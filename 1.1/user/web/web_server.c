/**
 * @file web_server.c
 * @brief 调参网页服务器实现 (esp_http_server)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-04
 */

#include "web_server.h"
#include "web_page.h"
#include "param.h"
#include "wifi_ap.h"
#include "motor_driver.h"
#include "encoder.h"
#include "mpu9250.h"
#include "servo.h"
#include "balance.h"
#include "trim.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "WEB";

/* httpd 实例 (web_server_stop 用) */
static httpd_handle_t s_httpd = NULL;

/* ================= 通用工具 ================= */

/**
 * @brief 从 JSON 里取一个字符串字段
 * @param json JSON 字符串
 * @param key  字段名 (不带引号)
 * @param out  [出参] 取到的字符串 (以 '\0' 结尾)
 * @param cap  out 缓冲区大小
 * @return true=找到并取到
 */
static bool json_str(const char *json, const char *key, char *out, uint32_t cap)
{
    if (out == NULL || cap == 0) return false;
    out[0] = '\0';

    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) return false;
    p += strlen(pat);

    while (*p == ' ' || *p == ':') p++;
    if (*p != '"') return false;
    p++;

    uint32_t i = 0;
    while (*p && *p != '"' && i < cap - 1) out[i++] = *p++;
    out[i] = '\0';
    return true;
}

/**
 * @brief 读完请求 body, 尾部补 '\0'
 * @return 实际读到的长度 (不含 '\0'), 失败返回 -1
 */
static int read_body(httpd_req_t *req, char *buf, uint32_t len)
{
    if (req->content_len >= len) return -1;     /* 太大, 丢弃 */
    int total = 0;
    while (total < req->content_len) {
        int n = httpd_req_recv(req, buf + total, req->content_len - total);
        if (n <= 0) {
            if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
            return -1;
        }
        total += n;
    }
    buf[total] = '\0';
    return total;
}

/**
 * @brief 发送一段 JSON 响应
 */
static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");   /* 调参要每次读到最新值 */
    return httpd_resp_sendstr(req, json);
}

/**
 * @brief 从 JSON 里取一个数字字段
 * @return true=找到并取到
 */
static bool json_num(const char *json, const char *key, float *out)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    char *endp = NULL;
    float v = strtof(p, &endp);
    if (endp == p) return false;
    *out = v;
    return true;
}

/* ========== 参数 JSON 缓冲区大小 ==========
 * 实测 19 个参数全量输出约 3000 字节 (hint 是中文, UTF-8 占 3 字节/字), 余量 1100。
 * ⚠ 以后加参数超过 ~28 个必须同步加大, 否则 param_to_json 会被**静默截断**,
 *   网页 JSON 解析失败 → 滑块全空, 而且日志里不会报错。 */
#define JSON_BUF_SIZE   4096

/* ================= 参数 / 保存 / 状态 ================= */

/**
 * @brief GET /api/params — 返回全部参数表
 */
static esp_err_t api_params_get(httpd_req_t *req)
{
    static char buf[JSON_BUF_SIZE];   /* 静态, 避免 4KB 大栈占用 */
    param_to_json(buf, sizeof(buf));
    return send_json(req, buf);
}

/**
 * @brief POST /api/params — 批量设置参数
 */
static esp_err_t api_params_post(httpd_req_t *req)
{
    char body[512];
    if (read_body(req, body, sizeof(body)) < 0) {
        return send_json(req, "{\"ok\":false,\"msg\":\"body too large\"}");
    }
    esp_err_t ret = param_from_json(body);
    char out[64];
    snprintf(out, sizeof(out), "{\"ok\":%s}", (ret == ESP_OK) ? "true" : "false");
    return send_json(req, out);
}

/**
 * @brief POST /api/save — 参数 + 配平表落盘
 * @note  两个都存: 参数失败也要把配平表试一下,
 *        免得一个失败导致另一个没存
 */
static esp_err_t api_save(httpd_req_t *req)
{
    esp_err_t ret = param_save();
    esp_err_t tr  = trim_save();
    char out[64];
    snprintf(out, sizeof(out), "{\"ok\":%s}",
             (ret == ESP_OK && tr == ESP_OK) ? "true" : "false");
    return send_json(req, out);
}

/**
 * @brief POST /api/reset — 恢复默认参数 (仅内存, 不落盘)
 * @note  配平表**不受影响** —— 那是标定数据, 不是参数
 */
static esp_err_t api_reset(httpd_req_t *req)
{
    param_reset_default();
    return send_json(req, "{\"ok\":true}");
}

/**
 * @brief GET /api/status — 实时状态
 * @note  平衡控制的中间量全部透出, 网页可以画曲线看响应
 */
static esp_err_t api_status(httpd_req_t *req)
{
    /* 640 字节: 现有字段约 480, 余量充足。
     * ⚠ snprintf 会静默截断, 截断的 JSON 解析失败会让网页所有实时数据都变 "-",
     *   所以字段变多时要同步加大。 */
    char buf[640];

    bal_data_t bal = {0};
    balance_get_data(&bal);
    static const char *bstate[] = { "IDLE", "STANDBY", "BALANCING", "FALLEN" };
    bal_state_t st = balance_get_state();
    if ((int)st < 0 || (int)st > BAL_STATE_FALLEN) st = BAL_STATE_IDLE;

    static const char *wstate[] = { "停止", "启动中", "就绪", "错误" };
    wifi_ap_state_t ws = wifi_ap_get_state();
    char ssid[32];
    wifi_ap_get_ssid(ssid, sizeof(ssid));

    snprintf(buf, sizeof(buf),
             "{\"wifi\":\"%s\",\"clients\":%u,\"ip\":\"%s\",\"ssid\":\"%s\","
             "\"pitch\":%.2f,\"pitch_raw\":%.2f,\"gyro\":%.2f,"
             "\"enc1\":%d,\"enc2\":%d,\"heap\":%u,\"uptime\":%u,"
             "\"bal_state\":\"%s\",\"bal_on\":%s,"
             "\"tgt_pitch\":%.2f,\"bal_out\":%.1f,\"speed\":%.1f,"
             "\"mot_a\":%d,\"mot_b\":%d,"
             "\"servo_deg\":%u,\"trim_comp\":%.3f,\"trim_usable\":%s,"
             "\"dt_ms\":%.1f,\"arm_ms\":%.0f}",
             wstate[ws], wifi_ap_get_client_count(), WIFI_AP_IP, ssid,
             (double)bal.pitch, (double)bal.pitch_raw, (double)bal.gyro,
             encoder_get_count(ENCODER_1), encoder_get_count(ENCODER_2),
             (unsigned)esp_get_free_heap_size(),
             (unsigned)(esp_timer_get_time() / 1000000ULL),
             bstate[st], balance_is_enabled() ? "true" : "false",
             (double)bal.target_pitch, (double)bal.bal_out, (double)bal.speed,
             bal.motor_a, bal.motor_b,
             servo_get_angle(SERVO_1),
             (double)bal.trim_comp,
             trim_is_usable(2) ? "true" : "false",
             (double)(bal.dt * 1000.0f),
             (double)balance_get_arm_ms());

    return send_json(req, buf);
}

/**
 * @brief POST /api/servo — 舵机回中 (param 里的 servo_c)
 */
static esp_err_t api_servo_center(httpd_req_t *req)
{
    servo_set_both((uint8_t)param_get_float(PARAM_SERVO_CENTER));
    return send_json(req, "{\"ok\":true}");
}

/**
 * @brief POST /api/reboot — 重启
 * @note  参数改崩了救命用
 */
static esp_err_t api_reboot(httpd_req_t *req)
{
    send_json(req, "{\"ok\":true}");
    vTaskDelay(pdMS_TO_TICKS(200));   /* 让响应先发出去 */
    esp_restart();
    return ESP_OK;
}

/* ================= 平衡控制 ================= */

/**
 * @brief POST /api/balance — 启停平衡控制, body = {"on":1|0}
 */
static esp_err_t api_balance(httpd_req_t *req)
{
    char body[64];
    if (read_body(req, body, sizeof(body)) < 0) {
        return send_json(req, "{\"ok\":false,\"msg\":\"body too large\"}");
    }

    float v;
    if (!json_num(body, "on", &v)) {
        return send_json(req, "{\"ok\":false,\"msg\":\"missing 'on'\"}");
    }

    esp_err_t ret = balance_set_enable(v >= 0.5f);
    char out[128];
    snprintf(out, sizeof(out), "{\"ok\":%s,\"state\":\"%s\"}",
             (ret == ESP_OK) ? "true" : "false",
             (ret == ESP_OK) ? (balance_is_enabled() ? "on" : "off") : "rejected");
    return send_json(req, out);
}

/**
 * @brief POST /api/balance_stop — 急停 (清 PID 积分)
 */
static esp_err_t api_balance_stop(httpd_req_t *req)
{
    balance_emergency_stop();
    return send_json(req, "{\"ok\":true}");
}

/**
 * @brief POST /api/balance_reload — 重新读取控制参数
 * @note  调大 Kp 之后建议调一次, 避免旧积分残留导致猛冲
 */
static esp_err_t api_balance_reload(httpd_req_t *req)
{
    balance_reload_params();
    return send_json(req, "{\"ok\":true}");
}

/* ================= 配平标定 =================
 *
 * @brief 标定的物理含义 (一句话)
 *   腿拨到某个角度 → **你用手把车身扶到接近平衡** → 读 Pitch → 记下来。
 *   攒够几个点, 控制器就知道"腿在 X 度时我该盯多少度"。
 *
 * @brief 和 1.0 的区别
 *   1.0 要拿尺子量"电机中心↔舵机轴心直距", 还要分左右两条腿各采一次。
 *   1.1 只需要舵机角一个量, 一次采完, 不用尺子、不用拆腿。
 *   原因: 腿高根本没法准 (车一动高度就变), 而控制器其实不需要知道腿高,
 *   只需要知道"这个腿角对应的平衡姿态是多少度"。
 *
 * @brief 怎么测才安全
 *   - 车扶住 (放桌上 / 夹具 / 手扶), 电机全程刹车
 *   - 每次都**重新扶一次**到平衡位 —— 上一轮扶的位不算数
 *   - 接口每次调用都强制刹车 + 停平衡控制, 电机绝不会转
 */

/* 采样参数: 停留稳定时间 / 采样次数 / 采样间隔 */
#define CAL_SETTLE_MS    600
#define CAL_SAMPLES      30
#define CAL_SAMPLE_MS    10

/**
 * @brief 让腿到指定角度, 等姿态稳, 连采 N 次求平均 Pitch
 * @param deg       目标舵机角 (°)
 * @param pitch_out [出参] 平均原始 Pitch (°)
 * @param rate_out  [出参] 平均角速度 (°/s), 用来提示车还在不在晃
 * @return ESP_OK 成功
 *
 * @note  停留 CAL_SETTLE_MS 让互补滤波收敛, 再连采求平均压噪声。
 * @note  为什么连采 30 次: 扶车的手不可能完全不抖, 单次读数噪声有 ±0.3°。
 *        取平均能压掉大部分。
 */
static esp_err_t cal_settle_and_read(float deg, float *pitch_out, float *rate_out)
{
    servo_set_both((uint8_t)deg);
    vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

    float sum_p = 0.0f, sum_r = 0.0f;
    int got = 0;
    for (int k = 0; k < CAL_SAMPLES; k++) {
        euler_angle_t e = {0};
        mpu9250_axis_t g = {0};
        if (mpu9250_get_pose(&e, &g) == ESP_OK) {
            sum_p += e.pitch;
            sum_r += g.x;   /* 俯仰角速度 = 绕 MPU 的 X 轴, 不是 Y
                              * (Y 是车体左右轴, 对前后倾是噪声, 会永远提示"在晃") */
            got++;
        }
        vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_MS));
    }
    if (got == 0) return ESP_FAIL;

    *pitch_out = sum_p / (float)got;
    if (rate_out != NULL) *rate_out = sum_r / (float)got;
    return ESP_OK;
}

/**
 * @brief GET /api/trim — 取整张配平表
 * @note  返回: { ok, n, usable, span, pts:[{deg, pitch}, ...] }
 */
static esp_err_t api_trim_get(httpd_req_t *req)
{
    const trim_t *t = trim_get();

    /* 容量: 头 ~80B + count * 48B/条, 留余量 */
    int cap = 128 + t->count * 48;
    char *buf = malloc((size_t)cap);
    if (buf == NULL) return send_json(req, "{\"ok\":false,\"msg\":\"no mem\"}");

    int w = snprintf(buf, (size_t)cap,
                     "{\"ok\":true,\"n\":%u,\"usable\":%s,\"span\":%.3f,\"pts\":[",
                     t->count, trim_is_usable(2) ? "true" : "false",
                     (double)trim_pitch_span());
    for (uint8_t i = 0; i < t->count && w > 0 && (size_t)w < (size_t)cap; i++) {
        w += snprintf(buf + w, (size_t)cap - (size_t)w,
                      "%s{\"deg\":%.1f,\"pitch\":%.3f}", i ? "," : "",
                      (double)t->pts[i].servo_deg, (double)t->pts[i].pitch_deg);
    }
    if (w > 0 && (size_t)w + 2 < (size_t)cap) {
        w += snprintf(buf + w, (size_t)cap - (size_t)w, "]}");
    }
    esp_err_t r = send_json(req, buf);
    free(buf);
    return r;
}

/**
 * @brief POST /api/trim — 拨腿 / 读数 / 记录 / 删除 / 清空
 * @note  body = {"act":"move","deg":80}     拨腿 + 等稳 + 返回 Pitch (不记录)
 *              {"act":"add", "deg":80}      拨腿 + 读数 + 记进表
 *              {"act":"read"}               只读当前 Pitch, 不动腿
 *              {"act":"del", "deg":80}      删掉某个角度的点
 *              {"act":"clear"}              清空整张表
 *
 * ⚠ 安全闸门: 标定期间绝不能让电机转。每次调用都强制刹车 + 停平衡控制。
 */
static esp_err_t api_trim_post(httpd_req_t *req)
{
    char body[128];
    if (read_body(req, body, sizeof(body)) < 0) {
        return send_json(req, "{\"ok\":false,\"msg\":\"body too large\"}");
    }

    char act[16];
    if (!json_str(body, "act", act, sizeof(act))) {
        return send_json(req, "{\"ok\":false,\"msg\":\"missing act\"}");
    }

    /* 安全闸门放在解析完 act 之后、任何动作之前, 保证每个子动作都受保护 */
    if (balance_is_enabled()) {
        balance_emergency_stop();
    }
    motor_brake_all();

    char out[224];

    /* ---------- del: 删一个点 (不动腿, 不读 MPU) ---------- */
    if (strcmp(act, "del") == 0) {
        float deg;
        if (!json_num(body, "deg", &deg)) {
            return send_json(req, "{\"ok\":false,\"msg\":\"missing deg\"}");
        }
        trim_del_point(deg);
        const trim_t *t = trim_get();
        snprintf(out, sizeof(out), "{\"ok\":true,\"n\":%u}", t->count);
        return send_json(req, out);
    }

    /* ---------- clear: 清空表 ---------- */
    if (strcmp(act, "clear") == 0) {
        trim_clear();
        return send_json(req, "{\"ok\":true}");
    }

    /* ---------- read: 只读当前 Pitch, 不动腿 ---------- */
    if (strcmp(act, "read") == 0) {
        euler_angle_t e = {0};
        if (mpu9250_get_pose(&e, NULL) != ESP_OK) {
            return send_json(req, "{\"ok\":false,\"msg\":\"MPU 读失败\"}");
        }
        snprintf(out, sizeof(out), "{\"ok\":true,\"deg\":%u,\"pitch\":%.3f}",
                 servo_get_angle(SERVO_1), (double)e.pitch);
        return send_json(req, out);
    }

    /* ---------- move / add: 拨腿 + 等稳 + 采样 ---------- */
    if (strcmp(act, "move") == 0 || strcmp(act, "add") == 0) {
        float deg;
        if (!json_num(body, "deg", &deg)) {
            return send_json(req, "{\"ok\":false,\"msg\":\"missing deg\"}");
        }
        float pitch = 0.0f, rate = 0.0f;
        esp_err_t r = cal_settle_and_read(deg, &pitch, &rate);
        if (r != ESP_OK) return send_json(req, "{\"ok\":false,\"msg\":\"MPU 读失败\"}");

        if (strcmp(act, "move") == 0) {
            snprintf(out, sizeof(out),
                     "{\"ok\":true,\"deg\":%.1f,\"pitch\":%.3f,\"rate\":%.2f}",
                     (double)deg, (double)pitch, (double)rate);
            return send_json(req, out);
        }

        /* add: 存进表 */
        esp_err_t ur = trim_set_point(deg, pitch);
        if (ur != ESP_OK) {
            const char *msg = (ur == ESP_ERR_NO_MEM) ? "点数已满" : "俯仰角超出合理范围";
            snprintf(out, sizeof(out), "{\"ok\":false,\"msg\":\"%s\"}", msg);
            return send_json(req, out);
        }
        const trim_t *t = trim_get();
        ESP_LOGI(TAG, "记下配平点 %.0f° -> %.2f° (共 %u 点)",
                 (double)deg, (double)pitch, t->count);
        snprintf(out, sizeof(out),
                 "{\"ok\":true,\"n\":%u,\"deg\":%.1f,\"pitch\":%.3f,"
                 "\"usable\":%s,\"span\":%.3f}",
                 t->count, (double)deg, (double)pitch,
                 trim_is_usable(2) ? "true" : "false", (double)trim_pitch_span());
        return send_json(req, out);
    }

    return send_json(req, "{\"ok\":false,\"msg\":\"unknown act\"}");
}

/* ================= 主页 / 404 ================= */

/**
 * @brief GET / — 调参主页
 */
static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, WEB_PAGE_HTML);
}

/**
 * @brief 404 — 提示正确地址
 */
static esp_err_t not_found(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    httpd_resp_set_status(req, "404 Not Found");
    return httpd_resp_sendstr(req, "Not found. 请打开 http://" WIFI_AP_IP "/");
}

/* ================= 服务器启动 ================= */

esp_err_t web_server_start(void)
{
    if (s_httpd != NULL) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn   = httpd_uri_match_wildcard;
    /* ⚠ 路由数必须 ≥ 实际注册数 (主页1 + 参数4 + 状态1 + 舵机1 + 重启1
     *    + 平衡3 + 配平2 = 13), 不够的话 httpd_register_uri_handler 会**静默失败**,
     *    表现是"某个接口 404 但日志里没有报错" */
    cfg.max_uri_handlers = 16;
    cfg.lru_purge_enable = true;          /* 浏览器刷新多, 及时回收缓存 */

    esp_err_t ret = httpd_start(&s_httpd, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start 失败: %s", esp_err_to_name(ret));
        s_httpd = NULL;
        return ret;
    }

    /* 主页 (整页 HTML) */
    httpd_uri_t u_root = { .uri = "/", .method = HTTP_GET, .handler = root_get };
    httpd_register_uri_handler(s_httpd, &u_root);

    /* 参数 */
    httpd_uri_t u_pg = { .uri = "/api/params", .method = HTTP_GET,  .handler = api_params_get  };
    httpd_uri_t u_pp = { .uri = "/api/params", .method = HTTP_POST, .handler = api_params_post };
    httpd_uri_t u_sv = { .uri = "/api/save",   .method = HTTP_POST, .handler = api_save        };
    httpd_uri_t u_rs = { .uri = "/api/reset",  .method = HTTP_POST, .handler = api_reset       };
    httpd_register_uri_handler(s_httpd, &u_pg);
    httpd_register_uri_handler(s_httpd, &u_pp);
    httpd_register_uri_handler(s_httpd, &u_sv);
    httpd_register_uri_handler(s_httpd, &u_rs);

    /* 状态与其他 */
    httpd_uri_t u_st = { .uri = "/api/status", .method = HTTP_GET,  .handler = api_status        };
    httpd_uri_t u_sc = { .uri = "/api/servo",  .method = HTTP_POST, .handler = api_servo_center  };
    httpd_uri_t u_rb = { .uri = "/api/reboot", .method = HTTP_POST, .handler = api_reboot        };
    httpd_register_uri_handler(s_httpd, &u_st);
    httpd_register_uri_handler(s_httpd, &u_sc);
    httpd_register_uri_handler(s_httpd, &u_rb);

    /* 平衡控制 */
    httpd_uri_t u_bl = { .uri = "/api/balance",        .method = HTTP_POST, .handler = api_balance        };
    httpd_uri_t u_bs = { .uri = "/api/balance_stop",   .method = HTTP_POST, .handler = api_balance_stop   };
    httpd_uri_t u_br = { .uri = "/api/balance_reload", .method = HTTP_POST, .handler = api_balance_reload };
    httpd_register_uri_handler(s_httpd, &u_bl);
    httpd_register_uri_handler(s_httpd, &u_bs);
    httpd_register_uri_handler(s_httpd, &u_br);

    /* 配平标定 */
    httpd_uri_t u_trg = { .uri = "/api/trim", .method = HTTP_GET,  .handler = api_trim_get  };
    httpd_uri_t u_trp = { .uri = "/api/trim", .method = HTTP_POST, .handler = api_trim_post };
    httpd_register_uri_handler(s_httpd, &u_trg);
    httpd_register_uri_handler(s_httpd, &u_trp);

    /* 兜底 404 */
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, not_found);

    ESP_LOGI(TAG, "网页服务器已启动, 打开 http://" WIFI_AP_IP);
    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    if (s_httpd == NULL) return ESP_OK;
    esp_err_t ret = httpd_stop(s_httpd);
    s_httpd = NULL;
    return ret;
}
