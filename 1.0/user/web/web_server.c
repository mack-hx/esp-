/**
 * @file web_server.c
 * @brief 调参网页服务器实现 (esp_http_server)
 * @author ESP32 机械狗项目
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
#include "legmap.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
 * @note  和 json_num() 成对使用。字符串值在 JSON 里必须用双引号包住。
 */
static bool json_str(const char *json, const char *key,
                     char *out, uint32_t cap)
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
 * @param req 请求
 * @param buf 缓冲区
 * @param len 缓冲区大小
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
 * @param json JSON 字符串
 * @param key  字段名 (不带引号)
 * @param out  [出参] 取到的值
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

/* ================= API 处理函数 ================= */

/* ========== 参数 JSON 缓冲区大小 ==========
 * 实测 19 个参数全量输出约 3000 字节, 给到 4096 留足余量。
 * 以后加参数超过 ~30 个必须同步加大这里, 否则 param_to_json 会被截断,
 * 网页 JSON 解析失败 → 滑块全空。 */
#define JSON_BUF_SIZE   4096

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
    char out[96];
    snprintf(out, sizeof(out), "{\"ok\":%s}",
             (ret == ESP_OK) ? "true" : "false");
    return send_json(req, out);
}

/**
 * @brief POST /api/balance — 平衡控制启停
 * @note  body = {"on":1|0}  1=启动平衡, 0=停止并回中
 * @warning ⚠ 骨架版: 三个 PID 的输出还没接到电机, 点了也不会真的平衡,
 *         只是状态机走起来。算法填好之后这个接口才真正生效。
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
 * @brief POST /api/balance_reload — 重新读取 param 里的控制参数
 * @note  调大 Kp 之后建议调一次, 避免旧积分残留导致猛冲
 */
static esp_err_t api_balance_reload(httpd_req_t *req)
{
    balance_reload_params();
    return send_json(req, "{\"ok\":true}");
}

/**
 * @brief POST /api/save — 参数落盘
 * @note  腿高标定表也一起存, 否则标定数据重启就丢
 */
static esp_err_t api_save(httpd_req_t *req)
{
    esp_err_t ret = param_save();
    /* 两个都存: 参数失败也要把标定表试一下, 免得一个失败导致另一个没存 */
    esp_err_t lm  = legmap_save();
    char out[96];
    snprintf(out, sizeof(out), "{\"ok\":%s}",
             (ret == ESP_OK && lm == ESP_OK) ? "true" : "false");
    return send_json(req, out);
}

/**
 * @brief POST /api/reset — 恢复默认 (仅内存, 不落盘)
 */
static esp_err_t api_reset(httpd_req_t *req)
{
    param_reset_default();
    return send_json(req, "{\"ok\":true}");
}

/**
 * @brief GET /api/status — 实时状态 (姿态/编码器/WiFi/平衡控制)
 */
static esp_err_t api_status(httpd_req_t *req)
{
    /* 768 字节: 现有字段约 500, 加腿高/Kp 后余量充足。
     * ⚠ snprintf 会静默截断, 截断的 JSON 解析失败会让网页所有实时数据都变 "-",
     *   所以字段变多时要同步加大。 */
    char buf[768];

    /* --- 平衡控制 (拿不到就全填 0) --- */
    bal_data_t bal = {0};
    balance_get_data(&bal);
    static const char *bstate[] = { "IDLE", "STANDBY", "BALANCING", "FALLEN" };
    bal_state_t st = balance_get_state();
    if ((int)st < 0 || (int)st > BAL_STATE_FALLEN) st = BAL_STATE_IDLE;

    /* --- WiFi --- */
    static const char *wstate[] = { "停止", "启动中", "就绪", "错误" };
    wifi_ap_state_t ws = wifi_ap_get_state();
    char ssid[32];
    wifi_ap_get_ssid(ssid, sizeof(ssid));

    snprintf(buf, sizeof(buf),
             "{\"wifi\":\"%s\",\"clients\":%u,\"ip\":\"%s\",\"ssid\":\"%s\","
             "\"pitch\":%.2f,\"gyro\":%.2f,\"enc1\":%d,\"enc2\":%d,"
             "\"heap\":%u,\"uptime\":%u,"
             /* 平衡控制: 中间量全部透出, 网页可画曲线看响应 */
             "\"bal_state\":\"%s\",\"bal_on\":%s,"
             "\"tgt_pitch\":%.2f,\"bal_out\":%.1f,\"spd_out\":%.2f,"
             "\"steer_out\":%.1f,\"speed\":%.1f,\"mot_a\":%d,\"mot_b\":%d,"
             /* 腿高标定: leg_h 是查表推出的直距(mm, 左路), usable=false 说明还没标定 */
             "\"leg_deg_l\":%u,\"leg_deg_r\":%u,\"leg_comp\":%.3f,\"leg_k\":%.3f,"
             "\"leg_h\":%.1f,\"leg_usable_l\":%s,\"leg_usable_r\":%s,\"kp_eff\":%.1f,"
             /* 实测周期 (ms): 正常应≈10。偏大说明控制任务被抢占,
              * 此时 PID 用的是实测 dt 而不是标称值, 增益会有偏差 */
             "\"dt_ms\":%.1f,\"arm_ms\":%.0f}",
             wstate[ws], wifi_ap_get_client_count(), WIFI_AP_IP, ssid,
             (double)bal.pitch, (double)bal.gyro_x,
             encoder_get_count(ENCODER_1), encoder_get_count(ENCODER_2),
             (unsigned)esp_get_free_heap_size(),
             (unsigned)(esp_timer_get_time() / 1000000ULL),
             bstate[st], balance_is_enabled() ? "true" : "false",
             (double)bal.target_pitch, (double)bal.bal_out,
             (double)bal.spd_out, (double)bal.steer_out,
             (double)bal.speed, bal.motor_a, bal.motor_b,
             servo_get_angle(SERVO_1), servo_get_angle(SERVO_2),
             (double)bal.leg_comp,
             (double)param_get_float(PARAM_LEG_PITCH_K),
             (double)bal.leg_height,
             legmap_is_usable(LEG_LEFT, 2)  ? "true" : "false",
             legmap_is_usable(LEG_RIGHT, 2) ? "true" : "false",
             (double)bal.kp_eff,
             (double)(bal.dt * 1000.0f),
             (double)balance_get_arm_ms());

    return send_json(req, buf);
}

/**
 * @brief POST /api/reboot — 重启
 * @note  参数改崩了救命用
 */
static esp_err_t api_reboot(httpd_req_t *req)
{
    send_json(req, "{\"ok\":true}");
    /* 延迟重启, 让响应先发出去 */
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

/**
 * @brief POST /api/servo — 舵机回中
 * @note  用 param 里的 servo_c (中心角)
 */
static esp_err_t api_servo_center(httpd_req_t *req)
{
    servo_set_both((uint8_t)param_get_float(PARAM_SERVO_CENTER));
    return send_json(req, "{\"ok\":true}");
}

/* ================= 腿高标定工具 (实测查表, 左右腿各一份) =================
 *
 * @brief 标定的物理含义
 *   MPU 装在车身上, 车身相对轮轴的角度由左右两腿弯曲**共同**决定。
 *   要建立的是两张实测表: 每条腿一份 { 舵机角, 电机中心↔舵机轴心直距, Pitch }。
 *
 * @brief 为什么"电机中心↔舵机轴心"而不用"轮心离地高度"
 *   离地高度在车摆动时会跟着变, 测一次一个数, 数据不稳。
 *   这两个点都是打印件/金属件上的硬位置, 尺子一量就是定值,
 *   不受车身姿态/MPU 漂移/弹簧回弹的影响, 录一次能用很久。
 *
 * @brief 为什么实测而不是四连杆运动学
 *   解析运动学要靠 4 根杆长 + 铰接点坐标反推机构姿态, 但杆长测量有误差,
 *   打印件有形变, 销孔有间隙 —— 在死点附近误差会被放大。
 *   实测量的是真实世界, 把这些误差全都包含进去了。配平用途实测远优于推算。
 *
 * @brief 怎么测才安全且准
 *   关键: **标定哪一条腿, 就只动哪一条, 另一条固定到中立位**。
 *   - 现在两路舵机硬件是镜像的, 一次"动腿"两条都动
 *   - 标定流程: 第一次标左腿 (右腿物理上拆开放到中立位), 第二次再换过来标右腿
 *   - 电机全程刹车, 平衡控制必须处于关闭状态 (接口会强制)
 *   - 车身扶住 / 夹具固定 / 放平桌上, 保证 Pitch 变化纯粹来自被标定那条腿
 *
 * @brief 直距是用户量的, ESP32 量不了自己
 *   流程: 网页让你把腿拨到某个角度并等姿态稳定 → 网页读 Pitch →
 *         你拿尺子量"电机中心↔舵机输出轴中心"距离填进去 → 提交。
 */

/* 标定参数: 停留稳定时间 / Pitch 采样次数 / 采样间隔 */
#define CAL_SETTLE_MS    300
#define CAL_SAMPLES      30
#define CAL_SAMPLE_MS    10

/**
 * @brief 让腿到指定角度并等姿态稳定, 返回该姿态的平均 Pitch
 * @param deg 目标舵机角 (°)
 * @param pitch_out [出参] 平均 Pitch (°)
 * @return ESP_OK 成功
 * @note  停留 300ms 让互补滤波收敛, 再连采 30 次求平均压噪声
 */
static esp_err_t cal_settle_and_read(float deg, float *pitch_out)
{
    servo_set_both((uint8_t)deg);
    vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

    float sum = 0.0f;
    int got = 0;
    for (int k = 0; k < CAL_SAMPLES; k++) {
        euler_angle_t e = {0};
        if (mpu9250_get_pose(&e, NULL) == ESP_OK) {
            sum += e.pitch;
            got++;
        }
        vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_MS));
    }
    if (got == 0) return ESP_FAIL;

    *pitch_out = sum / (float)got;
    return ESP_OK;
}

/**
 * @brief GET /api/legmap — 取左/右两腿的标定表
 * @note  返回结构: { ok, legs: { L: { n, usable, pts:[...] }, R: { n, usable, pts:[...] } } }
 */
static esp_err_t api_legmap_get(httpd_req_t *req)
{
    const legmap_all_t *all = legmap_get();

    /* 算容量: 头 ~128B + 两个表 * count * 72B/条, 留余量 */
    int cap = 256;
    for (int k = 0; k < LEG_COUNT; k++) cap += all->legs[k].count * 80;
    char *buf = malloc((size_t)cap);
    if (buf == NULL) return send_json(req, "{\"ok\":false,\"msg\":\"no mem\"}");

    int w = snprintf(buf, (size_t)cap,
                     "{\"ok\":true,\"legs\":{");
    for (int k = 0; k < LEG_COUNT && w > 0 && (size_t)w < (size_t)cap; k++) {
        const legmap_t *m = &all->legs[k];
        w += snprintf(buf + w, (size_t)cap - (size_t)w,
                      "%s\"%c\":{\"n\":%u,\"usable\":%s,\"pts\":[",
                      k ? "," : "",
                      legmap_leg_char((leg_id_t)k),
                      m->count,
                      legmap_is_usable((leg_id_t)k, 2) ? "true" : "false");
        for (uint8_t i = 0; i < m->count && w > 0 && (size_t)w < (size_t)cap; i++) {
            w += snprintf(buf + w, (size_t)cap - (size_t)w,
                          "%s{\"deg\":%.1f,\"h\":%.1f,\"pitch\":%.3f}",
                          i ? "," : "",
                          (double)m->pts[i].servo_deg,
                          (double)m->pts[i].height_mm,
                          (double)m->pts[i].pitch_deg);
        }
        if (w > 0 && (size_t)w + 4 < (size_t)cap) {
            w += snprintf(buf + w, (size_t)cap - (size_t)w, "]}");
        }
    }
    if (w > 0 && (size_t)w + 3 < (size_t)cap) {
        w += snprintf(buf + w, (size_t)cap - (size_t)w, "}}");
    }
    esp_err_t r = send_json(req, buf);
    free(buf);
    return r;
}

/**
 * @brief POST /api/legmap — 录入/清空/动腿/读取 (按 leg 字段)
 * @note  body = {"act":"clear",          "leg":"L"|"R"}   缺省 leg=清两路
 *              {"act":"clear_all"}
 *              {"act":"move",  "leg":"L", "deg":80}       动腿 + 读 Pitch (不录入)
 *              {"act":"read",  "leg":"L"}                  只读当前 Pitch
 *              {"act":"add",   "leg":"L", "deg":80, "h":81.5}
 *                                                     录一个点 (直距由用户量)
 *
 * ⚠ 安全闸门: 标定期间绝不能让电机转。每次调用都强制刹车 + 停平衡控制。
 *
 * 标定那一条腿要保证: **另一条腿在物理上不参与** (现在两路镜像同动,
 * 所以每次只能采一行, 左/右各采一次)。leg 字段决定往哪张表里写。
 */
static esp_err_t api_legmap_post(httpd_req_t *req)
{
    char body[160];
    if (read_body(req, body, sizeof(body)) < 0) {
        return send_json(req, "{\"ok\":false,\"msg\":\"body too large\"}");
    }

    char act[16];
    if (!json_str(body, "act", act, sizeof(act))) {
        return send_json(req, "{\"ok\":false,\"msg\":\"missing act\"}");
    }

    /* leg 字段: 可选, 缺省清空动作作用到两路, 其他动作作用于 L 路 */
    leg_id_t leg = LEG_LEFT;
    char leg_str[8] = {0};
    if (json_str(body, "leg", leg_str, sizeof(leg_str))) {
        if (legmap_parse_leg(leg_str, &leg) != ESP_OK) {
            return send_json(req, "{\"ok\":false,\"msg\":\"bad leg, want L/R\"}");
        }

    }

    /* 放在解析完 act 之后、任何动作之前, 保证每个子动作都受保护 */
    if (balance_is_enabled()) {
        balance_emergency_stop();
    }
    /* 强制刹车, 标定期间绝不能让电机转 */
    motor_brake_all();

    if (strcmp(act, "clear") == 0) {
        legmap_clear(leg);
        return send_json(req, "{\"ok\":true}");
    } else if (strcmp(act, "clear_all") == 0) {
        legmap_clear_all();
        return send_json(req, "{\"ok\":true}");
    } else if (strcmp(act, "move") == 0 || strcmp(act, "read") == 0) {
        float deg;
        if (!json_num(body, "deg", &deg)) {
            return send_json(req, "{\"ok\":false,\"msg\":\"missing deg\"}");
        }
        float pitch = 0.0f;
        esp_err_t r = cal_settle_and_read(deg, &pitch);
        if (r != ESP_OK) return send_json(req, "{\"ok\":false,\"msg\":\"MPU 读失败\"}");

        char out[160];
        snprintf(out, sizeof(out),
                 "{\"ok\":true,\"leg\":\"%c\",\"deg\":%.1f,\"pitch\":%.3f}",
                 legmap_leg_char(leg), (double)deg, (double)pitch);
        return send_json(req, out);

    } else if (strcmp(act, "add") == 0) {
        float deg, h;
        if (!json_num(body, "deg", &deg) || !json_num(body, "h", &h)) {
            return send_json(req, "{\"ok\":false,\"msg\":\"missing deg or h\"}");
        }
        /* 直距必须由用户提供 —— ESP32 量不了自己 */
        float pitch = 0.0f;
        esp_err_t r = cal_settle_and_read(deg, &pitch);
        if (r != ESP_OK) return send_json(req, "{\"ok\":false,\"msg\":\"MPU 读失败\"}");

        esp_err_t ur = legmap_upsert_point(leg, deg, h, pitch);
        if (ur != ESP_OK) {
            const char *msg = (ur == ESP_ERR_NO_MEM) ? "点数已满" : "直距超出合理范围";
            char out[128];
            snprintf(out, sizeof(out), "{\"ok\":false,\"msg\":\"%s\"}", msg);
            return send_json(req, out);
        }

        const legmap_t *m = legmap_get_one(leg);
        ESP_LOGI(TAG, "录入腿高[%c]: %.0f° -> %.1fmm, pitch=%.2f° (共 %u 点)",
                 legmap_leg_char(leg), (double)deg, (double)h, (double)pitch, m->count);

        char out[224];
        snprintf(out, sizeof(out),
                 "{\"ok\":true,\"leg\":\"%c\",\"n\":%u,\"deg\":%.1f,"
                 "\"h\":%.1f,\"pitch\":%.3f,\"usable\":%s}",
                 legmap_leg_char(leg), m->count,
                 (double)deg, (double)h, (double)pitch,
                 legmap_is_usable(leg, 2) ? "true" : "false");
        return send_json(req, out);
    }

    return send_json(req, "{\"ok\":false,\"msg\":\"unknown act\"}");
}

/**
 * @brief POST /api/legmap_fit_legk — 用现有标定点反推线性 leg_k (兜底方案)
 * @note  查表可用时不需要这个。留着是因为: 标定点不够 (比如只有 2 个)
 *        或用户只想快速试一个值时, 可以走线性补偿。
 *
 *        body = {"leg":"L"|"R"}  缺省用 L
 *
 * ⚠ 线性只能近似。实测的 pitch~腿角 关系常常是 U 型,
 *   直线拟合会丢掉近一半补偿量。所以优先用查表。
 */
static esp_err_t api_legmap_fit_legk(httpd_req_t *req)
{
    char body[64];
    if (read_body(req, body, sizeof(body)) < 0) {
        return send_json(req, "{\"ok\":false,\"msg\":\"body too large\"}");
    }

    leg_id_t leg = LEG_LEFT;
    char leg_str[8] = {0};
    if (json_str(body, "leg", leg_str, sizeof(leg_str))) {
        if (legmap_parse_leg(leg_str, &leg) != ESP_OK) {
            return send_json(req, "{\"ok\":false,\"msg\":\"bad leg\"}");
        }
    }

    const legmap_t *m = legmap_get_one(leg);
    if (m == NULL) {
        return send_json(req, "{\"ok\":false,\"msg\":\"bad leg\"}");
    }
    if (m->count < 2) {
        return send_json(req, "{\"ok\":false,\"msg\":\"至少要 2 个标定点\"}");
    }

    /* ⚠ 退化检查看**腿角跨度**, 不看分母:
     *   den = n·Σx² − (Σx)² 是平方和量级, 就算点全挤在 80° 附近(跨度 1°),
     *   den 也还是几万, fabs(den)<1e-6 永远不成立, 反而会算出 k=噪声/1°。
     */
    float xmin = m->pts[0].servo_deg, xmax = xmin;
    for (uint8_t i = 1; i < m->count; i++) {
        if (m->pts[i].servo_deg < xmin) xmin = m->pts[i].servo_deg;
        if (m->pts[i].servo_deg > xmax) xmax = m->pts[i].servo_deg;
    }
    if (xmax - xmin < 20.0f) {
        char out[160];
        snprintf(out, sizeof(out),
                 "{\"ok\":false,\"msg\":\"[%c] 腿角跨度只 %.0f°, 至少要 20°\"}",
                 legmap_leg_char(leg), (double)(xmax - xmin));
        return send_json(req, out);
    }

    /* 最小二乘拟合 pitch = k * deg + b, 扣除参考角后 k 就是 leg_k */
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (uint8_t i = 0; i < m->count; i++) {
        float x = m->pts[i].servo_deg, y = m->pts[i].pitch_deg;
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    float n   = (float)m->count;
    float den = n * sxx - sx * sx;
    if (fabsf(den) < 1e-6f) {
        return send_json(req, "{\"ok\":false,\"msg\":\"腿角太接近, 拉不开\"}");
    }
    float k = (n * sxy - sx * sy) / den;
    float b = (sy - k * sx) / n;

    /* 残差: 线性模型解释不了多少变化 → 说明曲线不是直线, 别用 leg_k */
    float max_res = 0.0f, span = 0.0f;
    for (uint8_t i = 0; i < m->count; i++) {
        float fit = k * m->pts[i].servo_deg + b;
        float res = fabsf(m->pts[i].pitch_deg - fit);
        if (res > max_res) max_res = res;
    }
    float pmin = m->pts[0].pitch_deg, pmax = pmin;
    for (uint8_t i = 1; i < m->count; i++) {
        if (m->pts[i].pitch_deg < pmin) pmin = m->pts[i].pitch_deg;
        if (m->pts[i].pitch_deg > pmax) pmax = m->pts[i].pitch_deg;
    }
    span = pmax - pmin;

    param_set_float(PARAM_LEG_PITCH_K, k);

    char out[256];
    snprintf(out, sizeof(out),
             "{\"ok\":true,\"leg\":\"%c\",\"k\":%.4f,\"n\":%u,"
             "\"max_res\":%.3f,\"span\":%.3f,\"res_ratio\":%.3f}",
             legmap_leg_char(leg),
             (double)k, m->count, (double)max_res, (double)span,
             (span > 0.01f) ? (double)(max_res / span) : 0.0);
    ESP_LOGI(TAG, "反推 leg_k[%c]=%.4f (%u 点, 残差<=%.3f, 变化幅度 %.3f)",
             legmap_leg_char(leg),
             (double)k, m->count, (double)max_res, (double)span);
    return send_json(req, out);
}

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
 * @brief 找不到路径 — 提示正确地址
 * @param req    请求
 * @param err    HTTP 错误码 (404)
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
    /* ⚠ 路由数必须 ≥ 实际注册数 (主页1 + 参数4 + 状态1 + 舵机1 + 重启1 + 平衡3 + 腿高3 = 14),
     *    不够的话 httpd_register_uri_handler 会静默失败, 表现是"某个接口 404 但没报错" */
    cfg.max_uri_handlers = 20;
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

    /* 参数接口 */
    httpd_uri_t u_pg = { .uri = "/api/params", .method = HTTP_GET,  .handler = api_params_get  };
    httpd_uri_t u_pp = { .uri = "/api/params", .method = HTTP_POST, .handler = api_params_post };
    httpd_uri_t u_sv = { .uri = "/api/save",   .method = HTTP_POST, .handler = api_save        };
    httpd_uri_t u_rs = { .uri = "/api/reset",  .method = HTTP_POST, .handler = api_reset       };
    httpd_register_uri_handler(s_httpd, &u_pg);
    httpd_register_uri_handler(s_httpd, &u_pp);
    httpd_register_uri_handler(s_httpd, &u_sv);
    httpd_register_uri_handler(s_httpd, &u_rs);

    /* 状态与其他 */
    httpd_uri_t u_st = { .uri = "/api/status", .method = HTTP_GET,  .handler = api_status       };
    httpd_uri_t u_svc= { .uri = "/api/servo",  .method = HTTP_POST, .handler = api_servo_center };
    httpd_uri_t u_rb = { .uri = "/api/reboot", .method = HTTP_POST, .handler = api_reboot       };
    httpd_register_uri_handler(s_httpd, &u_st);
    httpd_register_uri_handler(s_httpd, &u_svc);
    httpd_register_uri_handler(s_httpd, &u_rb);

    /* 平衡控制接口 */
    httpd_uri_t u_bl = { .uri = "/api/balance",      .method = HTTP_POST, .handler = api_balance        };
    httpd_uri_t u_bs = { .uri = "/api/balance_stop", .method = HTTP_POST, .handler = api_balance_stop   };
    httpd_uri_t u_br = { .uri = "/api/balance_reload", .method = HTTP_POST, .handler = api_balance_reload };
    httpd_register_uri_handler(s_httpd, &u_bl);
    httpd_register_uri_handler(s_httpd, &u_bs);
    httpd_register_uri_handler(s_httpd, &u_br);

    /* 腿高标定工具 */
    httpd_uri_t u_lmg = { .uri = "/api/legmap", .method = HTTP_GET,  .handler = api_legmap_get     };
    httpd_uri_t u_lmp = { .uri = "/api/legmap", .method = HTTP_POST, .handler = api_legmap_post    };
    httpd_uri_t u_lmf = { .uri = "/api/legmap_fit_legk", .method = HTTP_POST, .handler = api_legmap_fit_legk };
    httpd_register_uri_handler(s_httpd, &u_lmg);
    httpd_register_uri_handler(s_httpd, &u_lmp);
    httpd_register_uri_handler(s_httpd, &u_lmf);

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
