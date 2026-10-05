/**
 * @file web_server.c
 * @brief 调参网页服务器实现 (esp_http_server)
 * @note  单客户端即可, 所有 handler 都在 httpd 任务里跑, 不额外开任务。
 *        电机控制状态用静态变量存, 上电即停 (s_motor_on 初始为 0)。
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
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "WEB";

/* httpd 实例 (web_server_stop 用) */
static httpd_handle_t s_httpd = NULL;

/* ================= 电机手动测试状态 =================
 * 需求: 网页关闭 / 手机断开时电机必须停, 所以这里存"期望状态",
 * 由 watchdog 任务定期把实际输出拉回期望; 长时间没收到网页指令也自动停。 */
#define MOTOR_IDLE_STOP_MS   4000     /* 4 秒没收到网页请求就自动停 (看门狗兜底) */

/* 当前开启的电机: 0=停 1=电机A 2=电机B 3=A+B */
static uint8_t  s_motor_on  = 0;
/* 期望速度 (0~100) */
static int8_t   s_motor_spd = 0;
/* 期望方向: 1=正转, 2=反转 */
static uint8_t  s_motor_dir = 1;
/* 最后一次收到"网页还在线"证据的时间戳 (ms)
 * ⚠ 网页每 500ms 轮询一次 /api/status, 这就是心跳, 顺带当作电机看门狗的喂狗点。
 *    不能只在 /api/motor 时喂狗: 用户按住速度滑块拖动时不会重复发 /api/motor,
 *    只靠轮询喂狗才能保证拖滑块过程中电机不被误停。 */
static int64_t  s_motor_t   = 0;

/**
 * @brief 把当前期望状态真正写进电机
 * @note  也会被看门狗任务调用
 */
static void motor_apply(void)
{
    int8_t spd = (s_motor_spd > 100) ? 100 : s_motor_spd;
    if (s_motor_dir == 2) spd = -spd;         /* 反转取负 */
    if (spd < MOTOR_SPEED_MIN) spd = MOTOR_SPEED_MIN;
    if (spd > MOTOR_SPEED_MAX) spd = MOTOR_SPEED_MAX;

    int8_t sa = 0, sb = 0;
    if (s_motor_on == 1)      { sa = spd; }
    else if (s_motor_on == 2) { sb = spd; }
    else if (s_motor_on == 3) { sa = spd; sb = spd; }

    if (sa == 0 && sb == 0) motor_brake_all();
    else                    motor_set_speed_both(sa, sb);
}

/**
 * @brief 电机看门狗任务
 * @note  目的: 万一网页崩了 / 浏览器直接被杀掉 / 手机断网,
 *        电机不会一直转下去。超过 MOTOR_IDLE_STOP_MS 没收到请求就自动停。
 */
static void motor_watchdog_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_motor_on != 0 &&
            (esp_timer_get_time() / 1000 - s_motor_t) > MOTOR_IDLE_STOP_MS) {
            ESP_LOGW(TAG, "网页 %lld ms 无请求, 自动停电机",
                     (long long)MOTOR_IDLE_STOP_MS);
            s_motor_on = 0;
            motor_apply();
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

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
 */
static esp_err_t api_save(httpd_req_t *req)
{
    esp_err_t ret = param_save();
    char out[96];
    snprintf(out, sizeof(out), "{\"ok\":%s}",
             (ret == ESP_OK) ? "true" : "false");
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
 * @brief GET /api/status — 实时状态 (姿态/编码器/WiFi/电机/平衡控制)
 */
static esp_err_t api_status(httpd_req_t *req)
{
    char buf[512];

    /* 轮询本身就是"网页还在线"的心跳, 顺带喂电机看门狗 */
    s_motor_t = esp_timer_get_time() / 1000;

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
    const char *mname = s_motor_on == 0 ? "停止" :
                        (s_motor_dir == 2 ? "反转" : "正转");

    snprintf(buf, sizeof(buf),
             "{\"wifi\":\"%s\",\"clients\":%u,\"ip\":\"%s\",\"ssid\":\"%s\","
             "\"pitch\":%.2f,\"gyro\":%.2f,\"enc1\":%d,\"enc2\":%d,"
             "\"motor\":\"%s\",\"motor_on\":%u,\"heap\":%u,\"uptime\":%u,"
             /* 平衡控制: 中间量全部透出, 网页可画曲线看响应 */
             "\"bal_state\":\"%s\",\"bal_on\":%s,"
             "\"tgt_pitch\":%.2f,\"bal_out\":%.1f,\"spd_out\":%.2f,"
             "\"steer_out\":%.1f,\"speed\":%.1f,\"mot_a\":%d,\"mot_b\":%d,"
             /* 腿角几何补偿: leg_comp 是本周期实际减掉的量, 看它是否随腿角变化 */
             "\"leg_deg\":%u,\"leg_comp\":%.3f,\"leg_k\":%.3f}",
             wstate[ws], wifi_ap_get_client_count(), WIFI_AP_IP, ssid,
             (double)bal.pitch, (double)bal.gyro_x,
             encoder_get_count(ENCODER_1), encoder_get_count(ENCODER_2),
             mname, s_motor_on,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)(esp_timer_get_time() / 1000000ULL),
             bstate[st], balance_is_enabled() ? "true" : "false",
             (double)bal.target_pitch, (double)bal.bal_out,
             (double)bal.spd_out, (double)bal.steer_out,
             (double)bal.speed, bal.motor_a, bal.motor_b,
             servo_get_angle(SERVO_1),
             (double)bal.leg_comp,
             (double)param_get_float(PARAM_LEG_PITCH_K));

    return send_json(req, buf);
}

/**
 * @brief POST /api/motor — 电机手动测试
 * @note  body = {"on":1|2|3|0,"speed":0~100,"dir":1|2}
 */
static esp_err_t api_motor(httpd_req_t *req)
{
    char body[128];
    if (read_body(req, body, sizeof(body)) < 0) {
        return send_json(req, "{\"ok\":false,\"msg\":\"body too large\"}");
    }

    float v;
    if (json_num(body, "speed", &v)) {
        s_motor_spd = (int8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
    }
    if (json_num(body, "dir", &v)) {
        s_motor_dir = (uint8_t)(v < 1.5f ? 1 : 2);
    }
    if (json_num(body, "on", &v)) {
        s_motor_on = (uint8_t)(v < 0.5f ? 0 : (v > 3 ? 3 : v));
    }

    s_motor_t = esp_timer_get_time() / 1000;   /* 喂看门狗 */
    motor_apply();
    return send_json(req, "{\"ok\":true}");
}

/**
 * @brief POST /api/stop — 急停
 * @note  浏览器关页/断网时用 sendBeacon 打这个接口
 */
static esp_err_t api_stop(httpd_req_t *req)
{
    s_motor_on = 0;
    motor_apply();
    return send_json(req, "{\"ok\":true}");
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

/* ================= 腿角 → 俯仰角 标定工具 =================
 *
 * @brief 标定的物理含义
 *   MPU 装在车身上, 车身相对轮轴的角度由**腿弯曲多少**决定。
 *   我们要测的是"腿转 1° 时, MPU 读到的 Pitch 变多少度" = leg_k。
 *
 * @brief 怎么测才安全且准
 *   关键: **车身固定不动, 只让腿动**。
 *   - 电机全程刹车, 平衡控制必须处于关闭状态 (接口会强制)
 *   - 车身扶住 / 夹具固定 / 放平桌上, 保证 Pitch 不随腿动而变
 *   - 这样测出来的差值纯粹是"腿动的贡献", 混不进地面不平、轮子打滑的干扰
 *   - 全程车一动不动, 不存在摔倒风险
 *
 * @brief 采样策略
 *   每个腿角停留 CAL_SETTLE_MS 让姿态稳定, 再连采 CAL_SAMPLES 次求平均。
 *   平均能压掉互补滤波的残余抖动和 MPU 噪声。
 *
 * @brief 为什么要多个点拟合直线
 *   真实连杆有间隙/形变, 单点差值会带噪声。两个点算斜率误差大,
 *   4~5 个点做最小二乘拟合, 既能压噪又能检出"读数不随腿变"的情况
 *   (斜率≈0 说明 MPU 装的位置不受腿影响, 那就不需要补偿)。
 */

/* 标定参数: 停留稳定时间 / 每个点采样次数 / 采样间隔 */
#define CAL_SETTLE_MS    300
#define CAL_SAMPLES      30
#define CAL_SAMPLE_MS    10

/* 一次标定最多收几个点 (线性回归只需 2 个, 但多点拟合抗噪) */
#define CAL_MAX_POINTS   6

typedef struct {
    float phi;      /* 腿角 (°) */
    float pitch;    /* 该腿角下 Pitch 平均值 (°) */
} cal_point_t;

static cal_point_t s_cal_pts[CAL_MAX_POINTS];
static uint8_t     s_cal_cnt = 0;

/**
 * @brief 清空已收集的标定点
 */
static void cal_reset(void)
{
    s_cal_cnt = 0;
    memset(s_cal_pts, 0, sizeof(s_cal_pts));
}

/**
 * @brief POST /api/legcal — 腿角标定
 * @note  body = {"act":"reset"}
 *              {"act":"move","deg":80}
 *              {"act":"sample","deg":80}
 *              {"act":"fit"}
 *
 *   - reset : 清空已收的点
 *   - move  : 把腿设到 deg 度 (不采样), 让你先手动把车扶稳
 *   - sample: 把腿设到 deg 度, 等稳定后采 CAL_SAMPLES 次求平均, 存一个点
 *   - fit   : 对已收的点做最小二乘拟合, 算出 leg_k 写进 param (不落盘)
 */
static esp_err_t api_legcal(httpd_req_t *req)
{
    char body[96];
    if (read_body(req, body, sizeof(body)) < 0) {
        return send_json(req, "{\"ok\":false,\"msg\":\"body too large\"}");
    }

    char act[16];
    if (!json_str(body, "act", act, sizeof(act))) {
        return send_json(req, "{\"ok\":false,\"msg\":\"missing act\"}");
    }

    /* ⚠ 安全闸门: 标定期间绝不能让电机转。
     *   标定要靠车"不动"才准, 电机一转, Pitch 变化里就混进了车体运动,
     *   测出来的 leg_k 是错的。所以这里强制刹车 + 停平衡控制。
     *   放在解析完 act 之后、任何动作之前, 保证四个子动作都受保护。 */
    if (balance_is_enabled()) {
        balance_emergency_stop();
    }
    s_motor_on = 0;
    motor_apply();

    if (strcmp(act, "reset") == 0) {
        cal_reset();
        return send_json(req, "{\"ok\":true,\"n\":0}");

    } else if (strcmp(act, "move") == 0) {
        float v;
        if (!json_num(body, "deg", &v)) {
            return send_json(req, "{\"ok\":false,\"msg\":\"missing deg\"}");
        }
        servo_set_both((uint8_t)v);
        return send_json(req, "{\"ok\":true}");

    } else if (strcmp(act, "sample") == 0) {
        float v;
        if (!json_num(body, "deg", &v)) {
            return send_json(req, "{\"ok\":false,\"msg\":\"missing deg\"}");
        }
        if (s_cal_cnt >= CAL_MAX_POINTS) {
            return send_json(req, "{\"ok\":false,\"msg\":\"点太多了, 先 fit 或 reset\"}");
        }

        servo_set_both((uint8_t)v);
        vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));      /* 等姿态稳下来 */

        /* 采多次求平均, 压掉互补滤波残余抖动 */
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
        if (got == 0) {
            return send_json(req, "{\"ok\":false,\"msg\":\"MPU 读失败\"}");
        }

        s_cal_pts[s_cal_cnt].phi   = v;
        s_cal_pts[s_cal_cnt].pitch = sum / (float)got;
        s_cal_cnt++;

        char out[160];
        snprintf(out, sizeof(out),
                 "{\"ok\":true,\"n\":%u,\"deg\":%.1f,\"pitch\":%.3f,\"got\":%d}",
                 s_cal_cnt, (double)v, (double)(sum / (float)got), got);
        return send_json(req, out);

    } else if (strcmp(act, "fit") == 0) {
        if (s_cal_cnt < 2) {
            return send_json(req, "{\"ok\":false,\"msg\":\"至少要 2 个点\"}");
        }

        /* 最小二乘拟合 pitch = k * phi + b, k 就是 leg_k
         *
         * ⚠ 退化检查的正确做法是看**腿角跨度**, 不是看分母 den:
         *   den = n·Σx² − (Σx)² 是平方和量级, 就算 5 个点全挤在 80° 附近
         *   (跨度只有 1°), den 也还是几万, fabs(den)<1e-6 永远不成立,
         *   反而会算出 k = 噪声/1° 这种巨大的假斜率。
         *   所以直接要求: 腿角最大跨度至少要 20°, 否则拒绝。
         *
         * ⚠ 另外算出 k 之后要判断它是不是"噪声":
         *   |k| 很小 → MPU 位置几乎不受腿影响, 补偿没必要 (网页会提示)
         *   残差大   → 采样时车在动, 或腿角拉得不够开 (网页会提示重采) */
        float xmin = s_cal_pts[0].phi, xmax = xmin;
        for (uint8_t j = 1; j < s_cal_cnt; j++) {
            if (s_cal_pts[j].phi < xmin) xmin = s_cal_pts[j].phi;
            if (s_cal_pts[j].phi > xmax) xmax = s_cal_pts[j].phi;
        }
        if (xmax - xmin < 20.0f) {
            char out[128];
            snprintf(out, sizeof(out),
                     "{\"ok\":false,\"msg\":\"腿角跨度只 %.0f°, 至少要 20°\"}",
                     (double)(xmax - xmin));
            return send_json(req, out);
        }
        float sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (uint8_t j = 0; j < s_cal_cnt; j++) {
            float x = s_cal_pts[j].phi, y = s_cal_pts[j].pitch;
            sx += x; sy += y; sxx += x * x; sxy += x * y;
        }
        float n  = (float)s_cal_cnt;
        float den = n * sxx - sx * sx;
        /* 跨度检查已保证 den ≠ 0, 这里只做除零兜底 */
        if (fabsf(den) < 1e-6f) {
            return send_json(req, "{\"ok\":false,\"msg\":\"腿角太接近, 拉不开\"}");
        }
        float k   = (n * sxy - sx * sy) / den;
        float b   = (sy - k * sx) / n;

        /* 拟合优度: 各点残差的最大值, 太大说明读数不稳或中间有磕碰 */
        float max_res = 0.0f;
        for (uint8_t j = 0; j < s_cal_cnt; j++) {
            float res = fabsf(s_cal_pts[j].pitch - (k * s_cal_pts[j].phi + b));
            if (res > max_res) max_res = res;
        }

        /* 写进 param (只改内存, 要点"保存到 NVS"才落盘) */
        param_set_float(PARAM_LEG_PITCH_K, k);

        char out[224];
        snprintf(out, sizeof(out),
                 "{\"ok\":true,\"k\":%.4f,\"b\":%.3f,\"n\":%u,\"max_res\":%.3f}",
                 (double)k, (double)b, s_cal_cnt, (double)max_res);
        ESP_LOGI(TAG, "腿角标定: leg_k=%.4f (拟合 pitch=%.4f*phi%+.3f), %u 点, 残差<=%.3f",
                 (double)k, (double)k, (double)b, s_cal_cnt, (double)max_res);
        return send_json(req, out);
    }

    /* act 拼错时走到这: 正常只有 move/sample/fit/reset 四个 */
    return send_json(req, "{\"ok\":false,\"msg\":\"unknown act\"}");
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
    /* ⚠ 路由数必须 ≥ 实际注册数 (主页1 + 参数4 + 状态/执行6 + 平衡3 + 标定1 = 15),
     *    不够的话 httpd_register_uri_handler 会静默失败, 表现是"某个接口 404 但没报错" */
    cfg.max_uri_handlers = 18;
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

    /* 状态与执行 */
    httpd_uri_t u_st = { .uri = "/api/status", .method = HTTP_GET,  .handler = api_status       };
    httpd_uri_t u_mt = { .uri = "/api/motor",  .method = HTTP_POST, .handler = api_motor        };
    httpd_uri_t u_sp = { .uri = "/api/stop",   .method = HTTP_POST, .handler = api_stop         };
    httpd_uri_t u_svc= { .uri = "/api/servo",  .method = HTTP_POST, .handler = api_servo_center };
    httpd_uri_t u_rb = { .uri = "/api/reboot", .method = HTTP_POST, .handler = api_reboot       };
    httpd_register_uri_handler(s_httpd, &u_st);
    httpd_register_uri_handler(s_httpd, &u_mt);
    httpd_register_uri_handler(s_httpd, &u_sp);
    httpd_register_uri_handler(s_httpd, &u_svc);
    httpd_register_uri_handler(s_httpd, &u_rb);

    /* 平衡控制接口 */
    httpd_uri_t u_bl = { .uri = "/api/balance",      .method = HTTP_POST, .handler = api_balance        };
    httpd_uri_t u_bs = { .uri = "/api/balance_stop", .method = HTTP_POST, .handler = api_balance_stop   };
    httpd_uri_t u_br = { .uri = "/api/balance_reload", .method = HTTP_POST, .handler = api_balance_reload };
    httpd_register_uri_handler(s_httpd, &u_bl);
    httpd_register_uri_handler(s_httpd, &u_bs);
    httpd_register_uri_handler(s_httpd, &u_br);

    /* 腿角标定工具 */
    httpd_uri_t u_lc = { .uri = "/api/legcal", .method = HTTP_POST, .handler = api_legcal };
    httpd_register_uri_handler(s_httpd, &u_lc);

    /* 兜底 404 */
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, not_found);

    /* 电机看门狗 (首次创建, 保证电路上电电机一定是停的) */
    motor_apply();
    if (xTaskGetHandle("mwdt") == NULL) {
        xTaskCreate(motor_watchdog_task, "mwdt", 2560, NULL, 4, NULL);
    }

    ESP_LOGI(TAG, "网页服务器已启动, 打开 http://" WIFI_AP_IP);
    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    if (s_httpd == NULL) return ESP_OK;
    s_motor_on = 0;
    motor_apply();
    esp_err_t ret = httpd_stop(s_httpd);
    s_httpd = NULL;
    return ret;
}
