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
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

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
 * @brief GET /api/status — 实时状态 (姿态/编码器/WiFi/电机)
 */
static esp_err_t api_status(httpd_req_t *req)
{
    char buf[384];

    /* 轮询本身就是"网页还在线"的心跳, 顺带喂电机看门狗 */
    s_motor_t = esp_timer_get_time() / 1000;

    /* 姿态与角速度 (MPU 没接就填 0, 不报错) */
    float pitch = 0.0f, gyro_x = 0.0f;
    euler_angle_t euler = {0};
    mpu9250_axis_t  gyro  = {0};
    if (mpu9250_get_euler_angle(&euler) == ESP_OK) {
        pitch = euler.pitch + param_get_float(PARAM_ANGLE_ZERO);
    }
    if (mpu9250_read_gyro(&gyro) == ESP_OK) gyro_x = gyro.x;

    /* 编码器 */
    int enc1 = encoder_get_count(ENCODER_1);
    int enc2 = encoder_get_count(ENCODER_2);

    /* WiFi */
    static const char *wstate[] = { "停止", "启动中", "就绪", "错误" };
    wifi_ap_state_t ws = wifi_ap_get_state();
    char ssid[32];
    wifi_ap_get_ssid(ssid, sizeof(ssid));
    const char *mname = s_motor_on == 0 ? "停止" :
                        (s_motor_dir == 2 ? "反转" : "正转");

    snprintf(buf, sizeof(buf),
             "{\"wifi\":\"%s\",\"clients\":%u,\"ip\":\"%s\",\"ssid\":\"%s\","
             "\"pitch\":%.2f,\"gyro\":%.2f,\"enc1\":%d,\"enc2\":%d,"
             "\"motor\":\"%s\",\"motor_on\":%u,\"heap\":%u,\"uptime\":%u}",
             wstate[ws], wifi_ap_get_client_count(), WIFI_AP_IP, ssid,
             (double)pitch, (double)gyro_x, enc1, enc2,
             mname, s_motor_on,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)(esp_timer_get_time() / 1000000ULL));

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
    cfg.max_uri_handlers = 12;
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
