/**
 * @file onenet_client.c
 * @brief OneNet MQTT 客户端实现 (onenet_client.h)
 * @date 2026-10-10
 *
 * 依赖: ESP-IDF mqtt 客户端 (menuconfig: CONFIG_MQTT_PROTOCOL_311=y)
 *
 * 实现要点:
 *   1. token 计算: 用设备级 AccessKey 算 HMAC-SHA1, 跟 App 端 onenet.js 算法一致
 *   2. 后台任务: 100ms 周期 publish 设备属性 (姿态 / 电机 / 电池)
 *   3. 订阅: 物模型所有下行 topic, 收到后分发到 balance/param 模块
 *   4. 急停 / 起立 / 配平: 通过 service invoke 消息触发 (用 service 名查表)
 *
 * 不依赖外部 JSON 库, 手写最小解析 (OneJSON 格式很规整: 都是 {"key":{"value":..,"time":..}})
 */

#include "onenet_client.h"
#include "esp_log.h"
#include "mqtt_client.h"   // esp-mqtt 组件
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "mbedtls/md.h"
#include "mbedtls/base64.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "balance.h"
#include "param.h"
#include "led.h"
#include "servo.h"
#include "encoder.h"
#include "esp_timer.h"

static const char *TAG = "ONENET";

/* ========== URL 编码辅助 ========== */
static void url_encode_sign(const char *src, char *dst, size_t dst_size)
{
    size_t j = 0;
    for (size_t i = 0; src[i] && j < dst_size - 4; i++) {
        switch (src[i]) {
            case '+':  strcpy(dst + j, "%2B");  j += 3; break;
            case '/':  strcpy(dst + j, "%2F");  j += 3; break;
            case '=':  strcpy(dst + j, "%3D");  j += 3; break;
            case '%':  strcpy(dst + j, "%25");  j += 3; break;
            default:   dst[j++] = src[i];           break;
        }
    }
    dst[j] = '\0';
}

/* ========== 物模型 topic 拼接 ========== */
#define TOPIC_PROPERTY_POST        "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/post"
#define TOPIC_EVENT_POST           "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/event/post"
#define TOPIC_PROPERTY_SET         "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/set"
#define TOPIC_DESIRED_GET_REPLY    "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/desired/get/reply"
#define TOPIC_SERVICE_INVOKE_PRE   "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/service/"
#define TOPIC_SERVICE_INVOKE_SUF   "/invoke"

/* ========== 模块状态 ========== */
static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;
static TaskHandle_t s_pub_task = NULL;
static TaskHandle_t s_desired_pull = NULL;

/* 最近一次收到的 "期望属性" 缓存, 用于移动指令 (vx/vy/w) */
static volatile float s_target_vx = 0.0f;
static volatile float s_target_vy = 0.0f;
static volatile float s_target_w  = 0.0f;

/* ========== 工具: token 计算 ========== */

/**
 * @brief HMAC-SHA1 + base64, 跟 App 端 onenet.js 的 computeToken 等价
 * @param key_bin   设备 AccessKey base64 解码后的字节
 * @param key_len   长度
 * @param str_for_sig "et\nmethod\nres\nversion"
 * @param out_base64  输出 28 字节的 base64 (含 \0)
 * @return 0 成功
 */
static int hmac_sha1_base64(const uint8_t *key_bin, size_t key_len,
                            const char *str_for_sig, char *out_base64, size_t out_size)
{
    uint8_t mac[20];
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    if (mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA1), 1) != 0) {
        mbedtls_md_free(&ctx);
        return -1;
    }
    if (mbedtls_md_hmac_starts(&ctx, key_bin, key_len) != 0) {
        mbedtls_md_free(&ctx);
        return -1;
    }
    if (mbedtls_md_hmac_update(&ctx, (const uint8_t *)str_for_sig, strlen(str_for_sig)) != 0) {
        mbedtls_md_free(&ctx);
        return -1;
    }
    if (mbedtls_md_hmac_finish(&ctx, mac) != 0) {
        mbedtls_md_free(&ctx);
        return -1;
    }
    mbedtls_md_free(&ctx);

    size_t out_len = 0;
    if (mbedtls_base64_encode((unsigned char *)out_base64, out_size, &out_len, mac, sizeof(mac)) != 0) {
        return -1;
    }
    return 0;
}

/**
 * @brief 算 OneNet 设备 token, 写到 out 缓冲
 * @param out  输出 (至少 256 字节)
 * @param out_size
 * @return 0 成功
 */
static int compute_token(char *out, size_t out_size)
{
    /* 1. base64 解码 AccessKey */
    uint8_t key_bin[64];
    size_t key_len = 0;
    if (mbedtls_base64_decode(key_bin, sizeof(key_bin), &key_len,
                              (const uint8_t *)ONENET_DEVICE_AK, strlen(ONENET_DEVICE_AK)) != 0) {
        ESP_LOGE(TAG, "AccessKey base64 解码失败");
        return -1;
    }

    /* 2. et = 过期时间 (使用固定时间戳 2030.12.31, SDK 推荐值) */
    long et = 1924833600;  // 2030-12-30 12:00:00 UTC

    /* 3. res = products/{pid}/devices/{dev} */
    char res[128];
    snprintf(res, sizeof(res), "products/%s/devices/%s", ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);

    /* 4. StringForSignature = "{et}\n{method}\n{res}\n{version}" */
    /* 注意：version 必须放在最后！ */
    char str_for_sig[256];
    snprintf(str_for_sig, sizeof(str_for_sig), "%ld\nsha1\n%s\n2018-10-31", et, res);

    /* 5. sign = base64(hmac_sha1), 需要 URL 编码 */
    char sign_b64[64];
    if (hmac_sha1_base64(key_bin, key_len, str_for_sig, sign_b64, sizeof(sign_b64)) != 0) {
        ESP_LOGE(TAG, "HMAC-SHA1 计算失败");
        return -1;
    }

    /* 6. 拼 token, sign 中的 = 等需要 URL 编码 */
    char sign_encoded[128];
    url_encode_sign(sign_b64, sign_encoded, sizeof(sign_encoded));
    snprintf(out, out_size,
             "version=2018-10-31&res=products%%2F%s%%2Fdevices%%2F%s&et=%ld&method=sha1&sign=%s",
             ONENET_PRODUCT_ID, ONENET_DEVICE_NAME, et, sign_encoded);
    return 0;
}

/* ========== 工具: OneJSON 最小解析 ========== */

/**
 * @brief 在 {"key":{"value":N,"time":T}} 格式里提取指定 key 的 value (float)
 * @param json 完整 payload
 * @param key  要找的 key
 * @param out  找到后写到这里
 * @return 1=找到, 0=没找到
 */
static int json_find_value(const char *json, const char *key, float *out)
{
    /* 形如: "vx":{"value":0.3,"time":1700...}
     * 找 "key": 然后找 "value": 然后读数字 */
    char needle[32];
    snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char *p = strstr(json, needle);
    if (!p) return 0;
    p += strlen(needle);
    /* 找 "value" */
    const char *vp = strstr(p, "\"value\"");
    if (!vp) return 0;
    vp += 7;
    while (*vp == ' ' || *vp == ':') vp++;
    char *endp = NULL;
    float v = strtof(vp, &endp);
    if (endp == vp) return 0;
    *out = v;
    return 1;
}

/**
 * @brief 在 {"key":{"value":N,"time":T}} 格式里提取指定 key 的 value (int)
 * @param json 完整 payload
 * @param key  要找的 key
 * @param out  找到后写到这里
 * @return 1=找到, 0=没找到
 */
static int json_find_value_int(const char *json, const char *key, int *out)
{
    float v = 0;
    if (!json_find_value(json, key, &v)) return 0;
    *out = (int)v;
    return 1;
}

/**
 * @brief 在 {"key":{"value":N,"time":T}} 格式里提取指定 key 的 value (bool)
 * @param json 完整 payload
 * @param key  要找的 key
 * @param out  找到后写到这里
 * @return 1=找到, 0=没找到
 */
static int json_find_value_bool(const char *json, const char *key, bool *out)
{
    char needle[32];
    snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char *p = strstr(json, needle);
    if (!p) return 0;
    p += strlen(needle);

    /* 找 "value" */
    const char *vp = strstr(p, "\"value\"");
    if (!vp) return 0;
    vp += 7;
    while (*vp == ' ' || *vp == ':') vp++;

    /* 判断 true/false */
    if (strncmp(vp, "true", 4) == 0) {
        *out = true;
        return 1;
    } else if (strncmp(vp, "false", 5) == 0) {
        *out = false;
        return 1;
    }
    return 0;
}

/* ========== 工具: 拼属性 post payload ========== */

/**
 * @brief 用 snprintf 拼一个简版 OneJSON 属性 post body
 * @param buf   输出缓冲
 * @param size  缓冲大小
 * @param n     字段对数
 * @param ...   变参: 交替传 const char *key, double val
 */
static int build_property_post(char *buf, size_t size, int n, ...)
{
    long id = (long)(xTaskGetTickCount() * 1000ULL / configTICK_RATE_HZ) % 10000000000000L;
    long ts = (long)(xTaskGetTickCount() * 1000ULL / configTICK_RATE_HZ);

    int nw = snprintf(buf, size, "{\"id\":\"%013ld\",\"version\":\"1.0\",\"params\":{", id);
    if (nw < 0 || (size_t)nw >= size) return -1;

    va_list ap;
    va_start(ap, n);
    for (int i = 0; i < n; i++) {
        const char *k = va_arg(ap, const char *);
        double v = va_arg(ap, double);
        int w = snprintf(buf + nw, size - nw, "%s\"%s\":{\"value\":%g,\"time\":%ld}",
                         i ? "," : "", k, v, ts);
        if (w < 0 || (size_t)(nw + w) >= size) { va_end(ap); return -1; }
        nw += w;
    }
    va_end(ap);

    if ((size_t)(nw + 2) >= size) return -1;
    buf[nw++] = '}';
    buf[nw++] = '}';
    buf[nw]   = '\0';
    return nw;
}

/**
 * @brief 状态机转字符串
 */
static const char* state_to_string(bal_state_t state)
{
    switch (state) {
        case BAL_STATE_IDLE:      return "IDLE";
        case BAL_STATE_STANDBY:   return "STANDBY";
        case BAL_STATE_BALANCING: return "BALANCING";
        case BAL_STATE_FALLEN:    return "FALLEN";
        default:                  return "UNKNOWN";
    }
}

/* ========== 周期上报任务 ========== */

/**
 * @brief 周期 100ms 上报: 姿态 / 电机 / 编码器 / 设备状态
 * @note  走 $sys/.../thing/property/post
 */
static void publish_task(void *arg)
{
    ESP_LOGI(TAG, "publish 任务启动, 100ms 周期");
    int64_t start_us = esp_timer_get_time();  // 记录启动时间用于计算 uptime
    
    while (1) {
        if (s_connected) {
            bal_data_t d;
            if (balance_get_data(&d) == ESP_OK) {
                char body[768];  // 增大缓冲区以容纳更多属性
                int64_t uptime_s = (esp_timer_get_time() - start_us) / 1000000;
                
                int n = build_property_post(body, sizeof(body), 10,
                    "pitch",   (double)d.pitch,
                    "roll",    0.0,                      // 两轮平衡机器人无横滚角，上报0
                    "motor_l", (double)d.motor_a,
                    "motor_r", (double)d.motor_b,
                    "speed",   (double)d.speed,
                    "gz",      (double)d.gyro,
                    "enc_l",   (double)encoder_get_count(ENCODER_1),
                    "enc_r",   (double)encoder_get_count(ENCODER_2),
                    "state",   0.0,                      // 字符串后续单独处理
                    "uptime",  (double)uptime_s);
                if (n > 0) {
                    esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, body, n, 0, 0);
                    
                    // 单独上报 state 字符串属性
                    char state_body[128];
                    long id = (long)(xTaskGetTickCount() * 1000ULL / configTICK_RATE_HZ) % 10000000000000L;
                    int sn = snprintf(state_body, sizeof(state_body),
                        "{\"id\":\"%013ld\",\"version\":\"1.0\",\"params\":{\"state\":{\"value\":\"%s\",\"time\":%lld}}}",
                        id, state_to_string(d.state), (long long)(esp_timer_get_time() / 1000));
                    if (sn > 0) {
                        esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, state_body, sn, 0, 0);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief 周期 5s 拉一次期望属性 (云端的移动指令 vx/vy/w)
 * @note  也包含属性 post 时的最新一次值 (OneNet 期望值是云 → 设备 异步推送)
 */
static void desired_pull_task(void *arg)
{
    /* 启动后等几秒再拉, 让 MQTT 订阅生效 */
    vTaskDelay(pdMS_TO_TICKS(3000));
    const char *body = "{\"id\":\"0000000000001\",\"version\":\"1.0\",\"params\":[\"vx\",\"vy\",\"w\"]}";
    while (1) {
        if (s_connected) {
            esp_mqtt_client_publish(s_client,
                "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/desired/get",
                body, strlen(body), 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ========== 收到下行消息处理 ========== */

/**
 * @brief 处理灯控制属性
 * @param json 解析后的 JSON 字符串
 */
static void handle_led_control(const char *json)
{
    /* led_on: bool - 开/关 */
    bool led_on = false;
    if (json_find_value_bool(json, "led_on", &led_on)) {
        ESP_LOGI(TAG, "灯控制: led_on=%d", led_on);
        led_set_on(LED_1, led_on);
    }

    /* led_color: int - RGB 颜色值 (0x00RRGGBB)
     * 例如: 0xFF0000 = 红色, 0x00FF00 = 绿色, 0x0000FF = 蓝色 */
    int led_color = 0;
    if (json_find_value_int(json, "led_color", &led_color)) {
        uint8_t r = (led_color >> 16) & 0xFF;
        uint8_t g = (led_color >> 8) & 0xFF;
        uint8_t b = led_color & 0xFF;
        ESP_LOGI(TAG, "灯控制: led_color=0x%06X -> RGB(%d,%d,%d)", led_color, r, g, b);
        led_color_t color = { .r = r, .g = g, .b = b };
        led_set_color(LED_1, color);
    }

    /* led_brightness: int (0-255) - 亮度 */
    int led_brightness = 255;
    if (json_find_value_int(json, "led_brightness", &led_brightness)) {
        ESP_LOGI(TAG, "灯控制: led_brightness=%d", led_brightness);
        led_set_brightness(LED_1, (uint8_t)led_brightness);
    }

    /* led_preset: int - 预设颜色
     * 0=关闭, 1=红, 2=绿, 3=蓝, 4=黄, 5=青, 6=紫, 7=白 */
    int led_preset = -1;
    if (json_find_value_int(json, "led_preset", &led_preset)) {
        ESP_LOGI(TAG, "灯控制: led_preset=%d", led_preset);
        if (led_preset >= 0 && led_preset < LED_PRESET_MAX) {
            led_set_preset(LED_1, (led_preset_t)led_preset);
        }
    }
}

/**
 * @brief 处理舵机角度属性 (App 端下发 servo1_angle / servo2_angle)
 *        范围: SERVO_LIMIT_MIN(30) ~ SERVO_LIMIT_MAX(120)
 * @param json 解析后的 JSON 字符串
 */
static void handle_servo_control(const char *json)
{
    /* servo1_angle: int - 舵机1角度 (30-120) */
    int servo1_angle = -1;
    if (json_find_value_int(json, "servo1_angle", &servo1_angle)) {
        if (servo1_angle >= 0) {
            servo_set_angle(SERVO_1, (uint8_t)servo1_angle);
            ESP_LOGI(TAG, "舵机控制: servo1_angle=%d", servo1_angle);
        }
    }

    /* servo2_angle: int - 舵机2角度 (30-120) */
    int servo2_angle = -1;
    if (json_find_value_int(json, "servo2_angle", &servo2_angle)) {
        if (servo2_angle >= 0) {
            servo_set_angle(SERVO_2, (uint8_t)servo2_angle);
            ESP_LOGI(TAG, "舵机控制: servo2_angle=%d", servo2_angle);
        }
    }
}

/**
 * @brief 处理 $sys/.../thing/property/set 消息 (云下发的属性)
 *        主要是 vx / vy / w 移动指令 和 灯控制
 */
static void handle_property_set(const char *topic, int topic_len, const char *data, int data_len)
{
    char *json = strndup(data, data_len);
    if (!json) return;

    float vx = 0, vy = 0, w = 0;
    int has_vx = json_find_value(json, "vx", &vx);
    int has_vy = json_find_value(json, "vy", &vy);
    int has_w  = json_find_value(json, "w",  &w);

    if (has_vx) s_target_vx = vx;
    if (has_vy) s_target_vy = vy;
    if (has_w)  s_target_w  = w;

    ESP_LOGI(TAG, "property set: vx=%.2f vy=%.2f w=%.2f (has=%d/%d/%d)",
             vx, vy, w, has_vx, has_vy, has_w);

    /* 处理灯控制属性 */
    handle_led_control(json);

    /* 处理舵机角度属性 */
    handle_servo_control(json);

    free(json);
}

/**
 * @brief 处理 $sys/.../thing/property/desired/get/reply 消息
 *        (App 端走 SetDeviceDesiredProperty 推过来的移动指令)
 */
static void handle_desired_reply(const char *data, int data_len)
{
    char *json = strndup(data, data_len);
    if (!json) return;
    float vx = 0, vy = 0, w = 0;
    int has_vx = json_find_value(json, "vx", &vx);
    int has_vy = json_find_value(json, "vy", &vy);
    int has_w  = json_find_value(json, "w",  &w);
    if (has_vx) s_target_vx = vx;
    if (has_vy) s_target_vy = vy;
    if (has_w)  s_target_w  = w;
    ESP_LOGI(TAG, "desired reply: vx=%.2f vy=%.2f w=%.2f", vx, vy, w);
    free(json);
}

/**
 * @brief 处理 $sys/.../thing/service/{name}/invoke 消息
 *        (App 端的急停 / 起立 / 配平 / 参数读写 走这里)
 */
static void handle_service_invoke(const char *topic, int topic_len, const char *data, int data_len)
{
    /* 提取 service name: .../thing/service/{name}/invoke */
    /* 从 topic 末尾往回数第 2 段是 name, 最后是 invoke */
    char t[128] = {0};
    int copy = (topic_len < (int)sizeof(t) - 1) ? topic_len : (int)sizeof(t) - 1;
    memcpy(t, topic, copy);

    char *last = strrchr(t, '/');
    if (!last || strcmp(last, "/invoke") != 0) return;
    *last = '\0';
    char *name = strrchr(t, '/');
    if (!name) return;
    name++;

    ESP_LOGI(TAG, "service invoke: %s", name);

    if (strcmp(name, "stop") == 0) {
        balance_emergency_stop();
    } else if (strcmp(name, "stand") == 0) {
        balance_set_enable(true);
    } else if (strcmp(name, "calibrate") == 0) {
        balance_set_enable(false);
        vTaskDelay(pdMS_TO_TICKS(50));
        balance_set_enable(true);
    } else if (strcmp(name, "param_get") == 0) {
        /* 物模型定义 output: values: string
         * OneNet 要求应答格式: {"id":"...","code":0,"data":{"values":{"value":"{...}"}}} */
        char body[1024];
        int n = param_to_json(body, sizeof(body));
        if (n > 0) {
            char reply[1280];
            int rn = snprintf(reply, sizeof(reply),
                "{\"id\":\"%013ld\",\"code\":0,\"data\":{\"values\":{\"value\":\"%.*s\"}}}",
                (long)((xTaskGetTickCount() * 1000ULL / configTICK_RATE_HZ) % 10000000000000LL),
                n, body);
            if (rn > 0 && (size_t)rn < sizeof(reply)) {
                esp_mqtt_client_publish(s_client,
                    "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/service/param_get/invoke/reply",
                    reply, rn, 0, 0);
            }
        }
    } else if (strcmp(name, "param_set") == 0) {
        /* payload: {"id":"...","version":"1.0","params":{"values":{"value":"{...}","time":...}}}
         * values.value 是 JSON 字符串: "{\"bal_kp\":40,...}"
         * 提取内层字符串后直接喂给 param_from_json */
        char *json = strndup(data, data_len);
        if (json) {
            /* 找 "values":{"value":" ... "} */
            char *vp = strstr(json, "\"values\":{");
            if (vp) {
                vp += 10; // skip '"values":{'
                char *val_start = strstr(vp, "\"value\":\"");
                if (!val_start) val_start = strstr(vp, "\"value\": \"");
                if (val_start) {
                    val_start += 9;
                    char *val_end = strchr(val_start, '"');
                    if (val_end) {
                        int inner_len = (int)(val_end - val_start);
                        if (inner_len > 0 && inner_len < 1024) {
                            char inner[1024];
                            memcpy(inner, val_start, inner_len);
                            inner[inner_len] = '\0';
                            ESP_LOGI(TAG, "param_set inner JSON: %.*s", inner_len, val_start);
                            param_from_json(inner);
                        }
                    }
                }
            }
            free(json);
        }
    } else if (strcmp(name, "param_save") == 0) {
        param_save();
    }
}

/* ========== MQTT 事件回调 ========== */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT 已连上 broker");
        s_connected = true;
        /* 订阅物模型所有下行 topic */
        esp_mqtt_client_subscribe(s_client, TOPIC_PROPERTY_SET,        0);
        esp_mqtt_client_subscribe(s_client, TOPIC_DESIRED_GET_REPLY,   0);
        esp_mqtt_client_subscribe(s_client, TOPIC_SERVICE_INVOKE_PRE "stop/invoke",       0);
        esp_mqtt_client_subscribe(s_client, TOPIC_SERVICE_INVOKE_PRE "stand/invoke",      0);
        esp_mqtt_client_subscribe(s_client, TOPIC_SERVICE_INVOKE_PRE "calibrate/invoke",  0);
        esp_mqtt_client_subscribe(s_client, TOPIC_SERVICE_INVOKE_PRE "param_get/invoke",  0);
        esp_mqtt_client_subscribe(s_client, TOPIC_SERVICE_INVOKE_PRE "param_set/invoke",  0);
        esp_mqtt_client_subscribe(s_client, TOPIC_SERVICE_INVOKE_PRE "param_save/invoke", 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT 断开, 5s 后自动重连 (esp-mqtt 自带)");
        s_connected = false;
        break;
    case MQTT_EVENT_DATA: {
        char topic[160] = {0};
        int tlen = event->topic_len < (int)sizeof(topic) - 1 ? event->topic_len : (int)sizeof(topic) - 1;
        memcpy(topic, event->topic, tlen);
        topic[tlen] = '\0';

        if (strstr(topic, "/thing/property/set") && !strstr(topic, "/set/reply")) {
            handle_property_set(topic, tlen, event->data, event->data_len);
        } else if (strstr(topic, "/thing/property/desired/get/reply")) {
            handle_desired_reply(event->data, event->data_len);
        } else if (strstr(topic, "/thing/service/") && strstr(topic, "/invoke") && !strstr(topic, "/reply")) {
            handle_service_invoke(topic, tlen, event->data, event->data_len);
        } else {
            ESP_LOGI(TAG, "忽略 topic: %s (len=%d)", topic, event->data_len);
        }
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT 错误");
        break;
    default:
        break;
    }
}

/* ========== 对外 API ========== */

esp_err_t mqtt_client_start(void)
{
    if (s_client) return ESP_OK;

    /* 1. 算 token (在 mqtt 启动前算好) */
    char token[256];
    if (compute_token(token, sizeof(token)) != 0) {
        ESP_LOGI(TAG, "token 计算失败, mqtt 客户端不启动");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "token 算出");

    /* OneNET Studio MQTT Broker: studio-mqtt.heclouds.com:1883 (非加密) */
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://studio-mqtt.heclouds.com",
                .port = 1883,
            },
        },
        .credentials = {
            .username = ONENET_PRODUCT_ID,
            .client_id = ONENET_DEVICE_NAME,
            .authentication = {
                .password = token,
            },
        },
        .session = {
            .keepalive = 60,
            .disable_clean_session = false,
        },
        .network = {
            .timeout_ms = 15000,
        },
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) return ESP_ERR_NO_MEM;
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "esp_mqtt_client_start 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 3. 起两个后台任务 (增大栈避免溢出) */
    xTaskCreate(publish_task, "onenet_pub", 8192, NULL, 5, &s_pub_task);
    xTaskCreate(desired_pull_task, "onenet_desired", 4096, NULL, 4, &s_desired_pull);

    return ESP_OK;
}

esp_err_t mqtt_client_post_property(const char *json_payload)
{
    if (!s_connected || !s_client) return ESP_ERR_INVALID_STATE;
    int msg_id = esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST,
                                         json_payload, strlen(json_payload), 0, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_client_post_event(const char *name, const char *json_params)
{
    if (!s_connected || !s_client) return ESP_ERR_INVALID_STATE;
    char body[256];
    long id = (long)(xTaskGetTickCount() * 1000ULL / configTICK_RATE_HZ) % 10000000000000L;
    int n = snprintf(body, sizeof(body),
                     "{\"id\":\"%013ld\",\"version\":\"1.0\",\"name\":\"%s\",\"params\":%s}",
                     id, name, json_params);
    if (n <= 0 || (size_t)n >= sizeof(body)) return ESP_ERR_INVALID_SIZE;
    int msg_id = esp_mqtt_client_publish(s_client, TOPIC_EVENT_POST, body, n, 0, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

bool mqtt_client_is_connected(void)
{
    return s_connected;
}
