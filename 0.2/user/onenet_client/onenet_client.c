/**
 * @file onenet_client.c
 * @brief OneNet Studio MQTT 客户端实现 (onenet_client.h)
 * @date 2026-10-21
 *
 * 平台: OneNET Studio (Masa 平台), 产品 "平衡机械狗"
 *   - Broker: mqtt://studio-mqtt.heclouds.com:1883
 *   - username: 产品 ID, password: 设备级 token, clientId: 设备名
 *
 * OneJSON 协议要点 (官方文档:
 *   https://open.iot.10086.cn/doc/iot_platform/book/device-connect&manager/thing-model/protocol/OneJSON/property&event.html
 *   https://open.iot.10086.cn/doc/iot_platform/book/device-connect&manager/thing-model/protocol/OneJSON/service.html)
 *
 *   1. 属性上报 (设备→云): topic .../thing/property/post
 *      平台应答在 .../thing/property/post/reply (订阅它可以看到平台拒收哪个属性)
 *   2. 属性设置 (云→设备): topic .../thing/property/set
 *      params 里是**裸值**: {"params":{"vx":0.5}}, 不是 {"value":0.5}
 *      设备要回 .../thing/property/set_reply: {"id":"..","code":200,"msg":".."}
 *   3. 服务调用 (云→设备): topic .../thing/service/{identifier}/invoke
 *      设备要回 .../thing/service/{identifier}/invoke_reply (注意是 invoke_reply, 下划线)
 *      {"id":"<请求里的 id>","code":200,"msg":"success","data":{...}}
 *
 * @note  2026-10-21 修复: 服务应答 topic 之前写成 ".../invoke/reply" (斜杠),
 *        平台不认这个 topic, 收到就断开 MQTT 连接, 导致"调一次服务就掉线"。
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
#include "magnet.h"
#include "encoder.h"
#include "pcf8574.h"
#include "esp_timer.h"
#include "esp_netif_sntp.h"
#include <time.h>

static const char *TAG = "ONENET";

/* ========== 物模型 topic ==========
 * 官方 topic 表:
 *   属性上报   发布 .../thing/property/post      订阅 .../thing/property/post/reply
 *   属性设置   订阅 .../thing/property/set       发布 .../thing/property/set_reply
 *   服务调用   订阅 .../thing/service/{id}/invoke 发布 .../thing/service/{id}/invoke_reply
 */
#define TOPIC_PROPERTY_POST        "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/post"
#define TOPIC_PROPERTY_POST_REPLY  "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/post/reply"
#define TOPIC_PROPERTY_SET         "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/set"
#define TOPIC_PROPERTY_SET_REPLY   "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/set_reply"
#define TOPIC_DESIRED_GET          "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/desired/get"
#define TOPIC_DESIRED_GET_REPLY    "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/desired/get/reply"
#define TOPIC_EVENT_POST           "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/event/post"
#define TOPIC_SERVICE_INVOKE_PRE   "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/service/"
#define TOPIC_SERVICE_INVOKE_REPLY "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/service/%s/invoke_reply"
/* 服务类下行 topic 的通配符订阅 (一次性收齐所有服务的 invoke) */
#define TOPIC_SERVICE_WILDCARD    "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/service/#"

/* ========== 模块状态 ========== */
static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;
static TaskHandle_t s_pub_task = NULL;
static TaskHandle_t s_desired_pull = NULL;

/* 最近一次收到的远程指令 */
static volatile float s_target_vx = 0.0f;   /* 前后速度目标 (标幺 -1~1) */

/* ========== 工具: 消息 id ========== */

/**
 * @brief 生成一条唯一的消息 id (13 位数字)
 * @return 消息 id
 * @note  之前用 xTaskGetTickCount 的毫秒数当 id, 同一毫秒里发出的两帧
 *        (比如 float 属性帧和 int 属性帧) id 相同, 平台会回两条 id 一样的应答,
 *        设备这边按 id 对应时就对错了 (现象: 明明被拒收却显示"已就位")。
 *        所以改用自增序号, 保证每帧 id 唯一。
 */
static int64_t next_msg_id(void)
{
    static int64_t seq = 0;
    if (seq == 0) seq = esp_timer_get_time() / 1000;   /* 用开机毫秒数起头 */
    int64_t v = __sync_add_and_fetch(&seq, 1);
    return v % 10000000000000LL;
}

/* ========== 工具: token 计算 ========== */

/**
 * @brief HMAC-SHA1 + base64
 * @param key_bin   设备 AccessKey base64 解码后的字节
 * @param key_len   长度
 * @param str_for_sig "et\nmethod\nres\nversion"
 * @param out_base64  输出 base64 字符串 (含 '\0')
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
 * @brief 算 OneNet 设备 token
 * @param out  输出缓冲 (至少 256 字节)
 * @param out_size
 * @return 0 成功
 * @note  et 固定用 2030-12-30 的时间戳: OneNet MQTT broker 只认**长期有效**的
 *        token, 用"当前时间 + 24h"会被 broker 直接拒绝 (Bad username or password),
 *        而同一把 key 走 HTTP 接口却能通过。所以这里必须写死长期 et。
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

    /* 2. et = 固定过期时间 (2030-12-30 12:00:00 UTC) */
    long et = 1924833600;

    /* 3. res = products/{pid}/devices/{dev} */
    char res[128];
    snprintf(res, sizeof(res), "products/%s/devices/%s", ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);

    /* 4. StringForSignature = "{et}\n{method}\n{res}\n{version}" (version 放最后) */
    char str_for_sig[256];
    snprintf(str_for_sig, sizeof(str_for_sig), "%ld\nsha1\n%s\n2018-10-31", et, res);

    /* 5. sign = base64(hmac_sha1), 需要 URL 编码 */
    char sign_b64[64];
    if (hmac_sha1_base64(key_bin, key_len, str_for_sig, sign_b64, sizeof(sign_b64)) != 0) {
        ESP_LOGE(TAG, "HMAC-SHA1 计算失败");
        return -1;
    }

    /* 6. 拼 token (sign 里的 + / = 要 URL 编码) */
    char sign_encoded[128];
    size_t j = 0;
    for (size_t i = 0; sign_b64[i] && j < sizeof(sign_encoded) - 4; i++) {
        switch (sign_b64[i]) {
            case '+': memcpy(sign_encoded + j, "%2B", 3); j += 3; break;
            case '/': memcpy(sign_encoded + j, "%2F", 3); j += 3; break;
            case '=': memcpy(sign_encoded + j, "%3D", 3); j += 3; break;
            default:  sign_encoded[j++] = sign_b64[i];           break;
        }
    }
    sign_encoded[j] = '\0';

    snprintf(out, out_size,
             "version=2018-10-31&res=products%%2F%s%%2Fdevices%%2F%s&et=%ld&method=sha1&sign=%s",
             ONENET_PRODUCT_ID, ONENET_DEVICE_NAME, et, sign_encoded);
    return 0;
}

/* ========== 工具: OneJSON 最小解析 ========== */

/**
 * @brief 在 JSON 里找 "key" 对应的值
 *
 * 同时兼容两种写法 (平台在不同场景下发的格式不一样):
 *   a) 裸值:      "vx":0.5
 *   b) 带包装:    "vx":{"value":0.5,"time":1700000000000}
 *   c) 期望值:    "vx":{"expect_value":0.5,"time":...}
 *
 * @param json 完整 payload
 * @param key  要找的 key
 * @param out  找到后写到这里 (数值)
 * @return 1=找到, 0=没找到
 */
static int json_find_value(const char *json, const char *key, float *out)
{
    char needle[32];
    snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char *p = strstr(json, needle);
    if (!p) return 0;
    p += strlen(needle);
    while (*p == ' ') p++;

    /* 情况 b/c: 值是个对象, 进去找 value / expect_value */
    const char *vp = p;
    if (*vp == '{') {
        const char *v1 = strstr(p, "\"value\"");
        const char *v2 = strstr(p, "\"expect_value\"");
        if (v1 && v2)      vp = (v1 < v2) ? v1 : v2;
        else if (v1)       vp = v1;
        else if (v2)       vp = v2;
        else               return 0;
        vp += 6;
        while (*vp == ' ' || *vp == ':') vp++;
    }

    char *endp = NULL;
    float v = strtof(vp, &endp);
    if (endp == vp) return 0;
    *out = v;
    return 1;
}

/**
 * @brief 找 "key" 对应的整数值 (转调 json_find_value)
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
 * @brief 找 "key" 对应的布尔值
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
    while (*p == ' ') p++;

    if (*p == '{') {                      /* 包装写法: 进去找 value */
        const char *vp = strstr(p, "\"value\"");
        if (!vp) return 0;
        vp += 6;
        while (*vp == ' ' || *vp == ':') vp++;
        p = vp;
    }

    if (strncmp(p, "true", 4) == 0)  { *out = true;  return 1; }
    if (strncmp(p, "false", 5) == 0) { *out = false; return 1; }
    /* 数字写法: 0 = 假, 非 0 = 真 */
    if (*p == '0' || *p == '1') {
        *out = (*p != '0');
        return 1;
    }
    return 0;
}

/**
 * @brief 取 payload 里的消息 id (数字字符串)
 * @param json 完整 payload
 * @param out  输出缓冲
 * @param out_size
 * @return 1=找到, 0=没找到
 * @note  服务/属性应答必须回**同一个 id**, 平台靠它把应答和请求配对
 */
static int json_find_id(const char *json, char *out, size_t out_size)
{
    if (out_size == 0) return 0;
    out[0] = '\0';
    const char *p = strstr(json, "\"id\"");
    if (!p) return 0;
    p += 4;
    while (*p == ' ' || *p == ':') p++;
    if (*p != '"') return 0;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < out_size - 1) out[i++] = *p++;
    out[i] = '\0';
    return (i > 0);
}

/**
 * @brief 取 JSON 里某个 key 对应的**字符串值**, 并把转义还原
 * @param json      完整 payload
 * @param key       要找的 key
 * @param out       输出缓冲 (还原后的字符串)
 * @param out_size  缓冲大小
 * @return 1=找到, 0=没找到
 * @note  物模型里 values 这种 string 类型的参数, 平台下发时是
 *        "values":"{\"bal_kp\":40}", 内层的引号是转义的 (\")。
 *        直接用 strchr 找结束引号会在转义引号处截断, 必须自己走一遍转义。
 */
static int json_find_string(const char *json, const char *key, char *out, size_t out_size)
{
    if (out_size == 0) return 0;
    out[0] = '\0';

    char needle[32];
    snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char *p = strstr(json, needle);
    if (!p) return 0;
    p += strlen(needle);
    while (*p == ' ') p++;

    /* 兼容两种写法:
     *   a) "key":"字符串"
     *   b) "key":{"value":"字符串","time":...} */
    if (*p == '{') {
        const char *vp = strstr(p, "\"value\"");
        if (!vp) return 0;
        p = vp + 6;
        while (*p == ' ' || *p == ':') p++;
    }
    if (*p != '"') return 0;
    p++;

    size_t o = 0;
    while (*p && o < out_size - 1) {
        if (*p == '\\' && p[1]) {
            p++;
            switch (*p) {
                case 'n':  out[o++] = '\n'; break;
                case 't':  out[o++] = '\t'; break;
                case 'r':  out[o++] = '\r'; break;
                case '"':  out[o++] = '"';  break;
                case '\\': out[o++] = '\\'; break;
                case '/':  out[o++] = '/';  break;
                default:   out[o++] = *p;   break;
            }
            p++;
        } else if (*p == '"') {
            break;
        } else {
            out[o++] = *p++;
        }
    }
    out[o] = '\0';
    return 1;
}

/**
 * @brief 把普通字符串转成 JSON 字符串字面量 (带引号, 转义引号和反斜杠)
 * @param src  原始字符串
 * @param dst  输出缓冲
 * @param dst_size
 * @return 写入长度 (不含 '\0')
 */
static size_t json_escape_string(const char *src, char *dst, size_t dst_size)
{
    size_t o = 0;
    if (dst_size == 0) return 0;
    dst[o++] = '"';
    for (size_t i = 0; src[i] && o < dst_size - 3; i++) {
        if (src[i] == '"' || src[i] == '\\') {
            dst[o++] = '\\';
            dst[o++] = src[i];
        } else {
            dst[o++] = src[i];
        }
    }
    dst[o++] = '"';
    dst[o] = '\0';
    return o;
}

/* ========== 工具: 网络校时 (SNTP) ========== */

/**
 * @brief SNTP 校时成功的回调
 * @param timeinfo 校时后的时间信息
 */
static void sntp_time_sync_cb(struct timeval *timeinfo)
{
    ESP_LOGI(TAG, "SNTP 校时成功, 当前时间戳 %ld 秒", (long)timeinfo->tv_sec);
}

/**
 * @brief 启动 SNTP 校时 (后台异步, 不阻塞)
 * @note  平台会校验属性上报里的 time 字段, 必须是**真实时间戳 (Unix 毫秒)**。
 *        之前用开机以来的毫秒数上报, 平台直接拒收:
 *        code 2415 "invalid time:identifier:pitch"。
 */
static void sntp_start(void)
{
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("ntp.aliyun.com");
    cfg.sync_cb = sntp_time_sync_cb;
    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SNTP 启动失败 (%s), 属性上报将不带 time 字段", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "SNTP 已启动 (ntp.aliyun.com)");
    }
}

/**
 * @brief 取当前真实时间 (Unix 毫秒)
 * @return 毫秒时间戳; 没校上时返回 0 (调用方据此省略 time 字段, 该字段平台允许缺省)
 */
static int64_t now_epoch_ms(void)
{
    time_t t = time(NULL);
    if (t < 1704067200) return 0;   /* 2024-01-01 之前认为还没校时 */
    /* ⚠ 必须用 64 位: ESP32 的 long 是 32 位, Unix 毫秒时间戳 (~1.79e12)
     *   塞进 32 位会被截断成 2019 年的值, 平台直接回 "invalid time" 拒收。 */
    return (int64_t)t * 1000;
}

/* ========== 工具: 拼属性 post payload ========== */

/**
 * @brief 拼一个 OneJSON 属性 post body (float 属性)
 * @param buf   输出缓冲
 * @param size  缓冲大小
 * @param ts    属性生成时间 (Unix 毫秒, 64 位), 传 0 表示不带 time 字段
 * @param n     字段对数 (必须跟实际传入的键值对个数一致!)
 * @param ...   变参: 交替传 const char *key, double val
 * @return 实际长度 (不含 '\0'), <0 表示失败
 * @note  平台的 float 属性有 step 约束 (pitch/roll/yaw/gz 是 0.1, speed 是 0.01),
 *        值必须落在 step 的整数倍上, 否则平台回 2254 "float not conform step"。
 *        所以这里固定用 %.1f / %.2f 输出, 由调用方先把值圆整到 step。
 */
static int build_property_post_f(char *buf, size_t size, int64_t ts, int n, int decimals, ...)
{
    long id = (long)next_msg_id();

    int nw = snprintf(buf, size, "{\"id\":\"%013ld\",\"version\":\"1.0\",\"params\":{", id);
    if (nw < 0 || (size_t)nw >= size) return -1;

    va_list ap;
    va_start(ap, decimals);
    for (int i = 0; i < n; i++) {
        const char *k = va_arg(ap, const char *);
        double v = va_arg(ap, double);
        int w;
        if (ts > 0) {
            w = snprintf(buf + nw, size - nw, "%s\"%s\":{\"value\":%.*f,\"time\":%lld}",
                         i ? "," : "", k, decimals, v, (long long)ts);
        } else {
            w = snprintf(buf + nw, size - nw, "%s\"%s\":{\"value\":%.*f}",
                         i ? "," : "", k, decimals, v);
        }
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
 * @brief 拼一个 OneJSON 属性 post body (int32 属性)
 * @param buf   输出缓冲
 * @param size  缓冲大小
 * @param ts    属性生成时间 (Unix 毫秒, 64 位), 传 0 表示不带 time 字段
 * @param n     字段对数 (必须跟实际传入的键值对个数一致!)
 * @param ...   变参: 交替传 const char *key, long long val
 * @return 实际长度 (不含 '\0'), <0 表示失败
 * @note  int32 属性不能带小数 (平台回 2211 "int32 type error"), 所以单独一个函数。
 */
static int build_property_post_i(char *buf, size_t size, int64_t ts, int n, ...)
{
    long id = (long)next_msg_id();

    int nw = snprintf(buf, size, "{\"id\":\"%013ld\",\"version\":\"1.0\",\"params\":{", id);
    if (nw < 0 || (size_t)nw >= size) return -1;

    va_list ap;
    va_start(ap, n);
    for (int i = 0; i < n; i++) {
        const char *k = va_arg(ap, const char *);
        long long v = va_arg(ap, long long);
        int w;
        if (ts > 0) {
            w = snprintf(buf + nw, size - nw, "%s\"%s\":{\"value\":%lld,\"time\":%lld}",
                         i ? "," : "", k, v, (long long)ts);
        } else {
            w = snprintf(buf + nw, size - nw, "%s\"%s\":{\"value\":%lld}",
                         i ? "," : "", k, v);
        }
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
 * @brief 拼一个 OneJSON 属性 post body (单个字符串属性)
 * @param buf   输出缓冲
 * @param size  缓冲大小
 * @param ts    属性生成时间 (Unix 毫秒), 传 0 表示不带 time 字段
 * @param key   属性标识符
 * @param value 属性值 (字符串)
 * @return 实际长度 (不含 '\0'), <0 表示失败
 */
static int build_property_post_str(char *buf, size_t size, int64_t ts,
                                   const char *key, const char *value)
{
    long id = (long)next_msg_id();
    char vbuf[64];
    size_t vlen = json_escape_string(value, vbuf, sizeof(vbuf));
    if (vlen >= sizeof(vbuf)) return -1;

    int n;
    if (ts > 0) {
        n = snprintf(buf, size,
                     "{\"id\":\"%013ld\",\"version\":\"1.0\",\"params\":{\"%s\":{\"value\":%.*s,\"time\":%lld}}}",
                     id, key, (int)vlen, vbuf, (long long)ts);
    } else {
        n = snprintf(buf, size,
                     "{\"id\":\"%013ld\",\"version\":\"1.0\",\"params\":{\"%s\":{\"value\":%.*s}}}",
                     id, key, (int)vlen, vbuf);
    }
    if (n < 0 || (size_t)n >= size) return -1;
    return n;
}

/**
 * @brief 把浮点数圆整到 step 的整数倍, 并限制在 [min, max] 内
 * @param v    原始值
 * @param min 下限
 * @param max 上限
 * @param step 步长 (0.1 / 0.01 ...)
 * @return 圆整限幅后的值
 * @note  OneNet 平台的 float 属性有 step 约束, 值不是 step 的整数倍会被拒收
 *        (2254 float not conform step), 超范围会拒收 (2253 float over range)。
 */
static double clamp_step(double v, double min, double max, double step)
{
    if (v < min) v = min;
    if (v > max) v = max;
    return round(v / step) * step;
}

/**
 * @brief 把整数限制在 [min, max] 内
 */
static int clamp_int(int v, int min, int max)
{
    if (v < min) v = min;
    if (v > max) v = max;
    return v;
}

/**
 * @brief 状态机转字符串
 * @param state 状态
 * @return 状态名字符串
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

/**
 * @brief 读两个微动开关 (对接检测)
 * @param sw1 输出: 微动开关1 是否压下 (左)
 * @param sw2 输出: 微动开关2 是否压下 (右)
 * @note  PCF8574 的 P4/P5 接微动开关, 内部上拉, 压下拉低 (跟按键同约定)
 */
static void read_dock_switches(bool *sw1, bool *sw2)
{
    uint8_t state = 0xFF;
    if (pcf8574_read_all(&state) != ESP_OK) {
        *sw1 = false;
        *sw2 = false;
        return;
    }
    *sw1 = !(state & (1 << SW_K5));   /* P5 = 微动开关1 */
    *sw2 = !(state & (1 << SW_K6));   /* P4 = 微动开关2 */
}

/* ========== 周期上报任务 ========== */

/**
 * @brief 周期 200ms 上报: 姿态 / 角速度 / 电机 / 编码器 / 指令 / 状态
 * @note  走 $sys/.../thing/property/post
 *
 * @note  ⚠ 分帧上报的原因: 平台的 float 属性各自有 step 约束
 *        (pitch/roll/yaw 是 0.1, gz 是 0.01), 一帧里只要有一个属性
 *        不符合 step / 超范围, **整帧被拒收**, 其它属性也跟着丢。
 *        所以 step 不同的属性必须分帧发, 且每个属性的打印小数位
 *        要跟自己的 step 对齐。
 */
static void publish_task(void *arg)
{
    ESP_LOGI(TAG, "publish 任务启动, 200ms 周期");
    int64_t start_us = esp_timer_get_time();

    /* 上报用的缓冲放静态区, 不占任务栈 */
    static char body[512];    /* 角度帧 (step 0.1) */
    static char gzbody[128];  /* 角速度帧 (step 0.01) */
    static char ibody[512];   /* 整数帧 */
    static char sbody[128];   /* 字符串帧 */
    static char swbody[128];  /* 对接开关帧 */
    static char actbody[512]; /* 执行器状态帧 (灯/电磁铁) */
    static char svbody[256];  /* 舵机角度帧 */

    while (1) {
        if (s_connected) {
            bal_data_t d;
            if (balance_get_data(&d) == ESP_OK) {
                int64_t uptime_s = (esp_timer_get_time() - start_us) / 1000000;
                int64_t ts = now_epoch_ms();       /* 真实时间戳 (ms), 没校时就是 0 */

                /* --- 1. 角度帧 (step 0.1): pitch / roll / yaw --- */
                double pitch = clamp_step(d.pitch, -90.0, 90.0, 0.1);
                double roll  = 0.0;                       /* 两轮平衡机器人无横滚角 */
                double yaw   = 0.0;                       /* 无航向传感器 */
                int n = build_property_post_f(body, sizeof(body), ts, 3, 1,
                    "pitch", pitch,
                    "roll",  roll,
                    "yaw",   yaw);
                if (n > 0) {
                    esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, body, n, 0, 0);
                }

                /* --- 2. float 帧 (step 0.01): gz 角速度 + speed 移动指令 --- *
                 * speed 是"移动指令通道": 设备上报的是当前收到的指令值
                 * (0 / ±0.2~0.8 / ±1.2~1.8), 跟云端期望值语义一致。 */
                double gz    = clamp_step(d.gyro, -2000.0, 2000.0, 0.01);
                double speed = clamp_step((double)s_target_vx, -10.0, 10.0, 0.01);
                int ng = build_property_post_f(gzbody, sizeof(gzbody), ts, 2, 2,
                    "gz",    gz,
                    "speed", speed);
                if (ng > 0) {
                    esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, gzbody, ng, 0, 0);
                }

                /* --- 3. 整数帧: 电机输出 / 编码器 / 运行时长 --- */
                int n2 = build_property_post_i(ibody, sizeof(ibody), ts, 5,
                    "motor_l", (long long)clamp_int(d.motor_a, -100, 100),
                    "motor_r", (long long)clamp_int(d.motor_b, -100, 100),
                    "enc_l",   (long long)encoder_get_count(ENCODER_1),
                    "enc_r",   (long long)encoder_get_count(ENCODER_2),
                    "uptime",  (long long)uptime_s);
                if (n2 > 0) {
                    esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, ibody, n2, 0, 0);
                }

                /* --- 4. state 是字符串属性, 单独一帧 --- */
                int sn = build_property_post_str(sbody, sizeof(sbody), ts,
                                                 "state", state_to_string(d.state));
                if (sn > 0) {
                    esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, sbody, sn, 0, 0);
                }

                /* --- 5. 对接检测开关 (单独一帧, 1s 一帧) ---
                 * 物模型里还没建 sw1/sw2 时平台会拒收这一帧 (code 2306),
                 * 但不影响其它帧。建好之后自动就上报了。 */
                static int64_t last_sw_ms = 0;
                int64_t now_ms = esp_timer_get_time() / 1000;
                if (now_ms - last_sw_ms >= 1000) {
                    last_sw_ms = now_ms;
                    bool sw1 = false, sw2 = false;
                    read_dock_switches(&sw1, &sw2);
                    int n3 = build_property_post_i(swbody, sizeof(swbody), ts, 2,
                        "sw1", (long long)(sw1 ? 1 : 0),
                        "sw2", (long long)(sw2 ? 1 : 0));
                    if (n3 > 0) {
                        esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, swbody, n3, 0, 0);
                    }

                    /* --- 6. 执行器状态 (灯 / 电磁铁), 1s 一帧 ---
                     * 这几个属性 App 只是下发, 设备把当前状态报回去,
                     * App 才能显示真实状态 (而不是自己瞎猜)。 */
                    led_color_t lc = led_get_color(LED_1);
                    int32_t led_rgb = ((int32_t)lc.r << 16) | ((int32_t)lc.g << 8) | lc.b;
                    int n4 = build_property_post_i(actbody, sizeof(actbody), ts, 4,
                        "led_on",         (long long)(led_get_on(LED_1) ? 1 : 0),
                        "led_color",      (long long)led_rgb,
                        "led_brightness", (long long)led_get_brightness(LED_1),
                        "magnet_on",      (long long)(magnet_get(MAGNET_1) ? 1 : 0));
                    if (n4 > 0) {
                        esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, actbody, n4, 0, 0);
                    }

                    /* --- 7. 舵机角度 (float, step 1), 1s 一帧 --- */
                    int n5 = build_property_post_f(svbody, sizeof(svbody), ts, 2, 0,
                        "servo1_angle", (double)servo_get_angle(SERVO_1),
                        "servo2_angle", (double)servo_get_angle(SERVO_2));
                    if (n5 > 0) {
                        esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_POST, svbody, n5, 0, 0);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief 周期 5s 拉一次期望属性 (云端的期望值, 兜底)
 * @note  App 走 set-device-desired-property 时平台会直接推 property/set,
 *        这里再拉一次是为了 App 改了期望值但平台没推送的情况
 */
static void desired_pull_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    /* 拉取云端期望值 (兜底): 平台会把这些标识符的当前期望值放在 reply 里。
     * ⚠ 只能问**物模型里已经存在**的标识符, 只要有一个不存在,
     *   平台整条 reply 就回 2306 "not find identify: xxx"。
     *   本项目的移动指令通道是 speed (标幺 -1~1)。 */
    const char *body = "{\"id\":\"0000000000001\",\"version\":\"1.0\",\"params\":[\"speed\"]}";
    while (1) {
        if (s_connected) {
            esp_mqtt_client_publish(s_client, TOPIC_DESIRED_GET, body, strlen(body), 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ========== 收到下行消息处理 ========== */

/**
 * @brief 应用远程移动指令 (物模型属性 `speed`)
 * @param v 指令值 (编码见下)
 *
 * @note  指令编码 (平台属性 speed, 范围 -10~10):
 *
 *        0                        → 停止
 *        ±0.2 / ±0.4 / ±0.6 / ±0.8 → 前进(+) / 后退(-), 档位 20/40/60/80 (%)
 *        ±1.2 / ±1.4 / ±1.6 / ±1.8 → 左转(+) / 右转(-), 档位 20/40/60/80 (%)
 *
 *        档位与 UI 编码器调试页 (ui_encoder.c) 的调速档位同源,
 *        都取自 motor_spd_levels[]。
 *
 * @note  直驱期间平衡环不输出 (电机直接由本指令驱动), 跟编码器调试页一样是开环动作。
 */
static void apply_move_cmd(float v)
{
    float a = fabsf(v);
    bal_move_t dir = BAL_MOVE_STOP;
    int8_t level = 0;

    if (a <= 1.0f) {
        if (a >= 0.05f) {
            dir   = (v > 0.0f) ? BAL_MOVE_FWD : BAL_MOVE_BACK;
            level = (int8_t)lrintf(a * 100.0f);
        }
    } else if (a <= 2.0f) {
        dir   = (v > 0.0f) ? BAL_MOVE_LEFT : BAL_MOVE_RIGHT;
        level = (int8_t)lrintf((a - 1.0f) * 100.0f);
    }

    /* 记录原始指令值, 设备上报 `speed` 时用它 (后台能看出当前指令) */
    s_target_vx = v;

    balance_set_move(dir, level);
}

/**
 * @brief 处理灯控制属性
 * @param json 收到的 JSON
 */
static void handle_led_control(const char *json)
{
    /* led_on: bool - 开/关 */
    bool led_on = false;
    if (json_find_value_bool(json, "led_on", &led_on)) {
        ESP_LOGI(TAG, "灯控制: led_on=%d", led_on);
        led_set_on(LED_1, led_on);
    }

    /* led_color: int - RGB 颜色值 (0x00RRGGBB) */
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
        if (led_brightness < 0)   led_brightness = 0;
        if (led_brightness > 255) led_brightness = 255;
        ESP_LOGI(TAG, "灯控制: led_brightness=%d", led_brightness);
        led_set_brightness(LED_1, (uint8_t)led_brightness);
    }

    /* led_preset: int - 预设颜色 (0=关,1=红,...,7=白) */
    int led_preset = -1;
    if (json_find_value_int(json, "led_preset", &led_preset)) {
        if (led_preset >= 0 && led_preset < LED_PRESET_MAX) {
            led_set_preset(LED_1, (led_preset_t)led_preset);
        }
    }
}

/**
 * @brief 处理舵机角度属性
 * @param json 收到的 JSON
 * @note  servo1_angle / servo2_angle, 范围 30~120 度
 */
static void handle_servo_control(const char *json)
{
    int servo1_angle = -1;
    if (json_find_value_int(json, "servo1_angle", &servo1_angle)) {
        if (servo1_angle >= 0) {
            servo_set_angle(SERVO_1, (uint8_t)servo1_angle);
            ESP_LOGI(TAG, "舵机控制: servo1_angle=%d", servo1_angle);
        }
    }

    int servo2_angle = -1;
    if (json_find_value_int(json, "servo2_angle", &servo2_angle)) {
        if (servo2_angle >= 0) {
            servo_set_angle(SERVO_2, (uint8_t)servo2_angle);
            ESP_LOGI(TAG, "舵机控制: servo2_angle=%d", servo2_angle);
        }
    }
}

/**
 * @brief 处理电磁铁控制属性
 * @param json 收到的 JSON
 */
static void handle_magnet_control(const char *json)
{
    bool magnet_on = false;
    if (json_find_value_bool(json, "magnet_on", &magnet_on)) {
        magnet_set_both(magnet_on, magnet_on);
        ESP_LOGI(TAG, "电磁铁: %s", magnet_on ? "吸合" : "释放");
    }
}

/**
 * @brief 处理 $sys/.../thing/property/set 消息 (云下发的属性)
 * @param json 收到的 JSON (以 '\0' 结尾)
 */
static void handle_property_set(const char *json)
{
    ESP_LOGI(TAG, "收到属性设置: %s", json);

    /* --- 移动指令 ---
     * 物模型里没有 vx 属性, 用 speed 当移动指令通道。
     * 编码见 apply_move_cmd() 的注释。 */
    float vx = 0.0f;
    int has_vx = json_find_value(json, "vx", &vx);
    if (!has_vx) {
        float sp = 0.0f;
        if (json_find_value(json, "speed", &sp)) {
            vx = sp;
            has_vx = 1;
        }
    }
    if (has_vx) {
        apply_move_cmd(vx);
    } else {
        ESP_LOGI(TAG, "属性设置: 没有移动指令 (vx/speed), 只处理灯/舵机/电磁铁");
    }

    /* --- 灯 / 舵机 / 电磁铁 --- */
    handle_led_control(json);
    handle_servo_control(json);
    handle_magnet_control(json);
}

/**
 * @brief 处理 $sys/.../thing/property/desired/get/reply 消息
 * @param data     消息内容
 * @param data_len 长度
 */
static void handle_desired_reply(const char *data, int data_len)
{
    char *json = strndup(data, data_len);
    if (!json) return;
    /* 平台应答: code=200 才是正常带回了期望值, 其它都是错误 (比如标识符不存在) */
    int code = 200;
    json_find_value_int(json, "code", &code);
    if (code != 200) {
        static int64_t last_log_ms = 0;
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_log_ms > 30000) {
            last_log_ms = now;
            ESP_LOGW(TAG, "期望值应答异常: %.*s", data_len, data);
        }
    }
    /* 移动指令在物模型里叫 speed; vx 是老名字, 兼容一下 */
    float vx = 0.0f;
    int has = json_find_value(json, "speed", &vx);
    if (!has) has = json_find_value(json, "vx", &vx);
    if (has) {
        ESP_LOGI(TAG, "期望值应答: 移动指令=%.2f", (double)vx);
        apply_move_cmd(vx);
    }
    free(json);
}

/**
 * @brief 回一条服务应答
 * @param name    服务标识符 (stop / stand / ...)
 * @param req_id  请求里的消息 id (必须原样回传)
 * @param data_json data 字段的 JSON 片段 (没有就传 "{}")
 * @note  topic 是 .../thing/service/{name}/invoke_reply (下划线!)
 */
static void service_reply(const char *name, const char *req_id, const char *data_json)
{
    if (!s_connected || !s_client) return;

    static char topic[160];
    static char body[2048];     /* 静态区, 不占 MQTT 任务栈 */
    snprintf(topic, sizeof(topic), TOPIC_SERVICE_INVOKE_REPLY, name);

    int n = snprintf(body, sizeof(body),
                     "{\"id\":\"%s\",\"code\":200,\"msg\":\"success\",\"data\":%s}",
                     (req_id && req_id[0]) ? req_id : "0",
                     (data_json && data_json[0]) ? data_json : "{}");
    if (n <= 0 || (size_t)n >= sizeof(body)) return;

    esp_mqtt_client_publish(s_client, topic, body, n, 0, 0);
    ESP_LOGI(TAG, "服务应答 %s: %s", name, body);
}

/**
 * @brief 处理 $sys/.../thing/service/{name}/invoke 消息
 * @param topic     完整 topic (用来取服务名)
 * @param topic_len topic 长度
 * @param data      消息内容
 * @param data_len  长度
 */
static void handle_service_invoke(const char *topic, int topic_len, const char *data, int data_len)
{
    /* --- 1. 从 topic 尾部取出服务名 --- */
    static char t[160] = {0};       /* 静态区, 不占 MQTT 任务栈 */
    int copy = (topic_len < (int)sizeof(t) - 1) ? topic_len : (int)sizeof(t) - 1;
    memcpy(t, topic, copy);
    /* ⚠ 必须在这里补 '\0'! t 是 static 缓冲区, 上一次调用留下的尾巴
     *   不会被清掉。比如先收到 "param_save"(61 字符) 再收到 "stop"(55 字符),
     *   不补 '\0' 的话 t 会变成 ".../stop/invoke" + 残留的 "invoke",
     *   strrchr 找出来的最后一段是 "/invokeinvoke" != "/invoke",
     *   于是这个服务被直接丢掉, 平台收不到应答就报"设备响应超时"。 */
    t[copy] = '\0';

    char *last = strrchr(t, '/');
    if (!last || strcmp(last, "/invoke") != 0) return;
    *last = '\0';
    char *name = strrchr(t, '/');
    if (!name) return;
    name++;

    /* --- 2. 取请求 id (应答必须回同一个 id) --- */
    char *json = strndup(data, data_len);
    char req_id[32] = {0};
    if (json) json_find_id(json, req_id, sizeof(req_id));

    ESP_LOGI(TAG, "服务调用: %s (id=%s)", name, req_id[0] ? req_id : "-");

    /* --- 3. 执行 --- */
    if (strcmp(name, "stop") == 0) {
        /* 急停: 清直驱指令 + 停平衡 + 刹车 */
        balance_clear_move();
        balance_emergency_stop();
        apply_move_cmd(0.0f);
        service_reply(name, req_id, "{\"ok\":true}");
    } else if (strcmp(name, "stand") == 0) {
        /* 起立: 启动平衡控制。
         * 起立瞬间电机不会猛冲 —— balance 内部有软启动 (输出限幅从
         * BAL_RAMP_START_PCT 慢慢放开到 100%), 车还歪着也不会一上来就顶格。 */
        balance_clear_move();
        balance_set_enable(true);
        service_reply(name, req_id, "{\"ok\":true}");
    } else if (strcmp(name, "calibrate") == 0) {
        /* 配平: **只记录当前 pitch 作为平衡零点, 不驱动任何电机**。
         * 之前的实现是"停平衡 → 再开平衡", 电机会在重新起立时猛冲,
         * 看起来像配平引起剧烈抖动 —— 其实配平和电机没关系。 */
        balance_calibrate();
        service_reply(name, req_id, "{\"ok\":true}");
    } else if (strcmp(name, "param_get") == 0) {
        /* 物模型 output: values (string), 内容是整张参数表的 JSON (扁平键值对) */
        static char pbody[1024];    /* 静态区, 不占 MQTT 任务栈 */
        static char escaped[1200];
        static char data_json[1400];
        int n = param_to_map_json(pbody, sizeof(pbody));
        if (n > 0) {
            size_t elen = json_escape_string(pbody, escaped, sizeof(escaped));
            if (elen < sizeof(escaped)) {
                snprintf(data_json, sizeof(data_json), "{\"values\":%.*s}", (int)elen, escaped);
                service_reply(name, req_id, data_json);
            }
        }
    } else if (strcmp(name, "param_set") == 0) {
        /* 平台下发: {"id":"..","version":"1.0","params":{"values":"{\"bal_kp\":40,..}"}}
         * values 是物模型里声明的 string 类型, 内容是一段 JSON 字符串
         * (内层引号是转义的, 要用 json_find_string 还原) */
        int ok = 0;
        if (json) {
            static char inner[1024];   /* 静态区, 不占 MQTT 任务栈 */
            if (json_find_string(json, "values", inner, sizeof(inner)) && inner[0]) {
                ESP_LOGI(TAG, "param_set: %s", inner);
                ok = (param_from_json(inner) == ESP_OK);
            } else {
                ESP_LOGW(TAG, "param_set: 没找到 values 参数");
            }
        }
        /* ⚠ 物模型里 param_set 的**输出参数是空的** (其它 5 个服务都有 ok)。
         * 平台会校验应答里的标识符: 多一个它不认识的字段就报
         * "10415 response invalid", 所以这里只能回空对象。
         * 如果之后给 param_set 补上输出参数 ok (bool), 把下面这行
         * 改成 "{\"ok\":%s}" 即可 (ok ? "true" : "false")。 */
        (void)ok;
        service_reply(name, req_id, "{}");
    } else if (strcmp(name, "param_save") == 0) {
        esp_err_t ret = param_save();
        char data_json[32];
        snprintf(data_json, sizeof(data_json), "{\"ok\":%s}", (ret == ESP_OK) ? "true" : "false");
        service_reply(name, req_id, data_json);
    } else {
        service_reply(name, req_id, "{\"ok\":false}");
    }

    if (json) free(json);
}

/**
 * @brief 回一条属性设置应答
 * @param req_id  请求里的消息 id
 */
static void property_set_reply(const char *req_id)
{
    if (!s_connected || !s_client) return;
    char body[96];
    int n = snprintf(body, sizeof(body),
                     "{\"id\":\"%s\",\"code\":200,\"msg\":\"success\"}",
                     (req_id && req_id[0]) ? req_id : "0");
    if (n > 0 && (size_t)n < sizeof(body)) {
        esp_mqtt_client_publish(s_client, TOPIC_PROPERTY_SET_REPLY, body, n, 0, 0);
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
        /* 订阅所有下行 topic
         * 注意: 服务类 topic 用通配符一次性订阅
         *   $sys/{pid}/{dev}/thing/service/#
         * 之前逐个订阅 ".../service/{name}/invoke", 平台发下来的服务调用
         * (stop / stand / calibrate) 收不到, 表现为平台报"设备响应超时"。 */
        static const esp_mqtt_topic_t subs[] = {
            { TOPIC_PROPERTY_SET,       0 },
            { TOPIC_DESIRED_GET_REPLY,  0 },
            { TOPIC_PROPERTY_POST_REPLY, 0 },
            { TOPIC_SERVICE_WILDCARD,   0 },
        };
        int mid = esp_mqtt_client_subscribe_multiple(s_client, subs,
                                                     sizeof(subs) / sizeof(subs[0]));
        ESP_LOGI(TAG, "订阅 %d 个 topic -> msg_id=%d", (int)(sizeof(subs)/sizeof(subs[0])), mid);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT 断开, 5s 后自动重连 (esp-mqtt 自带)");
        s_connected = false;
        break;
    case MQTT_EVENT_DATA: {
        char topic[160] = {0};
        int tlen = event->topic_len < (int)sizeof(topic) - 1 ? event->topic_len : (int)sizeof(topic) - 1;
        memcpy(topic, event->topic, tlen);
        topic[tlen] = '\0';

        if (strstr(topic, "/thing/property/set") && !strstr(topic, "/set_reply")) {
            char *json = strndup(event->data, event->data_len);
            if (json) {
                char req_id[32] = {0};
                json_find_id(json, req_id, sizeof(req_id));
                handle_property_set(json);
                property_set_reply(req_id);
                free(json);
            }
        } else if (strstr(topic, "/thing/property/desired/get/reply")) {
            handle_desired_reply(event->data, event->data_len);
        } else if (strstr(topic, "/thing/property/post/reply")) {
            /* 平台对属性上报的应答: code != 200 说明这个属性被拒收 */
            char *json = strndup(event->data, event->data_len);
            if (json) {
                int code = 200;
                json_find_value_int(json, "code", &code);
                if (code != 200) {
                    /* 同一句拒收原因刷屏没意义, 5 分钟内只打一次 */
                    static int64_t last_log_ms = 0;
                    int64_t now = esp_timer_get_time() / 1000;
                    if (now - last_log_ms > 300000) {
                        last_log_ms = now;
                        ESP_LOGW(TAG, "平台拒收属性上报: %.*s", event->data_len, event->data);
                    }
                } else {
                    /* 只打第一次成功的, 用来确认平台成功应答长什么样 */
                    static bool logged_ok = false;
                    if (!logged_ok) {
                        logged_ok = true;
                        ESP_LOGI(TAG, "平台收下属性上报: %.*s", event->data_len, event->data);
                    }
                }
                free(json);
            }
        } else if (strstr(topic, "/thing/service/") && strstr(topic, "/invoke")) {
            ESP_LOGI(TAG, "收到服务调用 topic: %s", topic);
            handle_service_invoke(topic, tlen, event->data, event->data_len);
        } else {
            ESP_LOGI(TAG, "忽略 topic: %s (len=%d)", topic, event->data_len);
        }
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT 错误");
        break;
    default:
        break;
    }
}

/* ========== 对外 API ========== */

esp_err_t mqtt_client_start(void)
{
    if (s_client) return ESP_OK;

    /* 0. 启动 SNTP 校时 (属性上报的 time 字段必须是真实时间戳) */
    sntp_start();

    /* 1. 算 token (在 mqtt 启动前算好) */
    char token[256];
    if (compute_token(token, sizeof(token)) != 0) {
        ESP_LOGE(TAG, "token 计算失败, mqtt 客户端不启动");
        return ESP_FAIL;
    }

    /* 2. OneNET Studio MQTT Broker: studio-mqtt.heclouds.com:1883 (非加密) */
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://studio-mqtt.heclouds.com",
                .port = 1883,
            },
        },
        .credentials = {
            .username = ONENET_PRODUCT_ID,
            .client_id = ONENET_DEVICE_NAME,      /* clientId 必须等于设备名 */
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
        /* ⚠ MQTT 事件任务默认栈只有 4096, 服务调用应答 (param_get 要回 1KB+ 的
         *   JSON) 会直接把它撑爆 -> "stack overflow in task mqtt_task" -> 整机重启。
         *   这里给到 8192。 */
        .task = {
            .priority = 5,
            .stack_size = 8192,
        },
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) return ESP_ERR_NO_MEM;
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_start 失败: %s", esp_err_to_name(err));
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
    long id = (long)next_msg_id();
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
