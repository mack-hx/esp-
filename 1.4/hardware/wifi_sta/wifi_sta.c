/**
 * @file wifi_sta.c
 * @brief WiFi STA 模式 + AP 配网实现 (替代 1.3 的 wifi_ap.c)
 *
 * 设计:
 *   1. wifi_sta_start() 检查 NVS:
 *      - 有 ssid+pass → STA 模式连家里 WiFi
 *      - 没有 → AP 模式开 BalanceRobot_Setup 热点 + 起配网页
 *   2. STA 模式连上后, 配网页服务器**关闭** (释放 80 端口), onenet_client 可启动
 *   3. AP 配网模式下, 收到 /api/wifi POST → 存 NVS → 软重启
 *
 * @note  WiFi 事件处理用 esp_event 异步模式, 不开阻塞任务
 * @author ESP32 机械狗项目
 * @date 2026-10-12
 */

#include "wifi_sta.h"
#include "wifi_provision_page.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "WIFI_STA";

/* ========== 状态机 ========== */
static wifi_sta_state_t s_state = WIFI_STA_STOPPED;
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group = NULL;

/* WiFi 事件组的位定义 */
#define WIFI_STA_CONNECTED_BIT   BIT0   /* 已连上 */
#define WIFI_STA_FAIL_BIT        BIT1   /* 重试超限失败 */

/* STA 模式重试次数 - 需要足够多以便 DHCP 有时间获取 IP */
#define WIFI_STA_MAX_RETRY       15

/* STA 模式拿到的 IP (连上后填, 没连上时全 0) */
static esp_netif_ip_info_t s_sta_ip = {0};
static char s_sta_ssid[33] = {0};

/* AP 模式下的 netif 句柄 (创建后持有, 关闭时清理) */
static esp_netif_t *s_ap_netif = NULL;

/* 配网页 httpd 实例 (AP 模式下持有, STA 模式下销毁) */
static httpd_handle_t s_prov_httpd = NULL;

/* 真实生效的 AP SSID (基础名 + MAC 后两字节, 避免多台设备同 SSID) */
static char s_ap_ssid[32] = {0};

/* ========== NVS 读写 ========== */

/**
 * @brief 从 NVS 读保存的 SSID 和密码
 * @return true=读到了完整 ssid+pass
 */
static bool load_credentials_from_nvs(char *ssid, size_t ssid_len,
                                      char *pass, size_t pass_len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_STA_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return false;
    }

    size_t s_len = ssid_len, p_len = pass_len;
    err = nvs_get_str(h, WIFI_STA_NVS_KEY_SSID, ssid, &s_len);
    if (err != ESP_OK) {
        nvs_close(h);
        return false;
    }
    err = nvs_get_str(h, WIFI_STA_NVS_KEY_PASS, pass, &p_len);
    nvs_close(h);
    if (err != ESP_OK) {
        return false;
    }
    return (strlen(ssid) > 0);
}

/**
 * @brief 把 SSID+密码写 NVS (用单独 handle 避免读写冲突)
 * @return ESP_OK 成功
 */
static esp_err_t save_credentials_to_nvs(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_STA_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 失败: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_str(h, WIFI_STA_NVS_KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, WIFI_STA_NVS_KEY_PASS, pass);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/**
 * @brief 清空 NVS 里的 SSID/密码, 下次启动会进 AP 配网
 */
static esp_err_t clear_credentials_in_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_STA_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_erase_key(h, WIFI_STA_NVS_KEY_SSID);
    nvs_erase_key(h, WIFI_STA_NVS_KEY_PASS);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

/* ========== 配网页 HTTP handler ========== */

/**
 * @brief GET / → 返回配网页 HTML
 */
static esp_err_t prov_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, WIFI_PROV_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief 从 JSON 里取一个字符串字段
 * @param json 原始 JSON
 * @param key  字段名
 * @param out  [出参] 取出值
 * @param cap  out 大小
 * @return true=找到
 */
static bool json_str(const char *json, const char *key, char *out, size_t cap)
{
    if (out == NULL || cap == 0) return false;
    out[0] = '\0';

    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':' || *p == '\t') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < cap - 1) {
        /* JSON 中 \\ 和 \" 要反转义, 简化版: 遇到反斜杠跳过下一字符 */
        if (*p == '\\' && *(p + 1)) {
            p += 2;
            continue;
        }
        out[i++] = *p++;
    }
    out[i] = '\0';
    return (i > 0);
}

/**
 * @brief POST /api/wifi → 收 {ssid, pass}, 存 NVS, 返回 ok
 *        之后会软重启
 */
static esp_err_t prov_post_wifi_handler(httpd_req_t *req)
{
    char buf[256] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"msg\":\"empty body\"}", -1);
        return ESP_OK;
    }
    buf[ret] = '\0';

    char ssid[33] = {0}, pass[65] = {0};
    if (!json_str(buf, "ssid", ssid, sizeof(ssid))) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"msg\":\"missing ssid\"}", -1);
        return ESP_OK;
    }
    json_str(buf, "pass", pass, sizeof(pass));   /* pass 可空 (开放 WiFi) */

    esp_err_t err = save_credentials_to_nvs(ssid, pass);
    if (err != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"msg\":\"nvs save failed\"}", -1);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "已保存 WiFi 配置: SSID=%s, 重启中...", ssid);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", -1);

    /* 延迟 500ms 再重启, 让 HTTP 响应飞回浏览器 */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;  /* 不会到这里 */
}

/**
 * @brief 启动配网页 HTTP server (AP 模式下)
 */
static esp_err_t start_provision_httpd(void)
{
    if (s_prov_httpd != NULL) return ESP_OK;  /* 已在跑 */

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 4;
    cfg.stack_size = 8192;

    esp_err_t err = httpd_start(&s_prov_httpd, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配网页 httpd 启动失败: %s", esp_err_to_name(err));
        return err;
    }

    /* GET / → 配网页 */
    httpd_uri_t uri_get = {
        .uri = "/", .method = HTTP_GET,
        .handler = prov_get_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_get);

    /* GET / 也支持兜底, 浏览器没填路径时会自动跳到根 */
    httpd_uri_t uri_index = {
        .uri = "/index.html", .method = HTTP_GET,
        .handler = prov_get_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_index);

    /* POST /api/wifi → 收配置 */
    httpd_uri_t uri_post = {
        .uri = "/api/wifi", .method = HTTP_POST,
        .handler = prov_post_wifi_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_post);

    ESP_LOGI(TAG, "配网页已启动: http://" WIFI_STA_AP_IP "/");
    return ESP_OK;
}

/**
 * @brief 关闭配网页 HTTP server (STA 模式下)
 */
static void stop_provision_httpd(void) __attribute__((unused));
static void stop_provision_httpd(void)
{
    if (s_prov_httpd == NULL) return;
    httpd_stop(s_prov_httpd);
    s_prov_httpd = NULL;
    ESP_LOGI(TAG, "配网页已关闭");
}

/* ========== AP 模式 (配网用) ========== */

/**
 * @brief 启动 AP 模式 + 配网页
 */
static esp_err_t start_ap_mode(void)
{
    ESP_LOGI(TAG, "NVS 无 WiFi 配置, 进入 AP 配网模式");

    s_state = WIFI_STA_AP_MODE;

    /* 拼一个带 MAC 后缀的 SSID, 避免多台设备同名热点 */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s_%02X%02X",
             WIFI_STA_AP_SSID, mac[4], mac[5]);

    /* AP 必须在 STA 之前创建 netif, 用 AP-only 模式 */
    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 先创建 AP netif, 再设置 IP 和 WiFi config */
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ap_netif == NULL) {
        ESP_LOGE(TAG, "创建 AP netif 失败");
        return ESP_FAIL;
    }

    /* 固定 IP 192.168.4.1
     * esp_ip4addr_aton() 返回 uint32_t 网络序数值, 赋给 esp_ip4_addr_t 字段 */
    esp_netif_ip_info_t ipinfo = {0};
    ipinfo.ip.addr      = esp_ip4addr_aton((const char *)WIFI_STA_AP_IP);
    ipinfo.netmask.addr = esp_ip4addr_aton((const char *)"255.255.255.0");
    ipinfo.gw.addr      = esp_ip4addr_aton((const char *)WIFI_STA_AP_IP);
    ret = esp_netif_set_ip_info(s_ap_netif, &ipinfo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 AP IP 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 显式启动 DHCP server */
    ret = esp_netif_dhcps_start(s_ap_netif);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DHCP server 启动失败: %s (可能已自动启用)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DHCP server 已启动");
    }

    /* 配置 AP WiFi 参数 - 开放网络无密码 */
    wifi_config_t wc = {0};
    strlcpy((char *)wc.ap.ssid, s_ap_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(s_ap_ssid);
    wc.ap.channel = WIFI_STA_AP_CHANNEL;
    wc.ap.max_connection = WIFI_STA_AP_MAX_CONN;
    wc.ap.authmode = WIFI_AUTH_OPEN;  /* 开放网络 */
    wc.ap.beacon_interval = 100;

    ret = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "配网热点已开: SSID=%s  密码=%s", s_ap_ssid, WIFI_STA_AP_PASSWORD);
    ESP_LOGI(TAG, "请用手机连此热点, 浏览器打开 http://" WIFI_STA_AP_IP "/");

    /* 起配网页 */
    return start_provision_httpd();
}

/* ========== STA 模式 (连家里 WiFi) ========== */

/**
 * @brief 启动 STA 模式连家里 WiFi
 * @note  使用纯 STA 模式确保 DHCP 能正常工作
 */
static esp_err_t start_sta_mode(const char *ssid, const char *pass)
{
    ESP_LOGI(TAG, ">>> start_sta_mode() 被调用, SSID=%s", ssid);
    s_state = WIFI_STA_CONNECTING;
    s_retry_num = 0;
    strlcpy(s_sta_ssid, ssid, sizeof(s_sta_ssid));
    memset(&s_sta_ip, 0, sizeof(s_sta_ip));

    /* 直接使用 esp_netif_create_default_wifi_sta() 创建 netif
     * ESP-IDF 会自动处理 DHCP client */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGE(TAG, "创建STA netif失败");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "STA netif 创建成功");

    /* 设置纯 STA 模式 */
    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(STA) 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "WiFi 模式设置为 STA");

    /* 配置 STA WiFi 参数 */
    wifi_config_t sta_wc = {0};
    strlcpy((char *)sta_wc.sta.ssid, ssid, sizeof(sta_wc.sta.ssid));
    strlcpy((char *)sta_wc.sta.password, pass, sizeof(sta_wc.sta.password));

    /* 设置 BSSID 自动模式 (默认) */
    sta_wc.sta.bssid_set = false;

    /* 设置认证模式 */
    if (pass == NULL || pass[0] == '\0') {
        sta_wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
    } else {
        sta_wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &sta_wc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(STA) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi 配置完成, SSID=%s, 等待连接...", ssid);
    return ESP_OK;
}

/* ========== WiFi 事件处理 ========== */

/**
 * @brief WiFi 事件回调
 *        - STA 连上 → 拿 IP → 关配网页 (AP 仍保留做兜底)
 *        - STA 连失败 → 重试, 超过次数进 FAILED
 *        - AP 模式客户端连/断 → 打印日志
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "WiFi事件: base=%s, id=%ld", event_base, (long)event_id);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA 启动, 开始连接...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_STA_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "STA 断连, 重试 (%d/%d)", s_retry_num, WIFI_STA_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_STA_FAIL_BIT);
            s_state = WIFI_STA_FAILED;
            ESP_LOGE(TAG, "STA 重试超限, 连接失败");
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        /* WiFi 已连接，等待 DHCP 自动获取 IP */
        ESP_LOGI(TAG, "WiFi已连接, 等待DHCP获取IP...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        s_sta_ip = e->ip_info;
        ESP_LOGI(TAG, "★ STA 已连上, IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_num = 0;
        s_state = WIFI_STA_CONNECTED;
        xEventGroupSetBits(s_wifi_event_group, WIFI_STA_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "AP 客户端连入: " MACSTR, MAC2STR(e->mac));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "AP 客户端断开: " MACSTR, MAC2STR(e->mac));
    } else {
        ESP_LOGW(TAG, "未知WiFi事件: base=%s, id=%ld", event_base, (long)event_id);
    }
}

/* ========== 公共 API ========== */

esp_err_t wifi_sta_start(void)
{
    ESP_LOGI(TAG, "wifi_sta_start called, state=%d", s_state);

    if (s_state != WIFI_STA_STOPPED) {
        ESP_LOGW(TAG, "已在运行 (state=%d), 不重复启动", s_state);
        return ESP_OK;
    }

    /* 1. 初始化 TCP/IP + 默认事件循环 (幂等, main 已调过也 OK) */
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK) return ret;
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    /* 2. 创建事件组 (STA 等待用) */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) return ESP_ERR_NO_MEM;

    /* 3. 注册事件 */
    ret = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WIFI事件注册失败: %s", esp_err_to_name(ret));
    }
    ret = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IP事件注册失败: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "WiFi事件处理器已注册, state=%d", s_state);

    /* 4. WiFi 驱动初始化 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 5. 查 NVS, 决定走 STA 还是 AP, 然后设置模式并启动 */
    char ssid[33] = {0}, pass[65] = {0};
    if (load_credentials_from_nvs(ssid, sizeof(ssid), pass, sizeof(pass))) {
        /* 已有 WiFi 配置 → STA 模式
         * 顺序: 先创建netif, 再设置模式, 最后启动 */
        ret = start_sta_mode(ssid, pass);
        if (ret != ESP_OK) return ret;
        ret = esp_wifi_set_mode(WIFI_MODE_STA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_set_mode(STA) 失败: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        /* 首次使用或被清空 → AP 配网模式 */
        ret = start_ap_mode();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi 模式启动失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 6. 启动 WiFi 驱动 */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "esp_wifi_start() 成功, WiFi驱动已启动");

    return ESP_OK;
}

wifi_sta_state_t wifi_sta_get_state(void)
{
    return s_state;
}

bool wifi_sta_is_connected(void)
{
    return (s_state == WIFI_STA_CONNECTED);
}

esp_err_t wifi_sta_get_ip(esp_netif_ip_info_t *info)
{
    if (info == NULL) return ESP_ERR_INVALID_ARG;
    if (s_state != WIFI_STA_CONNECTED) return ESP_ERR_INVALID_STATE;
    *info = s_sta_ip;
    return ESP_OK;
}

esp_err_t wifi_sta_get_ssid(char *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    if (s_state != WIFI_STA_CONNECTED) return ESP_ERR_INVALID_STATE;
    strlcpy(buf, s_sta_ssid, len);
    return ESP_OK;
}

/**
 * @brief 启动 AP 模式 + 配网页 (用于运行时切换配网)
 * @note  从 STA 模式切换时会关闭 STA，切换到纯 AP 模式
 */
esp_err_t wifi_sta_start_ap(void)
{
    ESP_LOGI(TAG, "切换到 AP 配网模式");

    /* 停止现有 STA 连接 */
    if (s_state == WIFI_STA_CONNECTED || s_state == WIFI_STA_CONNECTING) {
        esp_wifi_disconnect();
    }

    /* 先创建 AP netif (必须先于 esp_wifi_set_mode) */
    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            ESP_LOGE(TAG, "创建 AP netif 失败");
            return ESP_FAIL;
        }
    }

    /* 先停止DHCP，确保干净 */
    esp_netif_dhcps_stop(s_ap_netif);

    /* 设置 AP IP (192.168.4.1) */
    esp_netif_ip_info_t ipinfo = {0};
    ipinfo.ip.addr      = esp_ip4addr_aton((const char *)"192.168.4.1");
    ipinfo.netmask.addr = esp_ip4addr_aton((const char *)"255.255.255.0");
    ipinfo.gw.addr      = esp_ip4addr_aton((const char *)"192.168.4.1");
    esp_err_t ret = esp_netif_set_ip_info(s_ap_netif, &ipinfo);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置 AP IP 失败: %s", esp_err_to_name(ret));
    }

    /* 启动 DHCP server */
    ret = esp_netif_dhcps_start(s_ap_netif);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DHCP server 启动失败: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DHCP server 已启动");
    }

    /* 拼一个带 MAC 后缀的 SSID */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s_%02X%02X",
             WIFI_STA_AP_SSID, mac[4], mac[5]);

    /* 切换到 AP-only 模式 */
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 配置 AP WiFi 参数 - 开放网络无密码 */
    wifi_config_t wc = {0};
    strlcpy((char *)wc.ap.ssid, s_ap_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(s_ap_ssid);
    wc.ap.channel = WIFI_STA_AP_CHANNEL;
    wc.ap.max_connection = WIFI_STA_AP_MAX_CONN;
    wc.ap.authmode = WIFI_AUTH_OPEN;  /* 开放网络 */
    wc.ap.beacon_interval = 100;

    ret = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 启动配网页 */
    ret = start_provision_httpd();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配网页启动失败: %s", esp_err_to_name(ret));
        return ret;
    }

    s_state = WIFI_STA_AP_MODE;
    ESP_LOGI(TAG, "AP 配网已开启: SSID=%s  IP=http://192.168.4.1/", s_ap_ssid);

    return ESP_OK;
}

void wifi_sta_reprovision(void)
{
    ESP_LOGW(TAG, "主动触发重新配网, 清 NVS, 重启...");
    clear_credentials_in_nvs();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}
