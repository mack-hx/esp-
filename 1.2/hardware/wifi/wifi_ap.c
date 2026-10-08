/**
 * @file wifi_ap.c
 * @brief WiFi 热点 (AP 模式) 驱动实现
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#include "wifi_ap.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "lwip/inet.h"

static const char *TAG = "WIFI_AP";

/* 当前状态 (事件回调里更新, 供 UI/网页查询) */
static wifi_ap_state_t s_state = WIFI_AP_STOPPED;

/* AP 的 netif 句柄 (在 esp_netif_create_default_wifi_ap() 时拿到) */
static esp_netif_t *s_ap_netif = NULL;

/* 当前已连接的客户端数 (由 WIFI_EVENT_AP_STACONNECTED / STADISCONNECTED 维护) */
static uint8_t s_client_cnt = 0;

/* 实际生效的 SSID (WIFI_AP_SSID + MAC 后两字节), wifi_ap_start() 里填好 */
static char s_ssid[32] = {0};

/**
 * @brief WiFi/IP 事件处理
 * @param event_id  事件类型
 * @param event_data 事件数据
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base != WIFI_EVENT) return;   /* 本模块只关心 WIFI_EVENT */

    switch (event_id) {
        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "热点已开启, SSID=%s", s_ssid);
            s_state = WIFI_AP_READY;
            break;
        case WIFI_EVENT_AP_STOP:
            ESP_LOGI(TAG, "热点已关闭");
            s_state = WIFI_AP_STOPPED;
            s_client_cnt = 0;
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            if (s_client_cnt < WIFI_AP_MAX_CONN) s_client_cnt++;
            ESP_LOGI(TAG, "客户端接入 (%u/%d)", s_client_cnt, WIFI_AP_MAX_CONN);
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            if (s_client_cnt > 0) s_client_cnt--;
            ESP_LOGI(TAG, "客户端断开, 剩余 (%u)", s_client_cnt);
            break;
        default:
            break;
    }
}

esp_err_t wifi_ap_start(void)
{
    if (s_state != WIFI_AP_STOPPED) {
        return ESP_OK;   // 已经在跑
    }
    s_state = WIFI_AP_STARTING;

    /* 1. 初始化 TCP/IP 协议栈 + 默认事件循环 (若 main 已调过会直接返回) */
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK) return ret;
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    /* 2. 注册事件处理 */
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) return ret;
    ret = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                             wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) return ret;

    /* 3. 初始化 WiFi 驱动, 设为 AP 模式 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(ret));
        s_state = WIFI_AP_ERROR;
        return ret;
    }

    /* 4. 取出 MAC, 拼一个唯一后缀, 避免多台设备同 SSID 混淆 */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ssid, sizeof(s_ssid), "%s_%02X%02X",
             WIFI_AP_SSID, mac[4], mac[5]);

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) return ret;

    wifi_config_t wc = {0};
    strlcpy((char *)wc.ap.ssid, s_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(s_ssid);
    wc.ap.channel = WIFI_AP_CHANNEL;
    wc.ap.max_connection = WIFI_AP_MAX_CONN;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    strlcpy((char *)wc.ap.password, WIFI_AP_PASSWORD, sizeof(wc.ap.password));
    wc.ap.beacon_interval = 100;

    ret = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (ret != ESP_OK) return ret;

    /* 5. 创建 AP 的 netif (固定 IP 192.168.4.1) */
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ap_netif == NULL) {
        s_state = WIFI_AP_ERROR;
        return ESP_FAIL;
    }

    esp_netif_ip_info_t ipinfo = {0};
    /* esp_ip4_addr_t 与 lwip 的 ip4_addr_t 是同一个布局, 直接转换 */
    ip4addr_aton(WIFI_AP_IP,      (ip4_addr_t *)&ipinfo.ip);
    ip4addr_aton(WIFI_AP_NETMASK, (ip4_addr_t *)&ipinfo.netmask);
    ip4addr_aton(WIFI_AP_GATEWAY, (ip4_addr_t *)&ipinfo.gw);
    esp_netif_set_ip_info(s_ap_netif, &ipinfo);

    /* 6. 启动 */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        s_state = WIFI_AP_ERROR;
        return ret;
    }

    ESP_LOGI(TAG, "热点启动中, 请连接: %s  密码: %s", s_ssid, WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "连接后浏览器打开: http://" WIFI_AP_IP);

    /* 7. 打印完整 MAC (AP 模式也可用) */
    ESP_LOGI(TAG, "本机 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return ESP_OK;
}

esp_err_t wifi_ap_stop(void)
{
    return esp_wifi_stop();
}

wifi_ap_state_t wifi_ap_get_state(void)
{
    return s_state;
}

esp_err_t wifi_ap_get_ip(esp_netif_ip_info_t *info)
{
    if (s_ap_netif == NULL || info == NULL) return ESP_ERR_INVALID_STATE;
    return esp_netif_get_ip_info(s_ap_netif, info);
}

uint8_t wifi_ap_get_client_count(void)
{
    return s_client_cnt;
}

void wifi_ap_get_ssid(char *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return;
    strlcpy(buf, s_ssid, len);
}
