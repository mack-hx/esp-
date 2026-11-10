/**
 * @file ui_netstat.c
 * @brief UI 网络状态页实现 (简化版)
 *
 * 显示:
 *   - 已连接WiFi名称 (SSID)
 *   - 设备IP地址
 *   - 当前状态: 已连接/未连接/AP配网模式
 *
 * 按键:
 *   - K1 返回菜单
 *   - K4 进入AP配网模式
 *
 * @author ESP32 机械狗项目
 */

#include "ui_netstat.h"
#include "wifi_sta.h"
#include "onenet_client.h"
#include "ble_ctrl.h"
#include "tft_display.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <stdio.h>
#include <string.h>

extern bool mqtt_client_is_connected(void);

static const char *TAG = "UI_NET";

/* ========== 滚动文本状态 ========== */
static uint32_t s_scroll_offset = 0;   /* AK 滚动偏移 */
static uint32_t s_last_scroll_ms = 0;  /* 上次滚动时间 */
#define SCROLL_INTERVAL_MS 500           /* 每 500ms 滚动一次 */

/* ========== 布局 ========== */
#define Y_TITLE       1
#define Y_WIFI        12      /* WiFi 连接状态 */
#define Y_SSID        26      /* WiFi 名称 */
#define Y_IP          40      /* IP 地址 */
#define Y_BLE         54      /* 蓝牙连接状态 */
#define Y_PEER        68      /* 对端地址 */
#define Y_MQTT        86      /* MQTT 连接状态 */
#define Y_DEV         100      /* 设备名 */
#define Y_AK          114     /* AccessKey */
#define Y_HINT        138     /* 按键提示 */

/**
 * @brief 绘制一条数据行: "标签: 值"
 * @param y        行 Y 坐标
 * @param label    标签文本
 * @param value    值文本
 * @param color    值的颜色
 */
static inline void draw_line(uint8_t y, const char *label, const char *value, uint16_t color)
{
    ui_draw_string(2, y, label, TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_fill_rect(36, y, 92, UI_FONT_H, TFT_COLOR_BLACK);  /* 清除值区域 */
    ui_draw_string(36, y, value, color, TFT_COLOR_BLACK);
}

static void draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 标题栏 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, Y_TITLE, "== NETWORK ==", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 标签 */
    ui_draw_string(2, Y_WIFI,  "WiFi:",  TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_SSID,  "SSID:",  TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_IP,    "IP:",    TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_BLE,   "BLE:",   TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_PEER,  "Peer:",  TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_MQTT,  "MQTT:",  TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_DEV,   "Dev:",   TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_AK,    "AK:",    TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    /* 底部提示 */
    ui_draw_string(2, Y_HINT, "[K1]Back [K4]AP Mode",
                   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

static void draw_data(void)
{
    char buf[32];
    wifi_sta_state_t st = wifi_sta_get_state();

    /* --- WiFi 连接状态 --- */
    uint16_t wifi_color;
    const char *wifi_text;
    if (st == WIFI_STA_CONNECTED) {
        wifi_color = TFT_COLOR_GREEN;
        wifi_text = "Connected";
    } else if (st == WIFI_STA_AP_MODE) {
        wifi_color = TFT_COLOR_YELLOW;
        wifi_text = "AP Mode";
    } else if (st == WIFI_STA_CONNECTING) {
        wifi_color = TFT_COLOR_YELLOW;
        wifi_text = "Connecting...";
    } else {
        wifi_color = TFT_COLOR_RED;
        wifi_text = st == WIFI_STA_FAILED ? "Failed" : "Disconnected";
    }
    draw_line(Y_WIFI, "WiFi:", wifi_text, wifi_color);

    /* --- SSID --- */
    char ssid[33] = {0};
    const char *ssid_text;
    if (st == WIFI_STA_CONNECTED && wifi_sta_get_ssid(ssid, sizeof(ssid)) == ESP_OK) {
        if (strlen(ssid) > 16) ssid[16] = '\0';
        ssid_text = ssid;
    } else if (st == WIFI_STA_AP_MODE && wifi_sta_get_ap_ssid(ssid, sizeof(ssid)) == ESP_OK) {
        if (strlen(ssid) > 16) ssid[16] = '\0';
        ssid_text = ssid;
    } else {
        ssid_text = st == WIFI_STA_AP_MODE ? "<AP>" : "-";
    }
    draw_line(Y_SSID, "SSID:", ssid_text, TFT_COLOR_WHITE);

    /* --- IP 地址 --- */
    esp_netif_ip_info_t ip = {0};
    const char *ip_text;
    if (st == WIFI_STA_CONNECTED && wifi_sta_get_ip(&ip) == ESP_OK) {
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip.ip));
        ip_text = buf;
    } else if (st == WIFI_STA_AP_MODE) {
        ip_text = "192.168.4.1";
    } else {
        ip_text = "-";
    }
    draw_line(Y_IP, "IP:", ip_text, TFT_COLOR_WHITE);

    /* --- 蓝牙: 连接状态 --- */
    uint16_t ble_color = ble_ctrl_is_connected() ? TFT_COLOR_GREEN : TFT_COLOR_YELLOW;
    const char *ble_text = ble_ctrl_is_connected() ? "Connected" : "Adv";
    draw_line(Y_BLE, "BLE:", ble_text, ble_color);

    /* --- 对端地址 --- */
    char peer[18] = {0};
    if (ble_ctrl_get_peer(peer, sizeof(peer))) {
        draw_line(Y_PEER, "Peer:", peer, TFT_COLOR_WHITE);
    } else {
        draw_line(Y_PEER, "Peer:", "-", TFT_COLOR_WHITE);
    }

    /* --- MQTT 连接状态 --- */
    uint16_t mqtt_color = mqtt_client_is_connected() ? TFT_COLOR_GREEN : TFT_COLOR_RED;
    const char *mqtt_text = mqtt_client_is_connected() ? "Connected" : "Disconnected";
    draw_line(Y_MQTT, "MQTT:", mqtt_text, mqtt_color);

    /* --- 设备名 --- */
    char dev[64] = {0};
    onenet_config_load(NULL, 0, dev, sizeof(dev), NULL, 0);
    draw_line(Y_DEV, "Dev:", dev[0] ? dev : "-", TFT_COLOR_WHITE);

    /* --- AccessKey (滚动显示) --- */
    char ak[128] = {0};
    onenet_config_load(NULL, 0, NULL, 0, ak, sizeof(ak));
    
    char display[24] = {0};
    if (ak[0]) {
        uint32_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - s_last_scroll_ms >= SCROLL_INTERVAL_MS) {
            s_last_scroll_ms = now_ms;
            int ak_len = strlen(ak);
            s_scroll_offset = (s_scroll_offset + 4) % ak_len;
        }
        
        int pos = s_scroll_offset;
        int i = 0;
        for (i = 0; i < 16 && ak[pos]; i++, pos = (pos + 1) % strlen(ak)) {
            display[i] = ak[pos];
        }
        display[i] = '\0';
        draw_line(Y_AK, "AK:", display, TFT_COLOR_WHITE);
    } else {
        draw_line(Y_AK, "AK:", "-", TFT_COLOR_WHITE);
    }
}

static void draw_all(void)
{
    draw_static();
    draw_data();
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    draw_all();
}

static void page_exit(void)
{
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        /* 返回菜单 */
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K4) {
        ESP_LOGW(TAG, "K4 切换到 AP 配网模式");
        wifi_sta_start_ap();
        /* 刷新显示 */
        draw_all();
    }
}

static void page_refresh(void)
{
    draw_data();
}

/* ========== 页面注册 ========== */

static const ui_page_t netstat_page = {
    .id         = UI_PAGE_NETSTAT,
    .title      = "Network",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_netstat_register(void)
{
    return lcd_ui_register_page(&netstat_page);
}
