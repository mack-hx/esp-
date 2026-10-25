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
 * @date 2026-10-19
 */

#include "ui_netstat.h"
#include "wifi_sta.h"
#include "onenet_client.h"
#include "tft_display.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "UI_NET";

/* ========== 布局 ========== */
/* 128/6 ≈ 21 字符/行 */
#define LINE_CHARS     20

#define Y_TITLE       1
#define Y_STATUS      12      /* 连接状态 */
#define Y_SSID        28      /* WiFi名称 */
#define Y_IP          44      /* IP地址 */
#define Y_AP_INFO     60      /* AP模式提示 */
#define Y_HINT        130     /* 按键提示 */

static inline void clear_val(uint8_t x, uint8_t y, uint8_t w)
{
    ui_fill_rect(x, y, w, UI_FONT_H, TFT_COLOR_BLACK);
}

static void draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 标题栏 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, Y_TITLE, "== NETWORK ==", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 标签 */
    ui_draw_string(2, Y_STATUS, "Status:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_SSID,   "WiFi:",  TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_IP,    "IP:",    TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    /* 底部提示 */
    ui_draw_string(2, Y_HINT, "[K1]Back [K4]AP Mode",
                   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

static void draw_data(void)
{
    char buf[32];
    wifi_sta_state_t st = wifi_sta_get_state();

    /* --- 状态行 --- */
    uint16_t status_color;
    const char *status_text;

    if (st == WIFI_STA_CONNECTED) {
        status_color = TFT_COLOR_GREEN;
        status_text = "Connected";
    } else if (st == WIFI_STA_AP_MODE) {
        status_color = TFT_COLOR_YELLOW;
        status_text = "AP Mode";
    } else if (st == WIFI_STA_CONNECTING) {
        status_color = TFT_COLOR_YELLOW;
        status_text = "Connecting...";
    } else if (st == WIFI_STA_FAILED) {
        status_color = TFT_COLOR_RED;
        status_text = "Failed";
    } else {
        status_color = TFT_COLOR_RED;
        status_text = "Disconnected";
    }

    clear_val(36, Y_STATUS, 92);
    ui_draw_string(36, Y_STATUS, status_text, status_color, TFT_COLOR_BLACK);

    /* --- SSID行 --- */
    clear_val(30, Y_SSID, 98);
    char ssid[33] = {0};

    if (st == WIFI_STA_CONNECTED && wifi_sta_get_ssid(ssid, sizeof(ssid)) == ESP_OK) {
        /* SSID可能长，截断到16字符 */
        if (strlen(ssid) > 16) ssid[16] = '\0';
        ui_draw_string(30, Y_SSID, ssid, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
    } else if (st == WIFI_STA_AP_MODE) {
        /* 显示真实热点名 (基础名 + MAC 后两字节), 手机必须按这个名字搜 */
        if (wifi_sta_get_ap_ssid(ssid, sizeof(ssid)) == ESP_OK) {
            if (strlen(ssid) > 16) ssid[16] = '\0';
            ui_draw_string(30, Y_SSID, ssid, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
        } else {
            ui_draw_string(30, Y_SSID, "<AP Mode>", TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
        }
    } else {
        ui_draw_string(30, Y_SSID, "Not Connected", TFT_COLOR_RED, TFT_COLOR_BLACK);
    }

    /* --- IP行 --- */
    clear_val(30, Y_IP, 98);
    esp_netif_ip_info_t ip = {0};

    if (st == WIFI_STA_CONNECTED && wifi_sta_get_ip(&ip) == ESP_OK) {
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip.ip));
        ui_draw_string(30, Y_IP, buf, TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    } else if (st == WIFI_STA_AP_MODE) {
        ui_draw_string(30, Y_IP, "192.168.4.1", TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
    } else {
        ui_draw_string(30, Y_IP, "-", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    }

    /* --- AP模式提示信息 --- */
    clear_val(0, Y_AP_INFO, UI_SCREEN_W);
    if (st == WIFI_STA_AP_MODE) {
        ui_draw_string(2, Y_AP_INFO, "Connect phone to AP",
                       TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
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
