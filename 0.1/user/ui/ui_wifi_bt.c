/**
 * @file ui_wifi_bt.c
 * @brief UI 蓝牙 & WiFi 调试页实现
 * @note  当前 WiFi / Bluetooth 驱动尚未实现, 此页仅做框架与占位:
 *        - 显示 WiFi / BT 模块状态 (OFF / INIT / READY)
 *        - 显示设备 MAC 后 3 字节 (从 esp_read_mac 读取, 可随时启用)
 *        - K2 模拟"WiFi 切换开关", K3 模拟"BT 切换开关"
 *        - K1 退出回菜单
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "ui_wifi_bt.h"
#include "tft_display.h"
#include "esp_system.h"
#include "esp_mac.h"
#include <stdio.h>

/* 模块状态枚举 (与字面字符串一一对应) */
typedef enum {
    WIFI_BT_STATE_OFF  = 0,    // 关闭
    WIFI_BT_STATE_INIT,        // 初始化中
    WIFI_BT_STATE_READY,       // 已就绪
} wifi_bt_state_t;

/* 状态字面字符串 */
static const char *state_str[] = { "OFF", "INIT", "READY" };

/* WiFi / BT 状态 (默认 OFF, 因为驱动尚未实现) */
static wifi_bt_state_t s_wifi_state = WIFI_BT_STATE_OFF;
static wifi_bt_state_t s_bt_state   = WIFI_BT_STATE_OFF;

/* MAC (从 esp_read_mac 读取一次后缓存) */
static uint8_t s_mac[6] = {0};

/**
 * @brief 读取 ESP32 MAC 地址 (不依赖 WiFi/BT 驱动, 始终可用)
 */
static void read_mac(void)
{
    if (s_mac[0] == 0 && s_mac[1] == 0 && s_mac[2] == 0) {
        esp_read_mac(s_mac, ESP_MAC_WIFI_STA);
    }
}

/**
 * @brief 绘制静态骨架
 */
static void ui_wifi_bt_draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 顶部标题 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, 1, "WIFI & BT", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* WiFi 区域 */
    ui_draw_string(2, 14, "WiFi:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    /* BT 区域 */
    ui_draw_string(2, 56, "BT:",   TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    /* 分隔线 */
    ui_draw_hline(0, 100, UI_SCREEN_W, TFT_COLOR_BLUE);

    /* MAC 区域 */
    ui_draw_string(2, 106, "MAC:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 底部提示 */
    ui_draw_hline(0, 130, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 136, "[K1] Back",        TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 146, "[K2] WiFi Toggle", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 156, "[K3] BT Toggle",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制实时数据
 */
static void ui_wifi_bt_draw_data(void)
{
    char buf[32];

    /* WiFi 状态 */
    ui_fill_rect(40, 14, 80, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%s", state_str[s_wifi_state]);
    uint16_t color = (s_wifi_state == WIFI_BT_STATE_READY) ? TFT_COLOR_GREEN :
                     (s_wifi_state == WIFI_BT_STATE_INIT)  ? TFT_COLOR_YELLOW :
                     TFT_COLOR_RED;
    ui_draw_string(40, 14, buf, color, TFT_COLOR_BLACK);

    /* WiFi 附加信息 (SSID/Mode) */
    ui_fill_rect(2, 26, UI_SCREEN_W, UI_FONT_H * 2, TFT_COLOR_BLACK);
    ui_draw_string(2, 26, "SSID: <not set>", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 36, "Mode: STA",       TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* BT 状态 */
    ui_fill_rect(40, 56, 80, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%s", state_str[s_bt_state]);
    color = (s_bt_state == WIFI_BT_STATE_READY) ? TFT_COLOR_GREEN :
            (s_bt_state == WIFI_BT_STATE_INIT)  ? TFT_COLOR_YELLOW :
            TFT_COLOR_RED;
    ui_draw_string(40, 56, buf, color, TFT_COLOR_BLACK);

    /* BT 附加信息 */
    ui_fill_rect(2, 68, UI_SCREEN_W, UI_FONT_H * 2, TFT_COLOR_BLACK);
    ui_draw_string(2, 68, "Name: ESP32-S3", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 78, "Scan: <none>",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* MAC */
    read_mac();
    ui_fill_rect(40, 106, 86, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X",
             s_mac[3], s_mac[4], s_mac[5]);
    ui_draw_string(40, 106, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    ui_wifi_bt_draw_static();
    ui_wifi_bt_draw_data();
}

static void page_exit(void)
{
    /* 此页不做任何硬件动作 (驱动未实现) */
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K2) {
        /* WiFi 状态循环: OFF -> INIT -> READY -> OFF */
        s_wifi_state = (wifi_bt_state_t)((s_wifi_state + 1) % 3);
        ui_wifi_bt_draw_data();
    } else if (key == UI_KEY_K3) {
        /* BT 状态循环 */
        s_bt_state = (wifi_bt_state_t)((s_bt_state + 1) % 3);
        ui_wifi_bt_draw_data();
    }
}

static void page_refresh(void)
{
    /* 当前每 150ms 全量刷一次 (驱动未实现, 暂时只展示静态) */
}

/* ========== 页面注册 ========== */

static const ui_page_t wifi_bt_page = {
    .id         = UI_PAGE_WIFI_BT,
    .title      = "WiFi & BT",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_wifi_bt_register(void)
{
    return lcd_ui_register_page(&wifi_bt_page);
}