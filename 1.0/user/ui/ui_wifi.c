/**
 * @file ui_wifi.c
 * @brief UI WiFi / 网页调参页实现
 * @note  屏上只显示"怎么连上去调参", 不做调参界面 (128x160 放不下):
 *        - 第 1 行: 热点状态 + 已接入客户端数
 *        - 第 2 行: 热点名 SSID
 *        - 第 3 行: 密码
 *        - 第 4 行: 打开的网址 (绿, 最关键)
 *        - 第 5 行: 本机 MAC
 *
 *        按键:
 *        - K1 返回菜单
 *        - K2 舵机回中 (用参数表里的 Servo Center)
 *        - K3 清零编码器计数
 *
 * @note  屏幕 128x160, 字体 6x8 => 每行 21 字符, 整行字符串一律 <= 20 字符,
 *        超出 ui_draw_string 会自动换行把布局冲掉。
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#include "ui_wifi.h"
#include "wifi_ap.h"
#include "param.h"
#include "servo.h"
#include "encoder.h"
#include "tft_display.h"
#include "esp_mac.h"
#include <stdio.h>
#include <string.h>

/* ========== 布局常量 ========== */
/* 128/6 ≈ 21 字符/行, x=2 起画只够 20 字符, 整行字符串一律 <= 20 字符 */
#define LINE_CHARS     20

#define Y_TITLE       1       /* 标题栏 (反白) */
#define Y_STATE       14      /* 状态行 */
#define Y_SSID        26      /* 热点名 */
#define Y_PASS        36      /* 密码 */
#define Y_URL         48      /* 网址 (绿) */
#define Y_MAC         60      /* MAC */
#define Y_SEP         72
#define Y_INFO        78      /* 说明文字 */
#define Y_INFO2       90
#define Y_HINT        120     /* 单行按键提示 */

#define X_LBL         2
#define X_VAL         38      /* 数值区起点 */
#define W_VAL_STATE   90
#define W_VAL_TXT     90      /* SSID/密码/网址 擦除宽度 */

/* WiFi 状态字面串 (下标与 wifi_ap_state_t 一一对应) */
static const char *state_str[] = { "OFF", "STARTING", "READY", "ERROR" };

/**
 * @brief 擦除一块矩形 (画动态数值前统一调用, 避免残留上一帧笔画)
 * @param x 左上角 X
 * @param y 左上角 Y
 * @param w 宽度 (像素)
 */
static void clear_val(uint8_t x, uint8_t y, uint8_t w)
{
    ui_fill_rect(x, y, w, UI_FONT_H, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制静态骨架 (只画一次)
 */
static void draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, Y_TITLE, "== WEB TUNER ==", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    ui_draw_string(X_LBL, Y_STATE, "WiFi:",  TFT_COLOR_CYAN,  TFT_COLOR_BLACK);
    ui_draw_string(X_LBL, Y_SSID,  "SSID:",  TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(X_LBL, Y_PASS,  "PASS:",  TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(X_LBL, Y_URL,   "URL:",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(X_LBL, Y_MAC,   "MAC:",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    ui_draw_hline(0, Y_SEP, UI_SCREEN_W, TFT_COLOR_BLUE);

    ui_draw_string(2, Y_INFO,  "Connect to WiFi,",  TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_INFO2, "open URL in browser",TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* 20 字符 = 120px, x=2 起刚好放满 */
    ui_draw_string(2, Y_HINT, "[K1]Bk [K2]Ctr [K3]Clr",
                   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制动态数值 (只重画数值区)
 */
static void draw_data(void)
{
    char buf[LINE_CHARS + 1];

    /* --- 状态: 绿=就绪, 黄=启动中, 红=失败 --- */
    wifi_ap_state_t st = wifi_ap_get_state();
    uint16_t color = (st == WIFI_AP_READY)   ? TFT_COLOR_GREEN :
                     (st == WIFI_AP_STARTING)? TFT_COLOR_YELLOW : TFT_COLOR_RED;
    clear_val(X_VAL, Y_STATE, W_VAL_STATE);
    snprintf(buf, sizeof(buf), "%s  CLI:%u", state_str[st],
             wifi_ap_get_client_count());
    ui_draw_string(X_VAL, Y_STATE, buf, color, TFT_COLOR_BLACK);

    /* --- 热点名 (用真实 SSID, ���际是 WIFI_AP_SSID + "_" + MAC后两字节) / 密码 --- */
    char ssid[32];
    wifi_ap_get_ssid(ssid, sizeof(ssid));
    clear_val(X_VAL, Y_SSID, W_VAL_TXT);
    ui_draw_string(X_VAL, Y_SSID, ssid, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    clear_val(X_VAL, Y_PASS, W_VAL_TXT);
    ui_draw_string(X_VAL, Y_PASS, WIFI_AP_PASSWORD, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* --- 网址 (最关键, 绿色) --- */
    clear_val(X_VAL, Y_URL, W_VAL_TXT);
    ui_draw_string(X_VAL, Y_URL, "http://" WIFI_AP_IP, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* --- MAC: 只显示后 3 字节 --- */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    clear_val(X_VAL, Y_MAC, W_VAL_TXT);
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X", mac[3], mac[4], mac[5]);
    ui_draw_string(X_VAL, Y_MAC, buf, TFT_COLOR_CYAN, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    draw_static();
    draw_data();
}

static void page_exit(void) {}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K2) {
        /* 舵机回中到参数表设定的中心角 */
        uint8_t center = (uint8_t)param_get_float(PARAM_SERVO_CENTER);
        servo_set_both(center);
    } else if (key == UI_KEY_K3) {
        /* 清零两个编码器计数, 方便重新测 */
        encoder_clear(ENCODER_1);
        encoder_clear(ENCODER_2);
    }
}

static void page_refresh(void)
{
    draw_data();
}

/* ========== 页面注册 ========== */

static const ui_page_t wifi_page = {
    .id         = UI_PAGE_WIFI,
    .title      = "Web Tuner",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_wifi_register(void)
{
    return lcd_ui_register_page(&wifi_page);
}
