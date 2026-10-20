/**
 * @file ui_home.c
 * @brief UI 主页实现 (上电默认显示)
 *        - 顶部反白标题栏 "ESP BALANCE ROBOT"
 *        - 第 1 行: 两路舵机当前角度
 *        - 第 2 行: 两路编码器当前计数
 *        - 第 3 行: WiFi 连接状态
 *        - 第 4 行: MPU 俯仰角
 *        - K1 → 切到菜单页
 * @author ESP32 机械狗项目
 * @date 2026-10-20
 */

#include "ui_home.h"
#include "servo.h"
#include "encoder.h"
#include "balance.h"
#include "wifi_sta.h"
#include "tft_display.h"
#include <stdio.h>
#include <string.h>

/* 显示坐标 */
#define HOME_TITLE_Y     1       // 标题文字 Y
#define HOME_SERVO_Y    14      // 舵机行 Y
#define HOME_ENC_Y      24      // 编码器行 Y
#define HOME_WIFI_Y     34      // WiFi状态行 Y
#define HOME_MPU_Y      44      // MPU俯仰角行 Y

#define HOME_L_LBL_X    2       // 左路标签 X
#define HOME_L_VAL_X    20      // 左路数值 X
#define HOME_R_LBL_X    66      // 右路标签 X
#define HOME_R_VAL_X    84      // 右路数值 X
#define HOME_VAL_W      42      // 数值擦除宽度

/**
 * @brief 绘制主页内容 (标题栏 + 舵机行 + 编码器行 + WiFi行 + MPU行)
 */
static void ui_home_draw(void)
{
    char buf[32];

    tft_clear(TFT_COLOR_BLACK);

    /* 顶部反白标题栏 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, HOME_TITLE_Y, "ESP BALANCE ROBOT",
                   TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 第 1 行: S1 / S2 当前角度 */
    ui_draw_string(HOME_L_LBL_X, HOME_SERVO_Y, "S1:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%3u", (unsigned)servo_get_angle(SERVO_1));
    ui_draw_string(HOME_L_VAL_X, HOME_SERVO_Y, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    ui_draw_string(HOME_R_LBL_X, HOME_SERVO_Y, "S2:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%3u", (unsigned)servo_get_angle(SERVO_2));
    ui_draw_string(HOME_R_VAL_X, HOME_SERVO_Y, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* 第 2 行: E1 / E2 编码值 */
    ui_draw_string(HOME_L_LBL_X, HOME_ENC_Y, "E1:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5d", encoder_get_count(ENCODER_1));
    ui_draw_string(HOME_L_VAL_X, HOME_ENC_Y, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    ui_draw_string(HOME_R_LBL_X, HOME_ENC_Y, "E2:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5d", encoder_get_count(ENCODER_2));
    ui_draw_string(HOME_R_VAL_X, HOME_ENC_Y, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* 第 3 行: WiFi 状态 */
    ui_draw_string(HOME_L_LBL_X, HOME_WIFI_Y, "WiFi:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    wifi_sta_state_t state = wifi_sta_get_state();
    uint16_t text_color;
    if (state == WIFI_STA_CONNECTED) {
        char ssid[32];
        wifi_sta_get_ssid(ssid, sizeof(ssid));
        snprintf(buf, sizeof(buf), "%s", ssid);
        text_color = TFT_COLOR_GREEN;
    } else if (state == WIFI_STA_AP_MODE) {
        snprintf(buf, sizeof(buf), "AP Mode");
        text_color = TFT_COLOR_YELLOW;
    } else {
        snprintf(buf, sizeof(buf), "Disconnected");
        text_color = TFT_COLOR_RED;
    }
    ui_draw_string(HOME_L_VAL_X, HOME_WIFI_Y, buf, text_color, TFT_COLOR_BLACK);

    /* 第 4 行: MPU 俯仰角 */
    ui_draw_string(HOME_L_LBL_X, HOME_MPU_Y, "Pitch:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    bal_data_t bal_data;
    if (balance_get_data(&bal_data) == ESP_OK) {
        snprintf(buf, sizeof(buf), "%+.1f", (double)bal_data.pitch);
    } else {
        snprintf(buf, sizeof(buf), "N/A");
    }
    ui_draw_string(HOME_L_VAL_X, HOME_MPU_Y, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    ui_home_draw();
}

static void page_exit(void)
{
    /* 主页无需释放资源 */
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        /* 进入菜单页 */
        lcd_ui_switch_to(UI_PAGE_MENU);
    }
    /* K2/K3/K4 在主页无操作 */
}

static void page_refresh(void)
{
    char buf[32];

    /* E1 */
    ui_fill_rect(HOME_L_VAL_X, HOME_ENC_Y, HOME_VAL_W, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5d", encoder_get_count(ENCODER_1));
    ui_draw_string(HOME_L_VAL_X, HOME_ENC_Y, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* E2 */
    ui_fill_rect(HOME_R_VAL_X, HOME_ENC_Y, HOME_VAL_W, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5d", encoder_get_count(ENCODER_2));
    ui_draw_string(HOME_R_VAL_X, HOME_ENC_Y, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* WiFi 状态 */
    wifi_sta_state_t state = wifi_sta_get_state();
    uint16_t text_color;
    if (state == WIFI_STA_CONNECTED) {
        char ssid[32];
        wifi_sta_get_ssid(ssid, sizeof(ssid));
        snprintf(buf, sizeof(buf), "%s", ssid);
        text_color = TFT_COLOR_GREEN;
    } else if (state == WIFI_STA_AP_MODE) {
        snprintf(buf, sizeof(buf), "AP Mode");
        text_color = TFT_COLOR_YELLOW;
    } else {
        snprintf(buf, sizeof(buf), "Disconnected");
        text_color = TFT_COLOR_RED;
    }
    ui_fill_rect(HOME_L_VAL_X, HOME_WIFI_Y, 60, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(HOME_L_VAL_X, HOME_WIFI_Y, buf, text_color, TFT_COLOR_BLACK);

    /* MPU 俯仰角 */
    bal_data_t bal_data;
    if (balance_get_data(&bal_data) == ESP_OK) {
        snprintf(buf, sizeof(buf), "%+.1f", (double)bal_data.pitch);
    } else {
        snprintf(buf, sizeof(buf), "N/A");
    }
    ui_fill_rect(HOME_L_VAL_X, HOME_MPU_Y, 40, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(HOME_L_VAL_X, HOME_MPU_Y, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
}

/* ========== 页面注册 ========== */

static const ui_page_t home_page = {
    .id         = UI_PAGE_HOME,
    .title      = "Home",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_home_register(void)
{
    return lcd_ui_register_page(&home_page);
}