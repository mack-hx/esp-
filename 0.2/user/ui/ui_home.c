/**
 * @file ui_home.c
 * @brief UI 主页实现 (上电默认显示)
 * @note  - 顶部反白标题栏 "ESP BALANCE ROBOT"
 *        - 第 1 行: 两路舵机当前角度 (S1:XXX S2:XXX)
 *        - 第 2 行: 两路编码器当前计数 (E1:XXXXX E2:XXXXX)
 *        - 其余区域留空
 *        - K1 → 切到菜单页, K2/K3/K4 在主页无操作
 *        - 每 150ms 刷新编码值 (其他行不刷)
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#include "ui_home.h"
#include "servo.h"
#include "encoder.h"
#include "tft_display.h"
#include <stdio.h>

/* 显示坐标 */
#define HOME_TITLE_Y     1       // 标题文字 Y
#define HOME_SERVO_Y     14      // 舵机行 Y 起点
#define HOME_ENC_Y       24      // 编码器行 Y 起点
/* 两路各占左半 / 右半: 标签 X + 数值 X, 两路对称 */
#define HOME_L_LBL_X     2       // 左路标签 "S1:" / "E1:" X
#define HOME_L_VAL_X     20      // 左路数值 X
#define HOME_R_LBL_X     66      // 右路标签 "S2:" / "E2:" X
#define HOME_R_VAL_X     84      // 右路数值 X
#define HOME_VAL_W       42      // 数值擦除宽度 (84+42=126, 留 2px 右边距) */

/**
 * @brief 绘制主页内容 (标题栏 + 舵机行 + 编码器行)
 */
static void ui_home_draw(void)
{
    char buf[24];

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

    /* 其余区域留空 (下方不画任何内容) */
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
    /* 仅刷新编码值 (每 150ms 调用一次), 标题/舵机/留空区不刷 */
    char buf[16];

    /* E1 */
    ui_fill_rect(HOME_L_VAL_X, HOME_ENC_Y, HOME_VAL_W, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5d", encoder_get_count(ENCODER_1));
    ui_draw_string(HOME_L_VAL_X, HOME_ENC_Y, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* E2 */
    ui_fill_rect(HOME_R_VAL_X, HOME_ENC_Y, HOME_VAL_W, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%5d", encoder_get_count(ENCODER_2));
    ui_draw_string(HOME_R_VAL_X, HOME_ENC_Y, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
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