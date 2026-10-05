/**
 * @file ui_hc595_test.c
 * @brief 74HC595 引脚映射测试页 (临时诊断用)
 * @note  - 进入: 两电机 PWM 输出 50% (持续), 74HC595 全 0 (BRAKE)
 *        - K2: 上一位  (清除上一位, 拉高下一位)
 *        - K3: 下一位  (清除上一位, 拉高下一位)
 *        - K1: 退出 + 全清 74HC595 + 停车
 *        - 屏幕显示: 当前拉高的是哪个 Qx, current_state, ENC1/ENC2 计数
 *        - 用户记录: 拉高 Qx 几秒后, 哪个电机开始转, 就知道该 Qx 接到哪个电机
 * @author ESP32 机械狗项目
 * @date 2026-10-03
 */

#include "ui_hc595_test.h"
#include "hc595.h"
#include "motor_driver.h"
#include "encoder.h"
#include "tft_display.h"
#include <stdio.h>

/* 当前测试位 (0~7) */
static int s_q = 0;

/* ========== 辅助函数 ========== */

/**
 * @brief 应用测试状态: 清掉所有位, 然后只拉高 s_q
 */
static void apply_test_state(void)
{
    hc595_clear_all();
    if (s_q >= 0 && s_q <= 7) {
        hc595_set_pin((hc595_pin_t)s_q, true);
    }
}

/**
 * @brief 重绘全部内容
 */
static void redraw(void)
{
    char buf[24];

    tft_clear(TFT_COLOR_BLACK);

    /* 顶部标题 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, 1, "HC595 PIN TEST", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 当前位 */
    ui_draw_string(2, 14, "Sel Q:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_fill_rect(54, 14, 40, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%d", s_q);
    ui_draw_string(54, 14, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* current_state */
    ui_draw_string(2, 24, "St  :", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_fill_rect(54, 24, 60, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "0x%02X", hc595_get_state());
    ui_draw_string(54, 24, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* 编码器 1/2 (看电机转没转) */
    ui_draw_string(2, 44, "ENC1:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    int c1 = encoder_get_count(ENCODER_1);
    ui_fill_rect(54, 44, 80, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%8d", c1);
    ui_draw_string(54, 44, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    ui_draw_string(2, 54, "ENC2:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    int c2 = encoder_get_count(ENCODER_2);
    ui_fill_rect(54, 54, 80, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%8d", c2);
    ui_draw_string(54, 54, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* 说明: 按 K2/K3 切位, 每位停 2 秒观察电机 */
    ui_draw_hline(0, 74, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 80, "Watch motor A & B", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 90, "Record which Qx",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 100, "spins each motor", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 底部按键提示 */
    ui_draw_hline(0, 120, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 126, "[K1]Quit  [K2]-  [K3]+", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 140, "PWM 50% on both motors", TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    ui_draw_string(2, 150, "BRAKE -> Sel Qx high",  TFT_COLOR_GREEN, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    s_q = 0;

    /* 清掉所有 hc595 输出 (避免上次的电平影响) */
    hc595_clear_all();

    /* 两电机 PWM 50% (持续), 方向 BRAKE (默认) */
    motor_set_speed_both(50, 50);

    apply_test_state();
    redraw();
}

static void page_exit(void)
{
    /* 清掉所有 hc595 输出 + 停车 */
    hc595_clear_all();
    motor_brake_all();
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K2) {
        /* 上一位 (Q-1) */
        s_q = (s_q == 0) ? 7 : (s_q - 1);
        apply_test_state();
        redraw();
    } else if (key == UI_KEY_K3) {
        /* 下一位 (Q+1) */
        s_q = (s_q + 1) % 8;
        apply_test_state();
        redraw();
    }
}

static void page_refresh(void)
{
    /* 150ms 刷新, 让用户看到编码器计数实时变化 (确认电机是否真的在转) */
    char buf[24];

    /* 更新 ENC1 */
    int c1 = encoder_get_count(ENCODER_1);
    ui_fill_rect(54, 44, 80, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%8d", c1);
    ui_draw_string(54, 44, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* 更新 ENC2 */
    int c2 = encoder_get_count(ENCODER_2);
    ui_fill_rect(54, 54, 80, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%8d", c2);
    ui_draw_string(54, 54, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
}

/* ========== 页面注册 ========== */

static const ui_page_t hc595_test_page = {
    .id         = UI_PAGE_HC595_TEST,
    .title      = "HC595 Pin Test",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_hc595_test_register(void)
{
    return lcd_ui_register_page(&hc595_test_page);
}
