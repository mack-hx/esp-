/**
 * @file ui_encoder.c
 * @brief UI 编码电机调试页实现
 * @note  - 实时显示 ENCODER_1/ENCODER_2 计数值与转速 (RPM)
 *        - K2 切换"选中电机" (Motor A / Motor B)
 *        - K3 当前选中电机速度 +10 (上限 100)
 *        - K1 退出回菜单, 自动停车
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "ui_encoder.h"
#include "encoder.h"
#include "motor_driver.h"
#include "tft_display.h"
#include <stdio.h>

/* 选中的电机 (0=Motor A, 1=Motor B) */
static uint8_t s_cur_motor = 0;
/* 两个电机的当前速度 (-100~100) */
static int8_t  s_speed[2]  = { 0, 0 };

/* RPM 计算节流计数器 (每 ~200ms 计算一次, 50ms × 4) */
static uint32_t s_rpm_tick = 0;
#define RPM_TICK_THRESHOLD  4

/* 上一次显示的 RPM (暂未使用, 但保留用于将来优化"仅变化时刷新") */
static float s_last_rpm[2] = { 0, 0 };

/**
 * @brief 绘制编码电机调试页静态部分 (不含实时刷新)
 */
static void ui_encoder_draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 顶部标题 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, 1, "ENCODER+MOTOR", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 编码器 1 / 2 区域 */
    ui_draw_string(2, 14, "ENC1:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, 24, "RPM :", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    ui_draw_string(2, 54, "ENC2:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, 64, "RPM :", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 分隔线 */
    ui_draw_hline(0, 94, UI_SCREEN_W, TFT_COLOR_BLUE);

    /* 电机控制区 */
    ui_draw_string(2,  100, "M-A:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(70, 100, "M-B:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    /* 底部提示 */
    ui_draw_hline(0, 130, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 136, "[K1] Back/Stop", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 146, "[K2] MotorSel",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 156, "[K3] Speed+",     TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制编码器计数、转速、电机速度
 */
static void ui_encoder_draw_data(void)
{
    char buf[20];

    /* ENC1 计数 */
    int c1 = encoder_get_count(ENCODER_1);
    ui_fill_rect(36, 14, 90, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%8d", c1);
    ui_draw_string(36, 14, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* ENC1 RPM */
    ui_fill_rect(36, 24, 90, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%7.1f", s_last_rpm[0]);
    ui_draw_string(36, 24, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* ENC2 计数 */
    int c2 = encoder_get_count(ENCODER_2);
    ui_fill_rect(36, 54, 90, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%8d", c2);
    ui_draw_string(36, 54, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* ENC2 RPM */
    ui_fill_rect(36, 64, 90, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%7.1f", s_last_rpm[1]);
    ui_draw_string(36, 64, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* 电机速度 */
    ui_fill_rect(36, 100, 30, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%4d", (int)s_speed[0]);
    ui_draw_string(36, 100, buf,
        (s_cur_motor == 0) ? TFT_COLOR_GREEN : TFT_COLOR_WHITE,
        TFT_COLOR_BLACK);

    ui_fill_rect(102, 100, 30, UI_FONT_H, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%4d", (int)s_speed[1]);
    ui_draw_string(102, 100, buf,
        (s_cur_motor == 1) ? TFT_COLOR_GREEN : TFT_COLOR_WHITE,
        TFT_COLOR_BLACK);

    /* 选中箭头 */
    ui_draw_arrow_right(2,  100, (s_cur_motor == 0) ? TFT_COLOR_GREEN : TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_arrow_right(70, 100, (s_cur_motor == 1) ? TFT_COLOR_GREEN : TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    s_cur_motor  = 0;
    s_speed[0]   = 0;
    s_speed[1]   = 0;
    s_rpm_tick   = 0;
    s_last_rpm[0] = 0;
    s_last_rpm[1] = 0;

    /* 进入时先停车 */
    motor_brake_all();

    ui_encoder_draw_static();
    ui_encoder_draw_data();
}

static void page_exit(void)
{
    /* 退出时务必停车 */
    motor_brake_all();
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        /* 退出回菜单 (on_exit 自动停车) */
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K2) {
        /* 切换选中电机 */
        s_cur_motor ^= 1;
        ui_encoder_draw_data();
    } else if (key == UI_KEY_K3) {
        /* 当前选中电机速度 +10 (上限 100) */
        if (s_speed[s_cur_motor] < 100) {
            int16_t candidate = (int16_t)s_speed[s_cur_motor] + 10;
            s_speed[s_cur_motor] = (candidate > 100) ? 100 : (int8_t)candidate;
            motor_set_speed((motor_id_t)s_cur_motor, s_speed[s_cur_motor]);
            ui_encoder_draw_data();
        }
    }
}

static void page_refresh(void)
{
    /* 每 ~200ms 重算 RPM */
    s_rpm_tick++;
    if (s_rpm_tick >= RPM_TICK_THRESHOLD) {
        s_rpm_tick = 0;
        s_last_rpm[0] = encoder_get_rpm(ENCODER_1, 200);
        s_last_rpm[1] = encoder_get_rpm(ENCODER_2, 200);
        ui_encoder_draw_data();
    }
}

/* ========== 页面注册 ========== */

static const ui_page_t encoder_page = {
    .id         = UI_PAGE_ENCODER,
    .title      = "Encoder & Motor",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_encoder_register(void)
{
    return lcd_ui_register_page(&encoder_page);
}