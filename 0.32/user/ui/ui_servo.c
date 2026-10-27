/**
 * @file ui_servo.c
 * @brief UI 舵机调试页实现
 * @note  - K2 切换"选中舵机" (S1 <-> S2)
 *        - K3 当前选中舵机 +5°, K4 当前选中舵机 -5°
 *        - 限位由 servo.c 处理 (SERVO_LIMIT_MIN ~ SERVO_LIMIT_MAX)
 *        - 跨页保持运行, 切页时舵机不会失力或跳变
 * @author ESP32 机械狗项目
 * @date 2026-10-03
 */

#include "ui_servo.h"
#include "servo.h"
#include "tft_display.h"
#include <stdio.h>

/* 当前选中的舵机 (0=SERVO_1, 1=SERVO_2) */
static uint8_t s_cur_servo = 0;

/* UI 上 +5° / -5° 步进 */
#define SERVO_STEP_DEG      5

/**
 * @brief 绘制舵机调试页固定内容
 */
static void ui_servo_draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 顶部标题栏 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, 1, "SERVO DEBUG", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 限位提示 */
    ui_draw_string(2, 14, "Range 30-120", TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    /* 分隔线 */
    ui_draw_hline(0, 24, UI_SCREEN_W, TFT_COLOR_BLUE);

    /* 当前选中舵机 (S1/S2) */
    char label[8];
    snprintf(label, sizeof(label), "S%u", (unsigned)(s_cur_servo + 1));
    ui_draw_string(2, 30, "Sel:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(36, 30, label, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    /* 选中舵机的当前角度 + 进度条 */
    uint8_t cur = servo_get_angle((servo_id_t)s_cur_servo);
    char buf[16];
    snprintf(buf, sizeof(buf), "%3d deg", cur);
    ui_fill_rect(40, 30, 80, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(40, 30, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    ui_fill_rect(2, 44, 124, 6, TFT_COLOR_BLUE);
    uint16_t pos = (uint16_t)(cur - SERVO_LIMIT_MIN) * 124
                   / (SERVO_LIMIT_MAX - SERVO_LIMIT_MIN);
    if (pos > 0) {
        ui_fill_rect(2, 44, (uint8_t)pos, 6, TFT_COLOR_GREEN);
    }

    /* S1 / S2 当前角度 (辅助显示, 不画选中箭头) */
    ui_draw_string(2, 70, "S1:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 86, "S2:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    snprintf(buf, sizeof(buf), "%3d deg", servo_get_angle(SERVO_1));
    ui_fill_rect(36, 70, 70, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(36, 70, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    snprintf(buf, sizeof(buf), "%3d deg", servo_get_angle(SERVO_2));
    ui_fill_rect(36, 86, 70, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(36, 86, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* 选中箭头 (放在 S1/S2 标签右侧) */
    uint8_t arrow_y = (s_cur_servo == 0) ? 70 : 86;
    ui_draw_arrow_right(20, arrow_y, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    /* 分隔线 */
    ui_draw_hline(0, 130, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 136, "[K1] Back",    TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 146, "[K2] Sel",     TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 156, "[K3/K4] +/-5", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    s_cur_servo = 0;
    ui_servo_draw_static();
}

static void page_exit(void)
{
    /* 不释放舵机, 后台任务继续保持输出 */
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        /* 退出回菜单, 舵机保持当前角度继续运行 */
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K2) {
        /* 切换选中舵机 (S1 <-> S2) */
        s_cur_servo ^= 1;
        ui_servo_draw_static();
    } else if (key == UI_KEY_K3) {
        /* +5° (限位由 servo_set_angle 内部处理) */
        uint8_t cur = servo_get_angle((servo_id_t)s_cur_servo);
        uint8_t next = (cur + SERVO_STEP_DEG > SERVO_LIMIT_MAX)
                       ? SERVO_LIMIT_MAX : (cur + SERVO_STEP_DEG);
        servo_set_angle((servo_id_t)s_cur_servo, next);
        ui_servo_draw_static();
    } else if (key == UI_KEY_K4) {
        /* -5° (限位由 servo_set_angle 内部处理) */
        uint8_t cur = servo_get_angle((servo_id_t)s_cur_servo);
        uint8_t next = (cur > SERVO_STEP_DEG) ? (cur - SERVO_STEP_DEG) : SERVO_LIMIT_MIN;
        servo_set_angle((servo_id_t)s_cur_servo, next);
        ui_servo_draw_static();
    }
}

static void page_refresh(void)
{
    /* 后台舵机由 servo_keep_alive_task 持续运行, 此页无需刷新 */
}

/* ========== 页面注册 ========== */

static const ui_page_t servo_page = {
    .id         = UI_PAGE_SERVO,
    .title      = "Servo Debug",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_servo_register(void)
{
    return lcd_ui_register_page(&servo_page);
}