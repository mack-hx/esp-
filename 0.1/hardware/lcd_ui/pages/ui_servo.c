/**
 * @file ui_servo.c
 * @brief UI 舵机调试页实现
 * @note  - 默认显示舵机 1 当前角度, 可视化条
 *        - K2 切换"选中舵机" (在 SERVO_1 / SERVO_2 之间)
 *        - K3 选中舵机角度 +10°, K2 长按角度 -10° (此版本只支持 -10 配合 K1)
 *          简化方案: K3 +10°, K1 减 -10° 并退出回菜单
 *          再简化: K3 +5°, K2 -5°, K1 退出回菜单
 *        - 进入页面时舵机自动归中 (90°), 退出时停止输出
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "ui_servo.h"
#include "servo.h"
#include "tft_display.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG __attribute__((unused)) = "UI_SERVO";

/* 当前选中的舵机 (0/1) */
static uint8_t s_cur_servo = 0;
/* 两只舵机当前角度 (0-180) */
static uint8_t s_angle[2] = { 90, 90 };

/**
 * @brief 绘制舵机调试页固定内容 (不含实时刷新部分)
 */
static void ui_servo_draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    // 顶部标题栏
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, 1, "SERVO DEBUG", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    // 当前选中舵机标记
    ui_draw_string(2, 14, "Sel: S", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    char buf[4];
    snprintf(buf, sizeof(buf), "%u", (unsigned)(s_cur_servo + 1));
    ui_draw_string(40, 14, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    // 分隔线
    ui_draw_hline(0, 24, UI_SCREEN_W, TFT_COLOR_BLUE);

    // 舵机 1 区域
    ui_draw_string(2, 30, "S1:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    if (s_cur_servo == 0) {
        ui_draw_arrow_right(40, 30, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    }

    // 舵机 2 区域
    ui_draw_string(2, 90, "S2:", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    if (s_cur_servo == 1) {
        ui_draw_arrow_right(40, 90, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    }

    // 底部提示
    ui_draw_hline(0, 130, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 136, "[K1] Back/Minus", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 146, "[K2] ServoSel",  TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 156, "[K3] Angle+",    TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制某个舵机的角度 + 可视化进度条
 */
static void ui_servo_draw_one(uint8_t idx, uint8_t y)
{
    // 角度文本
    char buf[16];
    snprintf(buf, sizeof(buf), "%3d deg", s_angle[idx]);
    ui_fill_rect(50, y, 60, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(50, y, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    // 进度条 (背景)
    ui_fill_rect(2, y + 10, 124, 6, TFT_COLOR_BLUE);
    // 进度条 (前景)
    uint8_t bar_w = (uint8_t)((uint16_t)s_angle[idx] * 124 / 180);
    if (bar_w > 0) {
        ui_fill_rect(2, y + 10, bar_w, 6, TFT_COLOR_GREEN);
    }
}

/**
 * @brief 立即把角度应用到舵机硬件
 */
static void apply_angle(uint8_t idx)
{
    servo_set_angle((servo_id_t)idx, s_angle[idx]);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    // 注意: 不在此重置 s_cur_servo / s_angle, 保留用户上次调整的状态
    // (避免快速来回切换时舵机剧烈抖动 + 屏幕闪烁)
    //
    // lcd_ui_switch_to() 已经做过 tft_clear + on_enter 调用,
    // 这里只负责绘制本页面内容 + 让舵机同步到当前 s_angle
    ui_servo_draw_static();
    ui_servo_draw_one(0, 30);
    ui_servo_draw_one(1, 90);
    // 同步舵机硬件到软件状态 (避免退出后再回来时舵机停在中间位置)
    servo_set_angle(SERVO_1, s_angle[0]);
    servo_set_angle(SERVO_2, s_angle[1]);
}

static void page_exit(void)
{
    // 退出时释放舵机, 防止抖动
    servo_release(SERVO_1);
    servo_release(SERVO_2);
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        // 退出回菜单
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K2) {
        // 切换选中舵机
        s_cur_servo ^= 1;  // 0 <-> 1
        // 重绘箭头位置
        if (s_cur_servo == 0) {
            ui_draw_arrow_right(40, 30, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
            ui_draw_arrow_right(40, 90, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
        } else {
            ui_draw_arrow_right(40, 30, TFT_COLOR_WHITE, TFT_COLOR_BLACK);
            ui_draw_arrow_right(40, 90, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
        }
    } else if (key == UI_KEY_K3) {
        // 当前舵机角度 +5° (上限 180)
        if (s_angle[s_cur_servo] <= 175) {
            s_angle[s_cur_servo] += 5;
        } else {
            s_angle[s_cur_servo] = 180;
        }
        apply_angle(s_cur_servo);
        ui_servo_draw_one(s_cur_servo, (s_cur_servo == 0) ? 30 : 90);
    }
}

static void page_refresh(void)
{
    // 当前角度已经在 K3 按下时同步到硬件, 无需再刷新显示
    // 后续可加入"心跳"或自动扫角等功能
}

/* ========== 页面注册 ========== */

static const ui_page_t servo_page = {
    .id        = UI_PAGE_SERVO,
    .title     = "Servo Debug",
    .on_enter  = page_enter,
    .on_exit   = page_exit,
    .on_key    = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_servo_register(void)
{
    return lcd_ui_register_page(&servo_page);
}