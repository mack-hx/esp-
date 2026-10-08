/**
 * @file ui_balance.c
 * @brief UI 平衡控制页实现
 * @note  屏幕只有 128x160, 每行最多 21 个字符, 所有显示串必须 ≤21 字符
 *        (超了 ui_draw_string 会自动换行, 把下面的布局冲掉)。
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-04
 */

#include "ui_balance.h"
#include "balance.h"
#include "tft_display.h"
#include <stdio.h>

/* ========== 绘制坐标 (Y) ========== */
#define Y_TITLE      1       /* 标题栏 */
#define Y_STATE      14      /* 状态机状态 + 使能标志 */
#define Y_PITCH      26      /* 姿态角 + 角速度 */
#define Y_TARGET     38      /* 目标倾角 */
#define Y_BALOUT     50      /* 平衡环输出 */
#define Y_MOTOR      62      /* 电机实际输出 */
#define Y_SPEED      74      /* 轮速 + 配平补偿 */
#define Y_SEP        86      /* 分隔线 */
#define Y_HINT       94      /* 按键提示第 1 行 */
#define Y_HINT2      106     /* 按键提示第 2 行 */

/* 数值区 X 起点 + 擦除宽度 (擦除宽度必须 >= 文本最长宽度, 否则位数变化会留残影) */
#define VAL_X        30
#define VAL_W        96

/* 状态机状态显示名 */
static const char *STATE_NAMES[] = {
    "IDLE", "STANDBY", "BALANCING", "FALLEN"
};

/**
 * @brief 擦除并重画一行数值
 */
static void draw_val(uint8_t y, const char *str, uint16_t color)
{
    ui_fill_rect(VAL_X, y, VAL_W, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(VAL_X, y, str, color, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制静态骨架 (标题 + 标签 + 分隔线 + 提示)
 */
static void ui_balance_draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, Y_TITLE, "BALANCE CTRL", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    ui_draw_string(2, Y_STATE, "St:",  TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_PITCH, "Pit:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_TARGET,"Tgt:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_BALOUT,"Bal:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_MOTOR, "Mot:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_SPEED, "Whl:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    ui_draw_hline(0, Y_SEP, UI_SCREEN_W, TFT_COLOR_BLUE);

    ui_draw_string(2, Y_HINT,  "[K1]Bk [K2]On/Off",
                   TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_HINT2, "[K3]E-Stop (clr int)",
                   TFT_COLOR_RED, TFT_COLOR_BLACK);
}

/**
 * @brief 刷新所有实时数值
 */
static void ui_balance_refresh_vals(void)
{
    bal_data_t d = {0};
    balance_get_data(&d);

    char buf[24];
    bal_state_t st = balance_get_state();
    if ((int)st < 0 || (int)st > BAL_STATE_FALLEN) st = BAL_STATE_IDLE;

    /* 状态行: BALANCING 绿色, FALLEN 红色, STANDBY 黄色 */
    uint16_t scolor = TFT_COLOR_WHITE;
    if (st == BAL_STATE_BALANCING) scolor = TFT_COLOR_GREEN;
    if (st == BAL_STATE_FALLEN)    scolor = TFT_COLOR_RED;
    if (st == BAL_STATE_STANDBY)  scolor = TFT_COLOR_YELLOW;

    if (balance_is_enabled()) {
        snprintf(buf, sizeof(buf), "%s ON", STATE_NAMES[st]);
    } else {
        snprintf(buf, sizeof(buf), "%s OFF", STATE_NAMES[st]);
    }
    draw_val(Y_STATE, buf, scolor);

    /* 修正后姿态角 + 角速度 */
    snprintf(buf, sizeof(buf), "%6.2f g%6.1f", (double)d.pitch, (double)d.gyro);
    draw_val(Y_PITCH, buf, TFT_COLOR_WHITE);

    snprintf(buf, sizeof(buf), "%6.2f deg", (double)d.target_pitch);
    draw_val(Y_TARGET, buf, TFT_COLOR_YELLOW);

    snprintf(buf, sizeof(buf), "%6.1f %%", (double)d.bal_out);
    draw_val(Y_BALOUT, buf, TFT_COLOR_MAGENTA);

    snprintf(buf, sizeof(buf), "A%+4d B%+4d", d.motor_a, d.motor_b);
    draw_val(Y_MOTOR, buf, TFT_COLOR_YELLOW);

    /* 轮速 + 配平补偿 (一眼看出补偿有没有在动) */
    snprintf(buf, sizeof(buf), "%5.1f T%5.2f", (double)d.speed, (double)d.trim_comp);
    draw_val(Y_SPEED, buf, TFT_COLOR_GREEN);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    /* 进入页面一律先停机, 防止在别的页面误启动后切进来电机已经在转 */
    balance_set_enable(false);
    ui_balance_draw_static();
    ui_balance_refresh_vals();
}

static void page_exit(void)
{
    balance_set_enable(false);   /* 退出页面时保证电机不转 */
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K2) {
        /* 车没扶正时 balance_set_enable 会拒绝, 状态行会显示出来 */
        balance_set_enable(!balance_is_enabled());
        ui_balance_refresh_vals();
    } else if (key == UI_KEY_K3) {
        balance_emergency_stop();   /* 清空 PID 积分, 避免下次启动猛冲 */
        ui_balance_refresh_vals();
    }
}

static void page_refresh(void)
{
    ui_balance_refresh_vals();
}

/* ========== 页面注册 ========== */

static const ui_page_t balance_page = {
    .id         = UI_PAGE_BALANCE,
    .title      = "Balance",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_balance_register(void)
{
    return lcd_ui_register_page(&balance_page);
}
