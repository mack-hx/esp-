/**
 * @file ui_led.c
 * @brief LED 检查/控制页实现
 * @note  左上角色块显示当前真实输出颜色 (已按亮度缩放)
 *
 * 按键: K1 退出回菜单 / K2 灯颜色循环 / K3 灯亮度 +16 循环
 *
 * @author ESP32 机械狗项目
 */

#include "ui_led.h"
#include "lcd_ui.h"
#include "tft_display.h"
#include "led.h"
#include <stdio.h>

/* ========== 页面状态 ========== */
/* 当前选中的颜色预设 (LED_PRESET_*) */
static led_preset_t s_preset = LED_PRESET_RED;
/* 当前亮度档位 (0~255) */
static uint8_t      s_bright = 255;

/* ========== 绘制坐标 (128x160, 字体 6x8) ========== */
#define Y_TITLE      1       /* 标题栏文字 */
#define Y_LED_LBL    12      /* "RGB LED" 分区标题 */
#define Y_SWATCH     20      /* 色块左上角 */
#define SWATCH_W     40      /* 色块宽 */
#define SWATCH_H     24      /* 色块高 */
#define Y_COLOR_TXT  20      /* 色块右侧: 颜色名 */
#define Y_BRI_TXT    30      /* 色块右侧: 亮度值 */
#define Y_STATE_TXT  40      /* 色块右侧: 亮/灭状态 */
#define X_INFO       48      /* 右侧文字起始 X */
#define Y_BAR        46      /* 亮度进度条 */
#define BAR_H        7       /* 进度条高度 */
#define Y_SEP2       124     /* 分隔线 */
#define Y_HINT1      130     /* 按键提示 */
#define Y_HINT2      141
#define Y_HINT3      152

/* 亮度步进 (K3 每按一次 +16, 0→16→…→240→255→0 循环) */
#define BRIGHT_STEP  16

/* 颜色预设表 (与 led.c 内部 preset_colors 一致, 顺序跟 led_preset_t 相同) */
static const led_color_t PRESET_RGB[LED_PRESET_MAX] = {
    [LED_PRESET_OFF]     = { 0,   0,   0   },
    [LED_PRESET_RED]     = { 255, 0,   0   },
    [LED_PRESET_GREEN]   = { 0,   255, 0   },
    [LED_PRESET_BLUE]    = { 0,   0,   255 },
    [LED_PRESET_YELLOW]  = { 255, 255, 0   },
    [LED_PRESET_CYAN]    = { 0,   255, 255 },
    [LED_PRESET_MAGENTA] = { 255, 0,   255 },
    [LED_PRESET_WHITE]   = { 255, 255, 255 },
};

/* 颜色预设名 */
static const char *PRESET_NAME[LED_PRESET_MAX] = {
    "OFF", "RED", "GREEN", "BLUE", "YELLOW", "CYAN", "MAGENTA", "WHITE"
};

/* ========== 内部工具 ========== */

/**
 * @brief 8 位 RGB 转 RGB565 (屏幕用的颜色格式)
 * @param r 红 0~255
 * @param g 绿 0~255
 * @param b 蓝 0~255
 * @return RGB565 颜色值
 */
static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/**
 * @brief 在颜色表里找某个 RGB 对应的预设序号
 * @param c 待查颜色
 * @return 预设序号; 找不到返回 -1
 */
static int preset_index_of(led_color_t c)
{
    for (int i = 0; i < LED_PRESET_MAX; i++) {
        if (PRESET_RGB[i].r == c.r && PRESET_RGB[i].g == c.g && PRESET_RGB[i].b == c.b) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 把当前选中的颜色 + 亮度真正下发给灯珠
 */
static void apply_led(void)
{
    led_set_color_brightness(LED_1, PRESET_RGB[s_preset], s_bright);
}

/**
 * @brief 画整页静态骨架 (标题栏 / 分区标题 / 分隔线 / 按键提示)
 */
static void draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 顶部标题栏 (反白) */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, Y_TITLE, "==== LED ====", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    ui_draw_string(2, Y_LED_LBL, "RGB LED", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_hline(0, Y_SEP2, UI_SCREEN_W, TFT_COLOR_BLUE);

    /* 亮度条底槽 */
    ui_fill_rect(2, Y_BAR, UI_SCREEN_W - 4, BAR_H, TFT_COLOR_BLUE);

    /* 按键提示 */
    ui_draw_string(2, Y_HINT1, "[K1] Back",   TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_HINT2, "[K2] Color",  TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, Y_HINT3, "[K3] Bright", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/**
 * @brief 刷新 LED 状态 (色块 / 颜色名 / 亮度值)
 * @note  颜色与亮度直接取驱动缓存, App 远程改灯时这一页也能反映真实状态
 */
static void draw_led(void)
{
    led_color_t c = led_get_color(LED_1);
    uint8_t br   = led_get_brightness(LED_1);
    bool on      = led_get_on(LED_1);

    /* 色块: 显示实际输出颜色 (按亮度缩放) */
    uint8_t sr = on ? (uint8_t)((c.r * br) / 255) : 0;
    uint8_t sg = on ? (uint8_t)((c.g * br) / 255) : 0;
    uint8_t sb = on ? (uint8_t)((c.b * br) / 255) : 0;
    ui_fill_rect(2, Y_SWATCH, SWATCH_W, SWATCH_H, rgb565(sr, sg, sb));
    ui_draw_rect(2, Y_SWATCH, SWATCH_W, SWATCH_H, TFT_COLOR_WHITE);

    int idx = preset_index_of(c);
    const char *name = (idx >= 0) ? PRESET_NAME[idx] : "CUSTOM";

    ui_fill_rect(X_INFO, Y_COLOR_TXT, UI_SCREEN_W - X_INFO - 2, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(X_INFO, Y_COLOR_TXT, name, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    char buf[16];
    snprintf(buf, sizeof(buf), "BRI %3u", (unsigned)br);
    ui_fill_rect(X_INFO, Y_BRI_TXT, UI_SCREEN_W - X_INFO - 2, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(X_INFO, Y_BRI_TXT, buf, TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    ui_fill_rect(X_INFO, Y_STATE_TXT, UI_SCREEN_W - X_INFO - 2, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(X_INFO, Y_STATE_TXT, on ? "LIT" : "DARK",
                   on ? TFT_COLOR_GREEN : TFT_COLOR_RED, TFT_COLOR_BLACK);
}

/**
 * @brief 刷新亮度进度条
 */
static void draw_bar(void)
{
    uint8_t br = led_get_brightness(LED_1);
    int w = (int)(br * (UI_SCREEN_W - 4) / 255);
    ui_fill_rect(2, Y_BAR, UI_SCREEN_W - 4, BAR_H, TFT_COLOR_BLUE);
    if (w > 0) {
        uint16_t col = (br < 64) ? TFT_COLOR_RED
                     : (br < 160) ? TFT_COLOR_YELLOW : TFT_COLOR_GREEN;
        ui_fill_rect(2, Y_BAR, (uint8_t)w, BAR_H, col);
    }
}

/**
 * @brief 页面状态跟硬件对齐 (App 或上一次操作可能已经改过)
 */
static void sync_from_hw(void)
{
    int idx = preset_index_of(led_get_color(LED_1));
    if (idx >= 0) {
        s_preset = (led_preset_t)idx;
    }
    s_bright = led_get_brightness(LED_1);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    sync_from_hw();
    draw_static();
    draw_led();
    draw_bar();
}

static void page_key(ui_key_t key)
{
    switch (key) {
        case UI_KEY_K1:
            lcd_ui_switch_to(UI_PAGE_MENU);
            break;

        case UI_KEY_K2:
            s_preset = (led_preset_t)((s_preset + 1) % LED_PRESET_MAX);
            apply_led();
            draw_led();
            draw_bar();
            break;

        case UI_KEY_K3:
            s_bright = (uint8_t)((s_bright + BRIGHT_STEP) & 0xFF);
            apply_led();
            draw_led();
            draw_bar();
            break;

        default:
            break;
    }
}

static void page_refresh(void)
{
    sync_from_hw();
    draw_led();
    draw_bar();
}

/* ========== 页面注册 ========== */

static const ui_page_t led_page = {
    .id         = UI_PAGE_LED,
    .title      = "LED",
    .on_enter   = page_enter,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_led_register(void)
{
    return lcd_ui_register_page(&led_page);
}
