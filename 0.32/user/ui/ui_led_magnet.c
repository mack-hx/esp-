/**
 * @file ui_led_magnet.c
 * @brief LED、电磁铁、微动开关 共用检查/控制页实现
 * @author ESP32 机械狗项目
 * @date 2026-10-25
 */

#include "ui_led_magnet.h"
#include "lcd_ui.h"
#include "tft_display.h"
#include "led.h"
#include "magnet.h"
#include "pcf8574.h"
#include <stdio.h>

/* ========== 页面状态 ========== */
/* 当前选中的颜色预设 (LED_PRESET_*) */
static led_preset_t s_preset = LED_PRESET_RED;
/* 当前亮度档位 (0~255) */
static uint8_t      s_bright = 255;
/* 电磁铁目标状态 (两个一起) */
static bool         s_magnet = false;

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
#define Y_SEP1       56      /* 分隔线 */
#define Y_MAG_LBL    60      /* "MAGNET" 分区标题 */
#define Y_MAG_BOX    69      /* 电磁铁格子 */
#define MAG_BOX_W    60      /* 格子宽 */
#define MAG_BOX_H    20      /* 格子高 */
#define X_BOX2       66      /* 第二个格子 X */
#define Y_SW_LBL     93      /* "DOCK SW" 分区标题 */
#define Y_SW_BOX     102     /* 微动开关格子 */
#define SW_BOX_H     18      /* 格子高 */
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
    ui_draw_string(2, Y_TITLE, "== LED & MAGNET ==", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 分区标题 */
    ui_draw_string(2, Y_LED_LBL, "RGB LED",   TFT_COLOR_CYAN,   TFT_COLOR_BLACK);
    ui_draw_string(2, Y_MAG_LBL, "MAGNET",    TFT_COLOR_CYAN,   TFT_COLOR_BLACK);
    ui_draw_string(2, Y_SW_LBL,  "DOCK SW",   TFT_COLOR_CYAN,   TFT_COLOR_BLACK);

    /* 分隔线 */
    ui_draw_hline(0, Y_SEP1, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_hline(0, Y_SEP2, UI_SCREEN_W, TFT_COLOR_BLUE);

    /* 亮度条底槽 */
    ui_fill_rect(2, Y_BAR, UI_SCREEN_W - 4, BAR_H, TFT_COLOR_BLUE);

    /* 电磁铁两个格子的外框 */
    ui_draw_rect(2, Y_MAG_BOX, MAG_BOX_W, MAG_BOX_H, TFT_COLOR_BLUE);
    ui_draw_rect(X_BOX2, Y_MAG_BOX, MAG_BOX_W, MAG_BOX_H, TFT_COLOR_BLUE);

    /* 微动开关两个格子的外框 */
    ui_draw_rect(2, Y_SW_BOX, MAG_BOX_W, SW_BOX_H, TFT_COLOR_BLUE);
    ui_draw_rect(X_BOX2, Y_SW_BOX, MAG_BOX_W, SW_BOX_H, TFT_COLOR_BLUE);

    /* 按键提示 */
    ui_draw_string(2, Y_HINT1, "[K1] Back",    TFT_COLOR_GREEN,  TFT_COLOR_BLACK);
    ui_draw_string(2, Y_HINT2, "[K2] Color",   TFT_COLOR_WHITE,  TFT_COLOR_BLACK);
    ui_draw_string(2, Y_HINT3, "[K3] Bright",  TFT_COLOR_WHITE,  TFT_COLOR_BLACK);
    ui_draw_string(60, Y_HINT3, "[K4] Magnet", TFT_COLOR_WHITE,  TFT_COLOR_BLACK);
}

/**
 * @brief 刷新 LED 状态 (色块 / 颜色名 / 亮度值 / 亮度条)
 */
static void draw_led(void)
{
    led_color_t c = led_get_color(LED_1);
    uint8_t br   = led_get_brightness(LED_1);
    bool on      = led_get_on(LED_1);

    /* --- 色块: 显示实际输出颜色 (按亮度缩放) --- */
    uint8_t sr = on ? (uint8_t)((c.r * br) / 255) : 0;
    uint8_t sg = on ? (uint8_t)((c.g * br) / 255) : 0;
    uint8_t sb = on ? (uint8_t)((c.b * br) / 255) : 0;
    ui_fill_rect(2, Y_SWATCH, SWATCH_W, SWATCH_H, rgb565(sr, sg, sb));
    /* 色块外框 (白), 让深色时也能看出块在哪 */
    ui_draw_rect(2, Y_SWATCH, SWATCH_W, SWATCH_H, TFT_COLOR_WHITE);

    /* --- 右侧文字 --- */
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
    /* 先把整条槽重画成蓝色底, 再覆盖已点亮部分 */
    ui_fill_rect(2, Y_BAR, UI_SCREEN_W - 4, BAR_H, TFT_COLOR_BLUE);
    if (w > 0) {
        uint16_t col = (br < 64) ? TFT_COLOR_RED
                     : (br < 160) ? TFT_COLOR_YELLOW : TFT_COLOR_GREEN;
        ui_fill_rect(2, Y_BAR, (uint8_t)w, BAR_H, col);
    }
}

/**
 * @brief 刷新两个电磁铁格子
 */
static void draw_magnet(void)
{
    bool m1 = magnet_get(MAGNET_1);
    bool m2 = magnet_get(MAGNET_2);

    /* 格子内画 "MAG1" + ON/OFF; 通电时整格填绿 + 黑字, 断电时黑底红字 */
    struct {
        uint8_t x;
        const char *label;
        bool on;
    } boxes[2] = {
        { 2,      "MAG1", m1 },
        { X_BOX2, "MAG2", m2 },
    };

    for (int i = 0; i < 2; i++) {
        uint8_t x = boxes[i].x;
        if (boxes[i].on) {
            ui_fill_rect(x, Y_MAG_BOX, MAG_BOX_W, MAG_BOX_H, TFT_COLOR_GREEN);
            ui_draw_rect(x, Y_MAG_BOX, MAG_BOX_W, MAG_BOX_H, TFT_COLOR_GREEN);
            ui_draw_string(x + 4, Y_MAG_BOX + 6,  boxes[i].label, TFT_COLOR_BLACK, TFT_COLOR_GREEN);
            ui_draw_string(x + 34, Y_MAG_BOX + 6, "ON",    TFT_COLOR_BLACK, TFT_COLOR_GREEN);
        } else {
            ui_fill_rect(x, Y_MAG_BOX, MAG_BOX_W, MAG_BOX_H, TFT_COLOR_BLACK);
            ui_draw_rect(x, Y_MAG_BOX, MAG_BOX_W, MAG_BOX_H, TFT_COLOR_BLUE);
            ui_draw_string(x + 4, Y_MAG_BOX + 6,  boxes[i].label, TFT_COLOR_CYAN, TFT_COLOR_BLACK);
            ui_draw_string(x + 34, Y_MAG_BOX + 6, "OFF",   TFT_COLOR_RED,   TFT_COLOR_BLACK);
        }
    }
}

/**
 * @brief 刷新两个对接微动开关格子
 */
static void draw_switch(void)
{
    bool sw1 = false, sw2 = false;
    pcf8574_read_dock_switches(&sw1, &sw2);

    struct {
        uint8_t x;
        const char *label;
        bool pressed;
    } boxes[2] = {
        { 2,      "SW1", sw1 },
        { X_BOX2, "SW2", sw2 },
    };

    for (int i = 0; i < 2; i++) {
        uint8_t x = boxes[i].x;
        if (boxes[i].pressed) {
            ui_fill_rect(x, Y_SW_BOX, MAG_BOX_W, SW_BOX_H, TFT_COLOR_YELLOW);
            ui_draw_rect(x, Y_SW_BOX, MAG_BOX_W, SW_BOX_H, TFT_COLOR_YELLOW);
            ui_draw_string(x + 4, Y_SW_BOX + 5,  boxes[i].label, TFT_COLOR_BLACK, TFT_COLOR_YELLOW);
            ui_draw_string(x + 34, Y_SW_BOX + 5, "HIT",   TFT_COLOR_BLACK, TFT_COLOR_YELLOW);
        } else {
            ui_fill_rect(x, Y_SW_BOX, MAG_BOX_W, SW_BOX_H, TFT_COLOR_BLACK);
            ui_draw_rect(x, Y_SW_BOX, MAG_BOX_W, SW_BOX_H, TFT_COLOR_BLUE);
            ui_draw_string(x + 4, Y_SW_BOX + 5,  boxes[i].label, TFT_COLOR_CYAN, TFT_COLOR_BLACK);
            ui_draw_string(x + 34, Y_SW_BOX + 5, "FREE",  TFT_COLOR_WHITE, TFT_COLOR_BLACK);
        }
    }
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    /* 初始状态跟硬件对齐 (App 或上一次操作可能已经改过) */
    led_color_t c = led_get_color(LED_1);
    int idx = preset_index_of(c);
    s_preset = (idx >= 0) ? (led_preset_t)idx : LED_PRESET_RED;
    s_bright = led_get_brightness(LED_1);
    s_magnet = magnet_get(MAGNET_1) || magnet_get(MAGNET_2);

    draw_static();
    draw_led();
    draw_bar();
    draw_magnet();
    draw_switch();
}

static void page_exit(void)
{
    /* 退出时断开电磁铁 —— 通电状态别带到别的页面去 */
    if (s_magnet) {
        magnet_release_all();
        s_magnet = false;
    }
}

static void page_key(ui_key_t key)
{
    switch (key) {
        case UI_KEY_K1:
            /* 退出回菜单 */
            lcd_ui_switch_to(UI_PAGE_MENU);
            break;

        case UI_KEY_K2:
            /* 换颜色预设 (循环) */
            s_preset = (led_preset_t)((s_preset + 1) % LED_PRESET_MAX);
            apply_led();
            draw_led();
            draw_bar();
            break;

        case UI_KEY_K3:
            /* 亮度 +16 (加到 255 之后回到 0 继续加) */
            s_bright = (uint8_t)((s_bright + BRIGHT_STEP) & 0xFF);
            apply_led();
            draw_led();
            draw_bar();
            break;

        case UI_KEY_K4:
            /* 电磁铁开/关 (两个一起, 跟 App 的 magnet_on 一致) */
            s_magnet = !s_magnet;
            magnet_set_both(s_magnet, s_magnet);
            draw_magnet();
            break;

        default:
            break;
    }
}

static void page_refresh(void)
{
    /* 页面局部状态跟着硬件走 (App 远程改过灯/磁铁时这里同步过来) */
    led_color_t c = led_get_color(LED_1);
    int idx = preset_index_of(c);
    if (idx >= 0) {
        s_preset = (led_preset_t)idx;
    }
    s_bright = led_get_brightness(LED_1);
    s_magnet = magnet_get(MAGNET_1) || magnet_get(MAGNET_2);

    draw_led();
    draw_bar();
    draw_magnet();
    draw_switch();
}

/* ========== 页面注册 ========== */

static const ui_page_t led_magnet_page = {
    .id         = UI_PAGE_LED_MAGNET,
    .title      = "LED & Magnet",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_led_magnet_register(void)
{
    return lcd_ui_register_page(&led_magnet_page);
}
