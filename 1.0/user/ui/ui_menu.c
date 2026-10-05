/**
 * @file ui_menu.c
 * @brief UI 菜单页实现
 * @note  - 显示所有可进入的功能页面条目 (第一项是 "Back to Home")
 *        - K2 上移 / K3 下移 切换选中条目 (循环)
 *        - 选中行画右箭头 ">" + 反白显示
 *        - K1 进入当前选中条目对应页面
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "ui_menu.h"
#include "tft_display.h"
#include <string.h>

/* 菜单条目 (顺序与 ui_page_id_t 一致, 但跳过 HOME/MENU 本身)
 * 第 0 项是特殊项 "Back to Home" - 不通过 ui_page_id_t 跳转 */
typedef struct {
    ui_page_id_t id;    // UI_PAGE_*; -1 表示 "Back to Home"
    const char  *name;
} menu_item_t;

static const menu_item_t menu_items[] = {
    { (ui_page_id_t)-1, "0. Back to Home"   },
    { UI_PAGE_BALANCE,   "1. Balance Ctrl"   },
    { UI_PAGE_SERVO,    "2. Servo Debug"    },
    { UI_PAGE_ENCODER,  "3. Encoder & Motor"},
    { UI_PAGE_MPU9250,  "4. MPU9250"        },
    { UI_PAGE_WIFI,     "5. Web Tuner"      },
    { UI_PAGE_HC595_TEST, "6. HC595 Test"   },
};
#define MENU_ITEM_COUNT  (sizeof(menu_items) / sizeof(menu_items[0]))
/* "Back to Home" 特殊 id 标识 */
#define MENU_ID_BACK_HOME  ((ui_page_id_t)-1)

/* 当前选中条目 (0 ~ MENU_ITEM_COUNT-1) */
static uint8_t s_selected = 0;

/* 每条目的起始 Y 坐标 */
#define ITEM_Y0   24                       // 第 1 条目 Y
#define ITEM_DY   16                       // 每条目间距 (像素)

/**
 * @brief 绘制整个菜单
 */
static void ui_menu_draw(void)
{
    tft_clear(TFT_COLOR_BLACK);

    /* 顶部标题栏 */
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, 1, "== MENU ==", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    /* 列表条目 */
    for (uint8_t i = 0; i < MENU_ITEM_COUNT; i++) {
        uint8_t y = ITEM_Y0 + i * ITEM_DY;
        if (i == s_selected) {
            /* 选中: 反白 + 右箭头 */
            ui_fill_rect(0, y, UI_SCREEN_W, UI_FONT_H, TFT_COLOR_WHITE);
            ui_draw_arrow_right(2, y, TFT_COLOR_BLACK, TFT_COLOR_WHITE);
            ui_draw_string(10, y, menu_items[i].name,
                           TFT_COLOR_BLACK, TFT_COLOR_WHITE);
        } else {
            /* 未选中: 右箭头占位 + 白字 */
            ui_draw_arrow_right(2, y, TFT_COLOR_BLACK, TFT_COLOR_BLACK);
            ui_draw_string(10, y, menu_items[i].name,
                           TFT_COLOR_WHITE, TFT_COLOR_BLACK);
        }
    }

    /* 底部操作提示 */
    ui_draw_hline(0, 130, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 136, "[K1] Enter",   TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    ui_draw_string(2, 146, "[K2/K3] Move", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    s_selected = 0;
    ui_menu_draw();
}

static void page_exit(void)
{
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        /* 进入选中条目 */
        if (s_selected < MENU_ITEM_COUNT) {
            if (menu_items[s_selected].id == MENU_ID_BACK_HOME) {
                /* "Back to Home" 特殊项: 直接跳转 HOME
                 * (不能走 lcd_ui_switch_to((ui_page_id_t)-1), 会被内部的
                 *  id >= UI_PAGE_MAX 校验判 INVALID_ARG) */
                lcd_ui_switch_to(UI_PAGE_HOME);
            } else {
                lcd_ui_switch_to(menu_items[s_selected].id);
            }
        }
    } else if (key == UI_KEY_K2) {
        /* 上移 (循环到底) */
        s_selected = (s_selected == 0) ? (MENU_ITEM_COUNT - 1) : (s_selected - 1);
        ui_menu_draw();
    } else if (key == UI_KEY_K3) {
        /* 下移 (循环到顶) */
        s_selected = (s_selected + 1) % MENU_ITEM_COUNT;
        ui_menu_draw();
    }
}

static void page_refresh(void)
{
    /* 菜单页无实时刷新 */
}

/* ========== 页面注册 ========== */

static const ui_page_t menu_page = {
    .id         = UI_PAGE_MENU,
    .title      = "Menu",
    .on_enter   = page_enter,
    .on_exit    = page_exit,
    .on_key     = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_menu_register(void)
{
    return lcd_ui_register_page(&menu_page);
}