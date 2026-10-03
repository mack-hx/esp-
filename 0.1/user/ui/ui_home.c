/**
 * @file ui_home.c
 * @brief UI 主页实现 (上电默认显示)
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "ui_home.h"
#include "tft_display.h"
#include "esp_log.h"

static const char *TAG __attribute__((unused)) = "UI_HOME";

/* 主页状态 (未来可放运行时变量: 时间/电量等) */
static uint32_t s_loop_count = 0;

/**
 * @brief 绘制主页固定内容
 */
static void ui_home_draw(void)
{
    tft_clear(TFT_COLOR_BLACK);

    // 顶部标题栏 (反白显示)
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, 1, "ESP BALANCE ROBOT", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    // 项目名称
    ui_draw_string(10, 24, "ESP32-S3R8N8", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(10, 36, "Balance Dog", TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    // 分隔线
    ui_draw_hline(0, 50, UI_SCREEN_W, TFT_COLOR_BLUE);

    // 状态信息
    ui_draw_string(2, 60, "FW Ver: 0.1", TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
    ui_draw_string(2, 72, "LCD: ST7735S", TFT_COLOR_CYAN, TFT_COLOR_BLACK);

    // 分隔线
    ui_draw_hline(0, 90, UI_SCREEN_W, TFT_COLOR_BLUE);

    // 操作提示
    ui_draw_string(2, 100, "[K1] Menu",  TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 112, "[K2/K3] N/A", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    // 底部状态栏
    ui_draw_hline(0, 134, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 140, "Running...", TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    ui_draw_string(2, 150, "Press K1",   TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    s_loop_count = 0;
    ui_home_draw();
}

static void page_exit(void)
{
    // 主页无需释放资源
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        // 进入菜单页
        lcd_ui_switch_to(UI_PAGE_MENU);
    }
    // K2/K3 在主页无操作
}

static void page_refresh(void)
{
    // 周期性刷新: 更新底部运行计数器 (模拟心跳)
    s_loop_count++;
    char buf[16];
    snprintf(buf, sizeof(buf), "Loop:%lu", (unsigned long)s_loop_count);
    ui_fill_rect(0, 140, UI_SCREEN_W, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(2, 140, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
}

/* ========== 页面注册 ========== */

static const ui_page_t home_page = {
    .id           = UI_PAGE_HOME,
    .title        = "Home",
    .on_enter     = page_enter,
    .on_exit      = page_exit,
    .on_key       = page_key,
    .on_refresh   = page_refresh,
};

esp_err_t ui_home_register(void)
{
    return lcd_ui_register_page(&home_page);
}