/**
 * @file lcd_ui.c
 * @brief LCD UI 系统主调度实现
 * @note  单任务轮询架构:
 *        - lcd_ui_init: 注册按键回调
 *        - lcd_ui_loop: 每 50ms 调用一次
 *            * 读取按键 → 派发到当前页面的 on_key
 *            * 累加 tick 计数器, 每 150ms 调用 on_refresh (实时数据刷新)
 *        - lcd_ui_switch_to: 处理旧页退出 + 新页进入
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "lcd_ui.h"
#include "tft_display.h"
#include "font6x8.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "LCD_UI";

/* ========== 内部状态 ========== */
#define MAX_PAGES  8
static const ui_page_t *pages[MAX_PAGES];
static uint8_t page_count = 0;

static ui_page_id_t current_page = UI_PAGE_HOME;
static ui_key_get_t key_get_cb = NULL;

/* 主循环周期: 50ms 按键扫描, 150ms 页面刷新 (3 个 tick) */
#define LOOP_TICK_MS         50
#define REFRESH_TICKS        3     // 150ms
static uint8_t tick_counter = 0;

/* ========== UI 工具 API: 绘图 (供所有页面调用) ========== */

void ui_draw_char(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bg)
{
    const uint8_t *font = font6x8_get((uint8_t)ch);

    // 设置写窗口: 字符 6x8 区域
    esp_err_t ret = tft_set_window(x, y, x + UI_FONT_W - 1, y + UI_FONT_H - 1);
    if (ret != ESP_OK) return;

    // 按行写入 (TFT RAMWR 按 X 推进: x0→x1→换行)
    // 每行 6 列, 每列 1 字节字模 = 6 列像素 (高位=第1行, ..., 低位=第8行)
    // 注意: 字模按列存, font[col] 的 bit(row) 表示 (col,row) 像素
    for (uint8_t row = 0; row < UI_FONT_H; row++) {
        uint8_t line_data[UI_FONT_W * 2];
        for (uint8_t col = 0; col < UI_FONT_W; col++) {
            uint16_t c = ((font[col] >> row) & 0x01) ? color : bg;
            line_data[col * 2]     = c >> 8;
            line_data[col * 2 + 1] = c & 0xFF;
        }
        tft_write_data(line_data, UI_FONT_W * 2);
    }
}

void ui_draw_string(uint8_t x, uint8_t y, const char *str,
                    uint16_t color, uint16_t bg)
{
    if (str == NULL) return;
    uint8_t cur_x = x;
    uint8_t cur_y = y;
    while (*str) {
        if (*str == '\n') {
            cur_x = x;
            cur_y += UI_FONT_H;
            str++;
            continue;
        }
        // 超出右边距, 换行
        if (cur_x + UI_FONT_W > UI_SCREEN_W) {
            cur_x = x;
            cur_y += UI_FONT_H;
        }
        // 超出下边距, 截断
        if (cur_y + UI_FONT_H > UI_SCREEN_H) {
            break;
        }
        ui_draw_char(cur_x, cur_y, *str, color, bg);
        cur_x += UI_FONT_W;
        str++;
    }
}

void ui_draw_hline(uint8_t x, uint8_t y, uint8_t w, uint16_t color)
{
    if (y >= UI_SCREEN_H) return;
    esp_err_t ret = tft_set_window(x, y, x + w - 1, y);
    if (ret != ESP_OK) return;
    uint8_t line_buf[UI_SCREEN_W * 2];
    for (uint8_t i = 0; i < w; i++) {
        line_buf[i * 2]     = color >> 8;
        line_buf[i * 2 + 1] = color & 0xFF;
    }
    tft_write_data(line_buf, w * 2);
}

void ui_draw_vline(uint8_t x, uint8_t y, uint8_t h, uint16_t color)
{
    if (x >= UI_SCREEN_W) return;
    esp_err_t ret = tft_set_window(x, y, x, y + h - 1);
    if (ret != ESP_OK) return;
    uint8_t col_buf[2] = { color >> 8, color & 0xFF };
    for (uint8_t i = 0; i < h; i++) {
        tft_write_data(col_buf, 2);
    }
}

void ui_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color)
{
    if (w == 0 || h == 0) return;
    ui_draw_hline(x, y, w, color);
    ui_draw_hline(x, y + h - 1, w, color);
    ui_draw_vline(x, y, h, color);
    ui_draw_vline(x + w - 1, y, h, color);
}

void ui_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color)
{
    if (w == 0 || h == 0) return;
    esp_err_t ret = tft_set_window(x, y, x + w - 1, y + h - 1);
    if (ret != ESP_OK) return;
    uint8_t line_buf[UI_SCREEN_W * 2];
    for (uint8_t i = 0; i < w; i++) {
        line_buf[i * 2]     = color >> 8;
        line_buf[i * 2 + 1] = color & 0xFF;
    }
    for (uint8_t i = 0; i < h; i++) {
        tft_write_data(line_buf, w * 2);
    }
}

/**
 * @brief 画一个 6x8 右箭头 "->" (3 列像素宽, 第 6 列空白)
 *        形状: 第1列顶部 1 点, 第2列中间 1 点, 第3列底部 1 点
 *        简化方案: 用 '>' 字符 (0x3E), 直接走 ui_draw_char
 */
void ui_draw_arrow_right(uint8_t x, uint8_t y, uint16_t color, uint16_t bg)
{
    ui_draw_char(x, y, '>', color, bg);
}

/* ========== UI 调度 API ========== */

esp_err_t lcd_ui_register_page(const ui_page_t *page)
{
    if (page == NULL || page->id >= UI_PAGE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (page_count >= MAX_PAGES) {
        ESP_LOGE(TAG, "页面注册数量超过上限 %d", MAX_PAGES);
        return ESP_ERR_NO_MEM;
    }
    pages[page_count++] = page;
    ESP_LOGI(TAG, "注册页面: %d (%s)", page->id, page->title ? page->title : "?");
    return ESP_OK;
}

esp_err_t lcd_ui_switch_to(ui_page_id_t id)
{
    if (id >= UI_PAGE_MAX) return ESP_ERR_INVALID_ARG;

    // 找到目标页面
    const ui_page_t *target = NULL;
    for (uint8_t i = 0; i < page_count; i++) {
        if (pages[i]->id == id) {
            target = pages[i];
            break;
        }
    }
    if (target == NULL) {
        ESP_LOGE(TAG, "目标页面 %d 未注册", id);
        return ESP_ERR_NOT_FOUND;
    }

    // 退出旧页
    for (uint8_t i = 0; i < page_count; i++) {
        if (pages[i]->id == current_page && pages[i]->on_exit) {
            pages[i]->on_exit();
        }
    }

    current_page = id;

    // 清屏 + 进入新页
    tft_clear(TFT_COLOR_BLACK);
    if (target->on_enter) {
        target->on_enter();
    }

    ESP_LOGI(TAG, "切换到页面: %d (%s)", target->id, target->title ? target->title : "?");
    return ESP_OK;
}

ui_page_id_t lcd_ui_get_current_page(void)
{
    return current_page;
}

esp_err_t lcd_ui_init(ui_key_get_t key_get)
{
    if (key_get == NULL) return ESP_ERR_INVALID_ARG;
    key_get_cb = key_get;

    current_page = UI_PAGE_HOME;

    // 上电显示主页 (清屏 + 触发 on_enter)
    tft_clear(TFT_COLOR_BLACK);
    for (uint8_t i = 0; i < page_count; i++) {
        if (pages[i]->id == UI_PAGE_HOME && pages[i]->on_enter) {
            pages[i]->on_enter();
            break;
        }
    }

    ESP_LOGI(TAG, "UI 系统初始化成功 (注册页面数: %d)", page_count);
    return ESP_OK;
}

/**
 * @brief 将 pcf8574 按键转换为 UI 按键
 *        K1 -> UI_KEY_K1
 *        K2 -> UI_KEY_K2
 *        K3 -> UI_KEY_K3
 *        K4 -> UI_KEY_K1 (长按/备用, 此版本暂忽略)
 * @note  当前版本未使用 (按键映射直接在 main.c 的 key_get_from_pcf8574 中实现),
 *        保留供后续切换到回调方式时使用
 */
__attribute__((unused))
static ui_key_t map_button_to_key(uint8_t btn_state)
{
    if (btn_state & (1 << BTN_K1)) return UI_KEY_K1;
    if (btn_state & (1 << BTN_K2)) return UI_KEY_K2;
    if (btn_state & (1 << BTN_K3)) return UI_KEY_K3;
    return UI_KEY_NONE;
}

void lcd_ui_loop(void)
{
    if (key_get_cb == NULL) return;

    // 1) 读取按键并派发
    ui_key_t key = key_get_cb();
    if (key != UI_KEY_NONE) {
        // 找到当前页面并派发
        for (uint8_t i = 0; i < page_count; i++) {
            if (pages[i]->id == current_page && pages[i]->on_key) {
                pages[i]->on_key(key);
                break;
            }
        }
    }

    // 2) 周期性刷新当前页面 (实时数据)
    tick_counter++;
    if (tick_counter >= REFRESH_TICKS) {
        tick_counter = 0;
        for (uint8_t i = 0; i < page_count; i++) {
            if (pages[i]->id == current_page && pages[i]->on_refresh) {
                pages[i]->on_refresh();
                break;
            }
        }
    }
}