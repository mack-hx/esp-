/**
 * @file lcd_ui.h
 * @brief LCD UI 系统主调度接口 (主页 / 菜单页 / 功能页面)
 * @note  UI 架构:
 *        - 上电默认显示主页
 *        - K1 按下: 进入菜单页
 *        - 菜单页中, K2/K3 切换选中条目, K1 进入当前选中功能页面
 *        - 功能页面中, K1 退出回到菜单页
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef LCD_UI_H
#define LCD_UI_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "pcf8574.h"
#include "tft_display.h"

/* ========== 屏幕尺寸快捷宏 (与 tft_display 一致) ========== */
#define UI_SCREEN_W       128
#define UI_SCREEN_H       160

/* ========== 字体尺寸 (与 font6x8 一致) ========== */
#define UI_FONT_W         6
#define UI_FONT_H         8
/* 每行字符数 (128 / 6 ≈ 21) */
#define UI_CHARS_PER_ROW  21
/* 每屏字符行数 (160 / 8 = 20) */
#define UI_ROWS_PER_SCR   20

/* ========== UI 页面 ID ========== */
typedef enum {
    UI_PAGE_HOME = 0,            // 主页
    UI_PAGE_MENU,                // 菜单页 (选择入口)
    UI_PAGE_BALANCE,             // 平衡控制 (串级控制器状态 + 启停)
    UI_PAGE_SERVO,               // 舵机调试
    UI_PAGE_ENCODER,             // 编码电机调试
    UI_PAGE_MPU9250,             // MPU9250 参数调试
    UI_PAGE_NETSTAT,             // 网络状态 (WiFi + OneNet)
    UI_PAGE_MAX                  // 哨兵
} ui_page_id_t;

/* ========== 按键事件 (UI 层定义, 与 pcf8574 解耦) ========== */
typedef enum {
    UI_KEY_NONE = 0,
    UI_KEY_K1,                   // 进入 / 确认 / 退出当前页
    UI_KEY_K2,                   // 上 / 左 / 减
    UI_KEY_K3,                   // 下 / 右 / 加
    UI_KEY_K4,                   // 减 (备用, 例如舵机调试页 K4 = -5°)
} ui_key_t;

/* ========== 按键事件回调 (主任务循环调用) ========== */
/**
 * @brief 检测并获取按键事件 (UI 层)
 * @return 当前按下的按键 (无键则返回 UI_KEY_NONE)
 */
typedef ui_key_t (*ui_key_get_t)(void);

/* ========== UI 页面接口 (每个 .c 文件实现) ========== */
/**
 * @brief 页面"进入"回调: 初始化状态, 重置显示
 */
typedef void (*ui_page_enter_t)(void);

/**
 * @brief 页面"退出"回调: 停止运动, 释放资源 (例如舵机停转)
 */
typedef void (*ui_page_exit_t)(void);

/**
 * @brief 页面"按键"回调: 处理 K1/K2/K3
 * @param key 按键
 */
typedef void (*ui_page_key_t)(ui_key_t key);

/**
 * @brief 页面"刷新"回调: 周期性刷新显示 (例如实时传感器数据)
 *        内部建议 100-200ms 调用一次, 由主循环触发
 */
typedef void (*ui_page_refresh_t)(void);

/* ========== 页面注册结构 ========== */
typedef struct {
    ui_page_id_t    id;
    const char     *title;       // 菜单中显示的中文/英文标题 (用 ASCII 写)
    ui_page_enter_t     on_enter;
    ui_page_exit_t      on_exit;
    ui_page_key_t       on_key;
    ui_page_refresh_t   on_refresh;
} ui_page_t;

/* ========== UI 主调度 API ========== */

/**
 * @brief 初始化 UI 系统 (需在 tft_display_init 之后调用)
 * @param key_get 按键获取回调 (通常为 pcf8574 包装)
 * @return ESP_OK 成功
 */
esp_err_t lcd_ui_init(ui_key_get_t key_get);

/**
 * @brief 注册页面 (供各页面模块在初始化时调用)
 * @param page 页面描述指针 (静态生命周期)
 * @return ESP_OK 成功, ESP_ERR_NO_MEM 超过最大页面数
 */
esp_err_t lcd_ui_register_page(const ui_page_t *page);

/**
 * @brief 切换到指定页面 (内部会调用旧页面 on_exit + 新页面 on_enter)
 * @param id 目标页面 ID
 * @return ESP_OK 成功
 */
esp_err_t lcd_ui_switch_to(ui_page_id_t id);

/**
 * @brief UI 主循环 (在 FreeRTOS 任务中循环调用)
 *        内部负责: 按键读取 + 派发 + 页面刷新
 *        建议调用周期 50-100ms (按键检测) + 内部计时刷新
 */
void lcd_ui_loop(void);

/* ========== UI 层给所有页面提供的工具 API ========== */

/**
 * @brief 画一个 6x8 ASCII 字符
 */
void ui_draw_char(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bg);

/**
 * @brief 画一串 ASCII 字符串 (遇 '\0x' 停止, 超出屏幕自动换行)
 */
void ui_draw_string(uint8_t x, uint8_t y, const char *str,
                    uint16_t color, uint16_t bg);

/**
 * @brief 画一条水平线
 */
void ui_draw_hline(uint8_t x, uint8_t y, uint8_t w, uint16_t color);

/**
 * @brief 画一条垂直线
 */
void ui_draw_vline(uint8_t x, uint8_t y, uint8_t h, uint16_t color);

/**
 * @brief 画矩形边框 (1 像素)
 */
void ui_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);

/**
 * @brief 填充矩形
 */
void ui_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);

/**
 * @brief 画箭头 (向右, 占位 6x8 区域)
 */
void ui_draw_arrow_right(uint8_t x, uint8_t y, uint16_t color, uint16_t bg);

/**
 * @brief 获取当前页面 ID
 */
ui_page_id_t lcd_ui_get_current_page(void);

#endif // LCD_UI_H