/**
 * @file tft_display.h
 * @brief 128x160 RGB TFT 显示屏驱动 (ST7735/ST7789)
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
#define TFT_SCL_GPIO      7       // SPI 时钟线
#define TFT_SDA_GPIO      8       // SPI 数据线 (MOSI)
// 原 TFT_RST_GPIO (GPIO9) 已释放给 SK6812 LED, RST 引脚已短接 VCC
// 由 TFT 模块内部 POR (Power-On-Reset) 完成上电自复位
#define TFT_DC_GPIO       10      // 数据/命令选择
#define TFT_CS_GPIO       11      // 片选信号
#define TFT_BLK_GPIO      12      // 背光控制 (PWM)

/* ========== 显示屏参数 ========== */
#define TFT_WIDTH         128     // 屏幕宽度 (像素)
#define TFT_HEIGHT        160     // 屏幕高度 (像素)
#define TFT_SPI_FREQ      20000000 // SPI 时钟频率 (20MHz, 保守值匹配 ST7735S)

/* ========== 屏幕方向 (与参考工程一致) ==========
 * 0/1: 竖屏 128×160
 * 2/3: 横屏 160×128
 * 当前固定 USE_HORIZONTAL=1, 配合 LCD 实际安装方向 */
#define USE_HORIZONTAL    1
#if (USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1)
    #define TFT_SCREEN_W  128
    #define TFT_SCREEN_H  160
#else
    #define TFT_SCREEN_W  160
    #define TFT_SCREEN_H  128
#endif

/* ========== 显示窗口偏移 (适配不同模组) ==========
 * 若屏幕显示偏移/有黑边, 调整这两个值 */
#define TFT_X_OFFSET  0
#define TFT_Y_OFFSET  0

/* ========== 颜色定义 (RGB565) ========== */
#define TFT_COLOR_BLACK   0x0000
#define TFT_COLOR_WHITE   0xFFFF
#define TFT_COLOR_RED     0xF800
#define TFT_COLOR_GREEN   0x07E0
#define TFT_COLOR_BLUE    0x001F
#define TFT_COLOR_YELLOW  0xFFE0
#define TFT_COLOR_CYAN    0x07FF
#define TFT_COLOR_MAGENTA 0xF81F

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 TFT 显示屏
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t tft_display_init(void);

/**
 * @brief 设置背光亮度
 * @param brightness 亮度 (0-100, 百分比)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t tft_set_backlight(uint8_t brightness);

/**
 * @brief 开启/关闭背光
 * @param on true=开启, false=关闭
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t tft_backlight_on(bool on);

/**
 * @brief 清屏 (填充指定颜色)
 * @param color 颜色 (RGB565)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t tft_clear(uint16_t color);

/**
 * @brief 在指定位置绘制一个像素
 * @param x X 坐标
 * @param y Y 坐标
 * @param color 颜色 (RGB565)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t tft_draw_pixel(uint8_t x, uint8_t y, uint16_t color);

/**
 * @brief 显示字符串
 * @param x X 坐标
 * @param y Y 坐标
 * @param str 要显示的字符串
 * @param color 文字颜色
 * @param bg_color 背景颜色
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t tft_draw_string(uint8_t x, uint8_t y, const char *str,
                          uint16_t color, uint16_t bg_color);

/**
 * @brief 设置显示窗口
 * @param x0 起始 X 坐标
 * @param y0 起始 Y 坐标
 * @param x1 结束 X 坐标
 * @param y1 结束 Y 坐标
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t tft_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

/**
 * @brief 写入显示数据
 * @param data 数据指针
 * @param len 数据长度
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t tft_write_data(const uint8_t *data, uint32_t len);

#endif // TFT_DISPLAY_H