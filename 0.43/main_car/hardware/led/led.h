/**
 * @file led.h
 * @brief SK6812 RGB LED 控制驱动 (ESP32 RMT 直驱版)
 * @note  SK6812 DIN 接 ESP32 GPIO9, 用 ESP32-S3 RMT 外设生成精确的
 *        SK6812 时序, 支持真实调色调光。
 *        TFT RST 短接 VCC, GPIO9 留给 SK6812
 *
 * @author ESP32 机械狗项目
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
// SK6812 DIN 直接连接到 ESP32 GPIO9 (由 RMT 外设驱动)
#define LED_DATA_GPIO        9

// RMT 配置
#define LED_RMT_RESOLUTION_HZ   (10 * 1000 * 1000)  // 10MHz, 100ns 精度
#define LED_RMT_TRANS_QUEUE_SIZE 4

/* ========== RGB 颜色结构 ========== */
typedef struct {
    uint8_t r;    // 红色分量 (0-255)
    uint8_t g;    // 绿色分量 (0-255)
    uint8_t b;    // 蓝色分量 (0-255)
} led_color_t;

/* ========== 预定义颜色 ========== */
#define LED_COLOR_OFF     ((led_color_t){0, 0, 0})
#define LED_COLOR_RED     ((led_color_t){255, 0, 0})
#define LED_COLOR_GREEN   ((led_color_t){0, 255, 0})
#define LED_COLOR_BLUE    ((led_color_t){0, 0, 255})
#define LED_COLOR_YELLOW  ((led_color_t){255, 255, 0})
#define LED_COLOR_CYAN    ((led_color_t){0, 255, 255})
#define LED_COLOR_MAGENTA ((led_color_t){255, 0, 255})
#define LED_COLOR_WHITE   ((led_color_t){255, 255, 255})

/* ========== 预设颜色枚举 ========== */
typedef enum {
    LED_PRESET_OFF = 0,
    LED_PRESET_RED,
    LED_PRESET_GREEN,
    LED_PRESET_BLUE,
    LED_PRESET_YELLOW,
    LED_PRESET_CYAN,
    LED_PRESET_MAGENTA,
    LED_PRESET_WHITE,
    LED_PRESET_MAX
} led_preset_t;

/* ========== LED ID (预留扩展为多颗灯) ========== */
typedef enum {
    LED_1 = 0,
    LED_NUM_MAX
} led_id_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 LED 模块 (创建 RMT 通道和编码器)
 * @note  调用后 LED 默认为关 (黑)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t led_init(void);
/**
 * @brief 开机自检: 红 → 绿 → 蓝 → 灭, 每档 300ms
 * @note  用来肉眼确认 "SK6812 到底有没有接到 GPIO9" —— 灯不亮时,
 *        先看这三个颜色有没有依次出现, 再去查 App / 云端。
 */
void led_selftest(void);

/**
 * @brief 设置 LED 颜色 (任意 RGB, 真实驱动)
 * @param id LED ID
 * @param color 颜色 (任意 RGB)
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG id 越界
 */
esp_err_t led_set_color(led_id_t id, led_color_t color);

/**
 * @brief 设置 LED 预设颜色 (枚举形式)
 * @param id LED ID
 * @param preset 预设颜色枚举
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG 参数越界
 */
esp_err_t led_set_preset(led_id_t id, led_preset_t preset);

/**
 * @brief 设置 LED 亮度 (0~255, 直接输出单色)
 * @note  纯亮度模式: 仅亮度参数有效, 颜色取当前缓存颜色的主导色
 * @param id LED ID
 * @param level 亮度 (0~255)
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG id 越界
 */
esp_err_t led_set_brightness(led_id_t id, uint8_t level);

/**
 * @brief 设置 LED 颜色 + 亮度 (推荐使用)
 * @param id LED ID
 * @param color 颜色 (RGB)
 * @param brightness 亮度 (0~255, 255 = 全亮)
 * @return ESP_OK 成功
 */
esp_err_t led_set_color_brightness(led_id_t id, led_color_t color, uint8_t brightness);

/**
 * @brief 开关 LED (直接 on/off)
 * @param id LED ID
 * @param on true=点亮 (恢复上次颜色), false=熄灭 (黑)
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG id 越界
 */
esp_err_t led_set_on(led_id_t id, bool on);

/**
 * @brief 获取 LED 当前开关状态
 * @param id LED ID
 * @return true=点亮, false=熄灭
 */
bool led_get_on(led_id_t id);

/**
 * @brief 获取 LED 当前颜色 (缓存值)
 * @param id LED ID
 * @return 颜色结构体
 */
led_color_t led_get_color(led_id_t id);

/**
 * @brief 获取 LED 当前亮度
 * @param id LED ID
 * @return 亮度 (0~255)
 */
uint8_t led_get_brightness(led_id_t id);
#endif // LED_H