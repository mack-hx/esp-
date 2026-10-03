/**
 * @file pcf8574.h
 * @brief PCF8574 I2C IO 扩展芯片驱动 (按键输入)
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef PCF8574_H
#define PCF8574_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
#define PCF8574_SCL_GPIO    17      // I2C 时钟线
#define PCF8574_SDA_GPIO    18      // I2C 数据线
#define PCF8574_INT_GPIO    21      // 中断引脚 (低电平有效)

/* ========== I2C 配置 ========== */
#define PCF8574_I2C_NUM     I2C_NUM_1
#define PCF8574_I2C_FREQ    400000  // 400kHz
#define PCF8574_I2C_ADDR    0x27    // 默认地址 (扫描确认 A0=A1=A2=1 → 0x27, 原配置 0x20)

/* ========== PCF8574 引脚定义 ========== */
typedef enum {
    PCF8574_P0 = 0,   // K1 按键1
    PCF8574_P1 = 1,   // K2 按键2
    PCF8574_P2 = 2,   // K3 按键3
    PCF8574_P3 = 3,   // K4 按键4
    PCF8574_P4 = 4,   // K6 微动开关2
    PCF8574_P5 = 5,   // K5 微动开关1
    PCF8574_P6 = 6,   // 保留
    PCF8574_P7 = 7    // 保留
} pcf8574_pin_t;

/* ========== 按键/开关 ID 定义 ========== */
typedef enum {
    BTN_K1 = 0,   // 按键1
    BTN_K2 = 1,   // 按键2
    BTN_K3 = 2,   // 按键3
    BTN_K4 = 3,   // 按键4
    SW_K5  = 4,   // 微动开关1
    SW_K6  = 5    // 微动开关2
} btn_id_t;

/* ========== 按键事件类型 ========== */
typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_PRESS,    // 按下
    BTN_EVENT_RELEASE   // 释放
} btn_event_t;

/* ========== 按键事件回调 ========== */
typedef void (*pcf8574_btn_cb_t)(btn_id_t id, btn_event_t event);

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 PCF8574
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t pcf8574_init(void);

/**
 * @brief 读取所有引脚状态
 * @param state 状态字节指针 (bit0=P0, bit1=P1, ...)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t pcf8574_read_all(uint8_t *state);

/**
 * @brief 读取单个引脚状态
 * @param pin 引脚编号
 * @param state 状态指针 (true=高电平, false=低电平)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t pcf8574_read_pin(pcf8574_pin_t pin, bool *state);

/**
 * @brief 检测按键是否按下 (高电平有效)
 * @param id 按键/开关 ID
 * @param pressed 状态指针 (true=按下)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t pcf8574_is_pressed(btn_id_t id, bool *pressed);

/**
 * @brief 扫描按键状态 (含多采样消抖 + 按下/释放状态机)
 * @param[out] pressed_mask 当前稳定按下的按键位图 (bit=1 表示按下, 全松开=0xFF)
 * @return ESP_OK 成功, 其他值失败
 *
 * @note  推荐调用周期 SCAN_INTERVAL_MS (8ms). 函数内部维护状态机,
 *        相同采样连续 DEBOUNCE_SAMPLES 次才视为稳定 (总消抖时间 ≈ 40ms).
 *        一次完整的按下-释放周期才会触发一次 BTN_EVENT_PRESS / BTN_EVENT_RELEASE 回调.
 */
esp_err_t pcf8574_scan_buttons(uint8_t *pressed_mask);

/**
 * @brief 注册按键事件回调
 * @param cb 回调函数指针
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t pcf8574_register_callback(pcf8574_btn_cb_t cb);

/**
 * @brief 启用中断 (GPIO 下降沿触发)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t pcf8574_enable_interrupt(void);

#endif // PCF8574_H