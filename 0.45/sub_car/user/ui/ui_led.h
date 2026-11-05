/**
 * @file ui_led.h
 * @brief LED 检查/控制页 (颜色预设 + 亮度, 屏上色块显示当前真实颜色)
 * @author ESP32 机械狗项目
 */

#ifndef UI_LED_H
#define UI_LED_H

#include "esp_err.h"

/**
 * @brief 注册 LED 控制页
 * @return ESP_OK 成功
 */
esp_err_t ui_led_register(void);

#endif // UI_LED_H
