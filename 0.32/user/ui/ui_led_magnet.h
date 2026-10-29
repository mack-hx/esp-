/**
 * @file ui_led_magnet.h
 * @brief LED、电磁铁、微动开关 共用检查/控制页
 * @author ESP32 机械狗项目
 * @date 2026-10-25
 */

#ifndef UI_LED_MAGNET_H
#define UI_LED_MAGNET_H

#include "esp_err.h"

/**
 * @brief 注册 LED & 电磁铁 控制页
 * @return ESP_OK 成功
 */
esp_err_t ui_led_magnet_register(void);

#endif // UI_LED_MAGNET_H
