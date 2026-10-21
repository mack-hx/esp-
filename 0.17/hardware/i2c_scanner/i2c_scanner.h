/**
 * @file i2c_scanner.h
 * @brief I2C 总线地址扫描工具 (诊断用)
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include "esp_err.h"

/**
 * @brief 扫描 PCF8574 所在 I2C 总线 (I2C_NUM_1), 打印 0x03-0x77 范围
 *        内所有 ACK 的从机地址
 * @return ESP_OK
 */
esp_err_t i2c_scanner_pcf8574(void);

#endif // I2C_SCANNER_H