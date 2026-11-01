/**
 * @file hc595.h
 * @brief 74HC595D 串行移位寄存器驱动
 * @note  Q0 保留为通用输出 (SK6812 LED 走 ESP32 GPIO9 RMT 直驱)
 * @author ESP32 机械狗项目
 */

#ifndef HC595_H
#define HC595_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 引脚配置 ==========
 * 硬件接线: GPIO4 -> STCP (锁存时钟), GPIO5 -> SHCP (移位时钟), GPIO6 -> DS (数据)
 */
#define HC595_SCLK_GPIO     5       // 串行移位时钟 (SHCP) - ESP32 GPIO 5
#define HC595_RCLK_GPIO     4       // 锁存时钟 (STCP)       - ESP32 GPIO 4
#define HC595_DS_GPIO       6       // 串行数据 (DS)         - ESP32 GPIO 6

/* ========== 输出引脚定义 ==========
 * 实际硬件映射: Q1 电磁铁2, Q2 电磁铁1, Q3/Q4 电机2 正/反转,
 *               Q5/Q6 电机1 正/反转, Q0/Q7 保留。
 * 此处 Q0~Q7 仅为物理引脚编号, 具体外设映射见 motor_driver.c / magnet.c
 */
typedef enum {
    HC595_Q0 = 0,   // 通用输出 (保留)
    HC595_Q1 = 1,   // 通用输出 (电磁铁 2)
    HC595_Q2 = 2,   // 通用输出 (电磁铁 1)
    HC595_Q3 = 3,   // 通用输出 (电机 2 正转)
    HC595_Q4 = 4,   // 通用输出 (电机 2 反转)
    HC595_Q5 = 5,   // 通用输出 (电机 1 正转)
    HC595_Q6 = 6,   // 通用输出 (电机 1 反转)
    HC595_Q7 = 7    // 通用输出 (保留)
} hc595_pin_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 74HC595
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_init(void);

/**
 * @brief 写入 8 位数据到 74HC595
 * @param data 8 位数据
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_write_byte(uint8_t data);

/**
 * @brief 设置单个输出引脚
 * @param pin 引脚编号 (0-7)
 * @param state 引脚状态 (true=高电平, false=低电平)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_set_pin(hc595_pin_t pin, bool state);
/**
 * @brief 清除所有输出 (设置为 0)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_clear_all(void);

/**
 * @brief 设置电磁铁 1 输出 (封装 hc595_set_pin Q2)
 * @param state true=高电平开启, false=低电平关闭
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_set_magnet1(bool state);

/**
 * @brief 设置电磁铁 2 输出 (封装 hc595_set_pin Q1)
 * @param state true=高电平开启, false=低电平关闭
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_set_magnet2(bool state);

#endif // HC595_H
