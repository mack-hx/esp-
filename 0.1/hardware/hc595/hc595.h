/**
 * @file hc595.h
 * @brief 74HC595D 串行移位寄存器驱动
 * @note  硬件改动 (2026-10-01):
 *        - Q0 不再用于 SK6812 LED (已迁移至 ESP32 GPIO9 RMT 直驱)
 *        - Q0 保留为通用输出, 上层可自由使用
 *        - 移除 hc595_set_led() 接口
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef HC595_H
#define HC595_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
#define HC595_SCLK_GPIO     4       // 串行时钟 (Shift Clock)
#define HC595_RCLK_GPIO     5       // 锁存时钟 (Latch Clock)
#define HC595_DS_GPIO       6       // 串行数据 (Data Serial)

/* ========== 输出引脚定义 ========== */
typedef enum {
    HC595_Q0 = 0,   // 通用输出 (SK6812 LED 已迁移至 GPIO9)
    HC595_Q1 = 1,   // 通用输出 (TB6612 AIN1)
    HC595_Q2 = 2,   // 通用输出 (TB6612 AIN2)
    HC595_Q3 = 3,   // 通用输出 (TB6612 BIN1)
    HC595_Q4 = 4,   // 通用输出 (TB6612 BIN2)
    HC595_Q5 = 5,   // 通用输出 (电磁铁1)
    HC595_Q6 = 6,   // 通用输出 (电磁铁2)
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
 * @brief 获取当前输出状态
 * @return 当前 8 位输出状态
 */
uint8_t hc595_get_state(void);

/**
 * @brief 清除所有输出 (设置为 0)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_clear_all(void);

/**
 * @brief 设置电磁铁 1 输出 (封装 hc595_set_pin Q5)
 * @param state true=高电平开启, false=低电平关闭
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_set_magnet1(bool state);

/**
 * @brief 设置电磁铁 2 输出 (封装 hc595_set_pin Q6)
 * @param state true=高电平开启, false=低电平关闭
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_set_magnet2(bool state);

#endif // HC595_H
