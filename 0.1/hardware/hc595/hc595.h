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
    HC595_Q1 = 1,   // TB6612 AIN1 (电机A方向1)
    HC595_Q2 = 2,   // TB6612 AIN2 (电机A方向2)
    HC595_Q3 = 3,   // TB6612 BIN1 (电机B方向1)
    HC595_Q4 = 4,   // TB6612 BIN2 (电机B方向2)
    HC595_Q5 = 5,   // 电磁铁1 IO1
    HC595_Q6 = 6,   // 电磁铁2 IO2
    HC595_Q7 = 7    // 保留
} hc595_pin_t;

/* ========== 电机方向定义 ========== */
typedef enum {
    MOTOR_A = 0,    // 电机 A
    MOTOR_B = 1     // 电机 B
} motor_id_t;

typedef enum {
    MOTOR_DIR_BRAKE = 0,    // 短路制动
    MOTOR_DIR_FORWARD = 1,  // 正转
    MOTOR_DIR_REVERSE = 2   // 反转
} motor_dir_t;

/* ========== 电磁铁定义 ========== */
typedef enum {
    MAGNET_1 = 0,   // 电磁铁1
    MAGNET_2 = 1    // 电磁铁2
} magnet_id_t;

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
 * @brief 设置电机方向
 * @param motor 电机 ID (MOTOR_A 或 MOTOR_B)
 * @param dir 电机方向
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_set_motor_dir(motor_id_t motor, motor_dir_t dir);

/**
 * @brief 控制电磁铁
 * @param magnet 电磁铁 ID
 * @param state true=开启, false=关闭
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t hc595_set_magnet(magnet_id_t magnet, bool state);

#endif // HC595_H
