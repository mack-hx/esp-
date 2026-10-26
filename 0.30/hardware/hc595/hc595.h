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

/* ========== 引脚配置 ==========
 * 硬件接线 (与图标注一致):
 *   ESP32 GPIO 4 (SCLK)  -> 74HC595 STCP  (锁存时钟, Register Clock)
 *   ESP32 GPIO 5 (RCLK)  -> 74HC595 SHCP  (移位时钟, Shift Clock)
 *   ESP32 GPIO 6 (DS)    -> 74HC595 DS    (串行数据, Data Serial)
 * 因此:
 *   - HC595_SCLK_GPIO 实际驱动 SHCP (移位时钟) —— 使用 GPIO 5
 *   - HC595_RCLK_GPIO 实际驱动 STCP (锁存时钟) —— 使用 GPIO 4
 *   - HC595_DS_GPIO   驱动 DS (数据)         —— 使用 GPIO 6
 */
#define HC595_SCLK_GPIO     5       // 串行移位时钟 (SHCP) - ESP32 GPIO 5
#define HC595_RCLK_GPIO     4       // 锁存时钟 (STCP)       - ESP32 GPIO 4
#define HC595_DS_GPIO       6       // 串行数据 (DS)         - ESP32 GPIO 6

/* ========== 输出引脚定义 ==========
 * 实际硬件映射 (2026-10-03 逐位测试确认, 详见 logs/2026-10-03.md):
 *   bit  ->  Qx  ->  实际外设
 *   0x01 (Q0)  ->  保留
 *   0x02 (Q1)  ->  电磁铁 2
 *   0x04 (Q2)  ->  电磁铁 1
 *   0x08 (Q3)  ->  电机 2 正转
 *   0x10 (Q4)  ->  电机 2 反转
 *   0x20 (Q5)  ->  电机 1 正转
 *   0x40 (Q6)  ->  电机 1 反转
 *   0x80 (Q7)  ->  保留
 * 注: 此处 Q0~Q7 仅为物理引脚编号, 不再与具体外设强绑定
 *      (具体外设映射见 motor_driver.c / magnet.c)
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
