/**
 * @file magnet.h
 * @brief 电磁铁控制驱动
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef MAGNET_H
#define MAGNET_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
// 电磁铁 1 -> 74HC595 Q5 (IO1)
// 电磁铁 2 -> 74HC595 Q6 (IO2)
// 实际驱动电流较大, 外部建议使用三极管/MOS 管扩流

/* ========== 电磁铁 ID 定义 ========== */
typedef enum {
    MAGNET_1 = 0,    // 电磁铁1
    MAGNET_2 = 1     // 电磁铁2
} magnet_id_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化电磁铁模块
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t magnet_init(void);

/**
 * @brief 控制单个电磁铁开关
 * @param id 电磁铁 ID
 * @param on true=开启 (通电吸合), false=关闭 (断电释放)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t magnet_set(magnet_id_t id, bool on);

/**
 * @brief 同时控制两个电磁铁
 * @param on1 电磁铁1 状态
 * @param on2 电磁铁2 状态
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t magnet_set_both(bool on1, bool on2);

/**
 * @brief 获取电磁铁当前状态
 * @param id 电磁铁 ID
 * @return true=开启, false=关闭
 */
bool magnet_get(magnet_id_t id);

/**
 * @brief 关闭所有电磁铁
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t magnet_release_all(void);

#endif // MAGNET_H