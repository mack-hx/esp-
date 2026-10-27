/**
 * @file ui_led_magnet.h
 * @brief LED、电磁铁、微动开关 共用检查/控制页
 * @note  一页里同时管灯和电磁铁:
 *        - 灯: 颜色预设 + 亮度, 屏上用色块实时显示当前真实颜色
 *        - 电磁铁: 两个电磁铁一起开关 (跟 App 的 magnet_on 语义一致)
 *        - 微动开关: 显示对接开关 SW1 / SW2 的压下状态
 *        - 显示的灯状态直接读 LED 驱动缓存, App 远程改过灯也能立刻反映
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
