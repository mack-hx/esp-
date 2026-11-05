/**
 * @file status_led.h
 * @brief 整车状态灯: 按车体状态自动设置 SK6812 颜色
 * @note  优先级 (高 → 低):
 *          摔倒红 > 舵机转动粉 > 合并蓝 > 未对齐红 > 对接中闪黄 > 站立绿 > 未站立黄
 *        手动设色 (App / LCD LED 页) 后保持用户颜色, 直到下一次状态变化才恢复自动。
 */

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

/**
 * @brief 启动状态灯后台任务
 * @note  依赖 led / servo / magnet / pcf8574 / control 已初始化
 * @return ESP_OK 成功, ESP_FAIL 任务创建失败
 */
esp_err_t status_led_start(void);

/**
 * @brief 声明"用户刚手动设过灯色"
 * @note  App 或 LCD LED 页设色后调用: 之后保持该颜色, 直到自动状态切换才被接管
 */
void status_led_manual_hold(void);

#endif // STATUS_LED_H
