/**
 * @file ui_balance.h
 * @brief UI 平衡控制页
 * @note  - 显示控制状态机状态 + 串级各环中间量
 *        - K2 启停平衡控制, K3 急停 (清 PID 积分)
 *        - K1 退出回菜单
 * @author ESP32 机械狗项目
 */

#ifndef UI_BALANCE_H
#define UI_BALANCE_H

#include "lcd_ui.h"

/**
 * @brief 注册平衡控制页
 * @return ESP_OK 成功
 */
esp_err_t ui_balance_register(void);

#endif // UI_BALANCE_H
