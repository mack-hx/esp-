/**
 * @file ui_encoder.h
 * @brief UI 编码电机调试页
 * @note  - 实时显示 ENCODER_1/ENCODER_2 计数值与转速 (RPM)
 *        - 可手动控制 MOTOR_A / MOTOR_B 速度
 *        - K2 切换"选中电机"
 *        - K3 当前电机速度 +10, K1 退出回菜单 (并停车)
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef UI_ENCODER_H
#define UI_ENCODER_H

#include "lcd_ui.h"

esp_err_t ui_encoder_register(void);

#endif // UI_ENCODER_H