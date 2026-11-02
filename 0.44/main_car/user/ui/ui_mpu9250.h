/**
 * @file ui_mpu9250.h
 * @brief UI MPU9250 IMU 调试页
 * @note  - 实时显示加速度 / 陀螺仪 / 温度
 *        - 显示互补滤波后的欧拉角 (Roll/Pitch/Yaw)
 * @author ESP32 机械狗项目
 */

#ifndef UI_MPU9250_H
#define UI_MPU9250_H

#include "lcd_ui.h"

esp_err_t ui_mpu9250_register(void);

#endif // UI_MPU9250_H