/**
 * @file ui_encoder.h
 * @brief UI 编码电机调试页
 * @note  - 实时显示 ENCODER_1/ENCODER_2 计数值与转速 (RPM, 各自一行)
 *        - 5 种模式 (K3 切换):
 *            M1   : 只转电机 A    K2 = 启动/关闭
 *            M2   : 只转电机 B    K2 = 启动/关闭
 *            A+B  : 两电机同转    K2 = 启动/关闭
 *            DIR  : 切换方向      K2 = FORWARD / REVERSE 切换 (设定, 不转)
 *            SPD  : 调速档位      K2 = 20/40/60/80 循环 (设定, 不转)
 *        - DIR 与 SPD 是"设定": 切到 DIR/SPD 时强制停车并清运行标志,
 *          状态行与实际硬件始终同步; 切回 M1/M2/A+B 后按 K2 才以当前方向和档位实际转动
 *        - 实际转动速度 = SPEED_LEVELS[s_speed_idx] (与 SPD 模式同步)
 *        - K1 退出回菜单 (自动停车)
 * @author ESP32 机械狗项目
 */

#ifndef UI_ENCODER_H
#define UI_ENCODER_H

#include "lcd_ui.h"

esp_err_t ui_encoder_register(void);

#endif // UI_ENCODER_H