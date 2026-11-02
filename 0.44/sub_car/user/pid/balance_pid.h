/**
 * @file balance_pid.h
 * @brief 平衡环 (内环): 姿态误差 → 电机输出 (%)
 * @author ESP32 平衡轮腿机器人
 */

#ifndef BALANCE_PID_H
#define BALANCE_PID_H

/**
 * @brief 初始化平衡环 (从 param 表读增益 + 清积分)
 * @note  上电初始化、参数重载时调用
 */
void balance_pid_init(void);

/**
 * @brief 从 param 表重新读取平衡环增益
 */
void balance_pid_load_params(void);

/**
 * @brief 清空平衡环的积分与微分历史
 */
void balance_pid_reset(void);

/**
 * @brief 执行一次平衡环运算
 */
float balance_pid_update(float dt, float pitch, float target_pitch, float gyro);

#endif // BALANCE_PID_H
