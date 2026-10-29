/**
 * @file steer_pid.h
 * @brief 转向环 (差速环): 左右轮速差 → 两轮输出差 (%)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#ifndef STEER_PID_H
#define STEER_PID_H

/* ========== 转向环参数 (%/rpm) ========== */
#define STEER_PID_KP         0.5f    /* 差速输出 (%/rpm) */
#define STEER_PID_KI         0.05f   /* 差速输出 (%/(rpm·s)) */
#define STEER_PID_OUT_MAX    20.0f   /* 差速输出上限 (%) */

/**
 * @brief 初始化转向环 (装载固定增益 + 清积分)
 */
void steer_pid_init(void);

/**
 * @brief 重新装载转向环增益 (走 param 表, 便于将来开放调参)
 */
void steer_pid_load_params(void);

/**
 * @brief 清空转向环的积分与微分历史
 */
void steer_pid_reset(void);

/**
 * @brief 执行一次转向环运算
 * @param dt     控制周期 (秒)
 * @param diff   实测 左−右 轮速差 (rpm, 前进为正, 左轮更快为正)
 * @param target 目标 左−右 轮速差 (rpm, 前进为正, 左轮更快为正)
 * @return 差速输出 (%), 正 = 左轮更快
 */
float steer_pid_update(float dt, float diff, float target);

#endif // STEER_PID_H
