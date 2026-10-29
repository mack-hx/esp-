/**
 * @file hold_pid.h
 * @brief 锁位速度环 (中环): 平移速度误差 (cm/s) → 目标倾角 (°)
 *
 * @note  反馈用里程计速度 (odom, 60 ms 窗口), 不是编码器轮速 (300 ms 窗口)。
 * @note  本环只在"站着不动"时运行 (balance.c 的 HOLD 模式);
 *        行走时由 speed_pid (轮速环) 给目标倾角。
 *
 * @author ESP32 平衡轮腿机器人
 */

#ifndef HOLD_PID_H
#define HOLD_PID_H

/* 锁位速度环内置默认增益 (°/(cm/s)): param 表里 hold_kp 为 0 时用它兜底 */
#define HOLD_PID_KP_DEFAULT  0.8f

/* 锁位速度环的速度死区 (cm/s): |误差| 小于此值时输出 0 */
#define HOLD_PID_V_DEADBAND  0.3f

/* 速度误差软化过渡带 (cm/s): 死区与全响应之间线性压缩, 保证连续 */
#define HOLD_PID_V_SOFT_BAND 0.3f

/* 锁位速度环的目标倾角上限 (°): 位置保持时车最多歪这么多 */
#define HOLD_PID_LEAN_MAX    3.0f

/* 积分状态上限 (cm): ∫(速度误差)dt 的限幅, 积分项贡献 = hold_ki × 积分值 */
#define HOLD_PID_I_MAX       5.0f

/* 目标倾角限速 (°/s) */
#define HOLD_PID_LEAN_SLEW   20.0f

/* 目标倾角的一阶低通时间常数 (秒) */
#define HOLD_PID_LEAN_LPF_TAU 0.05f

/**
 * @brief 初始化锁位速度环 (从 param 表读增益 + 清积分)
 * @note  上电初始化、参数重载时调用
 */
void hold_pid_init(void);

/**
 * @brief 从 param 表重新读取锁位速度环增益
 * @note  每个控制周期都调一次
 */
void hold_pid_load_params(void);

/**
 * @brief 清空积分与输出整形状态
 * @note  上电 / 摔倒恢复 / 开始行走 / 进入 HOLD 时调用
 */
void hold_pid_reset(void);

/**
 * @brief 执行一次锁位速度环运算
 * @param dt         控制周期 (秒)
 * @param v_cmps     实测平移速度 (cm/s, 前进为正): 里程计速度 (60 ms 窗口)
 * @param v_ref_cmps 目标平移速度 (cm/s, 前进为正): 位置环的输出
 * @return 目标倾角 (°, 正 = 车头往前倾)
 * @note  误差 = 目标速度 − 实测速度; 输出已按 HOLD_PID_LEAN_MAX 夹过,
 *        并做了限速 + 一阶低通。
 */
float hold_pid_update(float dt, float v_cmps, float v_ref_cmps);

#endif // HOLD_PID_H
