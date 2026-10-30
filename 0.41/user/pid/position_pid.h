/**
 * @file position_pid.h
 * @brief 位置保持环 (外环): 位置误差 (cm) + 平移速度 (cm/s) → 目标倾角 (°)
 *
 * @note  结构是"弹簧-阻尼": 位置偏差直接换算成回位力 (目标倾角),
 *        速度反馈当阻尼项, 推得越远回位力越大, 松手后自己回到原位。
 * @note  与速度环按指令速度加权混合 (见 balance.c): 指令速度为 0 时全由本环给目标倾角。
 * @note  反馈用里程计速度 (odom, 60 ms 窗口), 不是编码器轮速 (300 ms 窗口)。
 *
 * @author ESP32 平衡轮腿机器人
 */

#ifndef POSITION_PID_H
#define POSITION_PID_H

/* 位置误差死区 (cm): 偏差小于这个距离当作已经回到位 */
#define POS_PID_ERR_DEADBAND  0.1f

/* 死区软化过渡带宽度 (cm): 死区边缘用二次曲线平滑起步,
 * 带末端的有效误差 = 过渡带宽度, 与带外公式取值相同 → 输出处处连续 */
#define POS_PID_ERR_SOFT_BAND 0.2f

/* 目标倾角上限 (°): 位置偏差最大时车最多歪这么多 */
#define POS_PID_LEAN_MAX      5.0f

/* 积分状态上限 (cm·s): ∫(位置误差)dt 的限幅 */
#define POS_PID_I_MAX         20.0f

/* 死区内的积分衰减时间常数 (秒) */
#define POS_PID_I_DECAY_TAU   3.0f

/**
 * @brief 初始化位置保持环 (从 param 表读增益 + 清积分)
 * @note  上电初始化、参数重载时调用
 */
void position_pid_init(void);

/**
 * @brief 从 param 表重新读取位置保持环增益
 * @note  每个控制周期都调一次
 */
void position_pid_load_params(void);

/**
 * @brief 清空积分状态
 * @note  上电 / 摔倒恢复 / 开始行走时调用
 */
void position_pid_reset(void);

/**
 * @brief 执行一次位置保持环运算
 * @param dt      控制周期 (秒)
 * @param err_cm  位置误差 (cm): 正 = 目标位置在前方 (需要往前走)
 * @param v_cmps  实测平移速度 (cm/s, 前进为正): 里程计速度 (60 ms 窗口)
 * @return 目标倾角 (°, 正 = 车头往前倾)
 * @note  输出 = kp·位置误差 + ki·∫误差 − kd·速度 (前倾为正);
 *        误差小于 POS_PID_ERR_DEADBAND 时当作 0;
 *        输出经软饱和后不会超过 ±POS_PID_LEAN_MAX;
 *        pos_kp 与 pos_kd 都为 0 时本环输出恒 0 (关掉位置保持)。
 */
float position_pid_update(float dt, float err_cm, float v_cmps);

#endif // POSITION_PID_H
