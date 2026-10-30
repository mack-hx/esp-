/**
 * @file position_pid.h
 * @brief 位置保持环 (外环): 位置误差 (cm) + 相对速度 (cm/s) → 速度修正量 (cm/s)
 *
 * @note  串级结构: 本环输出叠加到速度环的目标轮速上, 速度环始终运行。
 * @note  位置误差 = 锚点位置 − 里程计位置; 锚点随指令速度积分, 指令结束后冻结。
 * @note  反馈用里程计速度 (odom, 60 ms 窗口), 不是编码器轮速 (300 ms 窗口)。
 *
 * @author ESP32 平衡轮腿机器人
 */

#ifndef POSITION_PID_H
#define POSITION_PID_H

/* 位置误差死区 (cm): 偏差小于这个距离当作已经回到位 */
#define POS_PID_ERR_DEADBAND  0.1f

/* 死区软化过渡带宽度 (cm): 死区边缘用二次曲线平滑起步 */
#define POS_PID_ERR_SOFT_BAND 0.2f

/* 速度修正量上限 (cm/s) */
#define POS_PID_OUT_MAX       20.0f

/* 积分状态上限 (cm·s): ∫(位置误差)dt 的限幅 */
#define POS_PID_I_MAX         20.0f

/* 死区内的积分衰减时间常数 (秒) */
#define POS_PID_I_DECAY_TAU   3.0f

/**
 * @brief 初始化位置保持环 (从 param 表读增益 + 清积分)
 */
void position_pid_init(void);

/**
 * @brief 从 param 表重新读取位置保持环增益 (每个控制周期调一次)
 */
void position_pid_load_params(void);

/**
 * @brief 清空积分状态
 */
void position_pid_reset(void);

/**
 * @brief 执行一次位置保持环运算
 * @param dt       控制周期 (秒)
 * @param err_cm   位置误差 (cm): 正 = 锚点在前方 (需要往前走)
 * @param v_rel_cmps 相对速度 (cm/s) = 实测平移速度 − 指令速度 (前进为正)
 * @return 速度修正量 (cm/s, 前进为正), 限幅 ±POS_PID_OUT_MAX
 * @note  输出 = kp·误差 + ki·∫误差 − kd·相对速度;
 *        pos_kp 与 pos_kd 都为 0 时输出恒 0 (关掉位置保持)。
 */
float position_pid_update(float dt, float err_cm, float v_rel_cmps);

#endif // POSITION_PID_H
