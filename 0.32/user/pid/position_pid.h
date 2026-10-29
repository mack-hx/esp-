/**
 * @file position_pid.h
 * @brief 位置环 (最外环): 位置误差 (cm) → 目标速度 (cm/s)
 *
 * @note  只输出"该走多快", 倾角由锁位速度环 (hold_pid.c) 给。
 * @note  本环只在"站住不动"时接管 (见 balance.c 的 pos_mode_t); 行走时由速度环给目标倾角。
 *
 * @author ESP32 平衡轮腿机器人
 */

#ifndef POSITION_PID_H
#define POSITION_PID_H

/* 位置误差死区 (cm): 小于这个距离当作没偏, 不要求移动 */
#define POS_PID_ERR_DEADBAND  0.3f

/* 死区软化过渡带 (cm): 死区与全响应之间线性压缩, 保证误差连续 */
#define POS_PID_ERR_SOFT_BAND 0.4f

/* 目标速度上限 (cm/s): 回位时最快要走多快 (安全阀) */
#define POS_PID_V_MAX         6.0f

/* 死区内的积分衰减时间常数 (秒): 积分按 exp(-dt/τ) 衰减到 0 */
#define POS_PID_I_DECAY_TAU   3.0f

/**
 * @brief 初始化位置环 (从 param 表读增益 + 清积分)
 * @note  上电初始化、参数重载时调用
 */
void position_pid_init(void);

/**
 * @brief 从 param 表重新读取位置环增益
 * @note  每个控制周期都调一次; 只改系数, 保留积分 (用 pid_set_gains)
 */
void position_pid_load_params(void);

/**
 * @brief 清空位置环的积分与微分历史
 * @note  上电 / 摔倒恢复 / 开始行走时调用
 */
void position_pid_reset(void);

/**
 * @brief 执行一次位置环运算
 * @param dt  控制周期 (秒)
 * @param err 位置误差 (cm): 正 = 目标位置在前方 (需要往前走)
 * @return 目标速度 (cm/s, 前进为正): 车应该走多快
 * @note  误差小于 POS_PID_ERR_DEADBAND 时当作 0; 输出已按 POS_PID_V_MAX 夹过。
 */
float position_pid_update(float dt, float err);

#endif // POSITION_PID_H
