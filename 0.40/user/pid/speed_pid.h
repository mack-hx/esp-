/**
 * @file speed_pid.h
 * @brief 速度环 (外环): 轮速误差 → 目标倾角 (°)
 *
 * @note  轮速约定: 全部用"前进为正"的轮速 (rpm)。
 *        前进 = 车头方向 (车头往下、pitch 为负 的那个方向)。
 *
 * @author ESP32 平衡轮腿机器人
 */

#ifndef SPEED_PID_H
#define SPEED_PID_H

/* 速度环误差死区 (rpm): |目标 − 实测| 小于此值时输出 0
 * 死区必须 ≥ 编码器量化台阶 (0.15 rpm @ gear=30, 300ms 窗口), 改 gear 后需重算 */
#define SPD_PID_ERR_DEADBAND  1.0f

/* 走路时允许的最大倾角目标 (°): 速度环输出的目标倾角夹在此范围内 */
#define SPD_PID_WALK_PITCH_MAX 4.0f

/* 死区软化的过渡带宽度 (rpm): 死区与死区+过渡带之间按比例线性压缩, 保证输出连续 */
#define SPD_PID_ERR_SOFT_BAND  0.5f

/* 死区内的积分衰减时间常数 (秒): 误差进入死区后积分按 exp(-dt/tau) 衰减到 0 */
#define SPD_PID_I_DECAY_TAU  3.0f

/* 速度环内置默认增益 (°/rpm): param 表里 spd_kp / spd_ki 为 0 时用它兜底 */
#define SPD_PID_KP_DEFAULT    0.06f
#define SPD_PID_KI_DEFAULT    0.01f

/**
 * @brief 初始化速度环 (从 param 表读增益 + 清积分)
 * @note  上电初始化、参数重载时调用
 */
void speed_pid_init(void);

/**
 * @brief 从 param 表重新读取速度环增益
 * @note  每个控制周期都调一次; 只改系数, 保留积分 (用 pid_set_gains)
 */
void speed_pid_load_params(void);

/**
 * @brief 清空速度环的积分与微分历史
 * @note  上电 / 摔倒恢复 / 换遥控指令 (方向变了) 时调用
 */
void speed_pid_reset(void);

/**
 * @brief 执行一次速度环运算
 * @param dt     控制周期 (秒)
 * @param speed  实测平均轮速 (rpm, 前进为正)
 * @param target 目标平均轮速 (rpm, 前进为正)
 * @return 目标倾角 (°), 已按输出限幅夹过
 * @note  误差 = 实测 − 目标。实测偏慢 → 误差为负 → 目标倾角为负 → 车前倾加速;
 *        实测偏快 → 误差为正 → 目标倾角为正 → 车后仰减速。
 */
float speed_pid_update(float dt, float speed, float target);

#endif // SPEED_PID_H
