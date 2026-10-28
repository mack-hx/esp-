/**
 * @file position_pid.h
 * @brief 位置环: 位置误差 (cm) → 目标轮速修正 (rpm)
 *
 * @note  控制结构 (串级, 从外到内):
 *            位置环 (position_pid.c): 位置误差 → 目标轮速修正 (rpm)
 *                 │
 *                 ▼
 *            速度环 (speed_pid.c): 轮速误差 → 目标倾角 (°)
 *                 │
 *                 ▼
 *            平衡环 (balance_pid.c): 姿态误差 → 电机输出 (%)
 *
 * @note  位置误差由**里程计** (user/odom/) 提供:
 *        编码器脉冲 → 位移 (cm), 换算靠参数表里的 wheel_dia (轮径 6.5 cm)。
 *        本模块不自己做里程计, 只负责把误差变成轮速修正量。
 *
 * @note  ⚠ 位置环只在"**站着不动**"时接管 (没有行走指令、速度目标已归零):
 *        行走时车是被人/App 指挥的, 位置环必须让位, 否则两个环对着干。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#ifndef POSITION_PID_H
#define POSITION_PID_H

/* ========== 位置环参数 (cm → rpm) ========== */

/* 位置误差死区 (cm): 小于这个距离当作没偏, 不纠。
 * 取 1.0 cm: 轮子周长才 20.4 cm (轮径 6.5), 1 cm 已是半个轮子半径的量级;
 * 再小就是编码器噪声 + 积分漂移, 会让车不停地微调。 */
#define POS_PID_ERR_DEADBAND  1.0f

/* 死区软化过渡带 (cm): 死区与全响应之间线性压缩, 保证误差连续。
 * 跟速度环的软化带一个道理: 硬归零会让目标轮速在边界跳变 → 车抖。 */
#define POS_PID_ERR_SOFT_BAND 1.0f

/* 死区内的积分衰减时间常数 (秒):
 * 死区把误差归零后传进 PID, 积分会**冻结** (i += 0×dt, 既不增也不减),
 * 攒下的残留积分会永久留在非零值上 → 车一直往一个方向偏。
 * 改成按 exp(-dt/τ) 衰减到 0, 输出连续, 不抖。 */
#define POS_PID_I_DECAY_TAU   3.0f

/* 位置环输出上限 (rpm): 纠偏速度不能大过正常行走速度,
 * 否则车为了纠一点点位置先猛冲一段。 */
#define POS_PID_OUT_MAX       20.0f

/* ========== 位置环输出的整形 ==========
 * 位置环的输出会**直接加到目标轮速上**, 所以它一跳变, 目标倾角就跟着跳,
 * 平衡环被迫猛纠 → 车前后摆。所以输出要"慢一点、稳一点":
 *   - 限速: 每周期最多变 POS_PID_OUT_SLEW (rpm/s)
 *   - 一阶低通: 时间常数 POS_PID_OUT_LPF_TAU (秒), 再滤一层噪声
 * 这两级是"外环整形", 不改变 PID 本身的稳态增益。 */
#define POS_PID_OUT_SLEW      12.0f   /* 输出限速 (rpm/s) */
#define POS_PID_OUT_LPF_TAU   0.20f   /* 一阶低通时间常数 (秒) */

/**
 * @brief 初始化位置环 (从 param 表读增益 + 清积分)
 * @note  上电初始化、参数重载时调用
 */
void position_pid_init(void);

/**
 * @brief 从 param 表重新读取位置环增益
 * @note  **每个控制周期都调一次**: App / 网页上改完, 下一周期立刻用新值。
 *        ⚠ 用 pid_set_gains() (只改系数, 保留积分), 不用 pid_config()。
 */
void position_pid_load_params(void);

/**
 * @brief 清空位置环的积分与微分历史
 * @note  上电 / 摔倒恢复 / 开始行走时调用 (行走不需要记位置积分)。
 */
void position_pid_reset(void);

/**
 * @brief 执行一次位置环运算
 * @param dt  控制周期 (秒)
 * @param err 位置误差 (cm): 正 = 目标位置在前方 (需要往前走)
 * @return 目标轮速修正量 (rpm), 正 = 需要往前走
 *
 * @note  误差死区: 小于 POS_PID_ERR_DEADBAND 时当作 0, 不纠。
 */
float position_pid_update(float dt, float err);

#endif // POSITION_PID_H
