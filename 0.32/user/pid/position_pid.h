/**
 * @file position_pid.h
 * @brief 位置环 (最外环): 位置误差 (cm) → 目标速度 (cm/s)
 *
 * 控制结构 (串级, 从外到内):
 *   位置环   (position_pid.c): 位置误差 → 目标速度 (cm/s)
 *        │
 *        ▼
 *   锁位速度环 (hold_pid.c):    平移速度误差 → 目标倾角 (°)
 *        │
 *        ▼
 *   平衡环   (balance_pid.c):   姿态误差 → 电机输出 (%)
 *
 * 位置环只输出"该走多快", 不管"歪多少才能走"。倾角由锁位速度环给:
 * 速度 = 位置误差的导数, 所以速度环天然提供阻尼, 位置环不需要再手写 D 项
 * (手写 D 用的是滞后的编码器轮速, 反而是负阻尼)。
 *
 * 本环只在"站住不动"时接管 (见 balance.c 的 pos_mode_t);
 * 行走时由速度环给目标倾角, 两个外环不同时说话。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-29
 */

#ifndef POSITION_PID_H
#define POSITION_PID_H

/* ========== 位置环参数 (cm → cm/s) ========== */

/* 位置误差死区 (cm): 小于这个距离当作没偏, 不要求移动。
 * 0.3 cm = 19 个编码器脉冲, 远大于里程计噪声 (量化台阶 0.0155 cm), 不会误纠偏。
 * ⚠ 这只是"允许多偏"的容差: 位置环不再要求移动, 但锁位速度环一直在线,
 *   车被推了照样被刹住 → 不会产生"死区极限环"。 */
#define POS_PID_ERR_DEADBAND  0.3f

/* 死区软化过渡带 (cm): 死区与全响应之间线性压缩, 保证误差连续。
 * 硬归零会让目标速度在边界跳变 → 车抖。 */
#define POS_PID_ERR_SOFT_BAND 0.4f

/* 目标速度上限 (cm/s): 回位时最快要走多快 (安全阀)。
 * 位置增益默认 2.0 (cm/s per cm) → 3 cm 误差就顶到上限;
 * 6 cm/s 走完 6 cm 只要 1 秒, 回位快, 且不会冲得太狠。 */
#define POS_PID_V_MAX         6.0f

/* 死区内的积分衰减时间常数 (秒):
 * 死区把误差归零后传进 PID, 积分会**冻结** (i += 0×dt), 攒下的残留积分
 * 会永久留在非零值上 → 车一直往一个方向偏。
 * 改成按 exp(-dt/τ) 衰减到 0, 输出连续不抖。
 * ⚠ 位置环积分默认是 0 (PARAM_POS_KI): "抗摩擦的常驻倾角"由**速度环的积分**
 *   负责 (hold_pid.c), 两个环都带积分容易互相追 → 车前后摆。 */
#define POS_PID_I_DECAY_TAU   3.0f

/**
 * @brief 初始化位置环 (从 param 表读增益 + 清积分)
 */
void position_pid_init(void);

/**
 * @brief 从 param 表重新读取位置环增益
 */
void position_pid_load_params(void);

/**
 * @brief 清空位置环的积分与微分历史
 */
void position_pid_reset(void);

/**
 * @brief 执行一次位置环运算
 * @param dt  控制周期 (秒)
 * @param err 位置误差 (cm): 正 = 目标位置在前方 (需要往前走)
 * @return 目标速度 (cm/s, 前进为正): 车应该走多快
 */
float position_pid_update(float dt, float err);

#endif // POSITION_PID_H
