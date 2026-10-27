/**
 * @file odom.h
 * @brief 里程计: 编码器脉冲 → 行进距离 (cm)
 *
 * @note  为什么需要它:
 *        位置环要有"位置反馈", 就必须知道车走了多远。
 *        编码器只能给瞬时转速, 位移要靠**把脉冲积分起来**才能得到:
 *
 *            距离 = 脉冲数 / (编码器每转脉冲数 × 减速比) × 轮子周长
 *                 = 脉冲数 / (44 × gear) × π × 轮径
 *
 *        轮径 (wheel_dia) 来自参数表, 本机 = **6.5 cm**。
 *        gear = 30 (编码器装在电机轴上, 减速箱 30:1) → 轮子转一圈 1320 个脉冲,
 *        1 个脉冲 = π×6.5/1320 ≈ **0.0155 cm**。
 *
 * @note  符号约定: 本模块只认"**前进为正**"的脉冲增量。
 *        编码器原始方向 = 车尾方向 (见 balance.h 的 BAL_ENC_FWD_SIGN),
 *        换算由调用方 (balance.c) 负责。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#ifndef ODOM_H
#define ODOM_H

/**
 * @brief 初始化里程计 (位置清零)
 * @note  上电时调一次
 */
void odom_init(void);

/**
 * @brief 位置清零 (编码器零点复位)
 * @note  把当前累计位移重新当作"原点"。
 *        起立 / 重新定位时调, 之后的位移都从这个点算起。
 */
void odom_reset(void);

/**
 * @brief 累加一次位移 (每 100ms 调一次)
 * @param d_left  左轮本窗口的脉冲增量 (**前进为正**)
 * @param d_right 右轮本窗口的脉冲增量 (**前进为正**)
 * @note  两轮取平均: 原地转向时两轮一正一负, 平均正好抵消 (车没平移)。
 * @note  ⚠ 必须挂在**每个窗口的脉冲增量**上 (100ms 一次),
 *        不要挂在 rpm 换算之后 —— 那样 300ms 才动一次, 位置反馈太粗。
 */
void odom_update(int d_left, int d_right);

/**
 * @brief 取当前累计位移
 * @return 相对最后一次 odom_reset() 的位移 (cm), 前进为正
 */
float odom_get_position_cm(void);

#endif // ODOM_H
