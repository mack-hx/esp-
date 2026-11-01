/**
 * @file odom.h
 * @brief 里程计: 编码器脉冲 → 行进距离 (cm)
 *
 * @note  位移 = 脉冲数 / (44 × gear) × π × 轮径。
 *        轮径 (wheel_dia) 来自参数表, 本机 = 6.5 cm;
 *        gear = 30 → 轮子转一圈 1320 个脉冲, 1 个脉冲 ≈ 0.0155 cm。
 *
 * @note  符号约定: 本模块只认"前进为正"的脉冲增量,
 *        编码器原始方向的换算由调用方 (balance.c) 负责。
 *
 * @author ESP32 平衡轮腿机器人
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
 * @note  两轮取平均: 原地转向时两轮一正一负, 平均相互抵消。
 * @note  必须挂在每个窗口的脉冲增量上 (100ms 一次), 不能挂在 rpm 换算之后。
 */
void odom_update(int d_left, int d_right);

/**
 * @brief 取当前累计位移
 * @return 相对最后一次 odom_reset() 的位移 (cm), 前进为正
 */
float odom_get_position_cm(void);

/**
 * @brief 取当前速度
 * @return 速度 (cm/s, 前进为正)
 * @note  由最近一个 100ms 里程计窗口的位移增量算出, 每 100ms 更新一次。
 * @note  位置保持环的速度阻尼用它, 不用编码器的 300ms 轮速 (滞后太大)。
 */
float odom_get_velocity_cmps(void);

#endif // ODOM_H
