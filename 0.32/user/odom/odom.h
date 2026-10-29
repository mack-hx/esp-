/**
 * @file odom.h
 * @brief 里程计: 编码器脉冲 → 行进距离 (cm)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#ifndef ODOM_H
#define ODOM_H

/**
 * @brief 初始化里程计 (位置清零)
 */
void odom_init(void);

/**
 * @brief 位置清零 (编码器零点复位)
 */
void odom_reset(void);

/**
 * @brief 累加一次位移 (每个控制周期 10ms 调一次)
 * @param d_left  左轮本周期的脉冲增量 (**前进为正**)
 * @param d_right 右轮本周期的脉冲增量 (**前进为正**)
 *
 * 两轮取平均: 原地转向时两轮一正一负, 平均正好抵消 (车没平移)。
 * ⚠ 必须挂在**每个控制周期的脉冲增量**上, 不要挂在 rpm 换算之后 ——
 *   那样 300ms 才动一次, 位置反馈太粗, 车纠偏会一顿一顿的。
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
 *
 * 由最近 60 ms 的位移增量算出 (滑动窗口, 每控制周期更新)。
 * ⚠ 锁位速度环的反馈用它, 不用编码器的 300ms 轮速:
 *   300ms 窗口滞后太大, 阻尼相位落后 → 等价负阻尼 → 车来回晃。
 */
float odom_get_velocity_cmps(void);

#endif // ODOM_H
