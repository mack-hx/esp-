/**
 * @file balance_pid.h
 * @brief 平衡环 (内环): 姿态误差 → 电机输出 (%)
 *
 * @note  控制结构 (串级, 从外到内):
 *            速度环 (speed_pid.c): 轮速误差 → 目标倾角 (°)
 *            平衡环 (balance_pid.c): 姿态误差 → 电机输出 (%)
 *
 * @note  本模块只负责"纯控制运算", 不碰硬件:
 *        传感器采样、状态机、电机输出全在 user/control/balance.c 里。
 *        这样调平衡环参数时, 不用在一堆无关代码里找。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#ifndef BALANCE_PID_H
#define BALANCE_PID_H

/**
 * @brief 初始化平衡环 (从 param 表读增益 + 清积分)
 * @note  上电初始化、参数重载时调用
 */
void balance_pid_init(void);

/**
 * @brief 从 param 表重新读取平衡环增益
 * @note  **每个控制周期都调一次**: App / 网页上拖滑块后, 下一次运算立刻用新值,
 *        不用重启。
 *        ⚠ 用的是 pid_set_gains() (只改系数, 保留积分/微分历史),
 *          不能用 pid_config() (它会清积分, 每周期调用会让积分永远为 0)。
 */
void balance_pid_load_params(void);

/**
 * @brief 清空平衡环的积分与微分历史
 * @note  上电 / 摔倒恢复 / 切模式时调用。
 *        不清积分的话, 上一次摔倒前累积的积分会让电机一启动就猛冲一下。
 */
void balance_pid_reset(void);

/**
 * @brief 执行一次平衡环运算
 * @param dt           控制周期 (秒)
 * @param pitch        修正后姿态角 (°)
 *                     ⚠ 本工程符号: **前倾 (车头往下) 为负数**
 * @param target_pitch 目标倾角 (°) —— 由速度环给出
 * @param gyro         俯仰方向角速度 (°/s), 与 pitch **同号**
 * @return 电机输出 (%), **未乘** BAL_MOTOR_SIGN (方向修正由 balance.c 做)
 *
 * @note  D 项用**角速度反馈**而不是误差微分 (见 pid_update_rate 的注释),
 *        且与 P 项同号: 车往前倒得越快, 轮子追得越凶 —— 这才是阻尼。
 */
float balance_pid_update(float dt, float pitch, float target_pitch, float gyro);

#endif // BALANCE_PID_H
