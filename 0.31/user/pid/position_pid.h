/**
 * @file position_pid.h
 * @brief 位置环 (预留): 位置误差 → 目标速度修正 (rpm)
 *
 * @note  ⚠ **本模块当前没有接入控制流程** (2026-10-27 建文件时定的):
 *   平衡车目前是"速度环 → 平衡环"两级串级, 站住不动时目标轮速恒为 0。
 *   位置环要放在速度环**外面** (位置误差 → 目标轮速修正 → 速度环),
 *   但工程里目前**没有里程计**: 编码器只能给瞬时转速, 积分成位移需要
 *   轮径参数 (参数表里没有), 也没有绝对位置基准 (没有编码器零点复位接口)。
 *   所以位置环的"位置反馈"目前拿不到, 强行接进去只会得到一个恒为 0 的环。
 *
 * @note  位置环的**误差由调用方给**, 本模块不自己做里程计:
 *   这样等里程计 (轮径参数 + 编码器零点) 做好了, 只要把误差算好传进来即可,
 *   本模块不用改。
 *
 * @note  将来接入的顺序 (届时再动 balance.c):
 *        位置环 (位置误差 → 目标轮速修正) → 速度环 → 平衡环
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#ifndef POSITION_PID_H
#define POSITION_PID_H

/* ========== 位置环参数 ==========
 * 单位: 位置误差用"轮子转过的圈数" (turn), 输出用 rpm。
 * 增益: rpm/turn —— 偏 1 圈时, 要求目标轮速修正多少 rpm。 */

/* 位置误差死区 (turn): 小于这个距离当作没偏, 不纠。
 * 0.05 turn ≈ 轮子转 18°, 小到只是"配平残留", 大到是真的走偏了。 */
#define POS_PID_ERR_DEADBAND  0.05f

/* 位置环增益 (rpm/turn) */
#define POS_PID_KP             8.0f
#define POS_PID_KI             0.5f

/* 位置环输出上限 (rpm): 位置纠偏速度不能大过正常行走速度,
 * 否则车会为了纠一点点位置先猛冲一段。 */
#define POS_PID_OUT_MAX        20.0f

/**
 * @brief 初始化位置环 (装载固定增益 + 清积分)
 */
void position_pid_init(void);

/**
 * @brief 重新装载位置环增益
 * @note  目前增益是固定宏, 等挪进 param 表后这里改成读参数。
 */
void position_pid_load_params(void);

/**
 * @brief 清空位置环的积分与微分历史
 * @note  上电 / 摔倒恢复 / 换目标位置时调用。
 */
void position_pid_reset(void);

/**
 * @brief 执行一次位置环运算
 * @param dt   控制周期 (秒)
 * @param err  位置误差 (turn): 正 = 目标在前方 (需要往前走)
 * @return 目标轮速修正量 (rpm), 正 = 需要往前走
 *
 * @note  误差死区: 小于 POS_PID_ERR_DEADBAND 时当作 0, 不纠。
 */
float position_pid_update(float dt, float err);

#endif // POSITION_PID_H
