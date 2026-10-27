/**
 * @file steer_pid.h
 * @brief 转向环 (差速环): 左右轮速差 → 两轮输出差 (%)
 *
 * @note  转向 = 两轮反向转 (原地转), 平均轮速保持 0 (不往前跑),
 *        平衡环照常扶住车身。差速环把"左−右 轮速差"换算成"两轮输出的差"。
 *
 * @note  ⚠ 差速输出是**直接叠加**到平衡输出上的, 所以它的上限不能太大:
 *   差速占掉太多 → 平衡环的出力被削 → 转向的时候车容易倒。
 *   输出上限 60% 留 20% 给差速, 平衡环至少还有 40% 的余量。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#ifndef STEER_PID_H
#define STEER_PID_H

/* ========== 转向环参数 (%/rpm) ========== */
#define STEER_PID_KP         0.5f    /* 差速输出 (%/rpm) */
#define STEER_PID_KI         0.05f   /* 差速输出 (%/(rpm·s)) */
#define STEER_PID_OUT_MAX    20.0f   /* 差速输出上限 (%) */

/**
 * @brief 初始化转向环 (装载固定增益 + 清积分)
 * @note  转向环增益目前是代码里的固定值 (不走 param 表),
 *        所以 init 只需设一次系数; 但保留 load_params 接口,
 *        将来要挪进参数表时不用改调用点。
 */
void steer_pid_init(void);

/**
 * @brief 重新装载转向环增益 (走 param 表, 便于将来开放调参)
 * @note  目前增益是固定宏, 所以这个函数等价于 init 的系数部分。
 */
void steer_pid_load_params(void);

/**
 * @brief 清空转向环的积分与微分历史
 * @note  上电 / 摔倒恢复 / 换遥控指令 (方向变了, 旧积分要清) 时调用。
 */
void steer_pid_reset(void);

/**
 * @brief 执行一次转向环运算
 * @param dt     控制周期 (秒)
 * @param diff   实测 左−右 轮速差 (rpm, 前进为正, 左轮更快为正)
 * @param target 目标 左−右 轮速差 (rpm, 前进为正, 左轮更快为正)
 * @return 差速输出 (%), 正 = 左轮更快
 *
 * @note  误差 = 目标差 − 实测差: 左轮该更快时误差为正 → 差速输出为正
 *        → 左轮输出更小 (往车头 = 前进) → 左轮更快 ✓
 */
float steer_pid_update(float dt, float diff, float target);

#endif // STEER_PID_H
