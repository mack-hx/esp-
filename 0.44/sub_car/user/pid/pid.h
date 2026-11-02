/**
 * @file pid.h
 * @brief 通用 PID 控制器 (位置式)
 *
 * @note  平衡车三个环共用本结构:
 *        - 平衡环 (balance_pid.c): 输入 = 姿态角误差 (°), 输出 = 电机输出 (%)
 *        - 速度环 (speed_pid.c):   输入 = 轮速误差 (rpm), 输出 = 目标倾角 (°)
 *        - 转向环 (steer_pid.c):   输入 = 转向误差, 输出 = 左右差速修正 (%)
 *
 * @note  dt 由调用方传入 (秒), 控制周期固定, 保证离线计算与真机运行一致。
 * @note  类型名必须是 bal_pid_t: <sys/types.h> 已 typedef pid_t, 重名会编译报错。
 *
 * @author ESP32 机械狗项目
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

/**
 * @brief PID 控制器状态结构
 */
typedef struct {
    float kp;        /* 比例系数 */
    float ki;        /* 积分系数 */
    float kd;        /* 微分系数 */
    float i_max;     /* 积分累积限幅 (防积分饱和) */
    float out_max;   /* 输出限幅 (正负对称) */

    /* 运行状态, 由 pid_update() 维护 */
    float i;         /* 积分累积值 */
    float prev_err;  /* 上一次的误差 (微分用) */
    bool  primed;    /* 是否已执行过至少一次 pid_update (false 时跳过微分) */
} bal_pid_t;

/**
 * @brief 算积分累加值的上限 (i_max)
 * @param ki      积分系数
 * @param out_max 该环的输出上限
 * @return 累加值上限
 * @note  输出项是 Ki·i, 故累加值上限取 (out_max/2)/Ki 才能让积分项不超过输出上限一半。
 *        ki≈0 时积分不参与运算, 直接返回 out_max。
 */
float pid_i_max(float ki, float out_max);

/**
 * @brief 死区软化: 把误差映射成"有效误差"
 * @param err       误差 (带符号)
 * @param deadband  死区宽度 (绝对值)
 * @param soft_band 过渡带宽度 (绝对值)
 * @return 有效误差 (带符号)
 *
 * @note  |err| <= deadband → 0
 * @note  过渡带内 (deadband < |err| < deadband+soft_band) → soft_band·t², t = (|err|−deadband)/soft_band
 * @note  过渡带外 → |err| − deadband
 * @note  过渡带末端的取值 = soft_band, 与带外公式在同一处相等 → 输出处处连续
 */
float pid_soft_deadband(float err, float deadband, float soft_band);

/**
 * @brief 初始化 PID 系数
 * @param pid     PID 结构指针
 * @param kp      比例系数
 * @param ki      积分系数
 * @param kd      微分系数
 * @param i_max   积分累积限幅 (绝对值, <=0 表示不限制)
 * @param out_max 输出限幅 (绝对值, <=0 表示不限制)
 */
void pid_config(bal_pid_t *pid, float kp, float ki, float kd,
                float i_max, float out_max);

/**
 * @brief 只改系数, 保留积分和微分历史
 * @param pid     PID 结构指针
 * @param kp      比例系数
 * @param ki      积分系数
 * @param kd      微分系数
 * @param i_max   积分累积限幅 (绝对值, <=0 表示不限制)
 * @param out_max 输出限幅 (绝对值, <=0 表示不限制)
 */
void pid_set_gains(bal_pid_t *pid, float kp, float ki, float kd,
                   float i_max, float out_max);

/**
 * @brief 清空积分和微分历史 (上电 / 摔倒恢复 / 切模式时调用)
 * @param pid PID 结构指针
 */
void pid_reset(bal_pid_t *pid);

/**
 * @brief 让积分项按系数衰减 (向 0 靠拢)
 * @param pid        PID 结构指针
 * @param decay      衰减系数 (0~1): 每次调用后 i *= decay
 * @param dt         距上次调用的时间间隔 (秒, 用于按时间常数折算)
 * @param time_const 衰减时间常数 (秒): 越小衰减越快; <=0 表示不衰减
 * @return 无
 * @note  decay = exp(-dt / time_const), time_const 即积分衰减到 37% 所需时间。
 */
void pid_decay_integral(bal_pid_t *pid, float dt, float time_const);

/**
 * @brief 执行一次 PID 运算 (误差微分型)
 * @param pid PID 结构指针
 * @param err 当前误差 (目标值 - 反馈值)
 * @param dt  距上次调用的时间间隔 (秒, 必须 > 0)
 * @return 控制输出 (已按 out_max 限幅)
 * @note  D 项是误差变化率, 会二次放大姿态量化噪声; 平衡环用 pid_update_rate()。
 */
float pid_update(bal_pid_t *pid, float err, float dt);

/**
 * @brief 执行一次 PID 运算, D 项用角速度反馈 (平衡环专用)
 * @param pid  PID 结构指针
 * @param err  当前误差 (目标值 - 反馈值)
 * @param rate 该误差对应方向上的角速度 (°/s), 已按正确符号
 * @param dt   距上次调用的时间间隔 (秒, 必须 > 0)
 * @return 控制输出 (已按 out_max 限幅)
 * @note  rate 必须是俯仰方向角速度, 且与 err 同向。
 *        本工程符号约定: 前倾 (车头往下) 时 pitch 与角速度均为负数。
 * @note  D 项与 P 项同号: out = kp·err + ki·i + kd·rate。
 * @note  首次调用 (primed=false) 时跳过 D 项。
 */
float pid_update_rate(bal_pid_t *pid, float err, float rate, float dt);

#endif // PID_H
