/**
 * @file pid.h
 * @brief 通用 PID 控制器 (位置式)
 * @note  平衡车用到的三个环都用这个结构:
 *        - 平衡环: 输入 = 姿态角误差 (°),   输出 = 电机目标差速 (%)
 *        - 速度环: 输入 = 轮速误差 (rpm),  输出 = 目标倾角 (°)
 *        - 转向环: 输入 = 转向误差,        输出 = 左右差速修正 (%)
 *
 * @note  dt 由调用方传入 (秒)。不要在 pid_update() 内部取时间:
 *        控制周期是固定的 (control.c 里 CONTROL_PERIOD_MS),
 *        传固定 dt 能保证同一组参数在离线计算和真机运行时表现一致。
 *
 * @note  ⚠ 类型名必须叫 bal_pid_t, 不能叫 pid_t:
 *        newlib 的 <sys/types.h> 已经 typedef 了 pid_t (进程 id, int),
 *        重名会直接编译报错 "conflicting types for 'pid_t'"。
 *
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

/**
 * @brief PID 控制器状态结构
 * @note  三个环各持有一个实例, 互不干扰
 */
typedef struct {
    float kp;        /* 比例系数 */
    float ki;        /* 积分系数 */
    float kd;        /* 微分系数 */
    float i_max;     /* 积分累积限幅 (防积分饱和) */
    float out_max;   /* 输出限幅 (正负对称) */

    /* ---- 以下为运行状态, 由 pid_update() 维护, 不要手动改 ---- */
    float i;         /* 积分累积值 */
    float prev_err;  /* 上一次的误差 (微分用) */
    bool  primed;    /* 是否已经执行过至少一次 pid_update (false 时跳过微分) */
} bal_pid_t;

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
 * @brief 清空积分和微分历史 (上电 / 摔倒恢复 / 切模式时调用)
 * @note  不清积分的话, 上一次摔倒前累积的积分会让电机一启动就猛冲一下
 * @param pid PID 结构指针
 */
void pid_reset(bal_pid_t *pid);

/**
 * @brief 执行一次 PID 运算
 * @param pid PID 结构指针
 * @param err 当前误差 (目标值 - 反馈值)
 * @param dt  距上次调用的时间间隔 (秒, 必须 > 0)
 * @return 控制输出 (已按 out_max 限幅)
 */
float pid_update(bal_pid_t *pid, float err, float dt);

#endif // PID_H
