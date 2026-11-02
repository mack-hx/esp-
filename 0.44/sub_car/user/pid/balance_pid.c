/**
 * @file balance_pid.c
 * @brief 平衡环 (内环) 实现: 姿态误差 → 电机输出 (%)
 * @author ESP32 平衡轮腿机器人
 */

#include "balance_pid.h"
#include "pid.h"
#include "param.h"

/* 模块内部唯一的 PID 实例 (平衡环) */
static bal_pid_t s_pid;

void balance_pid_load_params(void)
{
    float out_max = param_get_float(PARAM_MOTOR_OUT_MAX);

    float kp = param_get_float(PARAM_PID_BAL_KP);
    float ki = param_get_float(PARAM_PID_BAL_KI);
    float kd = param_get_float(PARAM_PID_BAL_KD);

    /* ⚠ 只改系数, 不清积分 (用 pid_set_gains 而不是 pid_config)。
     *   每周期都调 load_params, 用 pid_config 会让积分永远为 0。 */
    pid_set_gains(&s_pid,
                  kp, ki, kd,
                  pid_i_max(ki, out_max),   /* 积分项最大 = 输出一半 */
                  out_max);
}

void balance_pid_reset(void)
{
    pid_reset(&s_pid);
}

void balance_pid_init(void)
{
    balance_pid_load_params();
    balance_pid_reset();
}

float balance_pid_update(float dt, float pitch, float target_pitch, float gyro)
{
    float err = pitch - target_pitch;

    /* D 项用**角速度反馈**而不是误差微分 (见 pid_update_rate 的注释),
     * 且与 P 项同号: 车往前倒得越快, 轮子追得越凶 —— 这才是阻尼。 */
    return pid_update_rate(&s_pid, err, gyro, dt);
}
