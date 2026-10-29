/**
 * @file speed_pid.c
 * @brief 速度环 (外环) 实现: 轮速误差 → 目标倾角 (°)
 * @author ESP32 平衡轮腿机器人
 */

#include "speed_pid.h"
#include "pid.h"
#include "param.h"
#include <math.h>

/* 模块内部唯一的 PID 实例 (速度环) */
static bal_pid_t s_pid;

void speed_pid_load_params(void)
{
    float spdout_max = param_get_float(PARAM_SPEED_OUT_MAX);

    float kp = param_get_float(PARAM_PID_SPD_KP);
    float ki = param_get_float(PARAM_PID_SPD_KI);
    float kd = param_get_float(PARAM_PID_SPD_KD);

    /* 速度环: 输出 = 目标倾角 (°), 限幅夹到 SPD_PID_WALK_PITCH_MAX。
     * 限幅放在 PID 内部: 放外面的话积分会继续往上限攒, 误差反向时要很久才退回来。 */
    float walk_max = spdout_max;
    if (walk_max > SPD_PID_WALK_PITCH_MAX) walk_max = SPD_PID_WALK_PITCH_MAX;
    if (walk_max < 0.0f) walk_max = 0.0f;

    /* 速度环增益为 0 时用内置默认值 (防止旧版 App 把参数写回 0,
     * 结果方向键完全没反应) */
    float kp_eff = (kp > 0.001f) ? kp : SPD_PID_KP_DEFAULT;
    float ki_eff = (ki > 0.001f) ? ki : SPD_PID_KI_DEFAULT;
    float kd_eff = (kd > 0.001f) ? kd : 0.0f;

    /* ⚠ 只改系数, 不清积分 (每周期都调, 用 pid_config 会让积分永远为 0) */
    pid_set_gains(&s_pid,
                  kp_eff, ki_eff, kd_eff,
                  pid_i_max(ki_eff, walk_max),
                  walk_max);        /* 输出 = 目标倾角 (°) */
}

void speed_pid_reset(void)
{
    pid_reset(&s_pid);
}

void speed_pid_init(void)
{
    speed_pid_load_params();
    speed_pid_reset();
}

float speed_pid_update(float dt, float speed, float target)
{
    float err = speed - target;

    /* 误差死区 + 软化过渡带
     * 死区须 ≥ 编码器量化台阶 (0.15 rpm @ gear=30, 300ms 窗口) */
    float abs_err = fabsf(err);
    if (abs_err <= SPD_PID_ERR_DEADBAND) {
        err = 0.0f;

        /* 死区内让积分衰减而不是冻结: 死区把 err 归零后积分会 i += 0*dt,
         * 残留积分会永久冻结在非零值。这里用指数衰减 (SPD_PID_I_DECAY_TAU)
         * 让积分平滑滑向 0, 输出连续不抖。死区外正常累积。 */
        pid_decay_integral(&s_pid, dt, SPD_PID_I_DECAY_TAU);
    } else {
        /* 死区外: 有效误差 = 误差 − 死区宽度, 过渡带内平滑起步 (输出处处连续) */
        err = pid_soft_deadband(err, SPD_PID_ERR_DEADBAND, SPD_PID_ERR_SOFT_BAND);
    }

    return pid_update(&s_pid, err, dt);
}
