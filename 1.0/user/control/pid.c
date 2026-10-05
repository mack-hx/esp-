/**
 * @file pid.c
 * @brief 通用 PID 控制器实现 (位置式)
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#include "pid.h"

/**
 * @brief 把值限制到 [-limit, +limit]
 * @param v      原值
 * @param limit 限幅绝对值 (<=0 表示不限幅)
 * @return 限幅后的值
 */
static float clamp_sym(float v, float limit)
{
    if (limit <= 0.0f) return v;
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

void pid_config(bal_pid_t *pid, float kp, float ki, float kd,
                float i_max, float out_max)
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->i_max   = i_max;
    pid->out_max = out_max;
    pid_reset(pid);
}

void pid_reset(bal_pid_t *pid)
{
    pid->i        = 0.0f;
    pid->prev_err = 0.0f;
    pid->primed   = false;
}

float pid_update(bal_pid_t *pid, float err, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    /* --- 微分项 ---
     * 首次调用时 prev_err 还是 0, 直接拿它做微分会算出一个假的大跳变,
     * 所以 primed=false 时只记录不参与运算。 */
    float d = 0.0f;
    if (pid->primed) {
        d = (err - pid->prev_err) / dt;
    }
    pid->prev_err = err;
    pid->primed   = true;

    /* --- 积分项 (带累积限幅, 防积分饱和) --- */
    pid->i = clamp_sym(pid->i + err * dt, pid->i_max);

    /* --- 输出 --- */
    float out = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    return clamp_sym(out, pid->out_max);
}
