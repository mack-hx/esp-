#include "pid.h"
#include <math.h>

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

/**
 * @brief 判断"输出已饱和且误差还在往同一方向推" → 此时不能再积分
 * @param pid   PID 结构指针
 * @param out   当前完整输出 (含积分项)
 * @param err   当前误差
 * @return true=本周期应冻结积分
 */
static bool pid_windup_blocked(const bal_pid_t *pid, float out, float err)
{
    if (pid->out_max <= 0.0f) return false;   /* 输出不限幅 → 不需要抗饱和 */
    if (out >=  pid->out_max && err > 0.0f) return true;   /* 顶上限, 误差还在往上推 */
    if (out <= -pid->out_max && err < 0.0f) return true;   /* 顶下限, 误差还在往下推 */
    return false;
}

float pid_i_max(float ki, float out_max)
{
    if (ki <= 0.01f) return out_max;    /* Ki≈0 时积分不参与运算 */
    return (out_max * 0.5f) / ki;
}

void pid_set_gains(bal_pid_t *pid, float kp, float ki, float kd,
                   float i_max, float out_max)
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->i_max   = i_max;
    pid->out_max = out_max;
    /* ⚠ 不碰 i / prev_err / primed: 运行中改参数不能把积分清掉,
     *   否则每次改参数都会让积分项重新从 0 累积, 输出会出现台阶。 */
}

void pid_config(bal_pid_t *pid, float kp, float ki, float kd,
                float i_max, float out_max)
{
    pid_set_gains(pid, kp, ki, kd, i_max, out_max);
    pid_reset(pid);
}

void pid_reset(bal_pid_t *pid)
{
    pid->i        = 0.0f;
    pid->prev_err = 0.0f;
    pid->primed   = false;
}

void pid_decay_integral(bal_pid_t *pid, float dt, float time_const)
{
    if (dt <= 0.0f || time_const <= 0.0f) return;

    /* 指数衰减: i *= exp(-dt/tau)
     * tau 越小衰减越快。用 exp 而不是 (1 - dt/tau) 是因为后者在
     * dt > tau 时会变成负数, 把积分符号翻过来。 */
    pid->i *= expf(-dt / time_const);

    /* 数值上衰减到极小就归零, 避免留下永不消失的微小残值 */
    if (fabsf(pid->i) < 1e-6f) pid->i = 0.0f;
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
    float i_new = clamp_sym(pid->i + err * dt, pid->i_max);

    /* 抗积分饱和: 输出已顶到限幅且误差还在同向 → 冻结积分 */
    float out = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    if (!pid_windup_blocked(pid, out, err)) {
        pid->i = i_new;
    }

    /* --- 输出 --- */
    out = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    return clamp_sym(out, pid->out_max);
}


/**
 * @brief D 项用角速度反馈的 PID (平衡环专用)
 */
float pid_update_rate(bal_pid_t *pid, float err, float rate, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    /* D 项 = 角速度本身。首次调用跳过, 防止上电瞬间的假阻尼冲量。 */
    float d = pid->primed ? rate : 0.0f;

    pid->prev_err = err;
    pid->primed   = true;

    /* 抗积分饱和: 输出已顶到限幅且误差还在同向 → 冻结积分
     * (说明见 pid_windup_blocked 的注释) */
    float i_new = clamp_sym(pid->i + err * dt, pid->i_max);
    float out_now = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    if (!pid_windup_blocked(pid, out_now, err)) {
        pid->i = i_new;
    }
    float out = pid->kp * err + pid->ki * pid->i + pid->kd * d;
    return clamp_sym(out, pid->out_max);
}
