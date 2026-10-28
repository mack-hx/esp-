/**
 * @file position_pid.c
 * @brief 位置环实现: 位置误差 (cm) → 目标轮速修正 (rpm)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#include "position_pid.h"
#include "pid.h"
#include "param.h"
#include <math.h>

/* 模块内部唯一的 PID 实例 (位置环) */
static bal_pid_t s_pid;

/* ========== 输出整形 (限速 + 一阶低通) ==========
 * 位置环的输出会**直接加到目标轮速上** (见 balance.c 的 balance_control_step),
 * 所以它一跳变, 目标倾角就跟着跳, 平衡环被迫猛纠 → 车前后摆。
 * 两级整形放在本模块里 (而不是 balance.c), 好处是能离线单测。 */
static float s_out_slew = 0.0f;   /* 限速之后的输出 (rpm) */
static float s_out_lpf  = 0.0f;   /* 低通之后的输出 (rpm), 真正送出去的值 */

void position_pid_load_params(void)
{
    float kp = param_get_float(PARAM_POS_KP);
    float ki = param_get_float(PARAM_POS_KI);

    /* 增益为 0 时兜底: 位置环是可选功能, 但 0 也不该让车发疯。
     * 这里保持 0 就是"完全不纠偏", 相当于关掉位置环 (只留速度环)。 */
    float kp_eff = (kp > 0.001f) ? kp : 0.0f;
    float ki_eff = (ki > 0.001f) ? ki : 0.0f;

    /* ⚠ 只改系数, 不清积分 (每周期都调, 用 pid_config 会让积分永远为 0) */
    pid_set_gains(&s_pid,
                  kp_eff, ki_eff, 0.0f,
                  pid_i_max(ki_eff, POS_PID_OUT_MAX),
                  POS_PID_OUT_MAX);
}

void position_pid_reset(void)
{
    pid_reset(&s_pid);
    /* 整形状态一起清: 换指令/摔倒/上电时不该带着上一段动作的残留输出 */
    s_out_slew = 0.0f;
    s_out_lpf  = 0.0f;
}

void position_pid_init(void)
{
    position_pid_load_params();
    position_pid_reset();
}

/**
 * @brief 输出整形: 限速 + 一阶低通
 * @param dt  控制周期 (秒)
 * @param raw PID 原始输出 (rpm)
 * @return 整形后的输出 (rpm)
 */
static float position_pid_shape(float dt, float raw)
{
    /* --- 限速: 每周期最多变 POS_PID_OUT_SLEW (rpm/s) --- */
    float slew = POS_PID_OUT_SLEW * dt;
    if (raw > s_out_slew + slew)      s_out_slew += slew;
    else if (raw < s_out_slew - slew) s_out_slew -= slew;
    else                               s_out_slew = raw;

    /* --- 一阶低通: 滤掉残余的台阶/噪声 --- */
    float alpha = dt / (POS_PID_OUT_LPF_TAU + dt);
    s_out_lpf += (s_out_slew - s_out_lpf) * alpha;

    return s_out_lpf;
}

float position_pid_update(float dt, float err)
{
    float abs_err = fabsf(err);

    if (abs_err <= POS_PID_ERR_DEADBAND) {
        /* 死区内: 误差归零, 同时让积分**衰减**而不是冻结
         * (跟速度环同一个坑, 见 position_pid.h 的说明) */
        err = 0.0f;
        pid_decay_integral(&s_pid, dt, POS_PID_I_DECAY_TAU);
    } else if (abs_err < POS_PID_ERR_DEADBAND + POS_PID_ERR_SOFT_BAND) {
        /* 过渡带: 线性压缩, 保证误差连续 (消除边界跳变) */
        float t = (abs_err - POS_PID_ERR_DEADBAND) / POS_PID_ERR_SOFT_BAND;
        err = (err > 0.0f) ? t : -t;
    }

    float raw = pid_update(&s_pid, err, dt);
    return position_pid_shape(dt, raw);
}
