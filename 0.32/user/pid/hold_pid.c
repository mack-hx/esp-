/**
 * @file hold_pid.c
 * @brief 锁位速度环实现: 平移速度误差 (cm/s) → 目标倾角 (°)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-29
 */

#include "hold_pid.h"
#include "param.h"
#include <math.h>

/* 增益 (每周期从 param 表刷新) */
static float s_kp = HOLD_PID_KP_DEFAULT;   /* °/(cm/s) */
static float s_ki = 0.0f;                   /* °/cm */

/* 积分状态 (cm): ∫(速度误差)dt。
 * 它存的是"抗摩擦所需的常驻倾角"对应的积分量:
 * 站住不动时速度误差 ≈ 0, 但积分不为 0 → 车保持一个小的倾角 → 不漂。 */
static float s_i = 0.0f;

/* 输出整形状态: 限速之后的倾角 / 低通之后的倾角 (°, 正 = 前倾) */
static float s_out_slew = 0.0f;
static float s_out_lpf  = 0.0f;

void hold_pid_load_params(void)
{
    float kp = param_get_float(PARAM_HOLD_KP);
    float ki = param_get_float(PARAM_HOLD_KI);

    /* Kp 为 0 时兜底: 速度环是位置保持的主力, 关掉它等于"位置环无效",
     * 不应该让 App 误拖一下就把车变成"推了不回位"。
     * (参考 speed_pid 的 SPD_PID_KP_DEFAULT: 旧版 App 把参数写成 0 的教训) */
    s_kp = (kp > 0.001f) ? kp : HOLD_PID_KP_DEFAULT;

    /* Ki 默认 0 = 不带积分。
     * 积分能消掉稳态速度误差 (车在斜坡上/左右摩擦不对称时有用),
     * 但它同时是"常驻倾角"的来源, 配大了会攒出一个长驻倾角 → 车一直走。
     * 先用 0 跑, 真机确认有稳态速度误差再从 App 上加。 */
    s_ki = (ki > 0.0f) ? ki : 0.0f;
}

void hold_pid_reset(void)
{
    s_i = 0.0f;
    s_out_slew = 0.0f;
    s_out_lpf  = 0.0f;
}

void hold_pid_init(void)
{
    hold_pid_load_params();
    hold_pid_reset();
}

/**
 * @brief 输出整形: 限速 + 一阶低通
 * @param dt  控制周期 (秒)
 * @param raw PI 原始输出 (°, 正 = 前倾)
 * @return 整形后的倾角 (°, 正 = 前倾)
 */
static float hold_pid_shape(float dt, float raw)
{
    /* --- 限速: 每周期最多变 HOLD_PID_LEAN_SLEW (°/s) --- */
    float slew = HOLD_PID_LEAN_SLEW * dt;
    if (raw > s_out_slew + slew)      s_out_slew += slew;
    else if (raw < s_out_slew - slew) s_out_slew -= slew;
    else                               s_out_slew = raw;

    /* --- 一阶低通 --- */
    float alpha = dt / (HOLD_PID_LEAN_LPF_TAU + dt);
    s_out_lpf += (s_out_slew - s_out_lpf) * alpha;

    return s_out_lpf;
}

float hold_pid_update(float dt, float v_cmps, float v_ref_cmps)
{
    if (dt <= 0.0f) return 0.0f;

    /* 误差 = 目标速度 − 实测速度 (正 = 需要往前走) */
    float err = v_ref_cmps - v_cmps;

    /* --- P 项: 死区 + 软化带 ---
     * ⚠ 死区**只作用在 P 项**: 里程计速度有 0.26 cm/s 的量化台阶,
     *   不滤的话车会为了追一个噪声一直歪着。
     *   但积分必须用**原始误差**: 死区压缩后积分, "抗摩擦的常驻倾角"
     *   就攒不出来 → 站不住 (跟 speed_pid / position_pid 踩过的坑一样)。 */
    float p_err = err;
    float abs_err = fabsf(err);
    if (abs_err <= HOLD_PID_V_DEADBAND) {
        p_err = 0.0f;
    } else if (abs_err < HOLD_PID_V_DEADBAND + HOLD_PID_V_SOFT_BAND) {
        float t = (abs_err - HOLD_PID_V_DEADBAND) / HOLD_PID_V_SOFT_BAND;
        p_err = (err > 0.0f) ? t : -t;
    }

    /* --- I 项 + 抗积分饱和 ---
     * 先用**上一拍的积分**算输出, 判断要不要继续攒:
     * 输出已经顶在倾角上限、而误差还在同向 → 再攒就是积分饱和,
     * 等误差反号时车会先"退出饱和"走一段 → 表现为冲过头再倒回来。 */
    float raw = s_kp * p_err + s_ki * s_i;

    int saturated = ((raw >=  HOLD_PID_LEAN_MAX && err > 0.0f) ||
                     (raw <= -HOLD_PID_LEAN_MAX && err < 0.0f));
    if (!saturated) {
        s_i += err * dt;
        if (s_i >  HOLD_PID_I_MAX) s_i =  HOLD_PID_I_MAX;
        if (s_i < -HOLD_PID_I_MAX) s_i = -HOLD_PID_I_MAX;
    }

    raw = s_kp * p_err + s_ki * s_i;

    if (raw >  HOLD_PID_LEAN_MAX) raw =  HOLD_PID_LEAN_MAX;
    if (raw < -HOLD_PID_LEAN_MAX) raw = -HOLD_PID_LEAN_MAX;

    return hold_pid_shape(dt, raw);
}
