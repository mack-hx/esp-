/**
 * @file roll_compensator.c
 * @brief 横滚闭环实现 (PD + 弱积分, 输出左右舵机角度偏置)
 */

#include "roll_compensator.h"
#include <math.h>

static bool  s_enabled = false;
static float s_integral = 0.0f;   /* 积分项 (°·s 累积) */
static float s_prev_roll = 0.0f;  /* 上周期横滚角 (°), 用于微分 */
static float s_out = 0.0f;        /* 当前偏置 (°), 带变化率限制 */
static float s_no_resp = 0.0f;    /* 连续无响应时长 (s) */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 死区软化: |e| ≤ band 内按平方过渡, 输出处处连续 */
static float soft_deadband(float e)
{
    float a = fabsf(e);
    if (a <= ROLL_DEADZONE) return 0.0f;
    float x = (a - ROLL_DEADZONE) / ROLL_SOFT_BAND;
    if (x > 1.0f) x = 1.0f;
    float y = x * x * (3.0f - 2.0f * x);
    return (e > 0.0f) ? y * (a - 0.5f * ROLL_DEADZONE)
                      : -y * (a - 0.5f * ROLL_DEADZONE);
}

void roll_comp_init(void)
{
    roll_comp_reset();
    s_enabled = false;
}

void roll_comp_reset(void)
{
    s_integral = 0.0f;
    s_prev_roll = 0.0f;
    s_out = 0.0f;
    s_no_resp = 0.0f;
}

void roll_comp_set_enable(bool enable)
{
    if (!enable) {
        s_integral = 0.0f;
        s_prev_roll = 0.0f;
        s_no_resp = 0.0f;
    }
    s_enabled = enable;
}

bool roll_comp_is_enabled(void)
{
    return s_enabled;
}

void roll_comp_calculate(float dt, float roll_deg, roll_comp_output_t *output)
{
    if (!output) return;

    if (!isfinite(roll_deg)) roll_deg = 0.0f;
    if (!isfinite(dt) || dt <= 0.0f) dt = 0.01f;
    if (dt > 0.1f) dt = 0.1f;

    const bool valid = s_enabled && fabsf(roll_deg) < ROLL_FAULT_ANGLE;
    float target = 0.0f;

    if (valid) {
        float err = soft_deadband(roll_deg);

        /* 无响应看门狗: 出了力但横滚不动 = 该侧腿没受力, 冻结偏置和积分 */
        float droll_rate = (roll_deg - s_prev_roll) / dt;
        if (fabsf(s_out) > ROLL_RESP_MIN_OUT &&
            fabsf(droll_rate) < ROLL_RESP_MIN_RATE) {
            s_no_resp += dt;
        } else {
            s_no_resp = 0.0f;
        }
        const bool responsive = s_no_resp < ROLL_RESP_TIMEOUT;

        if (err != 0.0f && responsive) {
            s_integral += err * dt;
            float i_lim = ROLL_I_MAX_DEG / (ROLL_KI > 1e-6f ? ROLL_KI : 1.0f);
            s_integral = clampf(s_integral, -i_lim, i_lim);
        } else {
            /* 车身够平 或 该侧腿没受力: 积分缓慢泄放 (不锁存, 坡度变了能重新找平衡) */
            s_integral *= expf(-dt / ROLL_I_RELEASE_TAU);
        }

        float rate = (roll_deg - s_prev_roll) / dt;
        target = ROLL_KP * err + ROLL_KI * s_integral + ROLL_KD * rate;
    } else {
        s_integral = 0.0f;
        s_no_resp = 0.0f;
    }

    /* ROLL_COMP_SIGN 决定方向: 实测舵机角度增大抬高该侧, 故取 -1 —
     * roll > 0 (右侧高) 时右腿角度减小 (伸腿) 把右侧压下来 */
    target *= (float)ROLL_COMP_SIGN;
    target = clampf(target, -ROLL_COMP_MAX, ROLL_COMP_MAX);

    /* 变化率限制: 抬起快、收回慢, 避免横滚刚归零就把腿收掉 */
    float step = ((fabsf(target) > fabsf(s_out)) ? ROLL_SLEW_PER_S
                                                 : ROLL_RELEASE_PER_S) * dt;
    
    /* ⚠ 看门狗触发时冻结偏置，防止腿落地猛跳 */
    bool in_watchdog = s_enabled && fabsf(s_out) > ROLL_RESP_MIN_OUT &&
                       fabsf((roll_deg - s_prev_roll) / dt) < ROLL_RESP_MIN_RATE;
    if (!in_watchdog) {
        s_out += clampf(target - s_out, -step, step);
    }
    s_prev_roll = roll_deg;

    output->roll_angle = roll_deg;
    output->raw_out    = s_out;
    /* 正值: 右腿角度增大(收腿), 左腿角度减小(伸腿) */
    output->right_comp = (int8_t)lrintf(s_out);
    output->left_comp  = (int8_t)lrintf(-s_out);
}
