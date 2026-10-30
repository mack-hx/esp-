/**
 * @file lqr.c
 * @brief LQR 状态反馈实现
 * @author ESP32 平衡轮腿机器人
 */

#include "lqr.h"
#include "pid.h"
#include "position_pid.h"
#include <math.h>

/* 当前位置/速度增益与回位减速度上限 (取正值), 默认等于 lqr.h 的 -LQR_KX / -LQR_KV / LQR_AMAX */
static float s_kx = -LQR_KX;
static float s_kv = -LQR_KV;
static float s_amax = LQR_AMAX;

void lqr_set_pos_gains(float kx, float kv, float amax)
{
    s_kx = (kx > 0.0f) ? kx : 0.0f;
    s_kv = (kv > 0.0f) ? kv : 0.0f;
    s_amax = amax;
}

/**
 * @brief 偏差 → 期望回位速度 (cm/s, 与偏差同号)
 * @param e 软化后的位置偏差 (cm)
 * @param p 线性区斜率 KX/KV (1/s)
 */
static float desired_return_speed(float e, float p)
{
    float ea = fabsf(e);
    float v = p * ea;
    if (s_amax > 0.0f) {
        float lin_dist = s_amax / (p * p);          /* 线性区与 sqrt 区的交界偏差 */
        if (ea > lin_dist) v = sqrtf(2.0f * s_amax * (ea - 0.5f * lin_dist));
    }
    return copysignf(fminf(v, LQR_VMAX), e);
}

/** @brief 速度项增益: |v| ≤ LQR_VEL_LO 为 0, ≥ LQR_VEL_HI 为 1, 之间 smoothstep */
static float vel_gain(float v)
{
    float a = fabsf(v);
    if (a <= LQR_VEL_LO) return 0.0f;
    if (a >= LQR_VEL_HI) return 1.0f;
    float t = (a - LQR_VEL_LO) / (LQR_VEL_HI - LQR_VEL_LO);
    return t * t * (3.0f - 2.0f * t);
}

float lqr_pos_lean(float err_cm, float v_cmps)
{
    v_cmps *= vel_gain(v_cmps);
    float e = pid_soft_deadband(err_cm, LQR_ERR_DEADBAND, LQR_ERR_SOFT_BAND);
    float raw;
    if (s_kv > 1e-6f && s_kx > 1e-6f) {
        raw = s_kv * (desired_return_speed(e, s_kx / s_kv) - v_cmps) / LQR_KP;
    } else {
        raw = (s_kx * e - s_kv * v_cmps) / LQR_KP;
    }
    return POS_PID_LEAN_MAX * tanhf(raw / POS_PID_LEAN_MAX);
}

/* 位置积分项当前值 (°) */
static float s_pos_i = 0.0f;

void lqr_pos_integral_reset(void)
{
    s_pos_i = 0.0f;
}

float lqr_pos_integral(float dt, float err_cm, float v_cmps)
{
    float ea = fabsf(err_cm);
    int accumulate = ea > LQR_ERR_DEADBAND && ea < LQR_POS_I_BAND &&
                     fabsf(v_cmps) < LQR_POS_I_VEL && (s_pos_i * err_cm) >= 0.0f;

    if (accumulate) {
        s_pos_i += LQR_POS_KI * err_cm * dt;
        s_pos_i = fminf(fmaxf(s_pos_i, -LQR_POS_I_MAX), LQR_POS_I_MAX);
    } else {
        int near_anchor = ea < LQR_POS_I_BAND && fabsf(v_cmps) < LQR_POS_I_VEL;
        s_pos_i *= expf(-dt / (near_anchor ? LQR_POS_I_RELEASE : LQR_POS_I_DECAY));
    }
    return s_pos_i;
}

float lqr_balance_out(float pitch_err, float gyro)
{
    return LQR_KP * pitch_err + LQR_KD * gyro;
}
