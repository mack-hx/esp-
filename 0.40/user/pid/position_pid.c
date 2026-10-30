/**
 * @file position_pid.c
 * @brief 位置保持环实现: 位置误差 (cm) + 相对速度 (cm/s) → 速度修正量 (cm/s)
 * @author ESP32 平衡轮腿机器人
 */

#include "position_pid.h"
#include "param.h"
#include "pid.h"
#include <math.h>

/* 增益 (每周期从 param 表刷新); 0 = 这一项不参与运算 */
static float s_kp = 1.5f;                /* (cm/s)/cm     位置刚度 */
static float s_kd = 0.5f;                /* 无量纲         相对速度阻尼 */
static float s_ki = 0.0f;                /* (cm/s)/(cm·s) */

/* 积分状态 (cm·s): ∫(位置误差)dt */
static float s_i = 0.0f;

void position_pid_load_params(void)
{
    float kp = param_get_float(PARAM_POS_KP);
    float kd = param_get_float(PARAM_POS_KD);
    float ki = param_get_float(PARAM_POS_KI);

    s_kp = (kp > 0.0f) ? kp : 0.0f;
    s_kd = (kd > 0.0f) ? kd : 0.0f;
    s_ki = (ki > 0.0f) ? ki : 0.0f;
}

void position_pid_reset(void)
{
    s_i = 0.0f;
}

void position_pid_init(void)
{
    position_pid_load_params();
    position_pid_reset();
}

float position_pid_update(float dt, float err_cm, float v_rel_cmps)
{
    if (dt <= 0.0f) return 0.0f;

    float e;
    if (fabsf(err_cm) <= POS_PID_ERR_DEADBAND) {
        /* 死区内: 误差归零, 积分指数衰减 */
        e = 0.0f;
        s_i *= expf(-dt / POS_PID_I_DECAY_TAU);
    } else {
        e = pid_soft_deadband(err_cm, POS_PID_ERR_DEADBAND, POS_PID_ERR_SOFT_BAND);
    }

    /* 抗积分饱和: 输出已顶到上限且误差同向时冻结积分 */
    float raw = s_kp * e + s_ki * s_i - s_kd * v_rel_cmps;
    int saturated = ((raw >=  0.9f * POS_PID_OUT_MAX && err_cm > 0.0f) ||
                     (raw <= -0.9f * POS_PID_OUT_MAX && err_cm < 0.0f));
    if (!saturated) {
        s_i += e * dt;
        if (s_i >  POS_PID_I_MAX) s_i =  POS_PID_I_MAX;
        if (s_i < -POS_PID_I_MAX) s_i = -POS_PID_I_MAX;
        raw = s_kp * e + s_ki * s_i - s_kd * v_rel_cmps;
    }

    /* 软饱和: tanh 平滑逼近上限 */
    return POS_PID_OUT_MAX * tanhf(raw / POS_PID_OUT_MAX);
}
