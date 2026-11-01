/**
 * @file position_pid.c
 * @brief 位置保持环实现: 位置误差 (cm) + 平移速度 (cm/s) → 目标倾角 (°)
 * @author ESP32 平衡轮腿机器人
 */

#include "position_pid.h"
#include "param.h"
#include "pid.h"
#include <math.h>

/* 增益 (每周期从 param 表刷新); 0 = 这一项不参与运算 */
static float s_kp = 1.0f;                /* °/cm    弹簧刚度 */
static float s_kd = 0.3f;                /* °/(cm/s) 速度阻尼 */
static float s_ki = 0.1f;                /* °/(cm·s) */

/* 积分状态 (cm·s): ∫(位置误差)dt, 用来消掉站住之后的稳态位置偏差 */
static float s_i = 0.0f;

void position_pid_load_params(void)
{
    float kp = param_get_float(PARAM_POS_KP);
    float kd = param_get_float(PARAM_POS_KD);
    float ki = param_get_float(PARAM_POS_KI);

    /* 0 = 这一项不参与运算; pos_kp 与 pos_kd 都为 0 时本环输出恒 0
     * (等于关掉位置保持, 车被推了就不回位)。 */
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

float position_pid_update(float dt, float err_cm, float v_cmps)
{
    if (dt <= 0.0f) return 0.0f;

    float abs_err = fabsf(err_cm);
    float e;

    if (abs_err <= POS_PID_ERR_DEADBAND) {
        /* 死区内: 误差归零, 积分**衰减**而不是冻结
         * (冻结的话死区里会攒出一个常驻倾角 → 车一直往一个方向走) */
        e = 0.0f;
        s_i *= expf(-dt / POS_PID_I_DECAY_TAU);
    } else {
        /* 死区外: 有效误差 = 误差 − 死区宽度, 过渡带内平滑起步 (输出处处连续) */
        e = pid_soft_deadband(err_cm, POS_PID_ERR_DEADBAND,
                               POS_PID_ERR_SOFT_BAND);
    }

    /* --- 积分 + 抗积分饱和 ---
     * 输出已经顶在倾角上限而误差仍同向时冻结积分,
     * 避免误差反号时退出饱和导致冲过头。 */
    float raw = s_kp * e + s_ki * s_i - s_kd * v_cmps;

    int saturated = ((raw >=  0.9f * POS_PID_LEAN_MAX && err_cm > 0.0f) ||
                     (raw <= -0.9f * POS_PID_LEAN_MAX && err_cm < 0.0f));
    if (!saturated) {
        s_i += e * dt;
        if (s_i >  POS_PID_I_MAX) s_i =  POS_PID_I_MAX;
        if (s_i < -POS_PID_I_MAX) s_i = -POS_PID_I_MAX;
        raw = s_kp * e + s_ki * s_i - s_kd * v_cmps;
    }

    /* --- 软饱和 ---
     * 硬夹 (min/max) 会把控制器变成继电器: 输出在上限两侧来回跳,
     * 叠加执行延迟就是一圈又一圈的前后摆动, 停不下来。
     * tanh 平滑逼近上限, 输出处处连续, 越接近上限增益越小。 */
    return POS_PID_LEAN_MAX * tanhf(raw / POS_PID_LEAN_MAX);
}
