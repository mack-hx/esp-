/**
 * @file position_pid.c
 * @brief 位置环实现: 位置误差 (cm) → 目标速度 (cm/s)
 * @author ESP32 平衡轮腿机器人
 */

#include "position_pid.h"
#include "pid.h"
#include "param.h"
#include <math.h>

/* 模块内部唯一的 PID 实例 (位置环) */
static bal_pid_t s_pid;

void position_pid_load_params(void)
{
    float kp = param_get_float(PARAM_POS_KP);
    float ki = param_get_float(PARAM_POS_KI);

    /* 增益为 0 时兜底: 位置环是可选功能, 但 0 也不该让车发疯。
     * 保持 0 就是"完全不纠偏", 相当于关掉位置环 (只留速度环)。 */
    float kp_eff = (kp > 0.001f) ? kp : 0.0f;
    float ki_eff = (ki > 0.001f) ? ki : 0.0f;

    /* 只改系数, 不清积分 (每周期都调, 用 pid_config 会让积分永远为 0)
     * kd 传 0: 位置环的阻尼由锁位速度环提供 (见 hold_pid.h), 不需要手写 D 项。 */
    pid_set_gains(&s_pid,
                  kp_eff, ki_eff, 0.0f,
                  pid_i_max(ki_eff, POS_PID_V_MAX),
                  POS_PID_V_MAX);
}

void position_pid_reset(void)
{
    pid_reset(&s_pid);
}

void position_pid_init(void)
{
    position_pid_load_params();
    position_pid_reset();
}

float position_pid_update(float dt, float err)
{
    float abs_err = fabsf(err);

    /* 误差在死区外? 决定要不要要求车移动 */
    int outside = (abs_err > POS_PID_ERR_DEADBAND);

    if (!outside) {
        /* 死区内: 误差归零, 同时让积分**衰减**而不是冻结
         * (跟速度环同一个坑, 见 position_pid.h 的说明) */
        err = 0.0f;
        pid_decay_integral(&s_pid, dt, POS_PID_I_DECAY_TAU);
    } else {
        /* 死区外: 有效误差 = 误差 − 死区宽度, 过渡带内平滑起步 (输出处处连续) */
        err = pid_soft_deadband(err, POS_PID_ERR_DEADBAND, POS_PID_ERR_SOFT_BAND);
    }

    /* P (+可选 I): 输出 = 目标速度 (cm/s), 已在 PID 内部限幅到 ±V_MAX */
    return pid_update(&s_pid, err, dt);
}
