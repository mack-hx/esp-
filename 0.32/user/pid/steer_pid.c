/**
 * @file steer_pid.c
 * @brief 转向环 (差速环) 实现: 左右轮速差 → 两轮输出差 (%)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#include "steer_pid.h"
#include "pid.h"

/* 模块内部唯一的 PID 实例 (转向环) */
static bal_pid_t s_pid;

void steer_pid_load_params(void)
{
    /* 差速环 (转向): 输出 = 两轮的差 (%); 正 = 左轮更快 */
    pid_set_gains(&s_pid,
                  STEER_PID_KP, STEER_PID_KI, 0.0f,
                  pid_i_max(STEER_PID_KI, STEER_PID_OUT_MAX),
                  STEER_PID_OUT_MAX);
}

void steer_pid_reset(void)
{
    pid_reset(&s_pid);
}

void steer_pid_init(void)
{
    steer_pid_load_params();
    steer_pid_reset();
}

float steer_pid_update(float dt, float diff, float target)
{
    float err = target - diff;
    return pid_update(&s_pid, err, dt);
}
