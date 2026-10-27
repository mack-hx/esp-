/**
 * @file position_pid.c
 * @brief 位置环 (预留) 实现: 位置误差 → 目标轮速修正 (rpm)
 *
 * @note  ⚠ 本模块**当前没有接入控制流程** (见 position_pid.h 的说明):
 *   工程里没有里程计, 位置反馈拿不到, 所以 balance.c 不调用它。
 *   文件先建好, 接口定死, 等里程计做好了直接接。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#include "position_pid.h"
#include "pid.h"
#include <math.h>

/* 模块内部唯一的 PID 实例 (位置环) */
static bal_pid_t s_pid;

void position_pid_load_params(void)
{
    pid_set_gains(&s_pid,
                  POS_PID_KP, POS_PID_KI, 0.0f,
                  pid_i_max(POS_PID_KI, POS_PID_OUT_MAX),
                  POS_PID_OUT_MAX);
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
    if (fabsf(err) < POS_PID_ERR_DEADBAND) err = 0.0f;
    return pid_update(&s_pid, err, dt);
}
