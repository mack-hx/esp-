/**
 * @file odom.c
 * @brief 里程计实现: 编码器脉冲 → 行进距离 (cm) + 平移速度 (cm/s)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#include "odom.h"
#include "param.h"
#include "encoder.h"
#include <math.h>

/* 累计位移 (cm, 前进为正); 相对最后一次 odom_reset() */
static float s_pos_cm = 0.0f;

/* 平移速度 (cm/s, 前进为正): 最近 ODOM_VEL_N 个**控制周期**的位移增量求平均。
 *
 * ⚠ 为什么不用编码器轮速 (balance.c 的 s_speed_fwd):
 *   轮速是 300ms 滑动窗口, 滞后 ~150ms。阻尼相位落后 → 等价负阻尼 → 前后晃。
 * ⚠ 为什么不用"每 100ms 算一次、之后保持不变"的阶梯速度:
 *   阶梯信号让制动指令滞后整整一拍 (实测回位过冲 2~4 cm)。
 *   现在**每个控制周期 (10ms) 都更新**, 用 60ms 滑动窗口平滑:
 *   滞后从 ~100ms 降到 ~30ms, 同时量化台阶 = 0.0155/0.06 ≈ 0.26 cm/s,
 *   与锁位环死区 (0.3 cm/s) 同量级。 */
static float s_vel_cmps = 0.0f;

/* 一个控制周期的长度 (秒): balance.c 的 CONTROL_PERIOD_MS = 10 */
#define ODOM_TICK_S  0.01f

/* 速度滑动窗口的周期数 (6 × 10ms = 60ms) */
#define ODOM_VEL_N   6

/* 最近 ODOM_VEL_N 个周期的位移增量 (cm), 环形缓冲 */
static float s_vel_buf[ODOM_VEL_N];
static int   s_vel_idx = 0;

/**
 * @brief 每个编码器脉冲对应多少厘米
 * @return cm/pulse
 */
static float odom_cm_per_pulse(void)
{
    float gear = param_get_float(PARAM_ENCODER_GEAR);
    float dia  = param_get_float(PARAM_WHEEL_DIA);

    if (gear < 0.01f) gear = 0.01f;   /* 防除零 */
    if (dia  < 0.1f)  dia  = 0.1f;    /* 防除零 (轮径不可能这么小) */

    return (float)M_PI * dia / (ENCODER_PULSES_PER_REV * gear);
}

void odom_init(void)
{
    s_pos_cm = 0.0f;
    s_vel_cmps = 0.0f;
    s_vel_idx = 0;
    for (int i = 0; i < ODOM_VEL_N; i++) s_vel_buf[i] = 0.0f;
}

void odom_reset(void)
{
    s_pos_cm = 0.0f;
    s_vel_cmps = 0.0f;
    s_vel_idx = 0;
    for (int i = 0; i < ODOM_VEL_N; i++) s_vel_buf[i] = 0.0f;
}

void odom_update(int d_left, int d_right)
{
    /* 两轮取平均: 原地转向时两轮一正一负, 平均正好抵消 (车没有平移) */
    float pulses = ((float)d_left + (float)d_right) * 0.5f;
    float delta_cm = pulses * odom_cm_per_pulse();
    s_pos_cm += delta_cm;

    /* 速度 = 最近 60ms 的位移增量 / 60ms (滑动窗口, 每周期更新) */
    s_vel_buf[s_vel_idx] = delta_cm;
    s_vel_idx = (s_vel_idx + 1) % ODOM_VEL_N;

    float sum = 0.0f;
    for (int i = 0; i < ODOM_VEL_N; i++) sum += s_vel_buf[i];
    s_vel_cmps = sum / (ODOM_VEL_N * ODOM_TICK_S);
}

float odom_get_position_cm(void)
{
    return s_pos_cm;
}

float odom_get_velocity_cmps(void)
{
    return s_vel_cmps;
}
