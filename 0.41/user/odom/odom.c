/**
 * @file odom.c
 * @brief 里程计实现: 编码器脉冲 → 行进距离 (cm) + 平移速度 (cm/s)
 * @author ESP32 平衡轮腿机器人
 */

#include "odom.h"
#include "param.h"
#include "encoder.h"
#include <math.h>

/* 累计位移 (cm, 前进为正); 相对最后一次 odom_reset() */
static float s_pos_cm = 0.0f;

/* 平移速度 (cm/s, 前进为正): 最近 ODOM_VEL_N 个控制周期的位移增量求平均 */
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
 * @note  距离 = 脉冲数 / (每转脉冲数 × 减速比) × 轮子周长
 *        每转脉冲数 = 44 (硬件常数, 装在电机轴上)
 *        减速比     = gear (参数表, 默认 30 → 轮子转一圈 1320 个脉冲)
 *        轮子周长   = π × 轮径 (参数表 wheel_dia, 本机 6.5 cm)
 *        → 1 个脉冲 ≈ π×6.5/1320 ≈ 0.0155 cm
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
