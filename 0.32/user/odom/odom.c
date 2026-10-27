/**
 * @file odom.c
 * @brief 里程计实现: 编码器脉冲 → 行进距离 (cm)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#include "odom.h"
#include "param.h"
#include "encoder.h"
#include <math.h>

/* 累计位移 (cm, 前进为正); 相对最后一次 odom_reset() */
static float s_pos_cm = 0.0f;

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
}

void odom_reset(void)
{
    s_pos_cm = 0.0f;
}

void odom_update(int d_left, int d_right)
{
    /* 两轮取平均: 原地转向时两轮一正一负, 平均正好抵消 (车没有平移) */
    float pulses = ((float)d_left + (float)d_right) * 0.5f;
    s_pos_cm += pulses * odom_cm_per_pulse();
}

float odom_get_position_cm(void)
{
    return s_pos_cm;
}
