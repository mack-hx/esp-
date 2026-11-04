/**
 * @file servo_roll_table.c
 * @brief 舵机角度差 ↔ 车身横滚角 实测表与查表实现
 */

#include "servo_roll_table.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

/* 实测数据: 由串口命令 sweep 输出, 按行照抄即可
 *   { base, delta, roll }
 * 未测量的行把 roll 填 0.0f; 某组 delta=0 点缺失时该组不参与计算 */
const servo_roll_point_t SERVO_ROLL_TABLE[] = {
    /* --- base 40°: 固定左腿, 右腿加 --- */
    {  40,   0,  1.52f },
    {  40,  10,  6.17f },
    {  40,  20,  9.15f },
    {  40,  30, 12.16f },
    {  40,  40, 15.70f },

    /* --- base 40°: 固定右腿, 左腿加 --- */
    {  40,   0,  2.88f },
    {  40, -10,  0.12f },
    {  40, -20, -3.28f },
    {  40, -30, -6.72f },
    {  40, -40, -9.76f },

    /* --- base 100°: 固定左腿, 右腿减 --- */
    { 100,   0,  1.18f },
    { 100, -10, -2.58f },
    { 100, -20, -6.16f },
    { 100, -30, -9.93f },
    { 100, -40, -12.96f },

    /* --- base 100°: 固定右腿, 左腿减 --- */
    { 100,   0,  1.43f },
    { 100,  10,  5.94f },
    { 100,  20,  9.85f },
    { 100,  30, 13.43f },
    { 100,  40, 18.13f },
};

const unsigned SERVO_ROLL_TABLE_SIZE = sizeof(SERVO_ROLL_TABLE) / sizeof(SERVO_ROLL_TABLE[0]);

/**
 * @brief 求某一基准角度组的灵敏度 (最小二乘过原点)
 * @param base 两腿平均角度 (°)
 * @param[out] ok 该组数据是否可用
 * @return 灵敏度 (° roll / ° delta)
 */
static float group_gain(uint8_t base, bool *ok)
{
    float roll0_sum = 0.0f;  /* 零点横滚角累加 */
    int   n_zero    = 0;     /* 零点样本数 */
    float num = 0.0f;        /* Σ delta·Δroll */
    float den = 0.0f;        /* Σ delta² */
    bool  has_other = false;

    *ok = false;

    for (unsigned i = 0; i < SERVO_ROLL_TABLE_SIZE; i++) {
        if (SERVO_ROLL_TABLE[i].base != base) continue;
        if (SERVO_ROLL_TABLE[i].delta != 0) continue;
        roll0_sum += SERVO_ROLL_TABLE[i].roll_deg;
        n_zero++;
    }
    if (n_zero == 0) return 0.0f;

    /* 多次测得的零姿态取平均 */
    float roll0 = roll0_sum / (float)n_zero;

    for (unsigned i = 0; i < SERVO_ROLL_TABLE_SIZE; i++) {
        if (SERVO_ROLL_TABLE[i].base != base) continue;
        if (SERVO_ROLL_TABLE[i].delta == 0) continue;

        float d = (float)SERVO_ROLL_TABLE[i].delta;
        num += d * (SERVO_ROLL_TABLE[i].roll_deg - roll0);
        den += d * d;
        has_other = true;
    }

    if (!has_other || den < 1e-6f) return 0.0f;

    *ok = true;
    return num / den;
}

float servo_roll_gain(float base_angle)
{
    /* 取基准角最接近的一组, 不做跨组插值 (各组响应差一个量级) */
    bool  found = false;
    uint8_t best = 0;
    float best_d = 1e9f;

    for (unsigned i = 0; i < SERVO_ROLL_TABLE_SIZE; i++) {
        float d = fabsf(base_angle - (float)SERVO_ROLL_TABLE[i].base);
        if (d < best_d) {
            best_d = d;
            best = SERVO_ROLL_TABLE[i].base;
            found = true;
        }
    }
    if (!found) return 0.0f;

    bool ok = false;
    float g = group_gain(best, &ok);
    return ok ? g : 0.0f;
}
