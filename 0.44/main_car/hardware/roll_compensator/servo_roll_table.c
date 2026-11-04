/**
 * @file servo_roll_table.c
 * @brief 舵机角度差 ↔ 车身横滚角 实测表与查表实现
 */

#include "servo_roll_table.h"
#include <stdbool.h>
#include <stdlib.h>

/* 实测数据: 由串口命令 sweep 输出, 按行照抄即可
 *   { base, delta, roll }
 * 未测量的行把 roll 填 0.0f; 某组 delta=0 点缺失时该组不参与计算 */
const servo_roll_point_t SERVO_ROLL_TABLE[] = {
    /* --- 站立 (base = 40°) --- */
    {  40,   0,  0.0f },   /* 零点: 两腿等高 */
    {  40,   4,  0.0f },
    {  40,   8,  0.0f },
    {  40,  12,  0.0f },
    {  40,  16,  0.0f },
    {  40,  20,  0.0f },

    /* --- 蹲下 (base = 100°) --- */
    { 100,   0,  0.0f },   /* 零点: 两腿等高 */
    { 100,   4,  0.0f },
    { 100,   8,  0.0f },
    { 100,  12,  0.0f },
    { 100,  16,  0.0f },
    { 100,  20,  0.0f },
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
    float roll0 = 0.0f;
    float num = 0.0f;   /* Σ delta·Δroll */
    float den = 0.0f;   /* Σ delta² */
    bool  has_zero = false;
    bool  has_other = false;

    *ok = false;

    for (unsigned i = 0; i < SERVO_ROLL_TABLE_SIZE; i++) {
        if (SERVO_ROLL_TABLE[i].base != base) continue;

        if (SERVO_ROLL_TABLE[i].delta == 0) {
            roll0 = SERVO_ROLL_TABLE[i].roll_deg;
            has_zero = true;
            continue;
        }

        /* 未测的点保持 0, 与零点相同即视为没数据 */
        if (SERVO_ROLL_TABLE[i].roll_deg == 0.0f) continue;

        float d = (float)SERVO_ROLL_TABLE[i].delta;
        num += d * (SERVO_ROLL_TABLE[i].roll_deg - roll0);
        den += d * d;
        has_other = true;
    }

    if (!has_zero || !has_other || den < 1e-6f) return 0.0f;

    *ok = true;
    return num / den;
}

float servo_roll_gain(float base_angle)
{
    /* 找出围绕 base_angle 的上下两组基准角 */
    bool   has_lo = false, has_hi = false;
    uint8_t lo = 0, hi = 0;

    for (unsigned i = 0; i < SERVO_ROLL_TABLE_SIZE; i++) {
        uint8_t b = SERVO_ROLL_TABLE[i].base;
        if (b <= base_angle && (!has_lo || b > lo)) { lo = b; has_lo = true; }
        if (b >= base_angle && (!has_hi || b < hi)) { hi = b; has_hi = true; }
    }
    if (!has_lo) { lo = hi; }
    if (!has_hi) { hi = lo; }

    bool ok_lo = false, ok_hi = false;
    float g_lo = group_gain(lo, &ok_lo);
    float g_hi = group_gain(hi, &ok_hi);

    if (!ok_lo && !ok_hi) return 0.0f;
    if (!ok_hi || lo == hi) return g_lo;
    if (!ok_lo) return g_hi;

    float t = (base_angle - (float)lo) / (float)(hi - lo);
    return g_lo + t * (g_hi - g_lo);
}
