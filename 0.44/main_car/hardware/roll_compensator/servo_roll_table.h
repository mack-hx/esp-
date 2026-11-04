/**
 * @file servo_roll_table.h
 * @brief 舵机角度差 ↔ 车身横滚角 实测对照表
 *
 * @note 测量方法 (固件串口命令 sweep 自动跑, 见 user/rolltest):
 *   两腿平均角固定为 base, 左腿 = base - delta/2, 右腿 = base + delta/2,
 *   等车体静止后取 1 s 横滚角平均。delta 为正表示"左腿伸长"。
 *
 * @note 表只存实测原始值; 每组 (base) 以 delta=0 的点为零点,
 *       用最小二乘过原点拟合出该组的灵敏度 gain = Δroll / Δdelta。
 */

#ifndef SERVO_ROLL_TABLE_H
#define SERVO_ROLL_TABLE_H

#include <stdint.h>

/** 单个测量点 */
typedef struct {
    uint8_t base;      /* 两腿平均角度 (°) */
    int8_t  delta;     /* 角度差 (°): >0 = 左腿伸长 (左角 = base - delta/2) */
    float   roll_deg;  /* 实测横滚角 (°), 未扣零点 */
} servo_roll_point_t;

/**
 * @brief 实测表 (sweep 输出直接照抄)
 * @note  未测时 roll_deg 全为 0, 此时查表 gain = 0, 补偿自动不生效
 */
extern const servo_roll_point_t SERVO_ROLL_TABLE[];
extern const unsigned SERVO_ROLL_TABLE_SIZE;

/**
 * @brief 查某个基准角度下的灵敏度
 * @param base_angle 两腿平均角度 (°)
 * @return 灵敏度 (车身横滚角 ° / 左右角度差 °); 表内无数据时返回 0
 * @note 只在该组 delta=0 点存在时才能归一化, 否则返回 0
 */
float servo_roll_gain(float base_angle);

#endif // SERVO_ROLL_TABLE_H
