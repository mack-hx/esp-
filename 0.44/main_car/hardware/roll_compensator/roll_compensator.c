/**
 * @file roll_compensator.c
 * @brief 横滚角姿态补偿实现
 */

#include "roll_compensator.h"
#include "servo_roll_table.h"
#include <math.h>

/* 补偿使能, 默认关闭: 灵敏度要等实测表填好才有意义 */
static bool s_enabled = false;

roll_stance_t roll_comp_get_stance(uint8_t servo_left, uint8_t servo_right)
{
    uint8_t avg_angle = (servo_left + servo_right) / 2;
    return (avg_angle <= ROLL_STANCE_THRESHOLD) ? ROLL_STANCE_STAND : ROLL_STANCE_SQUAT;
}

void roll_comp_init(void)
{
    s_enabled = false;
}

void roll_comp_set_enable(bool enable)
{
    s_enabled = enable;
}

bool roll_comp_is_enabled(void)
{
    return s_enabled;
}

void roll_comp_calculate(float roll_deg, uint8_t servo_left, uint8_t servo_right,
                         roll_comp_output_t *output)
{
    if (!output) return;

    output->roll_angle = roll_deg;
    output->stance     = roll_comp_get_stance(servo_left, servo_right);
    output->left_comp  = 0;
    output->right_comp = 0;

    /* 灵敏度: 车身横滚角变化 / 左右角度差变化 */
    output->gain = servo_roll_gain((servo_left + servo_right) * 0.5f);

    if (!s_enabled || fabsf(roll_deg) < ROLL_DEADZONE) return;
    if (fabsf(output->gain) < 1e-3f) return;   /* 表里没有可用数据 */

    /* 抵消当前横滚角需要的角度差 (°) */
    float need = fabsf(roll_deg / output->gain);
    if (need > ROLL_COMP_MAX) need = ROLL_COMP_MAX;
    int comp_abs = (int)(need + 0.5f);

    /* 实测: 收腿(角度增大)会抬高该侧, 因此把高侧那条腿伸长(角度减小) */
    if (roll_deg > 0) output->right_comp = (int8_t)-comp_abs;  /* 右侧高 */
    else              output->left_comp  = (int8_t)-comp_abs;  /* 左侧高 */
}
