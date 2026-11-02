/**
 * @file roll_compensator.c
 * @brief 横滚角姿态补偿实现
 */

#include "roll_compensator.h"
#include <math.h>
#include <string.h>

/* ========== 内部状态 ========== */
static roll_comp_config_t s_config = {
    .gain       = ROLL_GAIN_DEFAULT,
    .deadzone   = ROLL_DEADZONE,
    .comp_max   = ROLL_COMP_MAX,
    .enabled    = false  /* 默认禁用，roll 漂移严重 */
};

/* ========== 内部函数 ========== */

/**
 * @brief 限幅函数
 */
static inline int8_t clamp_i8(int val, int8_t min, int8_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return (int8_t)val;
}

/**
 * @brief 判断姿态 (站立/蹲下)
 */
roll_stance_t roll_comp_get_stance(uint8_t servo_left, uint8_t servo_right) {
    uint8_t avg_angle = (servo_left + servo_right) / 2;
    return (avg_angle <= ROLL_STANCE_THRESHOLD) ? ROLL_STANCE_STAND : ROLL_STANCE_SQUAT;
}

/* ========== 公共接口 ========== */

void roll_comp_init(void) {
    // 配置已在静态初始化中完成
}

void roll_comp_set_gain(float gain) {
    if (gain >= 0.0f && gain <= 10.0f) {
        s_config.gain = gain;
    }
}

float roll_comp_get_gain(void) {
    return s_config.gain;
}

void roll_comp_set_enable(bool enable) {
    s_config.enabled = enable;
}

bool roll_comp_is_enabled(void) {
    return s_config.enabled;
}

void roll_comp_calculate(float roll_deg, uint8_t servo_left, uint8_t servo_right,
                         roll_comp_output_t *output) {
    if (!output) return;
    
    // 记录原始输入
    output->roll_angle = roll_deg;
    output->stance = roll_comp_get_stance(servo_left, servo_right);
    
    // 禁用状态或在死区内: 无补偿
    if (!s_config.enabled || fabsf(roll_deg) < s_config.deadzone) {
        output->left_comp = 0;
        output->right_comp = 0;
        return;
    }
    
    /* ========== 补偿策略 ==========
     * Roll 角正负约定: 待 MPU 实测确认, 暂假设:
     *   roll_deg > 0: 右侧高 (左倾)
     *   roll_deg < 0: 左侧高 (右倾)
     * 
     * 站立态 (≤70°): 高侧收腿 (角度↑)
     *   roll > 0 (右高): right_comp = +|roll|*gain, left_comp = 0
     *   roll < 0 (左高): left_comp = +|roll|*gain, right_comp = 0
     * 
     * 蹲下态 (>70°): 低侧伸腿 (角度↓)
     *   roll > 0 (右高左低): left_comp = -|roll|*gain, right_comp = 0
     *   roll < 0 (左高右低): right_comp = -|roll|*gain, left_comp = 0
     */
    
    float comp_raw = fabsf(roll_deg) * s_config.gain;
    int comp_abs = (int)(comp_raw + 0.5f);  // 四舍五入
    comp_abs = (comp_abs > s_config.comp_max) ? s_config.comp_max : comp_abs;
    
    if (output->stance == ROLL_STANCE_STAND) {
        // 站立态: 高侧收腿
        if (roll_deg > 0) {
            // 右侧高
            output->left_comp = 0;
            output->right_comp = comp_abs;
        } else {
            // 左侧高
            output->left_comp = comp_abs;
            output->right_comp = 0;
        }
    } else {
        // 蹲下态: 低侧伸腿
        if (roll_deg > 0) {
            // 右高左低
            output->left_comp = -comp_abs;
            output->right_comp = 0;
        } else {
            // 左高右低
            output->left_comp = 0;
            output->right_comp = -comp_abs;
        }
    }
}
