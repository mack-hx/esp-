/**
 * @file roll_compensator.h
 * @brief 横滚角姿态补偿 (适应左右高度差地形)
 * @note  根据 Roll 角 + 当前舵机姿态, 自动调整左右腿长度保持平衡。
 *        站立时: 高侧收腿; 蹲下时: 低侧伸腿。
 * @author ESP32 机械狗项目
 */

#ifndef ROLL_COMPENSATOR_H
#define ROLL_COMPENSATOR_H

#include <stdint.h>
#include <stdbool.h>

/* ========== 姿态阈值 ========== */
#define ROLL_STANCE_THRESHOLD   70      // 站立/蹲下分界 (°)
#define ROLL_STAND_NOMINAL      40      // 标准站姿 (°)
#define ROLL_SQUAT_NOMINAL      100     // 标准蹲姿 (°)

/* ========== 补偿参数 ========== */
#define ROLL_DEADZONE           10.0f   // Roll 角死区 (°, 小于此值不补偿, 避免漂移触发)
#define ROLL_GAIN_DEFAULT       0.0f    // 默认补偿增益 (禁用补偿)

/* ========== 补偿限制 ========== */
#define ROLL_COMP_MAX           25      // 单侧最大补偿量 (°)

/**
 * @brief 姿态类型
 */
typedef enum {
    ROLL_STANCE_STAND = 0,  // 站立态 (≤70°)
    ROLL_STANCE_SQUAT = 1   // 蹲下态 (>70°)
} roll_stance_t;

/**
 * @brief 横滚角补偿器配置
 */
typedef struct {
    float gain;             // 补偿增益 (°补偿/°Roll)
    float deadzone;         // Roll 角死区 (°)
    int8_t comp_max;        // 单侧最大补偿量 (°)
    bool enabled;           // 补偿使能
} roll_comp_config_t;

/**
 * @brief 补偿输出结果
 */
typedef struct {
    int8_t left_comp;       // 左腿补偿量 (°, 正=收腿/角度增大)
    int8_t right_comp;      // 右腿补偿量 (°, 正=收腿/角度增大)
    roll_stance_t stance;   // 当前姿态
    float roll_angle;       // 当前 Roll 角 (°)
} roll_comp_output_t;

/**
 * @brief 初始化横滚角补偿器 (加载默认配置)
 */
void roll_comp_init(void);

/**
 * @brief 设置补偿增益
 * @param gain 补偿增益 (°补偿/°Roll, 建议 1.0~3.0)
 */
void roll_comp_set_gain(float gain);

/**
 * @brief 获取当前补偿增益
 * @return 补偿增益
 */
float roll_comp_get_gain(void);

/**
 * @brief 使能/禁用补偿
 * @param enable true=使能, false=禁用
 */
void roll_comp_set_enable(bool enable);

/**
 * @brief 获取补偿使能状态
 * @return true=使能, false=禁用
 */
bool roll_comp_is_enabled(void);

/**
 * @brief 计算横滚角补偿量
 * @param roll_deg Roll 角 (°, 正=右侧高/左倾, 负=左侧高/右倾, 待MPU实测确认)
 * @param servo_left 左腿当前角度 (°)
 * @param servo_right 右腿当前角度 (°)
 * @param[out] output 补偿输出结果
 * @note  输出的 left_comp/right_comp 可直接叠加到舵机目标角度:
 *        servo_set_angle(SERVO_1, base_angle + output.left_comp);
 */
void roll_comp_calculate(float roll_deg, uint8_t servo_left, uint8_t servo_right,
                         roll_comp_output_t *output);

/**
 * @brief 判断当前姿态 (站立/蹲下)
 * @param servo_left 左腿角度 (°)
 * @param servo_right 右腿角度 (°)
 * @return 姿态类型
 */
roll_stance_t roll_comp_get_stance(uint8_t servo_left, uint8_t servo_right);

#endif // ROLL_COMPENSATOR_H
