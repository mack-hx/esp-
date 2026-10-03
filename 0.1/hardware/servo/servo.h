/**
 * @file servo.h
 * @brief 舵机控制驱动 (标准 PWM 舵机, 50Hz, 1-2ms 脉宽)
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
#define SERVO_1_GPIO        13      // 舵机1 控制信号 (头部转向)
#define SERVO_2_GPIO        14      // 舵机2 控制信号 (其他关节)

/* ========== PWM 配置 ========== */
#define SERVO_PWM_FREQ      50      // 50Hz (20ms 周期)
#define SERVO_PWM_RES       LEDC_TIMER_14_BIT  // 占空比分辨率
#define SERVO_PWM_TIMER     LEDC_TIMER_1
#define SERVO_PWM_MODE      LEDC_LOW_SPEED_MODE

#define SERVO_1_CHANNEL     LEDC_CHANNEL_2
#define SERVO_2_CHANNEL     LEDC_CHANNEL_3

/* ========== 舵机参数 ========== */
#define SERVO_PERIOD_MS     20      // 周期 20ms
#define SERVO_MIN_PULSE_US  1000    // 最小脉宽 (对应 0°)   1ms
#define SERVO_MAX_PULSE_US  2000    // 最大脉宽 (对应 180°) 2ms
#define SERVO_MID_PULSE_US  1500    // 中位脉宽 (对应 90°)  1.5ms

#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180

/* 上电默认角度 (servo_init 归位 + UI 页面 s_angle 初值都用这个)
 * 注: 限幅范围由 UI 层 SERVO_LOGIC_MIN/MAX 单独控制, 此处只决定"初始摆位" */
#define SERVO_DEFAULT_ANGLE 85

/* ========== 舵机 ID 定义 ========== */
typedef enum {
    SERVO_1 = 0,   // 舵机1 (头部转向)
    SERVO_2 = 1    // 舵机2 (其他关节)
} servo_id_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化所有舵机
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t servo_init(void);

/**
 * @brief 设置舵机角度
 * @param id 舵机 ID
 * @param angle 角度 (0-180°)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t servo_set_angle(servo_id_t id, uint8_t angle);

/**
 * @brief 设置舵机脉宽 (微秒)
 * @param id 舵机 ID
 * @param pulse_us 脉宽 (500-2500us, 典型 1000-2000us)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t servo_set_pulse(servo_id_t id, uint16_t pulse_us);

/**
 * @brief 释放舵机 (停止 PWM 输出)
 * @param id 舵机 ID
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t servo_release(servo_id_t id);

/**
 * @brief 将角度转换为占空比值
 * @param angle 角度 (0-180°)
 * @return LEDC 占空比值
 */
uint32_t servo_angle_to_duty(uint8_t angle);

#endif // SERVO_H