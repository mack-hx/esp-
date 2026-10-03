/**
 * @file servo.h
 * @brief 舵机控制驱动 (标准 PWM 舵机, 50Hz, 1-2ms 脉宽)
 * @note  架构:
 *        - 内部维护两路独立目标角度 s_angle[id] (用户操作的"逻辑角度")
 *        - 舵机1 物理角度 = 逻辑角度
 *        - 舵机2 物理角度 = 180 - 逻辑角度 (反向安装, 调 80° 实际转 100°)
 *        - UI 显示的"servo_get_angle(SERVO_2)"返回的是逻辑角度, 不是物理角度
 *        - main.c 启动后台任务 servo_keep_alive_task, 周期把两路角度写到硬件
 *        - 切页 / 状态变化时舵机不会失力或跳变, 一直保持当前角度
 * @author ESP32 机械狗项目
 * @date 2026-10-03
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
#define SERVO_1_GPIO        13      // 舵机1 控制信号 (头部转向, 正向安装)
#define SERVO_2_GPIO        14      // 舵机2 控制信号 (其他关节, 反向安装)

/* ========== PWM 配置 ==========
 * 注意: 与 MOTOR 共用 LEDC 通道 (见 motor_driver.h).
 *   SERVO_1 = LEDC_CHANNEL_2, SERVO_2 = LEDC_CHANNEL_3 (TIMER_1, 50Hz)
 *   MOTOR_A = LEDC_CHANNEL_4, MOTOR_B = LEDC_CHANNEL_5 (TIMER_2, 20kHz)
 *   通道不能重叠 (重叠会导致 SERVO GPIO 被重绑到 MOTOR 引脚, 失去 50Hz 输出). */
#define SERVO_PWM_FREQ      50
#define SERVO_PWM_RES       LEDC_TIMER_14_BIT
#define SERVO_PWM_TIMER     LEDC_TIMER_1
#define SERVO_PWM_MODE      LEDC_LOW_SPEED_MODE

#define SERVO_1_CHANNEL     LEDC_CHANNEL_2
#define SERVO_2_CHANNEL     LEDC_CHANNEL_3

/* ========== 舵机参数 ========== */
#define SERVO_PERIOD_MS     20      // 周期 20ms (1/50Hz)
#define SERVO_MIN_PULSE_US  1000    // 最小脉宽 (0°)   1ms
#define SERVO_MAX_PULSE_US  2000    // 最大脉宽 (180°) 2ms
#define SERVO_MID_PULSE_US  1500    // 中位脉宽 (90°)  1.5ms

#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180

/* ========== 限位 / 默认 ==========
 * 限位: 所有角度设置都按这个范围 clamp (UI 显示的也是这个范围)
 * 默认: 上电 / 初始化 / servo_set_both() 都用这个值 */
#define SERVO_LIMIT_MIN     30
#define SERVO_LIMIT_MAX     100
#define SERVO_DEFAULT_ANGLE 80

/* 后台同步任务周期 (ms): 周期性把当前角度写到两路舵机硬件 */
#define SERVO_KEEP_ALIVE_MS 50

/* ========== 舵机 ID 定义 ========== */
typedef enum {
    SERVO_1 = 0,   // 舵机1 (头部转向, 正向安装)
    SERVO_2 = 1    // 舵机2 (其他关节, 反向安装, 物理角度 = 180 - 逻辑)
} servo_id_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化所有舵机 (LEDC 通道 + 两路归位到 SERVO_DEFAULT_ANGLE)
 * @note  本函数不会启动后台任务, 需要在 main 中调用 servo_keep_alive_start()
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t servo_init(void);

/**
 * @brief 设置单路舵机角度 (限位 SERVO_LIMIT_MIN~MAX, 立即同步到该路硬件)
 * @param id 舵机 ID
 * @param angle 角度 (SERVO_LIMIT_MIN~SERVO_LIMIT_MAX)
 * @return ESP_OK 成功
 */
esp_err_t servo_set_angle(servo_id_t id, uint8_t angle);

/**
 * @brief 同时设置两路舵机角度 (限位 + 立即同步)
 * @param angle 两路都用这个值
 * @return ESP_OK 成功
 */
esp_err_t servo_set_both(uint8_t angle);

/**
 * @brief 获取单路舵机当前角度
 * @param id 舵机 ID
 * @return 当前角度
 */
uint8_t servo_get_angle(servo_id_t id);

/**
 * @brief 把两路舵机当前角度同步到硬件
 * @note  后台任务周期性调用, 防止切页 / 异常时舵机失力或跳变
 * @return ESP_OK 成功
 */
esp_err_t servo_apply_both(void);

/**
 * @brief 启动后台"保持运行"任务 (周期性 servo_apply_both)
 * @note  必须在 servo_init 之后调用, 任务运行后舵机会一直保持当前角度输出
 * @return ESP_OK 成功
 */
esp_err_t servo_keep_alive_start(void);

/**
 * @brief 设置舵机脉宽 (微秒) - 直接控制物理输出, 不走限位
 * @param id 舵机 ID
 * @param pulse_us 脉宽 (500-2500us, 典型 1000-2000us)
 * @return ESP_OK 成功
 */
esp_err_t servo_set_pulse(servo_id_t id, uint16_t pulse_us);

/**
 * @brief 将角度转换为占空比值
 * @param angle 角度 (0-180°)
 * @return LEDC 占空比值
 */
uint32_t servo_angle_to_duty(uint8_t angle);

#endif // SERVO_H