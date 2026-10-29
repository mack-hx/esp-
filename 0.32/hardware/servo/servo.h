/**
 * @file servo.h
 * @brief 舵机控制驱动 (标准 PWM 舵机, 50Hz, 1-2ms 脉宽)
 * @note  内部维护两路逻辑角度 s_angle[id]; 物理安装不对称, 由 servo_logic_to_phys()
 *        补偿, 同一逻辑角度机械上朝同一方向转。UI 显示与 servo_get_angle() 返回逻辑角度。
 * @note  main.c 启动后台任务周期性把角度写到硬件, 切页/状态变化时舵机不失力不跳变。
 * @author ESP32 机械狗项目
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "esp_err.h"

/* ========== 引脚配置 ==========
 * 编号统一为"物理左=1, 物理右=2": 舵机1 (左) = GPIO14 (正向舵机), 舵机2 (右) = GPIO13 (反向舵机) */
#define SERVO_1_GPIO        14      // 舵机1 控制信号 (物理左, 接正向舵机)
#define SERVO_2_GPIO        13      // 舵机2 控制信号 (物理右, 接反向舵机)

/* ========== PWM 配置 ==========
 * 与 MOTOR 共用 LEDC 硬件: SERVO 用 CHANNEL_2/3 (TIMER_1, 50Hz),
 * MOTOR 用 CHANNEL_4/5 (TIMER_2, 20kHz), 通道不能重叠。
 */
#define SERVO_PWM_FREQ      50
#define SERVO_PWM_RES       LEDC_TIMER_14_BIT
#define SERVO_PWM_TIMER     LEDC_TIMER_1
#define SERVO_PWM_MODE      LEDC_LOW_SPEED_MODE

#define SERVO_1_CHANNEL     LEDC_CHANNEL_2
#define SERVO_2_CHANNEL     LEDC_CHANNEL_3

/* ========== 舵机参数 ==========
 * 安装方向: GPIO14 = 正向, GPIO13 = 反向 (调 80° 实际转 100°)。
 * 物理映射: SERVO_1 → 180 - 逻辑; SERVO_2 不映射 (反向舵机自身反向, 净结果相同)。
 */
#define SERVO_MIN_PULSE_US  1000    // 最小脉宽 (0°)   1ms
#define SERVO_MAX_PULSE_US  2000    // 最大脉宽 (180°) 2ms

#define SERVO_MAX_ANGLE     180

/* ========== 限位 / 默认 ==========
 * 限位: 所有角度设置按此范围 clamp (UI 显示同范围)
 * 默认: 上电 / 初始化 / servo_set_both() 都用这个值
 * 行程: 伸腿 = SERVO_LIMIT_MIN(30), 蜷腿 = SERVO_LIMIT_MAX(120, 对应 2ms 脉宽物理极限) */
#define SERVO_LIMIT_MIN     30
#define SERVO_LIMIT_MAX     120
#define SERVO_DEFAULT_ANGLE 100

/* 后台同步任务周期 (ms): 周期性把当前角度写到两路舵机硬件 */
#define SERVO_KEEP_ALIVE_MS 20

/* ========== 舵机 ID 定义 ========== */
typedef enum {
    SERVO_1 = 0,   // 舵机1 (物理左, 接 GPIO14 的正向舵机)
    SERVO_2 = 1    // 舵机2 (物理右, 接 GPIO13 的反向舵机)
} servo_id_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化所有舵机 (LEDC 通道 + 两路归位到 SERVO_DEFAULT_ANGLE)
 * @note  不启动后台任务, 需在 main 中调用 servo_keep_alive_start()
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
 * @note  后台任务周期性调用
 * @return ESP_OK 成功
 */
esp_err_t servo_apply_both(void);

/**
 * @brief 启动后台"保持运行"任务 (周期性 servo_apply_both)
 * @note  必须在 servo_init 之后调用
 * @return ESP_OK 成功
 */
esp_err_t servo_keep_alive_start(void);

/**
 * @brief 获取舵机诊断信息 (用于调试)
 * @param[out] duty1 舵机1当前实际占空比
 * @param[out] duty2 舵机2当前实际占空比
 * @param[out] pulse1_us 舵机1当前脉宽(微秒)
 * @param[out] pulse2_us 舵机2当前脉宽(微秒)
 */
void servo_get_diagnostics(uint32_t *duty1, uint32_t *duty2,
                          uint32_t *pulse1_us, uint32_t *pulse2_us);

/**
 * @brief 将角度转换为占空比值
 * @param angle 角度 (0-180°)
 * @return LEDC 占空比值
 */
uint32_t servo_angle_to_duty(uint8_t angle);

#endif // SERVO_H