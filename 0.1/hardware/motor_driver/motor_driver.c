/**
 * @file motor_driver.c
 * @brief TB6612FNG 双路直流电机驱动实现
 * @note  方向控制引脚由 74HC595 提供, 此处通过调用 hc595 API 控制。
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "motor_driver.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hc595.h"

static const char *TAG = "MOTOR";

/* 电机 GPIO 与 LEDC 通道映射 */
static const struct {
    gpio_num_t gpio;
    ledc_channel_t channel;
} motor_map[MOTOR_B + 1] = {
    [MOTOR_A] = { .gpio = MOTOR_PWMA_GPIO, .channel = MOTOR_A_CHANNEL },
    [MOTOR_B] = { .gpio = MOTOR_PWMB_GPIO, .channel = MOTOR_B_CHANNEL },
};

/* 当前方向缓存 */
static motor_dir_t current_dir[MOTOR_B + 1] = {
    [MOTOR_A] = MOTOR_DIR_BRAKE,
    [MOTOR_B] = MOTOR_DIR_BRAKE,
};

/**
 * @brief 将速度 (-100~100) 转换为 LEDC 占空比
 */
static uint32_t motor_speed_to_duty(int8_t speed)
{
    if (speed < 0) speed = -speed;
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;
    uint32_t max_duty = (1 << 10) - 1;  // 10 bit 分辨率
    return (uint32_t)(((uint32_t)speed * max_duty) / MOTOR_SPEED_MAX);
}

/**
 * @brief 应用方向到 74HC595
 */
static esp_err_t motor_apply_direction(motor_id_t motor, motor_dir_t dir)
{
    // 通过 hc595 设置电机方向 (hc595 内已映射 Q1-Q4)
    esp_err_t ret = hc595_set_motor_dir((motor_id_t)(MOTOR_A + motor), dir);
    if (ret == ESP_OK) {
        current_dir[motor] = dir;
    }
    return ret;
}

/**
 * @brief 初始化电机驱动
 */
esp_err_t motor_driver_init(void)
{
    // 配置 LEDC 定时器
    ledc_timer_config_t timer_cfg = {
        .duty_resolution = MOTOR_PWM_RES,
        .freq_hz = MOTOR_PWM_FREQ,
        .speed_mode = MOTOR_PWM_MODE,
        .timer_num = MOTOR_PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC 定时器配置失败");
        return ret;
    }

    // 配置两个电机的 LEDC 通道
    for (int i = 0; i <= MOTOR_B; i++) {
        ledc_channel_config_t ch_cfg = {
            .channel = motor_map[i].channel,
            .duty = 0,
            .gpio_num = motor_map[i].gpio,
            .speed_mode = MOTOR_PWM_MODE,
            .timer_sel = MOTOR_PWM_TIMER,
        };

        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "电机 %c 通道配置失败", 'A' + i);
            return ret;
        }
    }

    // 初始状态: 制动 + 零速
    for (int i = 0; i <= MOTOR_B; i++) {
        motor_apply_direction((motor_id_t)i, MOTOR_DIR_BRAKE);
        motor_set_speed((motor_id_t)i, 0);
    }

    ESP_LOGI(TAG, "电机驱动初始化成功 (PWMA=%d, PWMB=%d, %dkHz)",
             MOTOR_PWMA_GPIO, MOTOR_PWMB_GPIO, MOTOR_PWM_FREQ / 1000);
    return ESP_OK;
}

/**
 * @brief 设置单个电机方向
 */
esp_err_t motor_set_direction(motor_id_t motor, motor_dir_t dir)
{
    if (motor > MOTOR_B) {
        return ESP_ERR_INVALID_ARG;
    }
    return motor_apply_direction(motor, dir);
}

/**
 * @brief 设置单个电机速度
 */
esp_err_t motor_set_speed(motor_id_t motor, int8_t speed)
{
    if (motor > MOTOR_B) {
        return ESP_ERR_INVALID_ARG;
    }

    // 根据速度符号自动调整方向
    motor_dir_t target_dir;
    if (speed > 0) {
        target_dir = MOTOR_DIR_FORWARD;
    } else if (speed < 0) {
        target_dir = MOTOR_DIR_REVERSE;
    } else {
        target_dir = MOTOR_DIR_BRAKE;
    }

    if (target_dir != current_dir[motor]) {
        esp_err_t ret = motor_apply_direction(motor, target_dir);
        if (ret != ESP_OK) return ret;
    }

    uint32_t duty = motor_speed_to_duty(speed);
    esp_err_t ret = ledc_set_duty(MOTOR_PWM_MODE, motor_map[motor].channel, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(MOTOR_PWM_MODE, motor_map[motor].channel);
}

/**
 * @brief 同时控制两个电机
 */
esp_err_t motor_set_speed_both(int8_t speed_a, int8_t speed_b)
{
    esp_err_t ret = motor_set_speed(MOTOR_A, speed_a);
    if (ret != ESP_OK) return ret;
    return motor_set_speed(MOTOR_B, speed_b);
}

/**
 * @brief 制动单个电机
 */
esp_err_t motor_brake(motor_id_t motor)
{
    if (motor > MOTOR_B) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ledc_set_duty(MOTOR_PWM_MODE, motor_map[motor].channel, 0);
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(MOTOR_PWM_MODE, motor_map[motor].channel);
    if (ret != ESP_OK) return ret;

    return motor_apply_direction(motor, MOTOR_DIR_BRAKE);
}

/**
 * @brief 制动两个电机
 */
esp_err_t motor_brake_all(void)
{
    esp_err_t ret = motor_brake(MOTOR_A);
    if (ret != ESP_OK) return ret;
    return motor_brake(MOTOR_B);
}