/**
 * @file servo.c
 * @brief 舵机控制驱动实现 (标准 PWM 舵机, 50Hz, 1-2ms 脉宽)
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "servo.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "SERVO";

/* 舵机 GPIO 与 LEDC 通道映射表 */
static const struct {
    gpio_num_t gpio;
    ledc_channel_t channel;
} servo_map[SERVO_2 + 1] = {
    [SERVO_1] = { .gpio = SERVO_1_GPIO, .channel = SERVO_1_CHANNEL },
    [SERVO_2] = { .gpio = SERVO_2_GPIO, .channel = SERVO_2_CHANNEL },
};

/**
 * @brief 将脉宽(us)转换为 LEDC 占空比
 * @param pulse_us 脉宽(微秒)
 * @return LEDC 占空比值
 */
static uint32_t servo_pulse_to_duty(uint16_t pulse_us)
{
    // 周期 = 1/50Hz = 20ms = 20000us
    // 占空比 = pulse_us / 20000 * (2^14 - 1)
    uint32_t max_duty = (1 << 14) - 1;  // 14 bit 分辨率
    return (uint32_t)((uint64_t)pulse_us * max_duty / 20000);
}

/**
 * @brief 将角度转换为 LEDC 占空比
 */
uint32_t servo_angle_to_duty(uint8_t angle)
{
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    uint16_t pulse = SERVO_MIN_PULSE_US +
                     (uint32_t)(angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US))
                     / SERVO_MAX_ANGLE;
    return servo_pulse_to_duty(pulse);
}

/**
 * @brief 初始化所有舵机
 */
esp_err_t servo_init(void)
{
    // 配置 LEDC 定时器
    ledc_timer_config_t timer_cfg = {
        .duty_resolution = SERVO_PWM_RES,
        .freq_hz = SERVO_PWM_FREQ,
        .speed_mode = SERVO_PWM_MODE,
        .timer_num = SERVO_PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC 定时器配置失败");
        return ret;
    }

    // 配置每个舵机的 LEDC 通道
    for (int i = 0; i <= SERVO_2; i++) {
        ledc_channel_config_t ch_cfg = {
            .channel = servo_map[i].channel,
            .duty = 0,
            .gpio_num = servo_map[i].gpio,
            .speed_mode = SERVO_PWM_MODE,
            .timer_sel = SERVO_PWM_TIMER,
        };

        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "舵机 %d 通道配置失败", i + 1);
            return ret;
        }
    }

    // 默认设置到中位
    servo_set_angle(SERVO_1, 90);
    servo_set_angle(SERVO_2, 90);

    ESP_LOGI(TAG, "舵机初始化成功 (PWM1=%d, PWM2=%d, 50Hz)",
             SERVO_1_GPIO, SERVO_2_GPIO);
    return ESP_OK;
}

/**
 * @brief 设置舵机角度
 */
esp_err_t servo_set_angle(servo_id_t id, uint8_t angle)
{
    if (id > SERVO_2) {
        ESP_LOGE(TAG, "无效的舵机 ID: %d", id);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t duty = servo_angle_to_duty(angle);
    esp_err_t ret = ledc_set_duty(SERVO_PWM_MODE, servo_map[id].channel, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(SERVO_PWM_MODE, servo_map[id].channel);
}

/**
 * @brief 设置舵机脉宽
 */
esp_err_t servo_set_pulse(servo_id_t id, uint16_t pulse_us)
{
    if (id > SERVO_2) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t duty = servo_pulse_to_duty(pulse_us);
    esp_err_t ret = ledc_set_duty(SERVO_PWM_MODE, servo_map[id].channel, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(SERVO_PWM_MODE, servo_map[id].channel);
}

/**
 * @brief 释放舵机 (停止 PWM 输出)
 */
esp_err_t servo_release(servo_id_t id)
{
    if (id > SERVO_2) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ledc_stop(SERVO_PWM_MODE, servo_map[id].channel, 0);
    return ret;
}