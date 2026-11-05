/**
 * @file motor_driver.c
 * @brief TB6612FNG 双路直流电机驱动实现
 * @note  方向控制引脚由 74HC595 提供, 此处通过调用 hc595 API 控制
 * @author ESP32 机械狗项目
 */

#include "motor_driver.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hc595.h"

static const char *TAG = "MOTOR";

/* 电机 GPIO 与 LEDC 通道映射 */
static const struct {
    gpio_num_t      gpio;
    ledc_channel_t  channel;
} motor_map[MOTOR_B + 1] = {
    [MOTOR_A] = { .gpio = MOTOR_PWMA_GPIO, .channel = MOTOR_A_CHANNEL },
    [MOTOR_B] = { .gpio = MOTOR_PWMB_GPIO, .channel = MOTOR_B_CHANNEL },
};

/* 当前方向缓存 */
static motor_dir_t current_dir[MOTOR_B + 1] = {
    [MOTOR_A] = MOTOR_DIR_BRAKE,
    [MOTOR_B] = MOTOR_DIR_BRAKE,
};

/* 直驱档位表: 20% / 40% / 60% / 80%
 * 整个工程只有这一张档位表 —— UI 编码器页的调速档位、App 的速度档位、
 * 遥控直驱用的 PWM 都取自这里, 保证三处永远一致。 */
const int8_t motor_spd_levels[MOTOR_SPD_LEVEL_COUNT] = { 20, 40, 60, 80 };

/**
 * @brief 把任意百分比吸附到最近的合法档位 (20/40/60/80)
 * @param pct 目标百分比 (0~100)
 * @return 合法档位百分比; 0 (停止) 原样返回
 */
int8_t motor_spd_level_snap(int8_t pct)
{
    if (pct <= 0) return 0;
    if (pct > 100) pct = 100;

    int8_t best = motor_spd_levels[0];
    int16_t best_diff = (int16_t)(pct - best);
    if (best_diff < 0) best_diff = -best_diff;

    for (int i = 1; i < MOTOR_SPD_LEVEL_COUNT; i++) {
        int16_t diff = (int16_t)(pct - motor_spd_levels[i]);
        if (diff < 0) diff = -diff;
        if (diff < best_diff) {
            best_diff = diff;
            best = motor_spd_levels[i];
        }
    }
    return best;
}

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
 * @brief 应用死区补偿后的占空比
 * @param duty      原始占空比 (0~1023)
 * @param deadzone  最小启动占空比 (0=不补偿)
 * @return 补偿后的占空比
 *
 * @note  补偿方式为加上 (duty = |duty| + dz): 硬件死区 D 被抵消成 max(0, D − dz),
 *        出力与控制器输出成线性关系。0 输出与制动不受影响。
 *        dz 应≈硬件真实死区 D: 偏小则补偿不够, 偏大则非零输出自带多余固定力矩。
 */
static uint32_t motor_apply_deadzone(uint32_t duty, int8_t deadzone)
{
    if (duty == 0) return 0;                 /* 真停: 不补偿 */
    if (deadzone <= 0) return duty;

    uint32_t dz = (uint32_t)(((uint32_t)deadzone * ((1 << 10) - 1)) / MOTOR_SPEED_MAX);
    /* "加上": 占空比往上抬 dz, 抵消硬件死区。
     * 注意方向已经在 motor_speed_to_duty() 里变成绝对值了 (duty 只有一个通道),
     * 所以这里直接加即可, 正反方向对称。 */
    uint32_t out = duty + dz;
    uint32_t max_duty = (1 << 10) - 1;
    return (out > max_duty) ? max_duty : out;
}

/* 当前死区设置 (%) 由 balance.c 写入; 默认 0 = 不补偿, 保持旧行为不变 */
static int8_t s_motor_deadzone = 0;

/**
 * @brief 应用方向到 74HC595
 * @note  硬件映射: 电机 A (PWMA=GPIO15) AIN1=Q5, AIN2=Q6;
 *        电机 B (PWMB=GPIO16) BIN1=Q3, BIN2=Q4。
 *        TB6612 真值表: (IN1=0, IN2=1)=正转, (IN1=1, IN2=0)=反转, 两者 0=制动
 */
static esp_err_t motor_apply_direction(motor_id_t motor, motor_dir_t dir)
{
    hc595_pin_t in1_pin = (motor == MOTOR_A) ? HC595_Q5 : HC595_Q3;
    hc595_pin_t in2_pin = (motor == MOTOR_A) ? HC595_Q6 : HC595_Q4;
    bool in1 = false, in2 = false;

    switch (dir) {
        case MOTOR_DIR_BRAKE:
            in1 = false; in2 = false;
            break;
        case MOTOR_DIR_FORWARD:
            in1 = false; in2 = true;
            break;
        case MOTOR_DIR_REVERSE:
            in1 = true; in2 = false;
            break;
        default:
            ESP_LOGE(TAG, "无效的电机方向: %d", dir);
            return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = hc595_set_pin(in1_pin, in1);
    if (ret != ESP_OK) return ret;
    ret = hc595_set_pin(in2_pin, in2);
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
    /* 配置 LEDC 定时器 */
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

    /* 配置两个电机的 LEDC 通道 */
    for (int i = 0; i <= MOTOR_B; i++) {
        ledc_channel_config_t ch_cfg = {
            .channel    = motor_map[i].channel,
            .duty       = 0,
            .gpio_num   = motor_map[i].gpio,
            .speed_mode = MOTOR_PWM_MODE,
            .timer_sel  = MOTOR_PWM_TIMER,
        };

        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "电机 %c 通道配置失败", 'A' + i);
            return ret;
        }
    }

    /* 初始状态: 制动 + 零速 */
    for (int i = 0; i <= MOTOR_B; i++) {
        motor_apply_direction((motor_id_t)i, MOTOR_DIR_BRAKE);
        motor_set_speed((motor_id_t)i, 0);
    }

    ESP_LOGI(TAG, "电机驱动初始化成功 (PWMA=%d, PWMB=%d, %dkHz)",
             MOTOR_PWMA_GPIO, MOTOR_PWMB_GPIO, MOTOR_PWM_FREQ / 1000);
    return ESP_OK;
}
/**
 * @brief 设置单个电机速度
 */
esp_err_t motor_set_speed(motor_id_t motor, int8_t speed)
{
    if (motor > MOTOR_B) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 根据速度符号自动调整方向 */
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
    duty = motor_apply_deadzone(duty, s_motor_deadzone);
    esp_err_t ret = ledc_set_duty(MOTOR_PWM_MODE, motor_map[motor].channel, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(MOTOR_PWM_MODE, motor_map[motor].channel);
}

/**
 * @brief 设置电机起动死区补偿值
 * @param deadzone 最小启动占空比 (0~30, 单位 %; 0=不补偿)
 * @return ESP_OK 成功; ESP_ERR_INVALID_ARG 超出合法范围
 *
 * @note  只影响**非零**输出, 见 motor_apply_deadzone() 的说明。
 * @note  超出 MOTOR_DEADBAND_MAX 时自动夹到上限, 不报错 (滑块可能超范围)。
 */
esp_err_t motor_set_deadband(int8_t deadzone)
{
    if (deadzone < 0) deadzone = 0;
    if (deadzone > MOTOR_DEADBAND_MAX) deadzone = MOTOR_DEADBAND_MAX;
    s_motor_deadzone = deadzone;
    return ESP_OK;
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

    /* 制动走 ledc_set_duty(0) 直写, 不经过 motor_apply_deadzone(),
     *   死区补偿不会把制动抬成最低转速 */
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