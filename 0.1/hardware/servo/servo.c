/**
 * @file servo.c
 * @brief 舵机控制驱动实现 (标准 PWM 舵机, 50Hz, 1-2ms 脉宽)
 * @note  - 双路独立目标角度 (用户操作的"逻辑角度")
 *        - 舵机2 物理反向安装: 实际硬件角度 = 180 - 逻辑角度
 *        - 后台任务 servo_keep_alive_task 每 SERVO_KEEP_ALIVE_MS ms
 *          自动重新同步硬件, 跨页不失力/不跳变
 * @author ESP32 机械狗项目
 * @date 2026-10-03
 */

#include "servo.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SERVO";

/* 舵机 GPIO 与 LEDC 通道映射表 */
static const struct {
    gpio_num_t      gpio;
    ledc_channel_t  channel;
} servo_map[SERVO_2 + 1] = {
    [SERVO_1] = { .gpio = SERVO_1_GPIO, .channel = SERVO_1_CHANNEL },
    [SERVO_2] = { .gpio = SERVO_2_GPIO, .channel = SERVO_2_CHANNEL },
};

/* 两路舵机独立的目标角度 (用户操作的"逻辑角度", UI 显示这个值) */
static uint8_t s_angle[SERVO_2 + 1] = {
    [SERVO_1] = SERVO_DEFAULT_ANGLE,
    [SERVO_2] = SERVO_DEFAULT_ANGLE,
};

/**
 * @brief 将逻辑角度转换为舵机 ID 对应的物理角度
 * @param id 舵机 ID
 * @param logic_angle 逻辑角度 (0-180)
 * @return 物理角度 (0-180)
 */
static uint8_t servo_logic_to_phys(servo_id_t id, uint8_t logic_angle)
{
    if (id == SERVO_1) {
        return logic_angle;
    }
    /* 舵机2 反向安装: 物理 = 180 - 逻辑 */
    int phys = (int)SERVO_MAX_ANGLE - (int)logic_angle;
    if (phys < 0) phys = 0;
    if (phys > SERVO_MAX_ANGLE) phys = SERVO_MAX_ANGLE;
    return (uint8_t)phys;
}

/**
 * @brief 将舵机 ID 的逻辑角度写到硬件 (经逻辑→物理映射)
 */
static esp_err_t servo_write_hw(servo_id_t id, uint8_t logic_angle)
{
    uint8_t  phys = servo_logic_to_phys(id, logic_angle);
    uint32_t duty = servo_angle_to_duty(phys);
    esp_err_t ret = ledc_set_duty(SERVO_PWM_MODE, servo_map[id].channel, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(SERVO_PWM_MODE, servo_map[id].channel);
}

/**
 * @brief 将脉宽(us)转换为 LEDC 占空比
 * @param pulse_us 脉宽(微秒)
 * @return LEDC 占空比值
 */
static uint32_t servo_pulse_to_duty(uint16_t pulse_us)
{
    /* 周期 = 1/50Hz = 20ms = 20000us, 14bit 分辨率 */
    uint32_t max_duty = (1 << 14) - 1;
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

    for (int i = 0; i <= SERVO_2; i++) {
        ledc_channel_config_t ch_cfg = {
            .channel    = servo_map[i].channel,
            .duty       = 0,
            .gpio_num   = servo_map[i].gpio,
            .speed_mode = SERVO_PWM_MODE,
            .timer_sel  = SERVO_PWM_TIMER,
        };

        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "舵机 %d 通道配置失败", i + 1);
            return ret;
        }
    }

    /* 两路归位到默认角度 (逻辑角度) */
    s_angle[SERVO_1] = SERVO_DEFAULT_ANGLE;
    s_angle[SERVO_2] = SERVO_DEFAULT_ANGLE;
    servo_apply_both();

    ESP_LOGI(TAG, "舵机初始化成功 (PWM1=%d, PWM2=%d, 50Hz, 默认=%d°, 限位=%d~%d, S2反向@180°)",
             SERVO_1_GPIO, SERVO_2_GPIO, SERVO_DEFAULT_ANGLE,
             SERVO_LIMIT_MIN, SERVO_LIMIT_MAX);
    return ESP_OK;
}

/**
 * @brief 设置单路舵机角度 (限位 + 立即同步到该路硬件)
 */
esp_err_t servo_set_angle(servo_id_t id, uint8_t angle)
{
    if (id > SERVO_2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (angle < SERVO_LIMIT_MIN) angle = SERVO_LIMIT_MIN;
    if (angle > SERVO_LIMIT_MAX) angle = SERVO_LIMIT_MAX;
    s_angle[id] = angle;
    return servo_write_hw(id, angle);
}

/**
 * @brief 同时设置两路舵机角度 (限位 + 立即同步)
 */
esp_err_t servo_set_both(uint8_t angle)
{
    if (angle < SERVO_LIMIT_MIN) angle = SERVO_LIMIT_MIN;
    if (angle > SERVO_LIMIT_MAX) angle = SERVO_LIMIT_MAX;
    s_angle[SERVO_1] = angle;
    s_angle[SERVO_2] = angle;
    esp_err_t ret = servo_write_hw(SERVO_1, angle);
    if (ret != ESP_OK) return ret;
    return servo_write_hw(SERVO_2, angle);
}

/**
 * @brief 获取单路舵机当前角度
 */
uint8_t servo_get_angle(servo_id_t id)
{
    if (id > SERVO_2) {
        return 0;
    }
    return s_angle[id];
}

/**
 * @brief 把两路舵机当前角度同步到硬件
 */
esp_err_t servo_apply_both(void)
{
    for (int i = 0; i <= SERVO_2; i++) {
        esp_err_t ret = servo_write_hw((servo_id_t)i, s_angle[i]);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

/**
 * @brief 后台"保持运行"任务: 周期性把两路角度同步到硬件
 *        作用: 防止任何路径(包括其他模块误改)导致舵机失力/跳变
 */
static void servo_keep_alive_task(void *arg)
{
    (void)arg;
    while (1) {
        servo_apply_both();
        vTaskDelay(pdMS_TO_TICKS(SERVO_KEEP_ALIVE_MS));
    }
}

/**
 * @brief 启动后台"保持运行"任务
 */
esp_err_t servo_keep_alive_start(void)
{
    static bool started = false;
    if (started) return ESP_OK;
    started = true;

    BaseType_t ret = xTaskCreate(servo_keep_alive_task, "servo_keep",
                                  2048, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "后台舵机保持任务创建失败");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "后台舵机保持任务已启动 (周期 %dms)", SERVO_KEEP_ALIVE_MS);
    return ESP_OK;
}

/**
 * @brief 设置舵机脉宽 (直接控制物理输出, 不走限位)
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