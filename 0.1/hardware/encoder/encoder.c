/**
 * @file encoder.c
 * @brief 增量式正交编码器驱动实现 (ESP32-S3 PCNT 外设)
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "encoder.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char *TAG = "ENCODER";

/* PCNT 单元分配 */
#define PCNT_UNIT_1   PCNT_UNIT_0
#define PCNT_UNIT_2   PCNT_UNIT_1

/* 编码器 GPIO 与 PCNT 单元映射 */
static const struct {
    gpio_num_t a_neg;       // A 相负 (实测低电平的信号线)
    gpio_num_t a_pos;       // A 相正 (实测高电平的信号线)
    pcnt_unit_t unit;
} encoder_map[ENCODER_2 + 1] = {
    // 注: 引脚配置说明中编码器1为 B相(41/42), 编码器2为 A相(45/46)
    // 在此映射中: a_neg 为 B-/A-, a_pos 为 B+/A+
    [ENCODER_1] = { .a_neg = ENC1_B_N_GPIO, .a_pos = ENC1_B_P_GPIO, .unit = PCNT_UNIT_1 },
    [ENCODER_2] = { .a_neg = ENC2_A_N_GPIO, .a_pos = ENC2_A_P_GPIO, .unit = PCNT_UNIT_2 },
};

/* 上次计数值快照 (用于 RPM) */
static int last_snapshot[ENCODER_2 + 1] = {0, 0};
static TickType_t last_snapshot_tick[ENCODER_2 + 1] = {0, 0};

/**
 * @brief 配置单个编码器的 PCNT 单元
 */
static esp_err_t encoder_config_unit(encoder_id_t id)
{
    pcnt_config_t cfg = {
        .pulse_gpio_num = encoder_map[id].a_pos,    // 脉冲信号
        .ctrl_gpio_num = encoder_map[id].a_neg,     // 控制信号 (方向)
        .lctrl_mode = PCNT_MODE_KEEP,               // 反转时计数保持不变 (仅 pulse 计)
        .hctrl_mode = PCNT_MODE_REVERSE,            // 高电平时反向计数
        .pos_mode = PCNT_COUNT_INC,                 // 正向沿递增
        .neg_mode = PCNT_COUNT_DIS,                 // 反向沿不计数
        .counter_h_lim = ENCODER_PCNT_HIGH_LIMIT,
        .counter_l_lim = ENCODER_PCNT_LOW_LIMIT,
        .unit = encoder_map[id].unit,
        .channel = PCNT_CHANNEL_0,
    };

    esp_err_t ret = pcnt_unit_config(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器 %d PCNT 配置失败", id + 1);
        return ret;
    }

    // 设置滤波 (最大 1023 个 APB 时钟周期, 此处用 100)
    pcnt_set_filter_value(encoder_map[id].unit, 100);
    pcnt_filter_enable(encoder_map[id].unit);

    // 暂停并清零
    pcnt_counter_pause(encoder_map[id].unit);
    pcnt_counter_clear(encoder_map[id].unit);

    // 开始计数
    pcnt_counter_resume(encoder_map[id].unit);

    return ESP_OK;
}

/**
 * @brief 初始化所有编码器
 */
esp_err_t encoder_init(void)
{
    for (int i = 0; i <= ENCODER_2; i++) {
        esp_err_t ret = encoder_config_unit((encoder_id_t)i);
        if (ret != ESP_OK) return ret;
    }

    last_snapshot[ENCODER_1] = 0;
    last_snapshot[ENCODER_2] = 0;
    last_snapshot_tick[ENCODER_1] = xTaskGetTickCount();
    last_snapshot_tick[ENCODER_2] = last_snapshot_tick[ENCODER_1];

    ESP_LOGI(TAG, "编码器初始化成功 (左轮=%d/%d, 右轮=%d/%d)",
             ENC1_B_N_GPIO, ENC1_B_P_GPIO,
             ENC2_A_N_GPIO, ENC2_A_P_GPIO);
    return ESP_OK;
}

/**
 * @brief 获取编码器当前计数值
 */
int encoder_get_count(encoder_id_t id)
{
    if (id > ENCODER_2) return 0;

    int count = 0;
    pcnt_get_counter_value(encoder_map[id].unit, &count);
    return count;
}

/**
 * @brief 获取编码器计数值并清除
 */
int encoder_get_and_clear(encoder_id_t id)
{
    if (id > ENCODER_2) return 0;

    int count = encoder_get_count(id);
    pcnt_counter_clear(encoder_map[id].unit);
    return count;
}

/**
 * @brief 清除编码器计数值
 */
esp_err_t encoder_clear(encoder_id_t id)
{
    if (id > ENCODER_2) return ESP_ERR_INVALID_ARG;

    pcnt_counter_pause(encoder_map[id].unit);
    pcnt_counter_clear(encoder_map[id].unit);
    pcnt_counter_resume(encoder_map[id].unit);
    return ESP_OK;
}

/**
 * @brief 计算转速 (RPM)
 */
float encoder_get_rpm(encoder_id_t id, uint32_t interval_ms)
{
    if (id > ENCODER_2) return 0.0f;

    TickType_t now = xTaskGetTickCount();
    TickType_t last = last_snapshot_tick[id];
    int last_cnt = last_snapshot[id];

    int cur_cnt = encoder_get_count(id);
    last_snapshot[id] = cur_cnt;
    last_snapshot_tick[id] = now;

    TickType_t elapsed = (now > last) ? (now - last) : 1;
    float elapsed_sec = (float)elapsed / configTICK_RATE_HZ;

    if (elapsed_sec <= 0.0f) return 0.0f;

    int delta = cur_cnt - last_cnt;
    // 转数 = 脉冲数 / 每转脉冲数
    float revs = (float)delta / ENCODER_PULSES_PER_REV;
    return revs * 60.0f / elapsed_sec;   // RPM
}

/**
 * @brief 计算累计转数
 */
float encoder_get_revolutions(encoder_id_t id)
{
    if (id > ENCODER_2) return 0.0f;
    int count = encoder_get_count(id);
    return (float)count / ENCODER_PULSES_PER_REV;
}