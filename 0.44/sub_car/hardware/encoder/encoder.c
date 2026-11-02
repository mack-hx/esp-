/**
 * @file encoder.c
 * @brief 增量式正交编码器驱动实现 (ESP32-S3 PCNT 外设)
 * @author ESP32 机械狗项目
 */

#include "encoder.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"  // gpio_num_t 类型定义
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ENCODER";

/* PCNT 单元分配 (ESP32-S3 上 PCNT_UNIT_0~3 独立存在, 不要混用!) */
#define PCNT_UNIT_ENC1   PCNT_UNIT_0
#define PCNT_UNIT_ENC2   PCNT_UNIT_1

/* 编码器 GPIO 与 PCNT 单元映射 */
static const struct {
    gpio_num_t a_neg;       // A 相负 (实测低电平的信号线)
    gpio_num_t a_pos;       // A 相正 (实测高电平的信号线)
    pcnt_unit_t unit;
} encoder_map[ENCODER_2 + 1] = {
    // 编号统一为"物理左=1, 物理右=2"
    [ENCODER_1] = { .a_neg = ENC1_A_N_GPIO, .a_pos = ENC1_A_P_GPIO, .unit = PCNT_UNIT_ENC1 },
    [ENCODER_2] = { .a_neg = ENC2_B_N_GPIO, .a_pos = ENC2_B_P_GPIO, .unit = PCNT_UNIT_ENC2 },
};

/* 上次计数值快照 (用于 RPM) */
static int last_snapshot[ENCODER_2 + 1] = {0, 0};
static TickType_t last_snapshot_tick[ENCODER_2 + 1] = {0, 0};

/**
 * @brief 配置单个编码器的 PCNT 单元为四倍频正交解码 (A、B 两相的每个边沿都计数)
 * @note  通道 0: P 相计数, N 相判方向; 通道 1: N 相计数, P 相判方向。
 *        invert=true 时所有边沿的计数方向取反 (物理左轮装配方向与右轮相反)。
 */
static esp_err_t encoder_config_unit(encoder_id_t id)
{
    const bool invert = (id == ENCODER_1);
    const pcnt_count_mode_t inc = invert ? PCNT_COUNT_DEC : PCNT_COUNT_INC;
    const pcnt_count_mode_t dec = invert ? PCNT_COUNT_INC : PCNT_COUNT_DEC;

    pcnt_config_t cfg = {
        .pulse_gpio_num = encoder_map[id].a_pos,
        .ctrl_gpio_num = encoder_map[id].a_neg,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = inc,
        .neg_mode = dec,
        .counter_h_lim = ENCODER_PCNT_HIGH_LIMIT,
        .counter_l_lim = ENCODER_PCNT_LOW_LIMIT,
        .unit = encoder_map[id].unit,
        .channel = PCNT_CHANNEL_0,
    };

    esp_err_t ret = pcnt_unit_config(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器 %d PCNT 通道0 配置失败", id + 1);
        return ret;
    }

    cfg.pulse_gpio_num = encoder_map[id].a_neg;
    cfg.ctrl_gpio_num = encoder_map[id].a_pos;
    cfg.lctrl_mode = PCNT_MODE_REVERSE;
    cfg.hctrl_mode = PCNT_MODE_KEEP;
    cfg.pos_mode = inc;
    cfg.neg_mode = dec;
    cfg.channel = PCNT_CHANNEL_1;

    ret = pcnt_unit_config(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "编码器 %d PCNT 通道1 配置失败", id + 1);
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

    ESP_LOGI(TAG, "编码器初始化成功 (左轮=%d/%d, 右轮=%d/%d, 四倍频, 左轮计数方向已反转)",
             ENC1_A_N_GPIO, ENC1_A_P_GPIO,
             ENC2_B_N_GPIO, ENC2_B_P_GPIO);
    return ESP_OK;
}

/**
 * @brief 获取编码器当前计数值
 */
int encoder_get_count(encoder_id_t id)
{
    if (id > ENCODER_2) return 0;

    int16_t count = 0;
    pcnt_get_counter_value(encoder_map[id].unit, &count);
    return count;
}
/**
 * @brief 计算转速 (RPM)
 * @note  周期由内部 tick 快照自动推导 (对比上次调用),
 *        所以调用间隔最好固定 (UI 里是 150ms);
 *        interval_ms 仅用于校验间隔是否明显偏离, 偏离时返回 0 避免算出错误的瞬时值
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

    /* 调用间隔与预期差太多时, 这次不参与计算 (避免把长间隔的速度当瞬时速度) */
    uint32_t expect_tick = pdMS_TO_TICKS(interval_ms);
    if (expect_tick > 0 && (elapsed < expect_tick / 2 || elapsed > expect_tick * 2)) {
        return 0.0f;
    }

    /* 增量要用 int16 取模: PCNT 计数到 ±32767 会回绕,
     * 按 int 直接相减会算出 65535 的假增量 (一次假的巨大转速)。 */
    int delta = (int16_t)(cur_cnt - last_cnt);
    /* 转数 = 脉冲数 / 每转脉冲数, 再换算到每分钟 */
    float revs = (float)delta / ENCODER_PULSES_PER_REV;
    return revs * 60.0f / elapsed_sec;
}
