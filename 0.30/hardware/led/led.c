/**
 * @file led.c
 * @brief SK6812 RGB LED 控制驱动实现 (ESP32 RMT 直驱版)
 * @note  硬件改动记录 (2026-10-01):
 *        - SK6812 DIN 已从 74HC595 Q0 改接至 ESP32 GPIO9
 *        - 使用 ESP32-S3 RMT TX 通道 0 驱动
 *        - RMT 时钟 10MHz, 100ns 精度, 满足 SK6812 时序要求
 *        - 使用 ESP-IDF 内置 led_strip_encoder 自动生成 0码/1码/复位码
 *
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "led.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "led_strip_encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "LED";

/* ========== RMT 句柄 ========== */
// RMT TX 通道 (ESP32-S3 有 4 个 TX 通道, 使用通道 0)
static rmt_channel_handle_t led_chan = NULL;
// LED 编码器 (将字节数据编码为 SK6812 时序)
static rmt_encoder_handle_t led_encoder = NULL;

/* ========== LED 状态缓存 ========== */
// 每颗 LED 当前颜色
static led_color_t current_color[LED_NUM_MAX] = {
    [LED_1] = LED_COLOR_OFF
};
// 每颗 LED 当前亮度 (0~255)
static uint8_t current_brightness[LED_NUM_MAX] = {
    [LED_1] = 255
};
// 每颗 LED 当前开关状态
static bool current_on[LED_NUM_MAX] = {
    [LED_1] = false
};

/* ========== 预设颜色表 ========== */
static const led_color_t preset_colors[LED_PRESET_MAX] = {
    [LED_PRESET_OFF]     = LED_COLOR_OFF,
    [LED_PRESET_RED]     = LED_COLOR_RED,
    [LED_PRESET_GREEN]   = LED_COLOR_GREEN,
    [LED_PRESET_BLUE]    = LED_COLOR_BLUE,
    [LED_PRESET_YELLOW]  = LED_COLOR_YELLOW,
    [LED_PRESET_CYAN]    = LED_COLOR_CYAN,
    [LED_PRESET_MAGENTA] = LED_COLOR_MAGENTA,
    [LED_PRESET_WHITE]   = LED_COLOR_WHITE,
};

/* ========== 内部函数 ========== */

/**
 * @brief 计算带亮度的实际颜色 (R/G/B 各分量乘以亮度系数)
 * @param color 原始颜色
 * @param brightness 亮度 (0~255)
 * @return 应用亮度后的颜色
 */
static led_color_t apply_brightness(led_color_t color, uint8_t brightness)
{
    // 使用整数运算避免浮点, 亮度系数 = brightness / 255
    return (led_color_t){
        .r = (uint8_t)((color.r * brightness) / 255),
        .g = (uint8_t)((color.g * brightness) / 255),
        .b = (uint8_t)((color.b * brightness) / 255),
    };
}

/**
 * @brief 刷新指定 LED 的 RMT 输出 (将缓存状态发送到 SK6812)
 * @param id LED ID
 * @return ESP_OK 成功, 其他值失败
 */
static esp_err_t flush_led(led_id_t id)
{
    if (id >= LED_NUM_MAX) return ESP_ERR_INVALID_ARG;

    // 计算实际输出颜色 (应用亮度)
    led_color_t out_color;
    if (current_on[id]) {
        out_color = apply_brightness(current_color[id], current_brightness[id]);
    } else {
        out_color = LED_COLOR_OFF;
    }

    // SK6812 数据格式: GRB (注意: 不是 RGB!)
    // 高位先发 (MSB first)
    uint8_t led_data[3] = { out_color.g, out_color.r, out_color.b };

    // 通过 RMT 发送
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,  // 不循环
    };

    esp_err_t ret = rmt_transmit(led_chan, led_encoder, led_data, sizeof(led_data), &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT 发送失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 等待发送完成
    ret = rmt_tx_wait_all_done(led_chan, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT 等待失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "LED[%d] RGB=(%d,%d,%d) 亮度=%d -> GRB=(%d,%d,%d)",
             id, current_color[id].r, current_color[id].g, current_color[id].b,
             current_brightness[id], led_data[0], led_data[1], led_data[2]);

    return ESP_OK;
}

/**
 * @brief 开机自检: 红 → 绿 → 蓝 → 灭, 每档 300ms
 * @note  用来肉眼确认 "SK6812 到底有没有接到 GPIO9" —— 灯不亮时,
 *        先看这三个颜色有没有依次出现, 再去查 App / 云端。
 */
void led_selftest(void)
{
    const struct { led_color_t c; const char *name; } seq[] = {
        { LED_COLOR_RED,   "红" },
        { LED_COLOR_GREEN, "绿" },
        { LED_COLOR_BLUE,  "蓝" },
    };

    for (int i = 0; i < 3; i++) {
        led_set_color(LED_1, seq[i].c);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    led_set_color(LED_1, LED_COLOR_OFF);
    ESP_LOGI(TAG, "开机自检结束 (红→绿→蓝)");
}

/* ========== 公开函数 ========== */

/**
 * @brief 初始化 LED 模块
 */
esp_err_t led_init(void)
{
    ESP_LOGI(TAG, "初始化 SK6812 LED (RMT 直驱, GPIO=%d)", LED_DATA_GPIO);

    // 1. 配置 RMT TX 通道
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,    // 默认时钟源 (通常 80MHz APB)
        .gpio_num = LED_DATA_GPIO,         // SK6812 DIN 引脚
        .mem_block_symbols = 64,           // 64 个 symbol, 足够一颗 LED
        .resolution_hz = LED_RMT_RESOLUTION_HZ,  // 10MHz, 100ns 精度
        .trans_queue_depth = LED_RMT_TRANS_QUEUE_SIZE,
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建 RMT TX 通道失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. 安装 LED 编码器
    led_strip_encoder_config_t encoder_config = {
        .resolution = LED_RMT_RESOLUTION_HZ,
    };

    ret = rmt_new_led_strip_encoder(&encoder_config, &led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建 LED 编码器失败: %s", esp_err_to_name(ret));
        rmt_del_channel(led_chan);
        led_chan = NULL;
        return ret;
    }

    // 3. 启用 RMT 通道
    ret = rmt_enable(led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启用 RMT 通道失败: %s", esp_err_to_name(ret));
        rmt_del_encoder(led_encoder);
        rmt_del_channel(led_chan);
        led_encoder = NULL;
        led_chan = NULL;
        return ret;
    }

    // 4. 初始化缓存: 全黑
    for (int i = 0; i < LED_NUM_MAX; i++) {
        current_color[i] = LED_COLOR_OFF;
        current_brightness[i] = 255;
        current_on[i] = false;
    }

    // 5. 先发一次空数据 (让 SK6812 进入已知状态)
    flush_led(LED_1);

    ESP_LOGI(TAG, "SK6812 LED 初始化完成 (RMT通道0, 10MHz时钟)");
    return ESP_OK;
}

/**
 * @brief 反初始化 LED 模块
 */
esp_err_t led_deinit(void)
{
    if (led_chan) {
        rmt_disable(led_chan);
        rmt_del_encoder(led_encoder);
        rmt_del_channel(led_chan);
        led_encoder = NULL;
        led_chan = NULL;
    }
    ESP_LOGI(TAG, "SK6812 LED 已反初始化");
    return ESP_OK;
}

/**
 * @brief 设置 LED 颜色 (任意 RGB, 真实驱动)
 */
esp_err_t led_set_color(led_id_t id, led_color_t color)
{
    if (id >= LED_NUM_MAX) return ESP_ERR_INVALID_ARG;

    current_color[id] = color;

    // 颜色非纯黑则视为"亮"
    bool on = (color.r > 0) || (color.g > 0) || (color.b > 0);
    current_on[id] = on;

    return flush_led(id);
}

/**
 * @brief 设置 LED 预设颜色
 */
esp_err_t led_set_preset(led_id_t id, led_preset_t preset)
{
    if (id >= LED_NUM_MAX) return ESP_ERR_INVALID_ARG;
    if (preset >= LED_PRESET_MAX) {
        ESP_LOGE(TAG, "无效的预设颜色: %d", preset);
        return ESP_ERR_INVALID_ARG;
    }
    return led_set_color(id, preset_colors[preset]);
}

/**
 * @brief 设置 LED 亮度
 */
esp_err_t led_set_brightness(led_id_t id, uint8_t level)
{
    if (id >= LED_NUM_MAX) return ESP_ERR_INVALID_ARG;

    current_brightness[id] = level;
    return flush_led(id);
}

/**
 * @brief 设置 LED 颜色 + 亮度
 */
esp_err_t led_set_color_brightness(led_id_t id, led_color_t color, uint8_t brightness)
{
    if (id >= LED_NUM_MAX) return ESP_ERR_INVALID_ARG;

    current_color[id] = color;
    current_brightness[id] = brightness;

    // 颜色和亮度都非零则视为"亮"
    bool on = (color.r > 0) || (color.g > 0) || (color.b > 0);
    if (brightness == 0) on = false;
    current_on[id] = on;

    return flush_led(id);
}

/**
 * @brief 开关 LED
 */
esp_err_t led_set_on(led_id_t id, bool on)
{
    if (id >= LED_NUM_MAX) return ESP_ERR_INVALID_ARG;

    current_on[id] = on;
    return flush_led(id);
}

/**
 * @brief 获取 LED 当前开关状态
 */
bool led_get_on(led_id_t id)
{
    if (id >= LED_NUM_MAX) return false;
    return current_on[id];
}

/**
 * @brief 获取 LED 当前颜色
 */
led_color_t led_get_color(led_id_t id)
{
    if (id >= LED_NUM_MAX) return LED_COLOR_OFF;
    return current_color[id];
}

/**
 * @brief 获取 LED 当前亮度
 */
uint8_t led_get_brightness(led_id_t id)
{
    if (id >= LED_NUM_MAX) return 0;
    return current_brightness[id];
}

/**
 * @brief 将 RGB 颜色转换为单字节亮度 (取最大分量)
 */
uint8_t led_color_to_brightness(led_color_t color)
{
    uint8_t max_c = color.r;
    if (color.g > max_c) max_c = color.g;
    if (color.b > max_c) max_c = color.b;
    return max_c;
}