/**
 * @file hc595.c
 * @brief 74HC595D 串行移位寄存器驱动实现
 * @note  Q0 不再用于 SK6812, 但仍可作为通用输出:
 *        用 hc595_set_pin(HC595_Q0, state) 控制
 * @author ESP32 机械狗项目
 */

#include "hc595.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "HC595";

/* 当前输出状态 */
static uint8_t current_state = 0x00;

/**
 * @brief 初始化 74HC595
 */
esp_err_t hc595_init(void)
{
    // 配置 GPIO 为输出模式
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HC595_SCLK_GPIO) | 
                        (1ULL << HC595_RCLK_GPIO) | 
                        (1ULL << HC595_DS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO 配置失败");
        return ret;
    }
    
    // 初始化引脚状态
    gpio_set_level(HC595_SCLK_GPIO, 0);
    gpio_set_level(HC595_RCLK_GPIO, 0);
    gpio_set_level(HC595_DS_GPIO, 0);

    // 清除所有输出
    hc595_clear_all();

    ESP_LOGI(TAG, "74HC595 初始化成功 (SCLK=%d, RCLK=%d, DS=%d)",
             HC595_SCLK_GPIO, HC595_RCLK_GPIO, HC595_DS_GPIO);

    return ESP_OK;
}

/**
 * @brief 写入 8 位数据到 74HC595
 * @note  74HC595 时序: 每次 SRCLK↑, SER 数据进入 QA(第一级),
 *        之前的 QA→QB→...→QH→QH'。RCLK↑ 锁存。
 *        因此必须**先发 bit0 (LSB)**, 8 次后 QA=bit0, QB=bit1, ..., QH=bit7。
 *        按 i=7→0 发送 (MSB first) 会导致 QF=QB+4, 与代码命名错位!
 */
esp_err_t hc595_write_byte(uint8_t data)
{
    // 逐位移位 (LSB first, 与 74HC595 QA→QH 物理顺序对齐)
    for (int i = 0; i < 8; i++) {
        // 设置数据位 (bit0 先发)
        gpio_set_level(HC595_DS_GPIO, (data >> i) & 0x01);

        // 产生串行时钟脉冲
        gpio_set_level(HC595_SCLK_GPIO, 1);
        esp_rom_delay_us(1);  // 短延迟确保时序
        gpio_set_level(HC595_SCLK_GPIO, 0);
        esp_rom_delay_us(1);
    }

    // 产生锁存时钟脉冲,更新输出
    gpio_set_level(HC595_RCLK_GPIO, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HC595_RCLK_GPIO, 0);

    // 更新当前状态
    current_state = data;

    return ESP_OK;
}

/**
 * @brief 设置单个输出引脚
 */
esp_err_t hc595_set_pin(hc595_pin_t pin, bool state)
{
    if (pin > HC595_Q7) {
        ESP_LOGE(TAG, "无效的引脚编号: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (state) {
        current_state |= (1 << pin);   // 设置位
    } else {
        current_state &= ~(1 << pin);  // 清除位
    }

    return hc595_write_byte(current_state);
}
/**
 * @brief 清除所有输出
 */
esp_err_t hc595_clear_all(void)
{
    return hc595_write_byte(0x00);
}

/**
 * @brief 设置电磁铁 1 输出 (Q2, 物理位 bit2 = 0x04)
 */
esp_err_t hc595_set_magnet1(bool state)
{
    return hc595_set_pin(HC595_Q2, state);
}

/**
 * @brief 设置电磁铁 2 输出 (Q1, 物理位 bit1 = 0x02)
 */
esp_err_t hc595_set_magnet2(bool state)
{
    return hc595_set_pin(HC595_Q1, state);
}
