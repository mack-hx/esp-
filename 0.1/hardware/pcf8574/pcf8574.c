/**
 * @file pcf8574.c
 * @brief PCF8574 I2C IO 扩展芯片驱动实现
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "pcf8574.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "PCF8574";

/* 按键事件回调 */
static pcf8574_btn_cb_t btn_callback = NULL;

/* 防抖参数 */
#define DEBOUNCE_MS         30      // 消抖时间 (ms)

/**
 * @brief 写入 1 字节 (用于 PCF8574 输出)
 */
static esp_err_t pcf8574_write_byte(uint8_t data)
{
    return i2c_master_write_to_device(PCF8574_I2C_NUM, PCF8574_I2C_ADDR,
                                      &data, 1, pdMS_TO_TICKS(1000));
}

/**
 * @brief 读取 1 字节 (含预写 0xFF 刷新 + 重试)
 * @note  PCF8574 是准双向 IO, 内部 100µA 上拉很弱.
 *       当某个引脚被按键拉低时, 公共 VCC 跌落会导致其他引脚被误读为低.
 *       解决办法:
 *         1. 读取前先写 0xFF 强制所有引脚为高电平 (刷新内部锁存器)
 *         2. 等待稳定 (10ms) 让上拉电流稳定
 *         3. 读取真实电平
 *         4. 如果读到 0x00 (全低, 多半是上拉太弱或 VCC 跌落), 视为全松开
 *            (返回 0xFF), 保守策略保护 UI 不被假信号触发
 *         5. 否则正常返回
 */
static esp_err_t pcf8574_read_byte(uint8_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    // 1. 强制所有引脚为高电平 (刷新内部锁存器)
    uint8_t all_high = 0xFF;
    esp_err_t ret = i2c_master_write_to_device(PCF8574_I2C_NUM, PCF8574_I2C_ADDR,
                                              &all_high, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;

    // 2. 等待 10ms 让上拉电流稳定 (主循环 50ms 周期, 多 10ms 不影响)
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. 读取真实电平
    ret = i2c_master_read_from_device(PCF8574_I2C_NUM, PCF8574_I2C_ADDR,
                                      data, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;

    // 4. 兜底: 如果读到 0x00 全低, 极有可能是 PCF8574 模块缺外部上拉
    //    (内部 100µA 不够强, 按下 1 个键时 VCC 跌落导致其他引脚被误判)
    //    视为全松开, 保守策略
    if (*data == 0x00) {
        *data = 0xFF;
    }
    return ESP_OK;
}

/**
 * @brief 初始化 I2C 总线 (PCF8574 专用)
 */
static esp_err_t pcf8574_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PCF8574_SDA_GPIO,
        .scl_io_num = PCF8574_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = PCF8574_I2C_FREQ,
    };

    esp_err_t ret = i2c_param_config(PCF8574_I2C_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 参数配置失败");
        return ret;
    }

    ret = i2c_driver_install(PCF8574_I2C_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 驱动安装失败");
        return ret;
    }

    ESP_LOGI(TAG, "I2C 初始化成功 (SCL=%d, SDA=%d, %dkHz)",
             PCF8574_SCL_GPIO, PCF8574_SDA_GPIO, PCF8574_I2C_FREQ / 1000);
    return ESP_OK;
}

/**
 * @brief 初始化 PCF8574
 */
esp_err_t pcf8574_init(void)
{
    esp_err_t ret = pcf8574_i2c_init();
    if (ret != ESP_OK) return ret;

    // 探测设备
    uint8_t dummy;
    ret = pcf8574_read_byte(&dummy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCF8574 设备未响应 (地址 0x%02X)", PCF8574_I2C_ADDR);
        return ret;
    }

    // 写 0xFF (高电平), 让所有引脚作为输入时处于上拉状态
    // 注: PCF8574 是准双向 IO, 输出 1 时为输入
    ret = pcf8574_write_byte(0xFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCF8574 初始化写入失败");
        return ret;
    }

    ESP_LOGI(TAG, "PCF8574 初始化成功 (INT=%d, 地址=0x%02X)",
             PCF8574_INT_GPIO, PCF8574_I2C_ADDR);
    return ESP_OK;
}

/**
 * @brief 读取所有引脚状态
 */
esp_err_t pcf8574_read_all(uint8_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    return pcf8574_read_byte(state);
}

/**
 * @brief 读取单个引脚状态
 */
esp_err_t pcf8574_read_pin(pcf8574_pin_t pin, bool *state)
{
    if (pin > PCF8574_P7 || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data;
    esp_err_t ret = pcf8574_read_byte(&data);
    if (ret != ESP_OK) return ret;

    *state = (data >> pin) & 0x01;
    return ESP_OK;
}

/**
 * @brief 检测按键是否按下
 */
esp_err_t pcf8574_is_pressed(btn_id_t id, bool *pressed)
{
    if (id > SW_K6 || pressed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return pcf8574_read_pin((pcf8574_pin_t)id, pressed);
}

/**
 * @brief 扫描按键状态 (含消抖)
 */
uint8_t pcf8574_scan_buttons(void)
{
    uint8_t state_now = 0;
    uint8_t state_last = 0;

    // 两次读取间隔 DEBOUNCE_MS, 仅当结果一致才认为有效
    if (pcf8574_read_byte(&state_now) != ESP_OK) return 0;
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    if (pcf8574_read_byte(&state_last) != ESP_OK) return 0;

    if (state_now == state_last) {
        return state_now;
    }
    return 0;  // 不稳定状态, 丢弃
}

/**
 * @brief 注册按键事件回调
 */
esp_err_t pcf8574_register_callback(pcf8574_btn_cb_t cb)
{
    if (cb == NULL) return ESP_ERR_INVALID_ARG;
    btn_callback = cb;
    return ESP_OK;
}

/**
 * @brief 中断处理函数 (由 ISR 调用)
 */
static void pcf8574_isr_handler(void *arg)
{
    // 中断只做标志位标记, 实际处理放到任务中
    static volatile bool pending = false;
    pending = true;
    (void)arg;
}

/**
 * @brief 启用中断
 */
esp_err_t pcf8574_enable_interrupt(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PCF8574_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,   // 下降沿触发
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    ret = gpio_isr_handler_add(PCF8574_INT_GPIO, pcf8574_isr_handler, NULL);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "PCF8574 中断已启用 (GPIO=%d)", PCF8574_INT_GPIO);
    return ESP_OK;
}