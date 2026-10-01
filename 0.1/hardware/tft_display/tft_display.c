/**
 * @file tft_display.c
 * @brief 128x160 RGB TFT 显示屏驱动实现 (ST7735/ST7789)
 * @note  硬件改动 (2026-10-01):
 *        - TFT RST 引脚已短接 VCC, 不再由 ESP32 控制
 *        - 上电复位依赖模块内部 POR (Power-On-Reset) 电路
 *        - 移除 tft_hard_reset() 调用, 移除 GPIO9 配置
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "tft_display.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TFT";

/* SPI 设备句柄 */
static spi_device_handle_t spi_dev = NULL;

/* 背光 PWM 配置 */
#define TFT_BLK_LEDC_TIMER   LEDC_TIMER_0
#define TFT_BLK_LEDC_CHANNEL LEDC_CHANNEL_0
#define TFT_BLK_LEDC_FREQ    5000
#define TFT_BLK_LEDC_RES     LEDC_TIMER_8_BIT

/**
 * @brief 写入 TFT 命令
 * @param cmd 命令字节
 */
static void tft_write_cmd(uint8_t cmd)
{
    gpio_set_level(TFT_DC_GPIO, 0);   // DC=0 表示命令
    gpio_set_level(TFT_CS_GPIO, 0);   // 拉低片选
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_polling_transmit(spi_dev, &t);
    gpio_set_level(TFT_CS_GPIO, 1);   // 拉高片选
}

/**
 * @brief 写入 TFT 数据
 * @param data 数据指针
 * @param len 数据长度
 */
static void tft_write_data_bytes(const uint8_t *data, uint32_t len)
{
    if (len == 0) return;
    gpio_set_level(TFT_DC_GPIO, 1);   // DC=1 表示数据
    gpio_set_level(TFT_CS_GPIO, 0);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_polling_transmit(spi_dev, &t);
    gpio_set_level(TFT_CS_GPIO, 1);
}

/**
 * @brief 初始化 SPI 总线
 */
static esp_err_t tft_spi_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = TFT_SDA_GPIO,
        .sclk_io_num = TFT_SCL_GPIO,
        .miso_io_num = -1,        // TFT 不需要 MISO
        // 一行像素 (128*2 = 256 字节) 作为一次 DMA 上限, 避免申请过大内存
        .max_transfer_sz = TFT_SCREEN_W * 2,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI 总线初始化失败");
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = TFT_SPI_FREQ,
        .mode = 0,
        .spics_io_num = -1,       // 使用 GPIO 手动控制 CS
        .queue_size = 7,
    };

    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI 设备添加失败");
        return ret;
    }

    ESP_LOGI(TAG, "SPI 初始化成功 (SCL=%d, SDA=%d, %dMHz)",
             TFT_SCL_GPIO, TFT_SDA_GPIO, TFT_SPI_FREQ / 1000000);
    return ESP_OK;
}

/**
 * @brief 初始化控制引脚 (DC, CS, BLK, 不再含 RST)
 */
static esp_err_t tft_gpio_init(void)
{
    gpio_config_t io_conf = {
        // RST_GPIO 已移除, 改用模块内部 POR
        .pin_bit_mask = (1ULL << TFT_DC_GPIO) |
                        (1ULL << TFT_CS_GPIO) |
                        (1ULL << TFT_BLK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    // 初始电平 (RST 不再控制)
    gpio_set_level(TFT_CS_GPIO, 1);
    gpio_set_level(TFT_DC_GPIO, 1);
    return ESP_OK;
}

/**
 * @brief 初始化背光 PWM
 */
static esp_err_t tft_backlight_pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .duty_resolution = TFT_BLK_LEDC_RES,
        .freq_hz = TFT_BLK_LEDC_FREQ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = TFT_BLK_LEDC_TIMER,
    };

    esp_err_t ls_ret = ledc_timer_config(&timer_cfg);
    if (ls_ret != ESP_OK) return ls_ret;

    ledc_channel_config_t ch_cfg = {
        .channel = TFT_BLK_LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = TFT_BLK_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = TFT_BLK_LEDC_TIMER,
    };

    ls_ret = ledc_channel_config(&ch_cfg);
    if (ls_ret != ESP_OK) return ls_ret;

    return ESP_OK;
}

/**
 * @brief ST7735S 完整初始化序列
 * @note  来源: 参考工程 LCD.c (江协科技/正点原子模板, 经 TI MSPM0 移植)
 *        RST 引脚已短接 VCC, 依赖模块内部 POR, 此处不再做软件复位
 */
static esp_err_t tft_st7735s_init_seq(void)
{
    // -------- Start Initial Sequence -------- //
    tft_write_cmd(0x11);  // SLPOUT - 必须先退出睡眠
    vTaskDelay(pdMS_TO_TICKS(120));

    // -------- Frame Rate -------- //
    tft_write_cmd(0xB1);
    tft_write_data_bytes((uint8_t[]){0x05, 0x3C, 0x3C}, 3);
    tft_write_cmd(0xB2);
    tft_write_data_bytes((uint8_t[]){0x05, 0x3C, 0x3C}, 3);
    tft_write_cmd(0xB3);
    tft_write_data_bytes((uint8_t[]){0x05, 0x3C, 0x3C, 0x05, 0x3C, 0x3C}, 6);

    tft_write_cmd(0xB4);  // Dot inversion
    tft_write_data_bytes((uint8_t[]){0x03}, 1);

    // -------- Power Sequence -------- //
    tft_write_cmd(0xC0);
    tft_write_data_bytes((uint8_t[]){0x28, 0x08, 0x04}, 3);
    tft_write_cmd(0xC1);
    tft_write_data_bytes((uint8_t[]){0xC0}, 1);
    tft_write_cmd(0xC2);
    tft_write_data_bytes((uint8_t[]){0x0D, 0x00}, 2);
    tft_write_cmd(0xC3);
    tft_write_data_bytes((uint8_t[]){0x8D, 0x2A}, 2);
    tft_write_cmd(0xC4);
    tft_write_data_bytes((uint8_t[]){0x8D, 0xEE}, 2);

    tft_write_cmd(0xC5);  // VCOM
    tft_write_data_bytes((uint8_t[]){0x1A}, 1);

    // -------- MADCTL: 扫描方向 + RGB/BGR 顺序 -------- //
    tft_write_cmd(0x36);
#if USE_HORIZONTAL == 0
    tft_write_data_bytes((uint8_t[]){0x00}, 1);
#elif USE_HORIZONTAL == 1
    tft_write_data_bytes((uint8_t[]){0xC0}, 1);
#elif USE_HORIZONTAL == 2
    tft_write_data_bytes((uint8_t[]){0x70}, 1);
#else
    tft_write_data_bytes((uint8_t[]){0xA0}, 1);
#endif

    // -------- Gamma Sequence -------- //
    tft_write_cmd(0xE0);
    tft_write_data_bytes((uint8_t[]){0x04, 0x22, 0x07, 0x0A, 0x2E, 0x30, 0x25, 0x2A,
                                     0x28, 0x26, 0x2E, 0x3A, 0x00, 0x01, 0x03, 0x13}, 16);
    tft_write_cmd(0xE1);
    tft_write_data_bytes((uint8_t[]){0x04, 0x16, 0x06, 0x0D, 0x2D, 0x26, 0x23, 0x27,
                                     0x27, 0x25, 0x2D, 0x3B, 0x00, 0x01, 0x04, 0x13}, 16);

    // -------- Color Mode -------- //
    tft_write_cmd(0x3A);  // COLMOD
    tft_write_data_bytes((uint8_t[]){0x05}, 1);  // 16bit/pixel (RGB565)

    // -------- Display On -------- //
    tft_write_cmd(0x29);  // DISPON
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

/**
 * @brief 初始化 TFT 显示屏
 */
esp_err_t tft_display_init(void)
{
    esp_err_t ret;

    ret = tft_gpio_init();
    if (ret != ESP_OK) return ret;

    ret = tft_spi_init();
    if (ret != ESP_OK) return ret;

    ret = tft_backlight_pwm_init();
    if (ret != ESP_OK) return ret;

    // 依赖模块内部 POR 完成硬件复位
    // POR 稳定 + SLPOUT 准备: 共等待 ~300ms
    vTaskDelay(pdMS_TO_TICKS(300));

    // ST7735S 完整初始化序列
    tft_st7735s_init_seq();

    // 默认开启背光
    tft_backlight_on(true);
    tft_clear(TFT_COLOR_BLACK);

    ESP_LOGI(TAG, "TFT 初始化成功 (ST7735S, %dx%d, MADCTL=0x%X, RST=VCC 内部 POR)",
             TFT_SCREEN_W, TFT_SCREEN_H,
#if USE_HORIZONTAL == 0
             0x00
#elif USE_HORIZONTAL == 1
             0xC0
#elif USE_HORIZONTAL == 2
             0x70
#else
             0xA0
#endif
             );
    return ESP_OK;
}

/**
 * @brief 设置背光亮度
 */
esp_err_t tft_set_backlight(uint8_t brightness)
{
    if (brightness > 100) brightness = 100;
    uint32_t max_duty = (1 << TFT_BLK_LEDC_RES) - 1;
    uint32_t duty = (max_duty * brightness) / 100;
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, TFT_BLK_LEDC_CHANNEL, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, TFT_BLK_LEDC_CHANNEL);
}

/**
 * @brief 开启/关闭背光
 */
esp_err_t tft_backlight_on(bool on)
{
    return tft_set_backlight(on ? 100 : 0);
}

/**
 * @brief 清屏
 */
esp_err_t tft_clear(uint16_t color)
{
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    uint8_t buf[TFT_SCREEN_W * 2];

    for (int x = 0; x < TFT_SCREEN_W; x++) {
        buf[x * 2] = hi;
        buf[x * 2 + 1] = lo;
    }

    tft_write_cmd(0x2A);  // CASET
    uint8_t caset[] = {
        (uint8_t)((TFT_X_OFFSET) >> 8), (uint8_t)(TFT_X_OFFSET & 0xFF),
        (uint8_t)((TFT_SCREEN_W - 1 + TFT_X_OFFSET) >> 8),
        (uint8_t)((TFT_SCREEN_W - 1 + TFT_X_OFFSET) & 0xFF)
    };
    tft_write_data_bytes(caset, 4);

    tft_write_cmd(0x2B);  // RASET
    uint8_t raset[] = {
        (uint8_t)((TFT_Y_OFFSET) >> 8), (uint8_t)(TFT_Y_OFFSET & 0xFF),
        (uint8_t)((TFT_SCREEN_H - 1 + TFT_Y_OFFSET) >> 8),
        (uint8_t)((TFT_SCREEN_H - 1 + TFT_Y_OFFSET) & 0xFF)
    };
    tft_write_data_bytes(raset, 4);

    tft_write_cmd(0x2C);  // RAMWR

    gpio_set_level(TFT_DC_GPIO, 1);
    gpio_set_level(TFT_CS_GPIO, 0);
    for (int y = 0; y < TFT_SCREEN_H; y++) {
        spi_transaction_t t = {
            .length = TFT_SCREEN_W * 2 * 8,
            .tx_buffer = buf,
        };
        esp_err_t ret = spi_device_polling_transmit(spi_dev, &t);
        if (ret != ESP_OK) {
            gpio_set_level(TFT_CS_GPIO, 1);
            return ret;
        }
    }
    gpio_set_level(TFT_CS_GPIO, 1);
    return ESP_OK;
}

/**
 * @brief 绘制一个像素
 */
esp_err_t tft_draw_pixel(uint8_t x, uint8_t y, uint16_t color)
{
    if (x >= TFT_SCREEN_W || y >= TFT_SCREEN_H) {
        return ESP_ERR_INVALID_ARG;
    }

    tft_write_cmd(0x2A);
    uint8_t caset[] = {
        (uint8_t)((x + TFT_X_OFFSET) >> 8), (uint8_t)((x + TFT_X_OFFSET) & 0xFF),
        (uint8_t)((x + TFT_X_OFFSET) >> 8), (uint8_t)((x + TFT_X_OFFSET) & 0xFF)
    };
    tft_write_data_bytes(caset, 4);
    tft_write_cmd(0x2B);
    uint8_t raset[] = {
        (uint8_t)((y + TFT_Y_OFFSET) >> 8), (uint8_t)((y + TFT_Y_OFFSET) & 0xFF),
        (uint8_t)((y + TFT_Y_OFFSET) >> 8), (uint8_t)((y + TFT_Y_OFFSET) & 0xFF)
    };
    tft_write_data_bytes(raset, 4);
    tft_write_cmd(0x2C);
    uint8_t color_bytes[] = {color >> 8, color & 0xFF};
    tft_write_data_bytes(color_bytes, 2);
    return ESP_OK;
}

/**
 * @brief 显示字符串 (此处使用最简实现, 实际项目需要集成中英文字模)
 */
esp_err_t tft_draw_string(uint8_t x, uint8_t y, const char *str,
                          uint16_t color, uint16_t bg_color)
{
    (void)x;
    (void)y;
    (void)str;
    (void)color;
    (void)bg_color;
    // 字符显示需要字模支持, 此处仅作接口预留
    ESP_LOGW(TAG, "tft_draw_string 需要字模支持");
    return ESP_OK;
}

/**
 * @brief 设置显示窗口
 */
esp_err_t tft_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    if (x1 >= TFT_SCREEN_W || y1 >= TFT_SCREEN_H || x0 > x1 || y0 > y1) {
        return ESP_ERR_INVALID_ARG;
    }

    tft_write_cmd(0x2A);
    uint8_t caset[] = {
        (uint8_t)((x0 + TFT_X_OFFSET) >> 8), (uint8_t)((x0 + TFT_X_OFFSET) & 0xFF),
        (uint8_t)((x1 + TFT_X_OFFSET) >> 8), (uint8_t)((x1 + TFT_X_OFFSET) & 0xFF)
    };
    tft_write_data_bytes(caset, 4);
    tft_write_cmd(0x2B);
    uint8_t raset[] = {
        (uint8_t)((y0 + TFT_Y_OFFSET) >> 8), (uint8_t)((y0 + TFT_Y_OFFSET) & 0xFF),
        (uint8_t)((y1 + TFT_Y_OFFSET) >> 8), (uint8_t)((y1 + TFT_Y_OFFSET) & 0xFF)
    };
    tft_write_data_bytes(raset, 4);
    tft_write_cmd(0x2C);
    return ESP_OK;
}

/**
 * @brief 写入显示数据
 */
esp_err_t tft_write_data(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    tft_write_data_bytes(data, len);
    return ESP_OK;
}