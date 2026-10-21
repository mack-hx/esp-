/**
 * @file i2c_scanner.c
 * @brief I2C 总线地址扫描工具 (诊断用)
 * @note  用于诊断 PCF8574 等模块的地址. 调用 i2c_scanner_pcf8574() 后
 *        串口会打印扫描结果, 然后设备进入死循环, 需要手动复位.
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "i2c_scanner.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "pcf8574.h"

static const char *TAG = "I2C_SCAN";

/* 扫描超时 (ms) */
#define SCAN_TIMEOUT_MS   20

/**
 * @brief 初始化 PCF8574 用的 I2C 总线 (如果 pcf8574_init 已失败, 这里手动初始化)
 * @return ESP_OK
 */
static esp_err_t ensure_i2c_bus(void)
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
        ESP_LOGE(TAG, "I2C 参数配置失败 (可能驱动已安装, 跳过)");
        // 已驱动安装在数组驱动中, ESP_ERR_INVALID_STATE 也是可接受的
        return ESP_OK;
    }

    ret = i2c_driver_install(PCF8574_I2C_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C 驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

/**
 * @brief 在指定 I2C 总线扫描 0x03-0x77 范围, 找出所有响应的地址
 * @param i2c_num I2C 总线号
 * @return ESP_OK
 */
static esp_err_t scan_bus(i2c_port_t i2c_num)
{
    ESP_LOGW(TAG, "==== I2C 扫描开始 (bus=%d, SCL=%d, SDA=%d) ====",
             i2c_num, PCF8574_SCL_GPIO, PCF8574_SDA_GPIO);
    ESP_LOGW(TAG, "扫描范围: 0x03 - 0x77 (含 PCF8574 候选 0x20-0x27)");

    int found_count = 0;

    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        // 快速探测: 1 字节读
        uint8_t dummy;
        esp_err_t ret = i2c_master_read_from_device(i2c_num, addr,
                                                     &dummy, 1,
                                                     pdMS_TO_TICKS(SCAN_TIMEOUT_MS));
        if (ret == ESP_OK) {
            ESP_LOGW(TAG, "  [0x%02X] ACK  <- 找到设备", addr);
            found_count++;
        }
        // 不打印 NAK, 否则日志刷屏
    }

    ESP_LOGW(TAG, "==== I2C 扫描完成: 找到 %d 个设备 ====", found_count);
    if (found_count == 0) {
        ESP_LOGE(TAG, "!!! 没找到任何设备, 请检查: !!!");
        ESP_LOGE(TAG, "  1. 模块是否上电 (VCC/GND)");
        ESP_LOGE(TAG, "  2. SDA/SCL 接线是否正确 (SCL=GPIO%d, SDA=GPIO%d)",
                 PCF8574_SCL_GPIO, PCF8574_SDA_GPIO);
        ESP_LOGE(TAG, "  3. I2C 总线是否有上拉电阻 (4.7kΩ 上拉到 3.3V)");
        ESP_LOGE(TAG, "  4. 接线是否松动/虚焊");
    }
    return ESP_OK;
}

esp_err_t i2c_scanner_pcf8574(void)
{
    ensure_i2c_bus();
    return scan_bus(PCF8574_I2C_NUM);
}