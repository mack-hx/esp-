/**
 * @file voice_module.c
 * @brief CI-03T 离线语音识别模块驱动实现
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "voice_module.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

static const char *TAG = "VOICE";

/* 命令回调 */
static voice_cmd_cb_t cmd_callback = NULL;

/* 接收事件队列 (内部使用) */
static QueueHandle_t s_uart_queue = NULL;

/**
 * @brief UART 事件处理任务
 */
static void voice_event_task(void *arg)
{
    (void)arg;
    uart_event_t event;
    uint8_t *buf = (uint8_t *)malloc(VOICE_UART_BUF_SIZE);

    while (1) {
        if (xQueueReceive(s_uart_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (event.type == UART_DATA && buf != NULL) {
            uint16_t len = 0;
            if (voice_receive(buf, VOICE_UART_BUF_SIZE, &len) == ESP_OK && len > 0) {
                // 简单协议: 第一字节为命令码
                voice_cmd_t cmd = (voice_cmd_t)buf[0];
                ESP_LOGI(TAG, "识别到命令: 0x%02X", (uint8_t)cmd);
                if (cmd_callback) {
                    cmd_callback(cmd);
                }
            }
        }
    }

    free(buf);
    vTaskDelete(NULL);
}

/**
 * @brief 初始化 CI-03T 语音模块
 */
esp_err_t voice_module_init(void)
{
    uart_config_t cfg = {
        .baud_rate = VOICE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(VOICE_UART_NUM, VOICE_UART_BUF_SIZE * 2,
                                        VOICE_UART_BUF_SIZE * 2, 20,
                                        &s_uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART 驱动安装失败");
        return ret;
    }

    ret = uart_param_config(VOICE_UART_NUM, &cfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(VOICE_UART_NUM, VOICE_TXD_GPIO, VOICE_RXD_GPIO,
                       -1, -1);
    if (ret != ESP_OK) return ret;

    // 创建接收处理任务
    xTaskCreate(voice_event_task, "voice_uart", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "CI-03T 语音模块初始化成功 (TX=%d, RX=%d, %d bps)",
             VOICE_TXD_GPIO, VOICE_RXD_GPIO, VOICE_UART_BAUDRATE);
    return ESP_OK;
}

/**
 * @brief 注册识别命令回调
 */
esp_err_t voice_register_callback(voice_cmd_cb_t cb)
{
    if (cb == NULL) return ESP_ERR_INVALID_ARG;
    cmd_callback = cb;
    return ESP_OK;
}

/**
 * @brief 接收数据 (非阻塞)
 */
esp_err_t voice_receive(uint8_t *buf, uint16_t max_len, uint16_t *out_len)
{
    if (buf == NULL || out_len == NULL) return ESP_ERR_INVALID_ARG;
    int len = uart_read_bytes(VOICE_UART_NUM, buf, max_len, pdMS_TO_TICKS(10));
    if (len < 0) return ESP_FAIL;
    *out_len = (uint16_t)len;
    return ESP_OK;
}

/**
 * @brief 发送数据
 */
esp_err_t voice_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    int sent = uart_write_bytes(VOICE_UART_NUM, data, len);
    if (sent < 0) return ESP_FAIL;
    return (sent == len) ? ESP_OK : ESP_FAIL;
}