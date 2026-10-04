/**
 * @file k230_uart.c
 * @brief K230 AI 视觉模块 UART 通信驱动实现
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "k230_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "K230";

/* 接收回调 */
static k230_rx_cb_t rx_callback = NULL;

/* 接收事件队列 (内部使用) */
static QueueHandle_t s_uart_queue = NULL;

/**
 * @brief UART 事件处理任务
 */
static void k230_uart_event_task(void *arg)
{
    (void)arg;
    uart_event_t event;
    uint8_t *buf = (uint8_t *)malloc(K230_UART_BUF_SIZE);

    while (1) {
        if (xQueueReceive(s_uart_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (event.type == UART_DATA && buf != NULL) {
            uint16_t len = 0;
            if (k230_uart_receive(buf, K230_UART_BUF_SIZE, &len) == ESP_OK && len > 0) {
                if (rx_callback) {
                    rx_callback(buf, len);
                }
            }
        }
    }

    free(buf);
    vTaskDelete(NULL);
}

/**
 * @brief 初始化 K230 UART
 */
esp_err_t k230_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = K230_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(K230_UART_NUM, K230_UART_BUF_SIZE * 2,
                                        K230_UART_BUF_SIZE * 2, 20,
                                        &s_uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART 驱动安装失败");
        return ret;
    }

    ret = uart_param_config(K230_UART_NUM, &cfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(K230_UART_NUM, K230_TXD_GPIO, K230_RXD_GPIO,
                       -1, -1);
    if (ret != ESP_OK) return ret;

    // 创建接收处理任务
    xTaskCreate(k230_uart_event_task, "k230_uart", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "K230 UART 初始化成功 (TX=%d, RX=%d, %d bps)",
             K230_TXD_GPIO, K230_RXD_GPIO, K230_UART_BAUDRATE);
    return ESP_OK;
}

/**
 * @brief 发送数据
 */
esp_err_t k230_uart_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    int sent = uart_write_bytes(K230_UART_NUM, data, len);
    if (sent < 0) return ESP_FAIL;
    return (sent == len) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief 计算校验和 (异或)
 */
static uint8_t k230_calc_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t cs = 0;
    for (uint16_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

/**
 * @brief 发送协议帧
 *        [HEAD] [LEN_HI] [LEN_LO] [PAYLOAD...] [CHECKSUM] [TAIL]
 */
esp_err_t k230_uart_send_frame(const uint8_t *payload, uint16_t len)
{
    if (payload == NULL || len > K230_FRAME_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[K230_FRAME_MAX_LEN + 5];
    uint16_t idx = 0;

    frame[idx++] = K230_FRAME_HEADER;
    frame[idx++] = (len >> 8) & 0xFF;
    frame[idx++] = len & 0xFF;
    memcpy(&frame[idx], payload, len);
    idx += len;
    frame[idx++] = k230_calc_checksum(payload, len);
    frame[idx++] = K230_FRAME_TAIL;

    return k230_uart_send(frame, idx);
}

/**
 * @brief 注册接收回调
 */
esp_err_t k230_uart_register_callback(k230_rx_cb_t cb)
{
    if (cb == NULL) return ESP_ERR_INVALID_ARG;
    rx_callback = cb;
    return ESP_OK;
}

/**
 * @brief 接收数据 (非阻塞)
 */
esp_err_t k230_uart_receive(uint8_t *buf, uint16_t max_len, uint16_t *out_len)
{
    if (buf == NULL || out_len == NULL) return ESP_ERR_INVALID_ARG;
    int len = uart_read_bytes(K230_UART_NUM, buf, max_len, pdMS_TO_TICKS(10));
    if (len < 0) return ESP_FAIL;
    *out_len = (uint16_t)len;
    return ESP_OK;
}