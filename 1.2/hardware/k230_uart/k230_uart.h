/**
 * @file k230_uart.h
 * @brief K230 AI 视觉模块 UART 通信驱动
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef K230_UART_H
#define K230_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
#define K230_TXD_GPIO       38      // ESP32 -> K230 (发送)
#define K230_RXD_GPIO       47      // K230 -> ESP32 (接收)

/* ========== UART 配置 ========== */
#define K230_UART_NUM       UART_NUM_1
#define K230_UART_BAUDRATE  115200
#define K230_UART_BUF_SIZE  256     // 接收缓冲区大小

/* ========== 协议帧格式 ========== */
#define K230_FRAME_HEADER   0xAA    // 帧头
#define K230_FRAME_TAIL     0x55    // 帧尾
#define K230_FRAME_MAX_LEN  64      // 单帧最大数据长度

/* ========== 接收回调 ========== */
typedef void (*k230_rx_cb_t)(const uint8_t *data, uint16_t len);

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 K230 UART
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t k230_uart_init(void);

/**
 * @brief 发送数据
 * @param data 数据指针
 * @param len 数据长度
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t k230_uart_send(const uint8_t *data, uint16_t len);

/**
 * @brief 发送协议帧
 * @param payload 负载数据
 * @param len 负载长度
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t k230_uart_send_frame(const uint8_t *payload, uint16_t len);

/**
 * @brief 注册接收回调
 * @param cb 回调函数指针
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t k230_uart_register_callback(k230_rx_cb_t cb);

/**
 * @brief 接收数据 (非阻塞)
 * @param buf 缓冲区
 * @param max_len 缓冲区最大长度
 * @param out_len 实际接收长度
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t k230_uart_receive(uint8_t *buf, uint16_t max_len, uint16_t *out_len);

#endif // K230_UART_H