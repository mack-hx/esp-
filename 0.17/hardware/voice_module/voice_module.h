/**
 * @file voice_module.h
 * @brief CI-03T 离线语音识别模块驱动
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef VOICE_MODULE_H
#define VOICE_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 引脚配置 ========== */
#define VOICE_RXD_GPIO      39      // CI-03T -> ESP32 (接收)
#define VOICE_TXD_GPIO      40      // ESP32 -> CI-03T (发送)

/* ========== UART 配置 ========== */
#define VOICE_UART_NUM      UART_NUM_2
#define VOICE_UART_BAUDRATE 9600
#define VOICE_UART_BUF_SIZE 128

/* ========== CI-03T 命令码定义 ========== */
typedef enum {
    VOICE_CMD_FORWARD    = 0x01,    // 前进
    VOICE_CMD_BACKWARD   = 0x02,    // 后退
    VOICE_CMD_TURN_LEFT  = 0x03,    // 左转
    VOICE_CMD_TURN_RIGHT = 0x04,    // 右转
    VOICE_CMD_STOP       = 0x05,    // 停止
    VOICE_CMD_DANCE      = 0x06,    // 跳舞
    VOICE_CMD_SIT        = 0x07,    // 坐下
    VOICE_CMD_STAND      = 0x08,    // 站立
    VOICE_CMD_MAX        = 0xFF
} voice_cmd_t;

/* ========== 识别结果回调 ========== */
typedef void (*voice_cmd_cb_t)(voice_cmd_t cmd);

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 CI-03T 语音模块
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t voice_module_init(void);

/**
 * @brief 注册识别命令回调
 * @param cb 回调函数指针
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t voice_register_callback(voice_cmd_cb_t cb);

/**
 * @brief 接收数据 (非阻塞)
 * @param buf 缓冲区
 * @param max_len 缓冲区最大长度
 * @param out_len 实际接收长度
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t voice_receive(uint8_t *buf, uint16_t max_len, uint16_t *out_len);

/**
 * @brief 发送命令到 CI-03T (例如播放语音回复)
 * @param data 数据指针
 * @param len 数据长度
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t voice_send(const uint8_t *data, uint16_t len);

#endif // VOICE_MODULE_H