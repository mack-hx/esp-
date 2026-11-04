/**
 * @file rolltest.h
 * @brief 横滚角实测工具: 串口命令设定左右舵机角度, 回读车身横滚角
 * @note  命令从 USB-Serial-JTAG 接收 (与日志同一根线), 直接读硬件 FIFO,
 *        不安装驱动, 不影响控制台输出。
 */

#ifndef ROLLTEST_H
#define ROLLTEST_H

#include "esp_err.h"

/**
 * @brief 启动测量任务
 * @return ESP_OK 成功
 * @note  需在 servo_init() 与 balance_init() 之后调用
 */
esp_err_t rolltest_start(void);

#endif // ROLLTEST_H
