/**
 * @file param_offline.h
 * @brief param.c 离线测试用的最小类型定义 (让 GCC 不去找 ESP-IDF)
 * @note  配合 -DPARAM_OFFLINE_TEST 使用。
 *        param.h 里对 esp_err.h 的 include 被 #ifndef PARAM_OFFLINE_TEST 屏蔽,
 *        param.c 里的 NVS 段也被屏蔽 —— 所以这里只需要给出 esp_err_t。
 *
 * @note  做法跟 test/trim_offline.h 一致: 在 param.h 里显式 #ifndef 屏蔽,
 *        而不是靠 -I test 让 GCC 碰巧找到假头文件。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-26
 */

#ifndef PARAM_OFFLINE_H
#define PARAM_OFFLINE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ESP-IDF 的 esp_err_t 就是 int, 这里保持一致 (只需比较, 不需具体值) */
typedef int esp_err_t;

#define ESP_OK                 0
#define ESP_FAIL               (-1)
#define ESP_ERR_NO_MEM         0x101
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103

#endif // PARAM_OFFLINE_H
