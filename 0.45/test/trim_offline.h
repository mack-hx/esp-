/**
 * @file trim_offline.h
 * @brief trim.c 离线测试用的最小类型定义 (让 GCC 不去找 ESP-IDF)
 * @note  配合 -DTRIM_OFFLINE_TEST 使用。
 *        trim.h 里对 esp_err.h 的 include 被 #ifndef TRIM_OFFLINE_TEST 屏蔽,
 *        trim.c 里的 NVS 段也被屏蔽 —— 所以这里只需要给出 esp_err_t。
 *
 * @note  为什么不用 test/ 下的 esp_err.h 桩文件:
 *        桩头文件方案要求被测代码无条件 #include "esp_err.h",
 *        然后靠 -I test 让 GCC 找到假的那个。这样"能找到假头文件"
 *        本身就变成了一个隐式依赖 —— 换个编译顺序就崩。
 *        直接在 trim.h 里加 #ifndef 屏蔽更明确。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-05
 */

#ifndef TRIM_OFFLINE_H
#define TRIM_OFFLINE_H

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

#endif // TRIM_OFFLINE_H
