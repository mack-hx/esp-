/* test/legmap_offline.h — 离线跑 legmap 测试用的 ESP 桩头
 *
 * 这份头要顶替 ESP-IDF 的 esp_err.h 和 esp_log.h。
 * 因为 legmap.h/legmap.c 直接 #include "esp_err.h" / "esp_log.h",
 * 所以我们在 -I 顺序里把 test/ 放在 ESP-IDF 头目录前面, 让 GCC 先找本份。
 * 但 ESP-IDF 头只在 ESP 编译时存在, 离线时 GCC 只在 -I 路径里搜不到 → 由本份提供。
 *
 * 设计要点:
 *   - esp_err_t 必须是 typedef, 与 esp_log 桩配套
 *   - 提供 ESP_OK / ESP_FAIL / ESP_ERR_* 几个常量
 *   - 提供 ESP_LOGI/W/E 空/简版
 *   - 离线版 legmap.c 用 #ifndef LEGMAP_OFFLINE_TEST 屏蔽 NVS 段
 */
#ifndef LEGMAP_OFFLINE_H
#define LEGMAP_OFFLINE_H

#include <stdint.h>
#include <stdio.h>

typedef int esp_err_t;
#define ESP_OK                    0
#define ESP_FAIL                 -1
#define ESP_ERR_NO_MEM            0x101
#define ESP_ERR_INVALID_ARG       0x102
#define ESP_ERR_INVALID_STATE     0x103
#define ESP_ERR_NOT_FOUND         0x104

#define ESP_LOGI(tag, ...)  ((void)0)
#define ESP_LOGW(tag, ...)  do { fprintf(stderr, "[W %s] ", tag); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)
#define ESP_LOGE(tag, ...)  do { fprintf(stderr, "[E %s] ", tag); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)

#endif /* LEGMAP_OFFLINE_H */