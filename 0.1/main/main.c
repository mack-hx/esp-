/**
 * @file main.c
 * @brief ESP32-S3R8N8 平衡轮腿机械狗主程序
 * @date 2026-10-01
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "hc595.h"
#include "tft_display.h"
#include "led.h"

static const char *TAG = "MAIN";

/**
 * @brief 主函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3R8N8 平衡轮腿机械狗启动...");

    // 初始化 NVS（Non-Volatile Storage）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "系统初始化完成");
    ESP_LOGI(TAG, "PSRAM: 8MB, Flash: 8MB");

    // 1. 初始化 74HC595 (电机方向/电磁铁控制)
    ESP_ERROR_CHECK(hc595_init());

    // 2. 初始化 TFT 显示屏 (RST 已接 VCC, 依赖内部 POR)
    ESP_ERROR_CHECK(tft_display_init());

    // 3. 初始化 SK6812 LED (RMT 直驱, 支持真实调色调光)
    ESP_ERROR_CHECK(led_init());

    // 4. 测试 SK6812 RGB 三色
    ESP_LOGI(TAG, "测试 SK6812: 红 -> 绿 -> 蓝 -> 白");
    led_set_preset(LED_1, LED_PRESET_RED);
    vTaskDelay(pdMS_TO_TICKS(500));
    led_set_preset(LED_1, LED_PRESET_GREEN);
    vTaskDelay(pdMS_TO_TICKS(500));
    led_set_preset(LED_1, LED_PRESET_BLUE);
    vTaskDelay(pdMS_TO_TICKS(500));
    led_set_preset(LED_1, LED_PRESET_WHITE);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 5. 测试屏幕: 蓝 -> 红 -> 绿 -> 白 -> 黑 (RGB 三色 + 验证 MADCTL)
    ESP_LOGI(TAG, "测试 TFT: 蓝 -> 红 -> 绿 -> 白");
    tft_set_backlight(100);
    tft_clear(TFT_COLOR_BLUE);
    vTaskDelay(pdMS_TO_TICKS(800));
    tft_clear(TFT_COLOR_RED);
    vTaskDelay(pdMS_TO_TICKS(800));
    tft_clear(TFT_COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(800));
    tft_clear(TFT_COLOR_WHITE);
    vTaskDelay(pdMS_TO_TICKS(800));
    tft_clear(TFT_COLOR_BLACK);  // 最终停留黑屏

    ESP_LOGI(TAG, "所有外设初始化完成");

    // 主循环
    while (1) {
        ESP_LOGI(TAG, "系统运行中...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}