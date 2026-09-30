/**
 * @file main.c
 * @brief ESP32-S3R8N8 平衡轮腿机械狗主程序
 * @date 2026-09-30
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

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
    
    // 主循环
    while (1) {
        ESP_LOGI(TAG, "系统运行中...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
