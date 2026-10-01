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
#include "servo.h"
#include "motor_driver.h"
#include "encoder.h"
#include "mpu9250.h"
#include "pcf8574.h"
#include "lcd_ui.h"
#include "pages/ui_home.h"
#include "pages/ui_menu.h"
#include "pages/ui_servo.h"
#include "pages/ui_encoder.h"
#include "pages/ui_mpu9250.h"
#include "pages/ui_wifi_bt.h"

static const char *TAG = "MAIN";

/* ========== 按键获取回调 (PCF8574 扫描 → UI 按键) ========== */
static ui_key_t key_get_from_pcf8574(void)
{
    uint8_t state = pcf8574_scan_buttons();
    // 优先级: K1 > K2 > K3 (一次只返回 1 个)
    if (state & (1 << BTN_K1)) return UI_KEY_K1;
    if (state & (1 << BTN_K2)) return UI_KEY_K2;
    if (state & (1 << BTN_K3)) return UI_KEY_K3;
    return UI_KEY_NONE;
}

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

    // 4. 初始化舵机
    ESP_ERROR_CHECK(servo_init());

    // 5. 初始化电机 (制动状态)
    ESP_ERROR_CHECK(motor_driver_init());
    ESP_ERROR_CHECK(motor_brake_all());

    // 6. 初始化编码器
    ESP_ERROR_CHECK(encoder_init());

    // 7. 初始化 MPU9250
    ESP_ERROR_CHECK(mpu9250_init());

    // 8. 初始化 PCF8574 (按键)
    ESP_ERROR_CHECK(pcf8574_init());

    // ========================================================
    // UI 系统初始化
    // ========================================================
    // 注册所有页面 (顺序不影响, 但 HOME 必须先注册)
    ESP_ERROR_CHECK(ui_home_register());
    ESP_ERROR_CHECK(ui_menu_register());
    ESP_ERROR_CHECK(ui_servo_register());
    ESP_ERROR_CHECK(ui_encoder_register());
    ESP_ERROR_CHECK(ui_mpu9250_register());
    ESP_ERROR_CHECK(ui_wifi_bt_register());

    // 初始化 UI 系统 (会在主页显示初始画面)
    ESP_ERROR_CHECK(lcd_ui_init(key_get_from_pcf8574));

    ESP_LOGI(TAG, "UI 系统就绪, 进入主循环");

    // 主循环 (按键扫描 + UI 调度 + 页面刷新)
    while (1) {
        lcd_ui_loop();
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms 周期
    }
}