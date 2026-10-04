/**
 * @file main.c
 * @brief ESP32-S3R8R8N8 平衡轮腿机械狗主程序
 * @date 2026-10-01
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "hc595.h"
#include "tft_display.h"
#include "led.h"
#include "servo.h"
#include "motor_driver.h"
#include "encoder.h"
#include "mpu9250.h"
#include "pcf8574.h"
#include "lcd_ui.h"
#include "wifi_ap.h"
#include "param.h"
#include "web_server.h"
#include "ui_home.h"
#include "ui_menu.h"
#include "ui_servo.h"
#include "ui_encoder.h"
#include "ui_mpu9250.h"
#include "ui_wifi.h"
#include "ui_hc595_test.h"

static const char *TAG = "MAIN";

/* ========== 按键获取回调 (PCF8574 扫描 → UI 按键) ========== */
/**
 * @brief 把 PCF8574 按键位图转换为 UI 按键
 *
 * 优先级: K1 > K3 > K2 > K4
 *   - K1: 进入/确认/退出当前页
 *   - K2: 上/左/减
 *   - K3: 下/右/加
 *   - K4: 减 (备用, 例如 SERVO 调试页 K4 = -5°)
 *
 * 多键同时按下: 只返回优先级最高的那个键
 *
 * 按下电平约定: P0~P7 内部上拉到 VCC (4.7kΩ 外部上拉), 按键按下时拉低 (接地).
 *   "按下"对应位 = 0, "松开"对应位 = 1
 *   例: 按下 K1 (P0) → 读 0xFE, 全松开 0xFF
 *
 * @note  本函数仅做 "位图 → UI 键值" 映射, 不做消抖
 *        消抖由 pcf8574_scan_buttons() 内部状态机 + lcd_ui_loop() 边沿检测共同完成
 */
static ui_key_t key_get_from_pcf8574(void)
{
    uint8_t state;
    if (pcf8574_scan_buttons(&state) != ESP_OK) {
        return UI_KEY_NONE;
    }
    if (!(state & (1 << BTN_K1))) return UI_KEY_K1;
    if (!(state & (1 << BTN_K3))) return UI_KEY_K3;
    if (!(state & (1 << BTN_K2))) return UI_KEY_K2;
    if (!(state & (1 << BTN_K4))) return UI_KEY_K4;
    return UI_KEY_NONE;
}

/**
 * @brief dummy 按键回调: 始终返回 NONE
 *        当 PCF8574 不可用时使用, 保证 UI 仍然显示
 */
static ui_key_t key_get_none(void)
{
    return UI_KEY_NONE;
}

/**
 * @brief 主函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3R8N8 平衡轮腿机械狗启动...");

    /* 初始化 NVS (Non-Volatile Storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "系统初始化完成");
    ESP_LOGI(TAG, "PSRAM: 8MB, Flash: 8MB");

    /* 0. 载入调参参数 (网页滑块调的就是这张表, 必须在用参数之前)
     * 注意: 依赖 NVS, 必须在 nvs_flash_init 之后 */
    param_load();

    /* 1. 初始化 74HC595 (电机方向/电磁铁控制) */
    ESP_ERROR_CHECK(hc595_init());

    /* 2. 初始化 TFT 显示屏 (RST 已接 VCC, 依赖内部 POR) */
    ESP_ERROR_CHECK(tft_display_init());

    /* 3. 初始化 SK6812 LED (RMT 直驱, 支持真实调色调光) */
    ESP_ERROR_CHECK(led_init());

    /* 4. 初始化舵机 (归位到 SERVO_DEFAULT_ANGLE, 默认舵机2 镜像) */
    ESP_ERROR_CHECK(servo_init());

    /* 4.1 启动后台"保持运行"任务: 持续 sync 舵机硬件, 跨页不失力/不跳变 */
    ESP_ERROR_CHECK(servo_keep_alive_start());

    /* 5. 初始化电机 (制动状态) */
    ESP_ERROR_CHECK(motor_driver_init());
    ESP_ERROR_CHECK(motor_brake_all());

    /* 6. 初始化编码器 */
    ESP_ERROR_CHECK(encoder_init());

    /* 7. 初始化 MPU9250 (允许失败: 模块可能未接, UI 会显示 "NOT READY") */
    esp_err_t mpu_ret = mpu9250_init();
    if (mpu_ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 初始化失败 (模块未接或 I2C 故障), UI 仍可正常工作");
        ESP_LOGE(TAG, "请检查 MPU9250 模块接线 (SCL=GPIO%d, SDA=GPIO%d, VCC, GND, AD0)",
                 MPU9250_SCL_GPIO, MPU9250_SDA_GPIO);
    }

    /* 8. 启动 WiFi 热点 + 调参网页
     * 注意: WiFi 驱动依赖 NVS, 必须在 nvs_flash_init 之后调用。
     *       两者都允许失败: 失败只是没法用网页调参, 屏上按键调试照常。 */
    esp_err_t wifi_ret = wifi_ap_start();
    if (wifi_ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi 热点启动失败, 网页调参不可用 (%s)",
                 esp_err_to_name(wifi_ret));
    } else {
        esp_err_t web_ret = web_server_start();
        if (web_ret != ESP_OK) {
            ESP_LOGE(TAG, "网页服务器启动失败 (%s)", esp_err_to_name(web_ret));
        }
    }

    /* 9. 初始化 PCF8574 (按键) - 允许失败: 模块可能未接 */
    esp_err_t pcf_ret = pcf8574_init();
    if (pcf_ret != ESP_OK) {
        ESP_LOGE(TAG, "PCF8574 初始化失败 (按键模块未接), UI 无法响应按键");
        ESP_LOGE(TAG, "请检查 PCF8574 模块接线 (SCL=GPIO%d, SDA=GPIO%d, INT=GPIO%d, VCC, GND)",
                 PCF8574_SCL_GPIO, PCF8574_SDA_GPIO, PCF8574_INT_GPIO);
    }

    /* ========================================================
     * UI 系统初始化
     * ======================================================== */
    /* 注册所有页面 (顺序不影响, 但 HOME 必须先注册) */
    ESP_ERROR_CHECK(ui_home_register());
    ESP_ERROR_CHECK(ui_menu_register());
    ESP_ERROR_CHECK(ui_servo_register());
    ESP_ERROR_CHECK(ui_encoder_register());
    ESP_ERROR_CHECK(ui_mpu9250_register());
    ESP_ERROR_CHECK(ui_wifi_register());
    ESP_ERROR_CHECK(ui_hc595_test_register());

    /* 根据 PCF8574 是否就绪, 选择按键回调
     * - 就绪: 从 PCF8574 读取真实按键
     * - 未就绪: 用 dummy 回调 (返回 NONE, UI 仍然显示但按键无效) */
    ui_key_get_t key_cb = (pcf_ret == ESP_OK) ? key_get_from_pcf8574 : key_get_none;

    /* 初始化 UI 系统 (会在主页显示初始画面) */
    ESP_ERROR_CHECK(lcd_ui_init(key_cb));

    ESP_LOGI(TAG, "UI 系统就绪, 进入主循环");

    /* 主循环 (按键扫描 + UI 调度 + 页面刷新) */
    while (1) {
        lcd_ui_loop();
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms 周期
    }
}