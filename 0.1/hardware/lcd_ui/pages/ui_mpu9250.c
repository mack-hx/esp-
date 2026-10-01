/**
 * @file ui_mpu9250.c
 * @brief UI MPU9250 IMU 调试页实现
 * @note  - 实时显示加速度 (ax/ay/az) / 陀螺仪 (gx/gy/gz) / 温度
 *        - 显示欧拉角 (Roll/Pitch/Yaw) - 取自 mpu9250_get_euler_angle()
 *        - K1 退出回菜单
 *        - K2 / K3 在此版本无操作 (后续可加入"校准/复位/切换量程")
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "ui_mpu9250.h"
#include "mpu9250.h"
#include "tft_display.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG __attribute__((unused)) = "UI_MPU9250";

/* 进入页面后是否需要触发陀螺仪校准 */
static bool s_need_calibrate = true;

/**
 * @brief 绘制静态骨架
 */
static void ui_mpu9250_draw_static(void)
{
    tft_clear(TFT_COLOR_BLACK);

    // 顶部标题
    ui_fill_rect(0, 0, UI_SCREEN_W, UI_FONT_H + 2, TFT_COLOR_WHITE);
    ui_draw_string(2, 1, "MPU9250", TFT_COLOR_BLACK, TFT_COLOR_WHITE);

    // 加速度 (m/s^2)
    ui_draw_string(2, 14, "ACC:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, 24, "ax=", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 34, "ay=", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 44, "az=", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    // 陀螺仪 (deg/s)
    ui_draw_string(2, 56, "GYR:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, 66, "gx=", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 76, "gy=", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 86, "gz=", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    // 欧拉角
    ui_draw_string(2, 100, "RPY:", TFT_COLOR_CYAN, TFT_COLOR_BLACK);
    ui_draw_string(2, 110, "T=", TFT_COLOR_WHITE, TFT_COLOR_BLACK);

    // 底部
    ui_draw_hline(0, 130, UI_SCREEN_W, TFT_COLOR_BLUE);
    ui_draw_string(2, 136, "[K1] Back", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 146, "[K2] Calib", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
    ui_draw_string(2, 156, "[K3] Reset", TFT_COLOR_WHITE, TFT_COLOR_BLACK);
}

/**
 * @brief 绘制实时数据
 */
static void ui_mpu9250_draw_data(void)
{
    mpu9250_data_t  data;
    euler_angle_t   angle;
    char buf[20];

    if (mpu9250_read_data(&data) != ESP_OK) {
        ui_fill_rect(36, 14, 90, UI_FONT_H, TFT_COLOR_BLACK);
        ui_draw_string(36, 14, "ERR", TFT_COLOR_RED, TFT_COLOR_BLACK);
        return;
    }
    if (mpu9250_get_euler_angle(&angle) != ESP_OK) {
        ui_fill_rect(36, 14, 90, UI_FONT_H, TFT_COLOR_BLACK);
        ui_draw_string(36, 14, "ERR", TFT_COLOR_RED, TFT_COLOR_BLACK);
        return;
    }

    // ax, ay, az
    ui_fill_rect(30, 24, 96, UI_FONT_H * 3, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%6.2f", data.accel.x);
    ui_draw_string(30, 24, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%6.2f", data.accel.y);
    ui_draw_string(30, 34, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%6.2f", data.accel.z);
    ui_draw_string(30, 44, buf, TFT_COLOR_YELLOW, TFT_COLOR_BLACK);

    // gx, gy, gz
    ui_fill_rect(30, 66, 96, UI_FONT_H * 3, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%6.1f", data.gyro.x);
    ui_draw_string(30, 66, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%6.1f", data.gyro.y);
    ui_draw_string(30, 76, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);
    snprintf(buf, sizeof(buf), "%6.1f", data.gyro.z);
    ui_draw_string(30, 86, buf, TFT_COLOR_GREEN, TFT_COLOR_BLACK);

    // Roll/Pitch/Yaw 单行
    char rpy[24];
    snprintf(rpy, sizeof(rpy), "R%4.0f P%4.0f Y%4.0f",
             angle.roll, angle.pitch, angle.yaw);
    ui_fill_rect(36, 100, 90, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(36, 100, rpy, TFT_COLOR_MAGENTA, TFT_COLOR_BLACK);

    // 温度
    snprintf(buf, sizeof(buf), "%4.1fC", data.temperature);
    ui_fill_rect(30, 110, 90, UI_FONT_H, TFT_COLOR_BLACK);
    ui_draw_string(30, 110, buf, TFT_COLOR_CYAN, TFT_COLOR_BLACK);
}

/* ========== 页面回调 ========== */

static void page_enter(void)
{
    s_need_calibrate = true;
    ui_mpu9250_draw_static();
    // 首次显示空数据 (等首次刷新填充)
    ui_mpu9250_draw_data();
}

static void page_exit(void)
{
    // MPU9250 持续运行, 无需释放
}

static void page_key(ui_key_t key)
{
    if (key == UI_KEY_K1) {
        lcd_ui_switch_to(UI_PAGE_MENU);
    } else if (key == UI_KEY_K2) {
        // 触发陀螺仪零点校准 (200 次采样, 约 2 秒)
        ESP_LOGI(TAG, "开始陀螺仪校准...");
        if (mpu9250_calibrate_gyro(200) == ESP_OK) {
            ESP_LOGI(TAG, "陀螺仪校准完成");
        } else {
            ESP_LOGE(TAG, "陀螺仪校准失败");
        }
    } else if (key == UI_KEY_K3) {
        // 软复位
        ESP_LOGI(TAG, "MPU9250 软复位");
        mpu9250_reset();
        // 重新初始化 (reset 后需重新调用 init)
        mpu9250_init();
    }
}

static void page_refresh(void)
{
    ui_mpu9250_draw_data();
}

/* ========== 页面注册 ========== */

static const ui_page_t mpu9250_page = {
    .id        = UI_PAGE_MPU9250,
    .title     = "MPU9250",
    .on_enter  = page_enter,
    .on_exit   = page_exit,
    .on_key    = page_key,
    .on_refresh = page_refresh,
};

esp_err_t ui_mpu9250_register(void)
{
    return lcd_ui_register_page(&mpu9250_page);
}