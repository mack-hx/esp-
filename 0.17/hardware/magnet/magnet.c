/**
 * @file magnet.c
 * @brief 电磁铁控制驱动实现
 * @note  通过 74HC595 Q5/Q6 控制电磁铁, 高电平开启。
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "magnet.h"
#include "hc595.h"
#include "esp_log.h"

static const char *TAG = "MAGNET";

/* 当前电磁铁状态缓存 */
static bool magnet_state[MAGNET_2 + 1] = {false, false};

/**
 * @brief 内部: 控制单个 74HC595 电磁铁引脚
 */
static esp_err_t magnet_pin_set(magnet_id_t id, bool on)
{
    if (id == MAGNET_1) {
        return hc595_set_magnet1(on);
    } else {
        return hc595_set_magnet2(on);
    }
}

/**
 * @brief 初始化电磁铁模块
 */
esp_err_t magnet_init(void)
{
    // 默认释放状态
    for (int i = 0; i <= MAGNET_2; i++) {
        magnet_state[i] = false;
    }
    magnet_release_all();

    ESP_LOGI(TAG, "电磁铁初始化完成 (通过 74HC595 Q5/Q6 控制, 默认关闭)");
    return ESP_OK;
}

/**
 * @brief 控制单个电磁铁开关
 */
esp_err_t magnet_set(magnet_id_t id, bool on)
{
    if (id > MAGNET_2) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = magnet_pin_set(id, on);
    if (ret == ESP_OK) {
        magnet_state[id] = on;
        ESP_LOGI(TAG, "电磁铁 %d: %s", id + 1, on ? "开启" : "关闭");
    }
    return ret;
}

/**
 * @brief 同时控制两个电磁铁
 */
esp_err_t magnet_set_both(bool on1, bool on2)
{
    esp_err_t ret = magnet_set(MAGNET_1, on1);
    if (ret != ESP_OK) return ret;
    return magnet_set(MAGNET_2, on2);
}

/**
 * @brief 获取电磁铁当前状态
 */
bool magnet_get(magnet_id_t id)
{
    if (id > MAGNET_2) return false;
    return magnet_state[id];
}

/**
 * @brief 关闭所有电磁铁
 */
esp_err_t magnet_release_all(void)
{
    esp_err_t ret = magnet_pin_set(MAGNET_1, false);
    if (ret != ESP_OK) return ret;
    ret = magnet_pin_set(MAGNET_2, false);
    if (ret != ESP_OK) return ret;

    magnet_state[MAGNET_1] = false;
    magnet_state[MAGNET_2] = false;
    return ESP_OK;
}