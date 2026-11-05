/**
 * @file status_led.c
 * @brief 整车状态灯实现
 * @note  周期 100ms 计算一次状态, 颜色变了才刷 RMT; 手动设色期间不动灯。
 */

#include "status_led.h"
#include "led.h"
#include "servo.h"
#include "magnet.h"
#include "pcf8574.h"
#include "balance.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STATUS_LED";

#define STATUS_PERIOD_MS   100    /* 状态轮询周期 (ms) */
#define SERVO_PINK_MS      1000   /* 舵机角度变化后粉色保持时间 (ms) */
#define DOCK_FLASH_MS      500    /* 对接中黄灯闪烁半周期 (ms) */

/* 自动状态 (不含手动设色) */
typedef enum {
    ST_IDLE = 0,      /* 未站立: 黄 */
    ST_STANDING,      /* 站立平衡中: 绿 */
    ST_DOCKING,       /* 执行对接但还没对上: 闪黄 */
    ST_MISALIGNED,    /* 只压下一个对接开关: 红 */
    ST_DOCKED,        /* 双车合并 (两个开关都压下): 蓝 */
    ST_SERVO_MOVE,    /* 舵机角度刚变化: 粉 */
    ST_FALLEN,        /* 摔倒: 红 */
    ST_MAX
} status_t;

static volatile bool s_manual = false;          // true = 手动设色期间, 不自动改灯
static status_t s_last_state = ST_MAX;          // 上一次的自动状态 (首次一定变化)
static led_color_t s_last_color = {255, 255, 255}; // 上一次下发给灯的颜色 (占位, 首轮必刷)
static uint32_t s_servo_seq = 0;                // 上一次看到的舵机角度变化计数
static uint32_t s_servo_change_ms = 0;          // 最近一次舵机角度变化的时刻 (ms)

static uint32_t now_ms(void)
{
    return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/**
 * @brief 计算当前自动状态
 * @param now 当前时刻 (ms)
 * @return 自动状态
 */
static status_t compute_state(uint32_t now)
{
    /* 摔倒优先: 电机已停, 必须让人一眼看到 */
    if (balance_get_state() == BAL_STATE_FALLEN) return ST_FALLEN;

    /* 舵机角度刚变: 粉色临时覆盖 1s */
    if (now - s_servo_change_ms < SERVO_PINK_MS) return ST_SERVO_MOVE;

    /* 电磁铁吸合 = 对接流程中; 开关状态区分 合并 / 未对齐 / 对接中 */
    if (magnet_get(MAGNET_1) || magnet_get(MAGNET_2)) {
        bool sw1 = false, sw2 = false;
        pcf8574_read_dock_switches(&sw1, &sw2);
        if (sw1 && sw2) return ST_DOCKED;
        if (sw1 || sw2) return ST_MISALIGNED;
        return ST_DOCKING;
    }

    if (balance_get_state() == BAL_STATE_BALANCING) return ST_STANDING;
    return ST_IDLE;
}

/**
 * @brief 按状态取颜色
 * @param st 状态
 * @param now 当前时刻 (ms, 用于闪烁)
 * @return 目标颜色
 */
static led_color_t state_color(status_t st, uint32_t now)
{
    switch (st) {
    case ST_STANDING:   return LED_COLOR_GREEN;
    case ST_DOCKING:    return ((now / DOCK_FLASH_MS) & 1) ? LED_COLOR_OFF : LED_COLOR_YELLOW;
    case ST_MISALIGNED: return LED_COLOR_RED;
    case ST_DOCKED:     return LED_COLOR_BLUE;
    case ST_SERVO_MOVE: return LED_COLOR_MAGENTA;
    case ST_FALLEN:     return LED_COLOR_RED;
    case ST_IDLE:
    default:            return LED_COLOR_YELLOW;
    }
}

/**
 * @brief 颜色有变化才真正下发 (RMT 发送会阻塞到发完, 不必每周期都刷)
 */
static void apply_color(led_color_t c)
{
    if (c.r == s_last_color.r && c.g == s_last_color.g && c.b == s_last_color.b) return;
    s_last_color = c;
    led_set_color(LED_1, c);
}

static void status_led_task(void *arg)
{
    (void)arg;

    /* 上电首轮不亮粉色: 把"上次舵机变化"推到 1 秒以前 */
    s_servo_seq       = servo_get_change_seq();
    s_servo_change_ms = now_ms() - SERVO_PINK_MS;

    while (1) {
        uint32_t now = now_ms();

        /* 舵机角度变化: 记时刻, 之后 1s 亮粉 */
        uint32_t seq = servo_get_change_seq();
        if (seq != s_servo_seq) {
            s_servo_seq = seq;
            s_servo_change_ms = now;
        }

        /* 自动状态切换 → 结束手动保持, 交还给自动颜色 */
        status_t st = compute_state(now);
        if (st != s_last_state) {
            s_last_state = st;
            s_manual = false;
            ESP_LOGI(TAG, "状态灯切换: %d", (int)st);
        }

        if (!s_manual) {
            apply_color(state_color(st, now));
        }

        vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));
    }
}

esp_err_t status_led_start(void)
{
    BaseType_t ret = xTaskCreatePinnedToCore(status_led_task, "status_led",
                                             3072, NULL, 3, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "状态灯任务创建失败");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "状态灯任务已启动 (周期 %dms, 粉 %dms, 闪烁 %dms)",
             STATUS_PERIOD_MS, SERVO_PINK_MS, DOCK_FLASH_MS);
    return ESP_OK;
}

void status_led_manual_hold(void)
{
    /* 跟随灯的实际颜色, 免得手动颜色跟缓存颜色不一致导致下一轮不再刷新 */
    s_last_color = led_get_color(LED_1);
    s_manual = true;
}
