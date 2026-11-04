/**
 * @file rolltest.c
 * @brief 横滚角实测工具实现
 *
 * 命令 (一行一条, 回车结束):
 *   help              打印命令表
 *   r <左> <右>       两腿设到指定角度, 静止 1 s 后测横滚角
 *   sweep             自动扫完 40°/100° 两组基准角 × 各档角度差, 输出制表行
 *   comp on|off       横滚补偿使能开关
 */

#include "rolltest.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hal/usb_serial_jtag_ll.h"

#include "servo.h"
#include "roll_compensator.h"
#include "balance.h"

static const char *TAG = "ROLLTEST";

#define LINE_LEN        64
#define POLL_MS         20
#define SETTLE_MS       1000    /* 改完角度到开始采样 */
#define SAMPLE_MS       10
#define SAMPLE_COUNT    100     /* 1 s 平均 */

/* 扫描用基准角 (两腿平均) 与角度差档位 */
static const uint8_t k_bases[]   = { 40, 100 };
static const uint8_t k_deltas[]  = { 0, 4, 8, 12, 16, 20 };

/**
 * @brief 测量一组舵机角度下的横滚角
 * @param left 左腿角度 (°)
 * @param right 右腿角度 (°)
 * @return 横滚角平均值 (°)
 */
static float measure_roll(uint8_t left, uint8_t right)
{
    servo_set_angle(SERVO_1, left);
    servo_set_angle(SERVO_2, right);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

    float sum = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sum += servo_get_roll_angle();
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
    return sum / (float)SAMPLE_COUNT;
}

/**
 * @brief 测一组并打印制表行
 * @param base 两腿平均角度 (°)
 * @param delta 角度差 (°), 正 = 左腿伸长
 */
static void measure_and_print(int base, int delta)
{
    int left  = base - delta / 2;
    int right = base + delta / 2;
    float roll = measure_roll((uint8_t)left, (uint8_t)right);

    printf("[ROLLTEST] { %3d, %3d, %6.2ff },   left=%d right=%d\n",
           base, delta, roll, left, right);
    fflush(stdout);
}

static void print_help(void)
{
    printf("[ROLLTEST] 命令: help | r <左> <右> | sweep | comp on|off\n");
    fflush(stdout);
}

static void run_sweep(void)
{
    if (balance_is_enabled()) {
        printf("[ROLLTEST] 平衡运行中, 先停车再测\n");
        fflush(stdout);
        return;
    }
    if (roll_comp_is_enabled()) {
        printf("[ROLLTEST] 补偿开着会把测量结果污染, 先 comp off\n");
        fflush(stdout);
        return;
    }

    printf("[ROLLTEST] 开始扫描 (每组约 2 s)\n");
    fflush(stdout);

    for (unsigned i = 0; i < sizeof(k_bases); i++) {
        for (unsigned j = 0; j < sizeof(k_deltas); j++) {
            measure_and_print(k_bases[i], k_deltas[j]);
        }
    }

    /* 收尾回到标准站姿 */
    servo_set_both(40);
    printf("[ROLLTEST] 扫描结束, 舵机回 40°\n");
    fflush(stdout);
}

/**
 * @brief 处理一行命令
 */
static void handle_line(char *line)
{
    if (strncmp(line, "help", 4) == 0) {
        print_help();
    } else if (strncmp(line, "sweep", 5) == 0) {
        run_sweep();
    } else if (strncmp(line, "comp", 4) == 0) {
        if (strstr(line, "on")) {
            roll_comp_set_enable(true);
            printf("[ROLLTEST] 补偿已开\n");
        } else if (strstr(line, "off")) {
            roll_comp_set_enable(false);
            printf("[ROLLTEST] 补偿已关\n");
        } else {
            printf("[ROLLTEST] 补偿: %s\n", roll_comp_is_enabled() ? "开" : "关");
        }
        fflush(stdout);
    } else if (line[0] == 'r' && line[1] == ' ') {
        int left = 0, right = 0;
        if (balance_is_enabled()) {
            printf("[ROLLTEST] 平衡运行中, 先停车再测\n");
            fflush(stdout);
        } else if (sscanf(line + 2, "%d %d", &left, &right) == 2) {
            measure_and_print((left + right) / 2, right - left);
        } else {
            printf("[ROLLTEST] 用法: r <左> <右>\n");
            fflush(stdout);
        }
    } else {
        print_help();
    }
}

static void rolltest_task(void *arg)
{
    (void)arg;
    char line[LINE_LEN];
    int  len = 0;

    print_help();

    for (;;) {
        uint8_t c;
        if (usb_serial_jtag_ll_read_rxfifo(&c, 1) != 1) {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                line[len] = '\0';
                handle_line(line);
                len = 0;
            }
            continue;
        }
        if (len < LINE_LEN - 1) line[len++] = (char)c;
    }
}

esp_err_t rolltest_start(void)
{
    BaseType_t ret = xTaskCreatePinnedToCore(rolltest_task, "rolltest",
                                             4096, NULL, 3, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "测量任务创建失败");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "横滚角测量任务已启动 (help 查看命令)");
    return ESP_OK;
}
