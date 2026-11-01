/**
 * @file trim.h
 * @brief 配平表: 舵机角 (°) → 把车身扶到平衡位时的俯仰角 (°)
 *
 * @brief 表是一维的: 舵机角 → 平衡俯仰角。标定步骤 (网页上做):
 *        扶住车 → 拨腿到某角度 → 手动扶车身到接近平衡 → 记下 Pitch,
 *        30°~100° 铺 6~8 个点 → 保存到 NVS。
 *
 * @note  补偿取相对参考角的差值:
 *          trim_pitch(φ) = 表插值(φ) − 表插值(φ_ref)
 *        φ_ref = param 里的 servo_c (舵机中心角), 该档补偿天然为 0。
 *
 * @author ESP32 平衡轮腿机器人
 */

#ifndef TRIM_H
#define TRIM_H

#include <stdint.h>
#include <stdbool.h>

/* 离线测试 (gcc 直接跑, -DTRIM_OFFLINE_TEST) 时用 test/trim_offline.h
 * 代替 ESP-IDF 的 esp_err.h; 桩头由本头文件自己带进来。 */
#ifdef TRIM_OFFLINE_TEST
#include "trim_offline.h"
#else
#include "esp_err.h"
#endif

/* ========== 容量 ==========
 * 舵机行程 30~100°, 每 5° 一个点, 最多 16 个点 */
#define TRIM_MAX_POINTS     16

/* ========== 俯仰角合理范围 (°) ==========
 * 超出这个范围说明 MPU 没焊好或者数据坏了, 直接拒收。 */
#define TRIM_PITCH_MIN      (-30.0f)
#define TRIM_PITCH_MAX      30.0f

/* ========== 一条标定记录 ========== */
typedef struct {
    float servo_deg;    /* 舵机逻辑角 (°), 与 servo_get_angle() 同量纲 */
    float pitch_deg;    /* 把车身扶到接近平衡时的 Pitch (°) */
} trim_point_t;

/* ========== 标定表 ========== */
typedef struct {
    uint8_t        count;                    /* 已录入的点数 */
    trim_point_t   pts[TRIM_MAX_POINTS];    /* 按 servo_deg 升序存放 */
} trim_t;

/* ================= API ================= */

/**
 * @brief 从 NVS 载入配平表
 * @return ESP_OK 成功 (无数据时用空表, 补偿量恒为 0)
 * @note  balance_init() 内部调用, 一般不直接用
 */
esp_err_t trim_load(void);

/**
 * @brief 保存配平表到 NVS
 * @return ESP_OK 成功
 * @note  网页点「保存到 NVS」时调用
 */
esp_err_t trim_save(void);

/**
 * @brief 取得配平表指针 (只读)
 * @return 指向全局表的指针, 不会为 NULL
 * @note  外部不要直接改; 要改走 trim_set_point()
 */
const trim_t *trim_get(void);

/**
 * @brief 某个舵机角对应的"平衡俯仰角" (插值后)
 * @param servo_deg 舵机逻辑角 (°)
 * @param ref_deg   参考角 (°), 通常是 servo_c —— 补偿在这个角度为 0
 * @return 补偿量 (°), = 表插值(servo_deg) − 表插值(ref_deg)
 *
 * @note  表点数 < 2 时返回 0 (不做补偿), 行为和没标定时完全一致。
 * @note  舵机角超出标定范围时夹到端点, 不外推 (外推值无意义)。
 */
float trim_pitch(float servo_deg, float ref_deg);

/**
 * @brief 录入/更新一个配定点
 * @param servo_deg 舵机逻辑角 (°)
 * @param pitch_deg 该姿态的 Pitch (°)
 * @return ESP_OK 成功; ESP_ERR_INVALID_ARG 角度超范围; ESP_ERR_NO_MEM 点数已满
 * @note  按舵机角升序插入 (不是 append)。已有同角度则覆盖。
 */
esp_err_t trim_set_point(float servo_deg, float pitch_deg);
/**
 * @brief 清空配平表 (只改内存)
 * @return ESP_OK 成功
 */
esp_err_t trim_clear(void);
/**
 * @brief 检查配平表是否可用 (点数 >= min_points 且俯仰角有合理变化)
 * @param min_points 最少点数
 * @return true=可用
 *
 * @note  俯仰角跨度过小 (< 0.05°) 视为数据无效。
 */
bool trim_is_usable(uint8_t min_points);

/**
 * @brief 取表内俯仰角的跨度 (最大值 − 最小值, °)
 * @return 跨度; 表空返回 0
 * @note  网页用来判断"这张表有没有实际内容"
 */
float trim_pitch_span(void);

#endif // TRIM_H
