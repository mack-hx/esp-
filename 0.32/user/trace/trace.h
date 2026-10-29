/**
 * @file trace.h
 * @brief 控制回路跟踪: 环形缓冲采集 + 独立任务回灌串口
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-27
 */

#ifndef TRACE_H
#define TRACE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 一条控制回路采样 ========== */
typedef struct {
    float t;          /* esp_timer 时刻 (秒) */
    float pitch;      /* 修正后姿态角 (°) */
    float gyro;       /* 俯仰角速度 (°/s) */
    float out;        /* 平衡环输出 (%) */
    float tgt;        /* 目标倾角 (°) */
    float spd;        /* 实测轮速 (rpm) */
    float yaw;        /* 差速输出 (%) */
    float pos;        /* 位置 (cm) */
    float pos_err;    /* 位置误差 (cm): 目标位置 - 当前位置 */
    float pos_out;    /* 位置环要求的倾角 (°, 正 = 车头往前倾) */
    float pos_vel;    /* 里程计速度 (cm/s): 位置环的阻尼输入, 看它对不对 */
    float kd;         /* 采样时刻的平衡环 Kd (参数可热改, 必须逐条记) */
    float dt;         /* 本周期实测间隔 (ms) */
    int8_t st;        /* 状态机状态 (bal_state_t) */
} trace_sample_t;

/**
 * @brief 初始化跟踪模块 (创建输出任务)
 */
void trace_init(void);

/**
 * @brief 采一条样存进环形缓冲 (控制任务调用, 不做串口 I/O)
 * @param s 采样数据 (内部按值拷贝)
 */
void trace_push(const trace_sample_t *s);

/**
 * @brief 请求把环形缓冲里的历史回灌到串口 (控制任务调用, 不阻塞)
 */
void trace_request_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* TRACE_H */
