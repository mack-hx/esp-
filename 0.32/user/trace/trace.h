/**
 * @file trace.h
 * @brief 控制回路跟踪: 环形缓冲采集 + 独立任务回灌串口
 *
 * @note  为什么单独一个组件 (而不是塞在 balance.c 里):
 *   1) **串口写会阻塞**。UART 波特有限, 一次写一百多字节要几毫秒;
 *      在控制任务里打印等于给平衡内环加了一段**几十毫秒的延迟** ——
 *      内环本来就在相位裕度边缘, 这一下足以把车顶出振荡,
 *      而且测出来的数据本身就是被打印污染的。
 *   2) 所以这里把两件事拆开:
 *      - **采集** (trace_push): 只往内存环形缓冲里写, 控制任务调用,
 *        耗时几百纳秒, **完全不碰串口**。
 *      - **输出** (trace 任务): 实时打印 + 收到请求后回灌历史,
 *        全部在一个钉在 **core 0** 的独立任务里做
 *        (控制任务钉在 core 1), 连 CPU 都不跟控制环抢。
 *
 * @note  数据格式 (固件 / 采集脚本 / 分析脚本 三方统一):
 *   TRACE t=1.234 pitch=-0.50 gyro=-12.3 out=8.0 tgt=0.20 spd=2.0
 *         yaw=0.0 pos=1.2 pos_out=0.0 kd=0.40 dt=10.0 st=2
 *   对应 CSV 表头:
 *     t_ms,pitch,gyro,out,tgt,spd,yaw,pos,pos_out,kd,dt,st
 *
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
    float pos_out;    /* 位置环输出 (rpm) */
    float kd;         /* 采样时刻的平衡环 Kd (参数可热改, 必须逐条记) */
    float dt;         /* 本周期实测间隔 (ms) */
    int8_t st;        /* 状态机状态 (bal_state_t) */
} trace_sample_t;

/**
 * @brief 初始化跟踪模块 (创建输出任务)
 * @note  必须在控制任务启动**之前**调用
 */
void trace_init(void);

/**
 * @brief 采一条样存进环形缓冲 (控制任务调用, 不做串口 I/O)
 * @param s 采样数据 (内部按值拷贝)
 * @note  耗时几百纳秒, 每个控制周期调一次即可 (10ms → 100Hz)
 */
void trace_push(const trace_sample_t *s);

/**
 * @brief 请求把环形缓冲里的历史回灌到串口 (控制任务调用, 不阻塞)
 * @note  真正的打印在独立任务里做, 调用方立即返回
 */
void trace_request_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* TRACE_H */
