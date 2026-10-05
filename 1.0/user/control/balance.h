/**
 * @file balance.h
 * @brief 平衡控制串级控制器 (骨架版: 接口已定型, 算法下一步填)
 *
 * @note  控制结构 (串级, 从外到内):
 *
 *   速度环 (外环, 慢)  ──> 目标倾角 target_pitch (°)
 *        │
 *        ▼
 *   平衡环 (内环, 快)  ──> 电机差速 drive (%)
 *        │
 *        ▼
 *   转向环 (并联)      ──> 左右轮差速修正 steer (%)
 *
 *   输入传感器: MPU9250 姿态角/角速度 + 编码器轮速
 *   输出执行器: 电机 A/B PWM + 两路舵机
 *
 * @note  为什么平衡环输出是"电机差速"而不是"舵机角度":
 *        这台车的平衡靠两轮差速 (坦克转向原理) 调整重心,
 *        舵机负责腿部姿态/转向, 不参与主平衡回路。
 *
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "pid.h"

/* ========== 控制周期 ==========
 * 内环 (平衡环) 必须快, 否则车会倒; 外环 (速度环) 可以慢。
 * 本版用单一周期 10ms 跑完整串级, 简单可靠。
 * ⚠ 下一步实现时如果内环不够快, 就把平衡环拆成独立的高频任务。 */
#define CONTROL_PERIOD_MS      10
#define CONTROL_DT            (CONTROL_PERIOD_MS / 1000.0f)

/* ========== 运行状态机 ========== */
typedef enum {
    BAL_STATE_IDLE = 0,     /* 未启动, 电机刹车, 等姿态稳定 */
    BAL_STATE_STANDBY,      /* 已上电但未解锁 (K1 触发) */
    BAL_STATE_BALANCING,    /* 平衡中, 电机输出跟随控制器 */
    BAL_STATE_FALLEN,       /* 检测到摔倒, 电机已停, 等重新扶正 */
} bal_state_t;

/* ========== 控制器运行数据 (供 UI / 网页显示, 也便于调参时看波形) ========== */

/**
 * @brief 一个控制周期的中间量快照
 * @note  这些量全部来自 param_get_float(), 网页滑块改了下一次运算立刻生效
 */
typedef struct {
    /* --- 传感器输入 --- */
    float pitch;        /* 姿态角 (°), 已扣零偏 PARAM_ANGLE_ZERO 和腿角补偿 */
    float gyro_x;       /* 俯仰方向角速度 (°/s) */
    float speed;        /* 两轮平均轮速 (rpm), 正值=前进 */
    float leg_comp;     /* 本周期腿角补偿量 (°), 观察补偿是否在工作 */

    /* --- 控制器中间量 --- */
    float target_pitch; /* 速度环算出的目标倾角 (°) */
    float bal_out;      /* 平衡环输出 (电机差速 %) */
    float spd_out;      /* 速度环输出 (目标倾角 °) */
    float steer_out;    /* 转向环输出 (左右差速修正 %) */

    /* --- 实际执行输出 --- */
    int8_t motor_a;     /* 电机 A 实际输出 (-100~100) */
    int8_t motor_b;     /* 电机 B 实际输出 (-100~100) */

    /* --- 状态 --- */
    bal_state_t state;  /* 当前状态机状态 */
} bal_data_t;

/**
 * @brief 初始化平衡控制模块
 * @note  读取 param 里的默认参数, 复位三个 PID, 启动控制任务
 * @return ESP_OK 成功
 */
esp_err_t balance_init(void);

/**
 * @brief 启动 / 停止平衡控制
 * @param on true=开始尝试平衡 (进入 STANDBY, 车扶正后转 BALANCING)
 *              false=立即停机 (电机刹车, 舵机回中心角)
 * @return ESP_OK 成功
 */
esp_err_t balance_set_enable(bool on);

/**
 * @brief 查询控制是否已启动
 * @return true=控制任务正在尝试平衡
 */
bool balance_is_enabled(void);

/**
 * @brief 急停: 立即刹车并进入 IDLE
 * @note  与 balance_set_enable(false) 的区别: 这个会清空所有 PID 积分
 *        (摔倒后再启动时不会带着旧积分猛冲)
 * @return ESP_OK 成功
 */
esp_err_t balance_emergency_stop(void);

/**
 * @brief 读取最近一个控制周期的数据快照
 * @param out 输出缓冲区
 * @return ESP_OK 成功
 */
esp_err_t balance_get_data(bal_data_t *out);

/**
 * @brief 读取当前状态机状态
 * @return 当前状态
 */
bal_state_t balance_get_state(void);

/**
 * @brief 从 param 表重新读取所有控制参数
 * @note  控制任务每周期本来就调 param_get_float(), 所以这个函数只是
 *        强制复位 PID 积分 (调大 Kp 之后建议调一次, 避免旧积分残留)
 * @return ESP_OK 成功
 */
esp_err_t balance_reload_params(void);

#endif // BALANCE_H
