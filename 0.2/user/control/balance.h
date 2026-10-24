/**
 * @file balance.h
 * @brief 平衡控制串级控制器 (双轮同向驱动)
 *
 * @note  控制结构 (串级, 从外到内):
 *
 *   速度环 (外环)  ──> 目标倾角 target_pitch (°)
 *        │
 *        ▼
 *   平衡环 (内环)  ──> 电机输出 (%)
 *
 *   输入: MPU9250 姿态角/角速度 + 编码器轮速
 *   输出: 电机 A/B PWM (两轮同向)
 *
 * @note  为什么是"双轮同向"而不是"差速/坦克转向":
 *        差速只能让车**原地转**, 产生不了把重心托回来所需的平移加速度,
 *        扶不住倒下的车身。要托住重心必须让两轮**一起**朝倒的方向跑,
 *        靠轮子的牵引力把重心甩到轮轴上方。
 *
 * @note  方向常量 (0.13 起从 param 表里删了, 烘焙进代码):
 *        BAL_MOTOR_SIGN:  平衡环输出符号, 由本机电机装配方向决定
 *                         改电机接线 / 反装编码器后, 才需要重新定
 *        GYRO_PITCH_SIGN: 陀螺俯仰角速度符号, 由 MPU 焊接朝向决定
 *                         0.11 第八阶段实测 +1 正确
 *
 * @note  三条"不经过速度环"的通道 (0.2 版, 2026-10-24):
 *        1) 遥控直驱 (balance_set_move): App 方向键直接给 PWM, 跟 UI
 *           编码器调试页一样, 车没起立也能让轮子转; 前进/后退两轮同向,
 *           左/右转两轮反向 (原地转向)。直驱期间暂停平衡环与状态机。
 *        2) 起立软启动: 进入平衡状态后输出限幅从 15% 渐开到 100% (1500ms),
 *           避免车还歪着时 PID 输出顶满导致电机猛冲抖动。
 *        3) 配平 (balance_calibrate): 只采 50 个 pitch 点取平均当"配平零点"
 *           (存 NVS `bal`/`pz`), **不驱动电机**; 姿态修正链为
 *           原始 pitch − 配平表补偿 − 配平零点。
 *        速度环目标恒为 0 (原地站立), 要走位一律走直驱通道。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-07
 */

#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "pid.h"

/* ========== 控制周期 ==========
 * 内环 (平衡环) 必须快, 否则车会倒; 外环 (速度环) 可以慢。
 * 本版用单一周期 10ms 跑完整串级, 简单可靠。 */
#define CONTROL_PERIOD_MS      10

/* ========== 速度环 (外环) ==========
 * 速度环把"轮速误差"换算成"目标倾角": 车要往前走, 得先往前倾一点。
 * 遥控指令走的是**直驱**通道 (balance_set_move), 不经过速度环,
 * 所以这里的目标轮速恒为 0 (原地站立, 不主动走)。 */

/* ========== 遥控直驱 (App 方向键) ==========
 * App 的方向键不经过速度环, 而是**直接给电机 PWM** (跟 UI 编码器调试页
 * ui_encoder.c 的行为完全一样), 这样在车没起立/没平衡时也能看到轮子转。
 *
 * 指令编码 (物模型属性 `speed`, 平台范围 -10~10):
 *     0                      → 停止
 *     ±0.2 / ±0.4 / ±0.6 / ±0.8 → 前进(+)/后退(-), 档位 20/40/60/80 (%)
 *     ±1.2 / ±1.4 / ±1.6 / ±1.8 → 左转(+)/右转(-), 档位 20/40/60/80 (%)
 * 档位值取自 motor_spd_levels[], 与 ui_encoder.c 的调速档位同源。
 */
#define BAL_MOVE_TURN_BASE      1.0f   /* 转向指令的基址 (>1 表示转向) */

/* ========== 转向时哪个电机是左轮 ==========
 * 1 = 电机 A 是左轮 (与"motor_l 上报 motor_a"的约定一致)
 * 若实际按左转却向右转, 把这里改成 0 即可, 不用改别的代码。 */
#define BAL_MOTOR_A_IS_LEFT     1

/* ========== 起立软启动 (防抖) ==========
 * 起立瞬间电机不能一上来就全力输出: 车多半还歪着/被人扶着,
 * 误差一大 PID 输出直接顶到限幅, 电机就会"猛冲"一下, 看着像剧烈抖动。
 * 所以进入平衡状态后, 输出限幅从 BAL_RAMP_START_PCT 慢慢放开到 100%。 */
#define BAL_RAMP_START_PCT      15.0f  /* 起立瞬间允许的最大输出 (%) */
#define BAL_RAMP_TIME_MS        1500.0f/* 限幅从起始值放开到 100% 所需时间 */


/* ========== 方向常量 (0.13 起不在网页上可调) ==========
 * 实测 2026-10-05: 启动后车往倒的反方向冲 → 平衡输出要反相 → -1
 * 陀螺符号: 第八阶段实测前倾 gx 为正、Pitch 也为正 → +1
 *
 * 改这两个值意味着: 拆电机、换 MPU 装配方向, 不是日常调试。 */
#define BAL_MOTOR_SIGN         (-1.0f)
#define GYRO_PITCH_SIGN        (+1.0f)

/* ========== 运行状态机 ========== */
typedef enum {
    BAL_STATE_IDLE = 0,     /* 未启动, 电机刹车 */
    BAL_STATE_STANDBY,      /* 已启动, 等姿态连续稳定后放行 (门控) */
    BAL_STATE_BALANCING,    /* 平衡中, 电机输出跟随控制器 */
    BAL_STATE_FALLEN,       /* 检测到摔倒, 电机已停, 等重新扶正 */
} bal_state_t;

/* ========== 遥控直驱方向 ========== */
typedef enum {
    BAL_MOVE_STOP  = 0,     /* 停止 */
    BAL_MOVE_FWD,           /* 前进 (两轮同向) */
    BAL_MOVE_BACK,          /* 后退 (两轮同向反向) */
    BAL_MOVE_LEFT,          /* 左转 (左轮反转 + 右轮正转, 原地转向) */
    BAL_MOVE_RIGHT,         /* 右转 (左轮正转 + 右轮反转, 原地转向) */
} bal_move_t;


/* ========== 一个控制周期的中间量快照 (供 UI / 网页显示) ========== */

typedef struct {
    /* --- 传感器输入 --- */
    float pitch;        /* 修正后姿态角 (°): 已扣配平表补偿 */
    float gyro;         /* 俯仰方向角速度 (°/s), 与 pitch 同向为正 */
    float speed;        /* 两轮平均轮速 (rpm), 正值=前进 */
    float trim_comp;    /* 配平表补偿量 (°), 观察补偿是否在工作 */
    float pitch_raw;    /* 修正前的原始 Pitch (°), 对照用 */

    /* --- 控制器中间量 --- */
    float target_pitch; /* 速度环算出的目标倾角 (°) */
    float bal_out;      /* 平衡环输出 (%, 已乘 BAL_MOTOR_SIGN) */
    float spd_out;      /* 速度环输出 (目标倾角 °) */
    float dt;           /* 本周期实测间隔 (秒), 正常应≈0.01 */

    /* --- 实际执行输出 --- */
    int8_t motor_a;     /* 电机 A 实际输出 (-100~100) */
    int8_t motor_b;     /* 电机 B 实际输出 (-100~100), 与 A 同向 */

    /* --- 状态 --- */
    bal_state_t state;  /* 当前状态机状态 */
} bal_data_t;

/* ================= 对外 API ================= */

/**
 * @brief 初始化平衡控制模块
 * @note  读取 param 参数, 载入配平表, 启动控制任务
 * @return ESP_OK 成功
 */
esp_err_t balance_init(void);

/**
 * @brief 启动 / 停止平衡控制
 * @param on true=进入 STANDBY 等门控放行
 *              false=立即停机 (电机刹车, 舵机回中心角)
 * @return ESP_OK 成功; ESP_ERR_INVALID_STATE 车歪着时拒绝启动
 */
esp_err_t balance_set_enable(bool on);

/**
 * @brief 查询控制是否已启动
 * @return true=控制任务正在尝试平衡
 */
bool balance_is_enabled(void);

/**
 * @brief 急停: 立即刹车并进入 IDLE
 * @note  会清空 PID 积分, 摔倒后再启动时不会带着旧积分猛冲
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
 * @brief 读取启动门控的已累计稳定时长
 * @return 毫秒; 0=当前不满足门控条件 (或还没 enable)
 * @note  与 PARAM_ARM_TIME 比较即可知道还要等多久。
 *        0 且状态为 STANDBY → 条件没满足 (车在晃或倾角超了)
 */
float balance_get_arm_ms(void);

/**
 * @brief 从 param 表重新读取所有控制参数并清 PID 积分
 * @note  调大 Kp 之后建议调一次, 避免旧积分残留导致猛冲
 * @return ESP_OK 成功
 */
esp_err_t balance_reload_params(void);

/**
 * @brief 设置遥控直驱指令 (App 方向键)
 * @param dir  方向 (前进/后退/左转/右转)
 * @param level_pct 档位百分比 (会被吸附到 20/40/60/80 之一)
 * @note  直驱期间**暂停平衡环输出** (电机直接由本指令驱动),
 *        跟 UI 编码器调试页一样是开环动作, 车会跑, 需要扶着。
 * @return ESP_OK 成功
 */
esp_err_t balance_set_move(bal_move_t dir, int8_t level_pct);

/**
 * @brief 清除遥控直驱指令 (电机交回平衡环; 若平衡没开则刹车)
 * @note  清除时会清 PID 积分, 免得直驱期间攒的积分让平衡环一恢复就猛冲
 */
void balance_clear_move(void);

/**
 * @brief 读取当前遥控直驱指令
 * @param out_dir    [出参] 当前方向 (可为 NULL)
 * @param out_level  [出参] 当前档位百分比 (可为 NULL)
 */
void balance_get_move(bal_move_t *out_dir, int8_t *out_level);

/**
 * @brief 配平: 记录当前姿态角作为平衡零点
 * @note  **不动电机**: 只在控制任务里连续采样约 0.5s 的 pitch 取平均,
 *        存成"配平零点", 之后平衡环以这个角度为 0。
 *        之前这个动作是"停平衡 → 再开平衡", 电机会在重新起立时猛冲,
 *        看着像剧烈抖动, 其实配平和电机没关系。
 * @return ESP_OK 成功; ESP_ERR_INVALID_STATE MPU 不可用
 */
esp_err_t balance_calibrate(void);

/**
 * @brief 读取已记录的配平零点 (°)
 * @return 零点角度; 没标定过返回 0
 */
float balance_get_pitch_zero(void);

#endif // BALANCE_H
