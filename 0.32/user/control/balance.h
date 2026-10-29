/**
 * @file balance.h
 * @brief 平衡控制串级控制器 (双轮同向驱动)
 *
 * @note  控制结构 (串级, 从外到内):
 *
 *   外环 (二选一, 按模式仲裁)  ──> 目标倾角 target_pitch (°)
 *     速度环 (行走时) / 位置环 (站住不动时)
 *        │
 *        ▼
 *   平衡环 (内环)  ──> 电机输出 (%)
 *
 *   输入: MPU9250 姿态角/角速度 + 编码器轮速
 *   输出: 电机 A/B PWM (两轮同向)
 *
 * @note  各控制环的 PID 运算在独立组件 user/pid/ 里:
 *        balance_pid.c (平衡环) / speed_pid.c (速度环) /
 *        steer_pid.c (转向环) / position_pid.c (位置环) / hold_pid.c (锁位速度环)。
 *        本头文件只放跨模块共享的内容: 控制周期、方向常量、
 *        行走/起立的机械参数、状态机枚举、对外 API。
 *
 * @note  转向靠倾角实现: 左倾 → 左轮减速/右轮加速 → 车往左走。
 *
 * @note  方向常量 (不在 param 表里):
 *        BAL_MOTOR_SIGN:  平衡环输出符号, 由电机装配方向决定
 *        GYRO_PITCH_SIGN: 陀螺俯仰角速度符号, 必须与 pitch 同号
 *                         (本工程: 前倾时两者都是负数)
 *
 * @note  方向键走速度环: App 方向键 → 目标轮速 → 目标倾角 → 平衡环,
 *        平衡环全程在线。
 *
 * @note  三条不经过速度环的通道:
 *        1) 起立软启动: 进入平衡状态后输出限幅从 15% 渐开到 100% (1500ms)。
 *        2) 配平 (balance_calibrate): 只采 50 个 pitch 点取平均当配平零点
 *           (存 NVS `bal`/`pz`), 不驱动电机; 姿态修正链为
 *           原始 pitch − 配平表补偿 − 配平零点。
 *        3) 未起立时的开环直驱: 平衡没开时方向键直接给 PWM。
 *
 * @author ESP32 平衡轮腿机器人
 */

#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 控制周期 ==========
 * 单一周期 10ms 跑完整串级 */
#define CONTROL_PERIOD_MS      10

/* ========== 速度环 (外环) ==========
 * 速度环把"轮速误差"换算成"目标倾角": 车要往前走, 得先往前倾一点。
 */

/* 100% 档位对应的目标轮速 (rpm): 档位 20/40/60/80 → 20/40/60/80 rpm
 * 生效上限取 param 表的 move_rpm_max, 本宏只是参数为 0 时的兜底 */
#define BAL_MOVE_RPM_MAX      100.0f
/* 目标轮速的斜坡 (rpm/s): 目标不跳变, 慢慢加上去, 免得一下子猛冲 */
#define BAL_SPD_RAMP_UP       15.0f
#define BAL_SPD_RAMP_DOWN     30.0f

/* 位置环锁位判据 (见 balance.c 的 pos_mode_t): 实测轮速 |s_speed_fwd| <= 此值
 * 才认为车停稳, 可以锁位。0.5 rpm 相对编码器量化台阶 0.15 rpm 算基本静止。 */
#define BAL_POS_SETTLE_RPM   0.5f
/* SETTLE 状态最长持续时间 (秒): 超时未停稳就强行锁位 */
#define BAL_POS_SETTLE_MAX_S 2.0f

/* 位置环输出整形 (限速 + 低通) 与其它常量同放在 user/pid/position_pid.h
 * 速度环死区 / 倾角上限 / 默认增益 → user/pid/speed_pid.h
 * 转向环增益与输出上限              → user/pid/steer_pid.h */

/* ========== 编码器方向换算 ==========
 * 编码器计数正方向 = 电机输出正方向 = 车尾方向, 换算成"前进为正"取负号。
 * 实车跑反时把本常量取反即可。 */
#define BAL_ENC_FWD_SIGN      (-1.0f)

/* ========== 遥控指令 (App 方向键) ==========
 * 方向键不直接给 PWM, 而是设目标轮速走速度环; 平衡没开时才退回开环直驱。
 *
 * 指令编码 (物模型属性 `speed`, 平台范围 -10~10):
 *     0                        → 停止
 *     ±0.2 / ±0.4 / ±0.6 / ±0.8 → 前进(+)/后退(-), 档位 20/40/60/80 (%)
 *     ±1.2 / ±1.4 / ±1.6 / ±1.8 → 左转(+)/右转(-), 档位 20/40/60/80 (%)
 * 档位值取自 motor_spd_levels[], 与 ui_encoder.c 的调速档位同源;
 * 档位代表"目标轮速的百分比"。生效上限取 param 表的 move_rpm_max,
 * BAL_MOVE_RPM_MAX 只是参数为 0 时的兜底。
 */
#define BAL_MOVE_TURN_BASE      1.0f   /* 转向指令的基址 (>1 表示转向) */

#define BAL_MOTOR_A_IS_LEFT     1

/* ========== 起立软启动 (防抖) ==========
 * 进入平衡状态后, 输出限幅从 BAL_RAMP_START_PCT 慢慢放开到 100% */
#define BAL_RAMP_START_PCT      15.0f  /* 起立瞬间允许的最大输出 (%) */
#define BAL_RAMP_TIME_MS        1500.0f/* 限幅从起始值放开到 100% 所需时间 */

/* ========== 方向常量 (不在网页上可调) ==========
 * 1) 本工程 pitch 符号: 前倾 (车头往下) 为负数, gyro 同为负。
 * 2) 电机输出为正时轮子往车尾方向转 (TB6612 接法决定)。
 * 3) 前倾 (pitch < 0) 时电机输出必须为负, 轮子才往车头跑接住车,
 *    故 BAL_MOTOR_SIGN 取 +1。
 * 改这两个值 = 拆电机 / 换 MPU 装配方向, 改完须复测。 */
#define BAL_MOTOR_SIGN         (+1.0f)
#define GYRO_PITCH_SIGN        (+1.0f)  /* 角速度与 pitch 同号 (前倾为负) */

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
    float pitch;        /* 修正后姿态角 (°): 已扣配平表补偿; 前倾 (车头往下) 为负数 */
    float gyro;         /* 俯仰方向角速度 (°/s), 与 pitch 同号 */
    float speed;        /* 两轮平均轮速 (rpm), **前进为正** */
    float trim_comp;    /* 配平表补偿量 (°), 观察补偿是否在工作 */
    float pitch_raw;    /* 修正前的原始 Pitch (°), 对照用 */

    /* --- 控制器中间量 --- */
    float target_pitch; /* 速度环算出的目标倾角 (°) */
    float bal_out;      /* 平衡环输出 (%, 已乘 BAL_MOTOR_SIGN) */
    float yaw_out;      /* 差速环输出 (%, 正 = 左轮更快) */
    float dt;           /* 本周期实测间隔 (秒), 正常应≈0.01 */
    float position_cm;  /* 里程计位移 (cm, 前进为正), 相对起立时的原点 */
    float pos_out;      /* 位置环要求的倾角 (°, 正 = 车头往前倾), 直接进平衡环 */

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
 * @brief 设置遥控指令 (App 方向键)
 * @param dir  方向 (前进/后退/左转/右转)
 * @param level_pct 档位百分比 (会被吸附到 20/40/60/80 之一)
 * @note  正常情况 (平衡已开) 下, 指令只用来设**目标轮速**,
 *        由速度环换算成目标倾角, 平衡环全程在线。
 *        只有平衡没开 (车没起立) 时才退回开环直驱 (直接给 PWM),
 *        方便单独调电机/编码器。
 * @return ESP_OK 成功
 */
esp_err_t balance_set_move(bal_move_t dir, int8_t level_pct);

/**
 * @brief 清除遥控指令 (目标轮速归零, 平衡环接管)
 * @note  清除时会清速度环/差速环的积分, 免得积分残留让车继续歪着走
 */
void balance_clear_move(void);

/**
 * @brief 配平: 记录当前姿态角作为平衡零点
 * @note  不动电机: 连续采样约 0.5s 的 pitch 取平均存成配平零点,
 *        之后平衡环以这个角度为 0。
 * @return ESP_OK 成功; ESP_ERR_INVALID_STATE MPU 不可用
 */
esp_err_t balance_calibrate(void);

/**
 * @brief 读取已记录的配平零点 (°)
 * @return 零点角度; 没标定过返回 0
 */
float balance_get_pitch_zero(void);

/**
 * @brief 记录一个配平表标定点 (当前舵机角 → 当前原始 pitch)
 * @return ESP_OK 记录成功; ESP_ERR_INVALID_STATE MPU 不可用;
 *         ESP_ERR_INVALID_ARG pitch 超出合理范围 (trim 层判定);
 *         ESP_ERR_NO_MEM 表已满 (最多 16 个点)
 * @note  存的是原始 pitch (pitch_raw, 互补滤波输出, 未扣任何补偿),
 *        修正链里还会再扣一次 trim_pitch()。
 * @note  舵机角取 servo_get_angle(SERVO_1) (两路舵机逻辑角始终相同)。
 */
esp_err_t balance_record_trim_point(void);

/**
 * @brief 把配平表导出成 JSON 字符串
 * @param buf 输出缓冲区
 * @param len 缓冲区长度
 * @return 写入的字符数 (0 = 缓冲区不够)
 * @note  格式: {"n":3,"ref":80,"pz":0.0,"rp":-1.2,"pts":[[40,-2.1],[60,-0.4],[80,0.0]]}
 *        ref = 配平表参考角 (param 里的 servo_c), pz = 当前配平零点 (°),
 *        rp = 当前原始 pitch (°), pts = [舵机角, 原始 pitch]
 */
int balance_get_trim_json(char *buf, uint32_t len);

#endif // BALANCE_H
