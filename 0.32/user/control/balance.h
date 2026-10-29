/**
 * @file balance.h
 * @brief 平衡控制串级控制器 (双轮同向驱动)
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-07
 */

#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 控制周期 ==========
 * 内环 (平衡环) 必须快, 否则车会倒; 外环 (速度环) 可以慢。
 * 本版用单一周期 10ms 跑完整串级, 简单可靠。 */
#define CONTROL_PERIOD_MS      10

/* ========== 速度环 (外环) ==========
 * 速度环把"轮速误差"换算成"目标倾角": 车要往前走, 得先往前倾一点。
 * App 方向键走这条通道: 设目标轮速 → 目标倾角 → 平衡环一直在线。
 *
 * 约定: 全部用"**前进为正**"的轮速 (rpm)。
 *   前进 = 车头方向 (车头往下、pitch 为负 的那个方向)。
 *   编码器的正方向 = 电机输出正方向 = **车尾**方向 (见下面的说明),
 *   所以前进速度 = −编码器速度, 换算符号见 BAL_ENC_FWD_SIGN。
 */

/* 100% 档位对应的目标轮速 (rpm): 档位 20/40/60/80 → 20/40/60/80 rpm
 *
 * 取 100: 本机轮子最高转速 333 rpm (空载), 100 rpm 约为 30% —— 留足余量,
 * 速度环还能跟得上 (真机速度环的跟踪能力在 100 rpm 以内是够的)。
 * 再往上目标倾角会顶到 SPD_PID_WALK_PITCH_MAX (4°) 安全阀,
 * 表现为车一直倾着追速度、左右猛摇。
 *
 * ⚠ 真正生效的上限是**参数表里的 move_rpm_max** (App 参数页可实时调),
 *   这个宏只是"参数为 0 时的兜底" (见 balance.c 的 balance_move_rpm_max())。 */
#define BAL_MOVE_RPM_MAX      100.0f

/* 目标轮速的斜坡 (rpm/s): 目标不跳变, 慢慢加上去, 免得一下子猛冲 */
#define BAL_SPD_RAMP_UP       15.0f
#define BAL_SPD_RAMP_DOWN     30.0f

/* 位置环"锁位"的判据 (见 balance.c 的 pos_mode_t):
 *   松手后车会滑行一段才停。滑行期间**不能**锁目标位置 ——
 *   锁的是滑行途中的位置, 车停稳后位置误差会突然变大 → 猛纠 → 前后大晃。
 *   判据: 实测轮速 |s_speed_fwd| <= 此值 就认为车停稳了, 可以锁位。
 *   取 0.5 rpm: 编码器量化台阶 0.15 rpm, 0.5 已经算"基本静止"。 */
#define BAL_POS_SETTLE_RPM   0.5f

/* SETTLE 状态最长持续时间 (秒): 超时还没停稳就强行锁位,
 *   免得车一直微幅晃动时永远进不了 HOLD (位置环就永远不接管)。 */
#define BAL_POS_SETTLE_MAX_S 2.0f

/* ⚠ 位置环的输出整形 (限速 + 低通) 放在 user/pid/position_pid.h 里,
 *   跟位置环的其它常量放在一起, 也能离线单测。 */

/* ⚠ 速度环的误差死区 / 倾角上限 / 默认增益 → 见 user/pid/speed_pid.h
 * ⚠ 转向环 (差速环) 的增益与输出上限   → 见 user/pid/steer_pid.h
 */

/* ========== 编码器方向换算 ==========
 * ⚠ 编码器计数的正方向 = 电机输出正方向 (encoder.c 里把左轮的计数方向
 *   反过来, 就是为了让它跟电机方向一致),
 *   而电机输出为正 = 轮子往**车尾**方向转 (TB6612 接法决定的)。
 *   → 编码器正方向 = 车尾 = 机器人**后退**方向。
 *   → 换算成"前进为正"要取负号。
 *
 * ⚠ 如果实车跑反了 (按前进时串口日志里 raw rpm 是正的),
 *   把这个常量取反即可, 其它地方都不用动。 */
#define BAL_ENC_FWD_SIGN      (-1.0f)

/* ========== 遥控指令 (App 方向键) ==========
 * App 的方向键**不再直接给 PWM**, 而是设目标轮速, 走速度环 (见文件头说明)。
 * 平衡没开 (车没起立) 时才退回开环直驱, 方便单独调电机/编码器。
 *
 * 指令编码 (物模型属性 `speed`, 平台范围 -10~10):
 *     0                        → 停止
 *     ±0.2 / ±0.4 / ±0.6 / ±0.8 → 前进(+)/后退(-), 档位 20/40/60/80 (%)
 *     ±1.2 / ±1.4 / ±1.6 / ±1.8 → 左转(+)/右转(-), 档位 20/40/60/80 (%)
 * 档位值取自 motor_spd_levels[], 与 ui_encoder.c 的调速档位同源;
 * 档位现在代表"目标轮速的百分比" (100% = 目标轮速上限)。
 * ⚠ 真正生效的上限是**参数表里的 move_rpm_max** (默认 100 rpm, App 可实时调),
 *   BAL_MOVE_RPM_MAX 这个宏只是"参数为 0 时的兜底" (见 balance.c 的
 *   balance_move_rpm_max())。改 move_rpm_max 后各档的 rpm 要跟着重算
 *   (App 物模型文档里的指令表已经按 100 rpm 写好了)。
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


/* ========== 方向常量 (不在网页上可调) ==========
 *
 *   1) 本工程的 pitch 符号: **前倾 (车头往下) 为负数**。
 *      MPU 装在车体上时 Y 轴朝车头, 加速度算出来的俯仰角 = -(前倾角);
 *      实测: 车头下压时 pitch 从 -1.4 一路走到 -22°, 角速度 gyro 同为负。
 *
 *   2) 电机输出的极性: 电机输出为**正**时, 轮子往**车尾**方向转
 *      (TB6612 的 AIN1/AIN2 接法决定的)。
 *
 *   3) 平衡环要"往倒的方向追": 前倾 (pitch < 0) 时, 电机输出必须是**负**的,
 *      轮子才会往车头跑出去接住车。
 *      → out = bal_out * BAL_MOTOR_SIGN, 要让 bal_out<0 → out<0,
 *        BAL_MOTOR_SIGN 必须取 **+1**。
 *
 *   取反的两种等价改法: ① BAL_MOTOR_SIGN 改 +1 (本工程采用, 只动一个常量);
 *                      ② 把 pitch/角速度整体取反, 让"前倾=正", 保留 -1。
 *      选 ① 是因为 pitch/配平零点/配平表/网页曲线的符号约定全都在 MPU
 *      那一侧, 改那边牵连一大片。
 *
 * ⚠ 改这两个值 = 拆电机 / 换 MPU 装配方向, 不是日常调试。
 *   改完必须复测: 前倾时轮子要往车头方向跑。 */
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
    float pitch;        /* 修正后姿态角 (°): 已扣配平表补偿。
                         * ⚠ 本工程符号: **前倾 (车头往下) 为负数** */
    float gyro;         /* 俯仰方向角速度 (°/s), 与 pitch **同号**
                         * (前倾倒得越快, 越负) */
    float speed;        /* 两轮平均轮速 (rpm), **前进为正** */
    float speed_raw;    /* 左轮编码器原始轮速 (rpm, 编码器空间, 调试用) */
    float trim_comp;    /* 配平表补偿量 (°), 观察补偿是否在工作 */
    float pitch_raw;    /* 修正前的原始 Pitch (°), 对照用 */

    /* --- 控制器中间量 --- */
    float target_pitch; /* 速度环算出的目标倾角 (°) */
    float bal_out;      /* 平衡环输出 (%, 已乘 BAL_MOTOR_SIGN) */
    float spd_out;      /* 速度环输出 (目标倾角 °) */
    float target_speed; /* 目标轮速 (rpm, 前进为正) */
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
 */
float balance_get_arm_ms(void);

/**
 * @brief 从 param 表重新读取所有控制参数并清 PID 积分
 * @return ESP_OK 成功
 */
esp_err_t balance_reload_params(void);

/**
 * @brief 设置遥控指令 (App 方向键)
 * @param dir  方向 (前进/后退/左转/右转)
 * @param level_pct 档位百分比 (会被吸附到 20/40/60/80 之一)
 * @return ESP_OK 成功
 */
esp_err_t balance_set_move(bal_move_t dir, int8_t level_pct);

/**
 * @brief 清除遥控指令 (目标轮速归零, 平衡环接管)
 */
void balance_clear_move(void);

/**
 * @brief 读取当前遥控指令
 * @param out_dir    [出参] 当前方向 (可为 NULL)
 * @param out_level  [出参] 当前档位百分比 (可为 NULL)
 */
void balance_get_move(bal_move_t *out_dir, int8_t *out_level);

/**
 * @brief 配平: 记录当前姿态角作为平衡零点
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
 */
esp_err_t balance_record_trim_point(void);

/**
 * @brief 把配平表导出成 JSON 字符串
 * @param buf 输出缓冲区
 * @param len 缓冲区长度
 * @return 写入的字符数 (0 = 缓冲区不够)
 */
int balance_get_trim_json(char *buf, uint32_t len);

#endif // BALANCE_H
