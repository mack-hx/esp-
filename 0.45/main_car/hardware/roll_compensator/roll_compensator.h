/**
 * @file roll_compensator.h
 * @brief 横滚闭环: 用左右腿长差把车身横滚拉回 0
 * @note  输入横滚角, 输出左右舵机角度偏置 (对称, 一收一伸)。
 *        只在平衡起立时生效; 未使能时偏置按斜率退回 0。
 * @author ESP32 机械狗项目
 */

#ifndef ROLL_COMPENSATOR_H
#define ROLL_COMPENSATOR_H

#include <stdint.h>
#include <stdbool.h>

/* ========== 增益 (输出 = 舵机角度差, 单位 °) ==========
 * 机构实测: 舵机差 30° 只产生约 4° 横滚 (≈0.14 °/°), 增益按此放大 */
#define ROLL_KP           2.00f   /* °舵机 / °横滚 */
#define ROLL_KI           0.40f   /* °舵机 / (°·s) */
#define ROLL_KD           0.10f   /* °舵机 / (°/s) */
#ifndef ROLL_COMP_SIGN
#define ROLL_COMP_SIGN    (-1)    /* +1 = 高侧收腿(角度增大); 实测角度增大抬高该侧, 取 -1 */
#endif

/* ========== 限制 ========== */
#define ROLL_DEADZONE     1.5f    /* ° 横滚死区: 车身本来就允许带一点倾角, 不硬掰到 0 */
#define ROLL_SOFT_BAND    1.0f    /* ° 死区外平滑过渡带 */
#define ROLL_COMP_MAX     15.0f   /* ° 单侧最大偏置 */
#define ROLL_I_MAX_DEG    8.0f    /* ° 积分项单独贡献的上限 (只用来吃掉静态偏置) */

/* 偏置变化率: 抬起比收回快。收回慢是为了不在横滚刚变平的瞬间"啪"地把腿收掉 */
#define ROLL_SLEW_PER_S     30.0f /* °/s 偏置增大时的速率上限 */
#define ROLL_RELEASE_PER_S  12.0f /* °/s 偏置回零时的速率上限 */

/* 死区内积分泄放时间常数 (s): 车身已经够平时, 积分慢慢退, 让腿缓慢回中位。
 * 不做锁存 —— 坡道坡度变了仍能重新找平衡 */
#define ROLL_I_RELEASE_TAU  3.0f

/* ========== 无响应看门狗 ==========
 * 输出已经不小、横滚却几乎不动 → 该侧腿没受力 (悬空 / 打滑 / 卡住),
 * 此时继续积分只会攒出巨大的偏置, 落地的瞬间腿猛跳 (实测"一条腿抽风")。 */
#define ROLL_RESP_MIN_OUT    3.0f  /* ° 输出超过它才开始判响应 */
#define ROLL_RESP_MIN_RATE   2.0f  /* °/s 横滚变化率低于它算"没响应" */
#define ROLL_RESP_TIMEOUT    0.6f  /* s 连续没响应多久后冻结积分 */

/** 横滚异常阈值: 超过它认为车已经侧翻, 偏置退回 0 */
#define ROLL_FAULT_ANGLE  35.0f

/** 补偿输出结果 */
typedef struct {
    int8_t left_comp;       /* 左腿舵机角度偏置 (°, 正 = 角度增大 = 收腿) */
    int8_t right_comp;      /* 右腿舵机角度偏置 (°) */
    float  roll_angle;      /* 本周期使用的横滚角 (°, 正 = 右侧高) */
    float  raw_out;         /* 未取整的偏置 (°), 调试用 */
} roll_comp_output_t;

/** @brief 复位内部状态 (积分清零, 偏置归零) */
void roll_comp_init(void);

/** @brief 清掉积分和当前偏置, 下一周期从 0 重新开始 (配平后调用) */
void roll_comp_reset(void);

/** @brief 使能/禁用补偿 */
void roll_comp_set_enable(bool enable);

/** @brief 补偿是否使能 */
bool roll_comp_is_enabled(void);

/**
 * @brief 计算横滚闭环偏置
 * @param dt         控制周期 (s)
 * @param roll_deg   横滚角 (°, 正 = 右侧高; 已扣配平零点)
 * @param[out] output 左右舵机角度偏置
 * @note  未使能 / 角度超限 / 非有限值时, 偏置按 ROLL_RELEASE_PER_S 退回 0
 */
void roll_comp_calculate(float dt, float roll_deg, roll_comp_output_t *output);

#endif // ROLL_COMPENSATOR_H
