/**
 * @file param.h
 * @brief 全局可调参数表 (网页滑块调参的唯一数据源)
 * @note  设计目的:
 *        1) 网页上每个滑块 = 一条 PARAM_ITEM 描述, 前端不用硬编码字段名
 *        2) 参数值存 NVS, 断电不丢; 上电自动读回来
 *        3) 以后写 PID 控制时, 直接 param_get_float(PARAM_PID_BAL_KP) 取值
 *        4) 参数表是扁平的 id -> {名称, 默认值, 最小, 最大, 小数位, 分组}
 *           加新参数只要加一行, 网页自动多出一个滑块, 无需改前端代码
 *
 * @note  加参数的步骤 (以后实际开发时照此做):
 *        1) 在下方 PARAM_ID 里加一个枚举
 *        2) 在 g_param_items[] 里加一行描述 (名字/默认值/范围/步进/分组)
 *        3) 完事。网页和 NVS 都不用改
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 参数分组 (网页按分组折叠显示) ========== */
typedef enum {
    PARAM_GRP_BALANCE = 0,   // 平衡环 (直立)
    PARAM_GRP_SPEED,         // 速度环 (前后走)
    PARAM_GRP_STEER,         // 转向环 (左右转)
    PARAM_GRP_LIMIT,         // 限幅 / 死区 / 滤波
    PARAM_GRP_MAX            // 哨兵
} param_group_t;

/* ========== 参数 ID (每个滑块一个) ========== */
typedef enum {
    /* --- 平衡环: 靠 Pitch 角度把车扶正 --- */
    PARAM_PID_BAL_KP = 0,    // 平衡 Kp (越大越"硬", 但容易抖)
    PARAM_PID_BAL_KI,        // 平衡 Ki (消除持续倾斜, 太大积分饱和)
    PARAM_PID_BAL_KD,        // 平衡 Kd (角速度阻尼, 抑制超调)
    /* --- 速度环: 控制前进后退速度 --- */
    PARAM_PID_SPD_KP,        // 速度 Kp
    PARAM_PID_SPD_KI,        // 速度 Ki
    PARAM_PID_SPD_KD,        // 速度 Kd
    /* --- 转向环: 控制偏航角 / 差速 --- */
    PARAM_PID_STEER_KP,      // 转向 Kp
    PARAM_PID_STEER_KI,      // 转向 Ki
    PARAM_PID_STEER_KD,      // 转向 Kd
    /* --- 限幅 / 死区 / 滤波 --- */
    PARAM_MOTOR_OUT_MAX,     // 电机输出限幅 (%)
    PARAM_SPEED_OUT_MAX,     // 速度环输出限幅 (%)
    PARAM_ANGLE_LIMIT,       // 倾角保护阈值 (°), 超过判定摔倒
    PARAM_ANGLE_ZERO,        // 机械零点偏置 (°), 配平用
    PARAM_LEG_PITCH_K,       // 腿角→俯仰角斜率 (°/°), 腿弯了 MPU 读数的修正量
    PARAM_GYRO_LPF,          // 陀螺仪低通滤波系数 (0~1, 越小越平滑)
    PARAM_SPEED_DEADBAND,    // 速度死区 (编码器计数/采样周期), 抑制原地抖动
    PARAM_ENCODER_GEAR,      // 减速比 (电机转速/轮转速)
    /* --- 硬件调试 --- */
    PARAM_SERVO_CENTER,      // 舵机中心角 (°)
    PARAM_SERVO_LIMIT,       // 舵机摆动幅度 (°)
    PARAM_MOTOR_PWM_MAX,     // 电机 PWM 上限 (%)
    PARAM_MAX                 // 哨兵
} param_id_t;

/* ========== 参数条目的静态描述 (加参数只改这张表) ========== */

/**
 * @brief 一条参数的描述信息
 */
typedef struct {
    param_id_t    id;            // 参数 ID (也是 NVS 里的存盘顺序)
    const char   *key;           // 短标识, JSON 传输用 (如 "bal_kp")
    const char   *label;         // 网页上显示的名字 (如 "Balance Kp")
    const char   *unit;          // 单位 (无单位写 "")
    float         def;           // 默认值
    float         min;           // 滑块最小值
    float         max;           // 滑块最大值
    float         step;          // 步进 (滑块每格走多少)
    uint8_t       decimals;      // 显示小数位数
    param_group_t grp;           // 所属分组
    const char   *hint;          // 网页上的一句话说明
} param_item_t;

/**
 * @brief 参数表总条目数
 */
#define PARAM_ITEM_COUNT  ((uint8_t)(sizeof(g_param_items) / sizeof(g_param_items[0])))

/**
 * @brief 全部参数的描述表
 * @note  顺序必须与 param_id_t 一致 (NVS 按下标顺序存 blob)
 */
extern const param_item_t g_param_items[];

/* ========== 参数读写 API ========== */

/**
 * @brief 从 NVS 载入参数 (上电调用一次)
 * @note  首次上电 / NVS 无数据 / 数据版本不匹配时, 自动填默认值
 * @return ESP_OK 成功
 */
esp_err_t param_load(void);

/**
 * @brief 保存参数到 NVS (网页点"保存"时调用)
 * @return ESP_OK 成功
 */
esp_err_t param_save(void);

/**
 * @brief 恢复默认参数 (不落盘, 只改内存)
 * @return ESP_OK 成功
 */
esp_err_t param_reset_default(void);

/**
 * @brief 读取一个浮点参数
 * @param id 参数 ID
 * @return 参数值
 */
float param_get_float(param_id_t id);

/**
 * @brief 写入一个浮点参数 (只改内存, 需要 param_save() 才落盘)
 * @param id  参数 ID
 * @param val 新值 (超范围会自动 clamp 到 min~max)
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG id 非法
 */
esp_err_t param_set_float(param_id_t id, float val);

/**
 * @brief 取某条参数的描述
 * @param id 参数 ID
 * @return 描述指针, id 非法返回 NULL
 */
const param_item_t *param_get_item(param_id_t id);

/**
 * @brief 把整个参数表序列化成 JSON (网页加载时拉一次)
 * @param buf  输出缓冲区
 * @param len  缓冲区大小
 * @return 实际写入长度 (不含 '\0')
 */
int param_to_json(char *buf, uint32_t len);

/**
 * @brief 从 JSON 片段批量设置参数
 * @param json JSON 对象字符串, 形如 {"bal_kp":12.5,"bal_ki":3}
 * @return ESP_OK 成功
 */
esp_err_t param_from_json(const char *json);

#endif // PARAM_H
