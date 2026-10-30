/**
 * @file param.h
 * @brief 全局可调参数表 (网页滑块调参的唯一数据源)
 *
 * 方向符号参数 (bal_sign / gyro_sign) 与 MPU 安装零偏 (ang_zero) 不在这张表里,
 * 是代码里的常量 (balance.h / mpu9250.c)。
 *
 * @note  加参数两步: PARAM_ID 里加枚举 + param.c 的 g_param_items[] 里加一行描述,
 *        网页滑块与 NVS 无需改动。
 * @author ESP32 平衡轮腿机器人
 */

#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <stdbool.h>

#ifndef PARAM_OFFLINE_TEST
#include "esp_err.h"
#else
#include "param_offline.h"   /* 离线测试用的最小 esp_err_t 桩头 */
#endif

/* ========== 参数分组 (网页按分组折叠显示) ========== */
typedef enum {
    PARAM_GRP_BALANCE = 0,   // 平衡环 (直立)
    PARAM_GRP_SPEED,         // 速度环 (前后走)
    PARAM_GRP_LIMIT,         // 限幅 / 死区 / 启动门控
    PARAM_GRP_MAX            // 哨兵
} param_group_t;

/* ========== 参数 ID (每个滑块一个) ==========
 * 顺序 = NVS blob 的下标顺序, 只在末尾追加; 改了数量就把 PARAM_DATA_VERSION +1 */
typedef enum {
    /* --- 平衡环: 靠 Pitch 把车扶正 --- */
    PARAM_PID_BAL_KP = 0,    // 平衡 Kp (越大越"硬", 但容易抖)
    PARAM_PID_BAL_KI,        // 平衡 Ki (消除持续倾斜)
    PARAM_PID_BAL_KD,        // 平衡 Kd (角速度阻尼)
    /* --- 速度环: 控制前进后退速度 --- */
    PARAM_PID_SPD_KP,
    PARAM_PID_SPD_KI,
    PARAM_PID_SPD_KD,
    /* --- 限幅 / 死区 / 滤波 / 摔倒检测 --- */
    PARAM_MOTOR_OUT_MAX,     // 电机输出限幅 (%)
    PARAM_SPEED_OUT_MAX,     // 速度环输出限幅 (°)
    PARAM_ANGLE_LIMIT,       // 倾角保护阈值 (°), 超过判定摔倒
    PARAM_GYRO_LPF,          // 陀螺仪低通滤波系数 (0~1)
    PARAM_ENCODER_GEAR,      // 减速比 (电机转速/轮转速)
    /* --- 电机死区 --- */
    PARAM_MOTOR_DEADBAND,    // 电机最小启动 PWM (%) 死区补偿
    /* --- 安全门控: 启动前必须连续稳定这么久 --- */
    PARAM_ARM_PITCH,         // 允许的最大倾角 (°)
    PARAM_ARM_RATE,          // 允许的最大角速度 (°/s)
    PARAM_ARM_TIME,          // 需要连续稳定的时长 (ms)
    /* --- 舵机 --- */
    PARAM_SERVO_CENTER,      // 舵机中心角 (°) —— 配平表的参考角
    PARAM_MOVE_RPM_MAX,     // 走路上限 (rpm): App 方向键 100% 档位对应的目标轮速
    PARAM_WHEEL_DIA,        // 轮子直径 (cm): 里程计用, 距离 = 脉冲数 / (44 × gear) × π × 轮径
    PARAM_POS_KP,           // 位置保持环比例增益 (°/cm)
    PARAM_POS_KD,           // 位置保持环速度阻尼增益 (°/(cm/s))
    PARAM_POS_KI,           // 位置保持环积分增益 (°/(cm·s))
    PARAM_LQR_KX,           // LQR 位置偏差增益 (%/cm)
    PARAM_LQR_KV,           // LQR 平移速度增益 (%/(cm/s))
    PARAM_LQR_AMAX,         // LQR 回位减速度上限 (cm/s²)
    PARAM_MAX                // 哨兵
} param_id_t;

/* ========== 参数条目的静态描述 (加参数只改这张表) ========== */

/**
 * @brief 一条参数的描述信息
 */
typedef struct {
    param_id_t    id;            // 参数 ID (也是 NVS 里的存盘顺序)
    const char   *key;           // 短标识, JSON 传输用 (如 "bal_kp")
    const char   *label;         // 网页上显示的名字
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

/* ================= 参数读写 API ================= */

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
 * @brief 取当前参数表结构版本号 (PARAM_DATA_VERSION)
 * @return 版本号
 * @note  App 下发参数可带 "_v" 字段表示其内置的参数表版本 (见 onenet_client.c 的 param_set 分支)
 */
uint16_t param_get_data_version(void);

/**
 * @brief 标记"最近一次参数写入来自旧版 App"
 * @param on true=标记 (会随 param_get 的 "_w" 字段提示给 App), false=清除
 * @note  车端只提示不拦截, 下发参数照常生效; 标记随 param_save() 存进 NVS。
 */
void param_mark_stale_write(bool on);
/**
 * @brief 读取一个浮点参数
 * @param id 参数 ID
 * @return 参数值
 */
float param_get_float(param_id_t id);

/**
 * @brief 写入一个浮点参数 (只改内存, 需要 param_save() 才落盘)
 * @param id  参数 ID
 * @param val 新值 (超范围会自动 clamp 到 min~max, 并对齐到 step)
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG id 非法或 val 是 NaN/Inf
 */
esp_err_t param_set_float(param_id_t id, float val);
/**
 * @brief 把整个参数表序列化成"扁平键值对" JSON
 * @param buf  输出缓冲区
 * @param len  缓冲区大小
 * @return 实际写入长度 (不含 '\0')
 * @note  格式: {"bal_kp":40,"bal_ki":0,...}, 与 param_from_json 的输入格式一致
 */
int param_to_map_json(char *buf, uint32_t len);

/**
 * @brief 从 JSON 片段批量设置参数
 * @param json JSON 对象字符串, 形如 {"bal_kp":12.5,"bal_ki":3}
 * @return ESP_OK 至少成功设置了一个
 */
esp_err_t param_from_json(const char *json);

#endif // PARAM_H
