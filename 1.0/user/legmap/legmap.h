/**
 * @file legmap.h
 * @brief 腿高/直距查表（左右腿各一份）: 舵机角度 ↔ 电机中心↔舵机轴心距离
 *
 * @note  为什么用查表而不是四连杆运动学解析解
 *   解析运动学要靠 4 根杆长 + 铰接点坐标反推机构姿态, 但这些尺寸:
 *     - 来自 SolidWorks 理想模型, 打印件有形变
 *     - 销孔有间隙, 装配有偏差
 *     - 杆长 5% 的测量误差, 在死点附近会被放大到腿高 20% 的误差
 *   而**实测**的查表法把这些误差全部包含进去了 —— 它量的是真实世界,
 *   不是重建一个理想机构。对"配平"这种用途, 实测永远优于推算。
 *
 * @note  每张表里存什么
 *   每条记录 = { 舵机逻辑角(°), 电机中心↔舵机输出轴中心直线距离(mm),
 *                 该姿态的车身 Pitch(°) }
 *   第二个字段由用户在网页上手动量尺录入(ESP32 量不了自己)。
 *   第三个字段是顺手测的: 标定那一路时, 另一路拆开放到中立位(不参与),
 *   车身固定不动, 读 MPU 的 Pitch。
 *
 * @note  为什么是"电机中心↔舵机轴心直距"而不是"轮心离地高度"
 *   测离地高度时车身稍微一摆, 高度就跟着变, 数据不稳。
 *   两点都是机械上的硬位置(打印件/金属件上不动), 用尺子一量就是定值,
 *   不受车身姿态/MPU 漂移/弹簧回弹的影响, 录一次能用很久。
 *   名字仍叫 height_mm 只是为了保留插值/判定代码不变 —— **含义已变**。
 *
 * @note  左右两腿为什么分开存
 *   两路舵机硬件有差异 + 装配有偏差, 即使"同步发同样 PWM"两腿的直距也不完全一样。
 *   合一张表 → 补偿用一个数 → 永远有一边欠补偿或过补偿, 表现就是车往一边偏。
 *   分两张表 → 查表分别查 → 各补各的, 偏差就被各自吸收掉。
 *
 * @note  两张表怎么采
 *   现在两路舵机是镜像的, 一次"动腿并读取"两腿都动, 没法同时采左右。
 *   **正确流程**: 采两次。
 *     ① 左腿表: 把右腿(连杆/舵机一起)固定到中立位**不参与**, 只动左腿,
 *        读到的 Pitch 变化 = 左腿单独贡献。标完左表。
 *     ② 拆下来换右腿采: 固定左腿, 只动右腿, 读 Pitch = 右腿单独贡献。标右表。
 *   网页顶部有 L/R Tab 切换"现在录的是哪张表", 后端按 leg 字段分流。
 *
 * @note  没标定时的行为
 *   任一张表为空 → 该路查表返回 LEGMAP_HEIGHT_UNKNOWN,
 *   balance 自动退回"该路不补偿 + leg_k 兜底", 保证不标定也能安全跑。
 *
 * @author ESP32 机械狗项目
 * @date 2026-10-05
 */

#ifndef LEGMAP_H
#define LEGMAP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 左右两路 ========== */
typedef enum {
    LEG_LEFT = 0,   /* 左腿表 (通常配合 SERVO_1) */
    LEG_RIGHT = 1,  /* 右腿表 (通常配合 SERVO_2) */
    LEG_COUNT = 2,
} leg_id_t;

/* ========== 容量 ==========
 * 标定时腿角跨度 30~100°, 每 5° 一个点就是 15 个点, 富余留到 24 个。 */
#define LEGMAP_MAX_POINTS     24

/* 直距超出这个范围就认为数据不可信 (mm), 只做合理性检查 */
#define LEGMAP_HEIGHT_MIN     20.0f
#define LEGMAP_HEIGHT_MAX     300.0f

/* 查表为空 / 查询失败时的返回值, 上层据此判断"要不要退回不补偿" */
#define LEGMAP_HEIGHT_UNKNOWN (-999.0f)

/* ========== 一条标定记录 ========== */
typedef struct {
    float servo_deg;    /* 舵机逻辑角 (°), 与 servo_get_angle(SERVO_1) 同量纲 */
    float height_mm;    /* 电机中心↔舵机轴心直线距离 (mm), 用户实测 (语义见文件头) */
    float pitch_deg;    /* 该姿态的车身 Pitch(°), 标定那一路时另一路中立, MPU 读 */
    bool  valid;        /* true=用户已录入有效数据 */
} legmap_point_t;

/* ========== 标定数据表 (每路腿一份) ========== */
typedef struct {
    uint8_t         count;                    /* 已录入的有效点数 */
    legmap_point_t  pts[LEGMAP_MAX_POINTS];   /* 按 servo_deg 升序存放 */
} legmap_t;

/* ========== 双路汇总 (内部存储/对外发布都用这一份) ========== */
typedef struct {
    legmap_t legs[LEG_COUNT];
} legmap_all_t;

/* ========== 全局访问 ========== */

/**
 * @brief 初始化 (从 NVS 载入已存的标定数据, 左/右两路都读)
 * @note  main.c 启动时调用一次
 * @return ESP_OK 成功
 */
esp_err_t legmap_init(void);

/**
 * @brief 从 NVS 载入已存的标定数据
 * @return ESP_OK 成功
 * @note  legmap_init() 内部调用, 一般不直接用
 */
esp_err_t legmap_load(void);

/**
 * @brief 取得当前标定数据指针 (只读)
 * @return 指向全局双路表的指针, 不会为 NULL
 * @note  返回的是控制任务的"输入", 外部不要改; 要改请走 legmap_upsert_point()
 */
const legmap_all_t *legmap_get(void);

/**
 * @brief 取得单腿的标定表指针 (只读)
 * @param leg 左/右
 * @return 指向该路标定表的指针; leg 非法时返回 NULL
 */
const legmap_t *legmap_get_one(leg_id_t leg);

/**
 * @brief 当前舵机角对应的"电机中心↔舵机轴心直距" (插值后)
 * @param leg       左/右
 * @param servo_deg 舵机逻辑角 (°)
 * @return 直距 (mm); 表空/越界/leg 非法返回 LEGMAP_HEIGHT_UNKNOWN
 * @note  落在最前/最后两个点之外则夹到端点值 (外推不安全)。
 */
float legmap_height(leg_id_t leg, float servo_deg);

/**
 * @brief 当前舵机角对应的 Pitch 几何补偿量
 * @param leg       左/右
 * @param servo_deg 舵机逻辑角 (°)
 * @param ref_deg   参考腿角 (°), 通常是舵机中心角 (补偿在此处为 0)
 * @return 补偿量 (°)
 * @note  返回 pitch(servo_deg) - pitch(ref_deg),
 *        上层用它修正姿态读数: pitch_修正 = pitch_原始 - 补偿
 *
 * ⚠ 与 legmap_height() 的降级行为不同, 这里**永远不返回哨兵值**:
 *   补偿量本身就是"零就是不做补偿", 语义天然自洽, 上层无需特判。
 *
 * ⚠ 超出标定范围时**夹到端点**, 不返回 0 也不外推:
 *   - 返回 0 会在范围边界造成补偿突变
 *   - 夹到端点则补偿幅值始终有界, 与 legmap_height() 行为一致
 */
float legmap_pitch_comp(leg_id_t leg, float servo_deg, float ref_deg);

/**
 * @brief 录入/更新一个标定点 (到指定腿)
 * @param leg       左/右
 * @param servo_deg 舵机逻辑角 (°)
 * @param height_mm 直距 (mm), 超出 LEGMAP_HEIGHT_MIN~MAX 会被拒
 * @param pitch_deg 该姿态的 Pitch (°)
 * @return ESP_OK 成功; ESP_ERR_INVALID_ARG 腿非法/高度非法; ESP_ERR_NO_MEM 点数已满
 * @note  按 servo_deg 升序插入 (已有同角度则覆盖)。insert 不是 append。
 */
esp_err_t legmap_upsert_point(leg_id_t leg, float servo_deg, float height_mm, float pitch_deg);

/**
 * @brief 清空指定腿的标定点 (只改内存)
 * @param leg 左/右
 * @return ESP_OK 成功; ESP_ERR_INVALID_ARG 腿非法
 */
esp_err_t legmap_clear(leg_id_t leg);

/**
 * @brief 清空两腿全部标定点
 * @return ESP_OK 成功
 */
esp_err_t legmap_clear_all(void);

/**
 * @brief 保存标定数据到 NVS (两路一起存)
 * @return ESP_OK 成功
 * @note  网页上点"保存到 NVS"时一并调用
 */
esp_err_t legmap_save(void);

/**
 * @brief 检查指定腿的数据是否可用 (点数足够且直距曲线形状合理)
 * @param leg        左/右
 * @param min_points 最少点数
 * @return true=可用; leg 非法返回 false
 */
bool legmap_is_usable(leg_id_t leg, uint8_t min_points);

/**
 * @brief 快捷把 leg_id_t 写到字符串 ('L'/'R'), 用于 HTTP/日志
 * @param leg 左/右
 * @return 'L' / 'R'; leg 非法返回 '?'
 */
char legmap_leg_char(leg_id_t leg);

/**
 * @brief 解析字符串到 leg_id_t ("L"/"R"/"LEFT"/"RIGHT" 都接受)
 * @param s  字符串
 * @param out [出参] 解出的 leg
 * @return ESP_OK 解析成功; ESP_ERR_INVALID_ARG 不认得
 */
esp_err_t legmap_parse_leg(const char *s, leg_id_t *out);

#endif // LEGMAP_H