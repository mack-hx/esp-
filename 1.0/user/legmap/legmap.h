/**
 * @file legmap.h
 * @brief 腿高查表: 舵机角度 ↔ 轮心高度 的实测映射
 *
 * @note  为什么用查表而不是四连杆运动学解析解
 *   解析运动学要靠 4 根杆长 + 铰接点坐标反推机构姿态, 但这些尺寸:
 *     - 来自 SolidWorks 理想模型, 打印件有形变
 *     - 销孔有间隙, 装配有偏差
 *     - 杆长 5% 的测量误差, 在死点附近会被放大到腿高 20% 的误差
 *   而**实测**的查表法把这些误差全部包含进去了 —— 它量的是真实世界,
 *   不是重建一个理想机构。对"配平"这种用途, 实测永远优于推算。
 *
 * @note  查表里存什么
 *   每条记录 = { 舵机逻辑角(°), 轮心高度(mm), 该姿态的 Pitch(°) }
 *   第二个字段由用户在网页上手动量尺录入 (ESP32 量不了自己)。
 *   第三个字段是顺手测的: 车身固定、腿到该高度时读 Pitch。
 *
 * @note  没标定时的行为
 *   查表为空 → legmap_height() 返回 LEGMAP_HEIGHT_UNKNOWN,
 *   上层自动退回"不补偿"(等价于 leg_k=0), 保证不标定也能安全跑。
 *
 * @author ESP32 机械狗项目
 * @date 2026-10-05
 */

#ifndef LEGMAP_H
#define LEGMAP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ========== 容量 ==========
 * 标定时腿角跨度 30~100°, 每 5° 一个点就是 15 个点, 富余留到 24 个。 */
#define LEGMAP_MAX_POINTS     24

/* 高度超出这个范围就认为数据不可信 (mm), 只做合理性检查 */
#define LEGMAP_HEIGHT_MIN     20.0f
#define LEGMAP_HEIGHT_MAX     200.0f

/* 查表为空 / 查询失败时的返回值, 上层据此判断"要不要退回不补偿" */
#define LEGMAP_HEIGHT_UNKNOWN (-999.0f)

/* ========== 一条标定记录 ========== */
typedef struct {
    float servo_deg;    /* 舵机逻辑角 (°), 与 servo_get_angle(SERVO_1) 同量纲 */
    float height_mm;    /* 该角度下轮心离地高度 (mm), 用户实测 */
    float pitch_deg;    /* 该高度下 MPU 读到的车身 Pitch (°), 车身固定时测 */
    bool  valid;        /* true=用户已录入有效数据 */
} legmap_point_t;

/* ========== 标定数据表 ========== */
typedef struct {
    uint8_t         count;                    /* 已录入的有效点数 */
    legmap_point_t  pts[LEGMAP_MAX_POINTS];   /* 按 servo_deg 升序存放 */
} legmap_t;

/* ========== 全局访问 ========== */

/**
 * @brief 初始化 (从 NVS 载入已存的标定数据)
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
 * @return 指向全局表的指针, 不会为 NULL
 * @note  返回的是控制任务的"输入", 外部不要改; 要改请走 legmap_upsert_point()
 */
const legmap_t *legmap_get(void);

/**
 * @brief 当前腿角对应的轮心高度
 * @param servo_deg 舵机逻辑角 (°)
 * @return 高度 (mm); 未标定或越界返回 LEGMAP_HEIGHT_UNKNOWN
 * @note  表内按 servo_deg 升序, 这里做线性插值。
 *        落在最前/最后两个点之外则夹到端点值 (外推不安全)。
 */
float legmap_height(float servo_deg);

/**
 * @brief 当前腿角对应的 Pitch 几何补偿量
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
 *   - 返回 0 会在范围边界造成补偿突变 (腿刚过边界, 补偿突然归零)
 *   - 夹到端点则补偿幅值始终有界, 与 legmap_height() 行为一致
 */
float legmap_pitch_comp(float servo_deg, float ref_deg);

/**
 * @brief 录入/更新一个标定点
 * @param servo_deg 舵机逻辑角 (°)
 * @param height_mm 轮心高度 (mm), 超出 LEGMAP_HEIGHT_MIN~MAX 会被拒
 * @param pitch_deg 该姿态的 Pitch (°)
 * @return ESP_OK 成功; ESP_ERR_INVALID_ARG 高度非法; ESP_ERR_NO_MEM 点数已满
 * @note  按 servo_deg 升序插入 (已有同角度则覆盖)。insert 不是 append。
 */
esp_err_t legmap_upsert_point(float servo_deg, float height_mm, float pitch_deg);

/**
 * @brief 清空所有标定点 (只改内存)
 * @return ESP_OK 成功
 */
esp_err_t legmap_clear(void);

/**
 * @brief 保存标定数据到 NVS
 * @return ESP_OK 成功
 * @note  网页上点"保存到 NVS"时一并调用
 */
esp_err_t legmap_save(void);

/**
 * @brief 检查当前数据是否可用 (点数足够且腿高曲线形状合理)
 * @param min_points 最少点数
 * @return true=可用
 * @note  腿高对角度必须是**单峰**的(先降后升, 最多 1 个拐点) —— 这正是
 *        四连杆的真实形状: 腿伸直时轮心高, 弯到中间最低, 再弯又开始抬。
 *        ⚠ 不能要求单调递增, 那样会误杀所有合法的 U 型数据。
 *        检查的是"有没有来回抖"(拐点 > 1), 那才是尺子读错或腿没到位。
 *        1mm 容差吸收量尺误差与机构回差。
 */
bool legmap_is_usable(uint8_t min_points);

#endif // LEGMAP_H
