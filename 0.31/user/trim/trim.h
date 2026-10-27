/**
 * @file trim.h
 * @brief 配平表: 舵机角 (°) → 把车身扶到平衡位时的俯仰角 (°)
 *
 * @brief 这是干什么的 —— 一句话
 *   你把腿拨到某个角度, 手动把车身扶到"快要平衡"的位置,
 *   读一下 Pitch, 记下来。攒够几个点, 控制器就知道
 *   "腿在 X 度时, 我要盯着的 Pitch 应该是 Y 度"。
 *
 * @brief 为什么表里只有"舵机角"一个自变量
 *   之前 0.1 用的是「舵机角 → 腿高(拿尺子量) → Pitch」, 两个自变量。
 *   问题是腿高根本没法准: 车一动高度就变, 尺子读数要靠手感,
 *   打印件有形变、销孔有间隙 —— 腿高算出来是编的。
 *
 *   实际上**不需要知道腿高**。控制器只关心一件事:
 *   "现在舵机在这个角度, 我应该盯多少度才算平衡"。
 *   腿高只是这个答案的中间变量, 不是必要输入。去掉它, 表就退化成
 *   一维: 舵机角 → 平衡俯仰角。这是你要求的做法, 也是最直接的做法。
 *
 * @brief 标定流程 (网页上做, 30 秒一个点)
 *   1. 车扶住 (放桌上/夹具/手扶), 电机全程刹车
 *   2. 滑块拨到某个舵机角, 点「拨腿」→ 两路舵机转到该角度
 *   3. **用手把车身扶到接近平衡** (车会往哪边倒就往另一边扶)
 *   4. 点「记下这个角度」→ 读 Pitch, 存进表
 *   5. 换角度重复, 铺满 30°~100° 大约 6~8 个点
 *   6. 点顶部「保存到 NVS」落盘
 *
 * @brief 参考角与补偿
 *   记下来的 Pitch 是"绝对"的 (含 MPU 焊歪的固定角)。控制器要的是
 *   "**相对** 某个参考姿态的偏差", 所以用差值:
 *
 *       trim_pitch(φ) = 表插值(φ) − 表插值(φ_ref)
 *
 *   φ_ref = param 里的 servo_c (舵机中心角), 也就是"回中姿态"。
 *   这样 servo_c 那一档补偿天然为 0, ang_zero 只需要补 MPU 的安装偏差。
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-05
 */

#ifndef TRIM_H
#define TRIM_H

#include <stdint.h>
#include <stdbool.h>

/* 离线测试 (gcc 直接跑, -DTRIM_OFFLINE_TEST) 时用 test/trim_offline.h
 * 代替 ESP-IDF 的 esp_err.h。
 * ⚠ 桩头必须由本头文件自己带进来, 不能指望调用方先 include ——
 *   trim.c 是 `#include "trim.h"` 打头的, 靠调用方兜底会很脆。 */
#ifdef TRIM_OFFLINE_TEST
#include "trim_offline.h"
#else
#include "esp_err.h"
#endif

/* ========== 容量 ==========
 * 舵机行程 30~100°, 每 5° 一个点是 15 个, 富余留到 16。
 * 比 0.1 的 24 少 —— 一维表点太多反而难采准, 12 个点足够描述一条平滑曲线。 */
#define TRIM_MAX_POINTS     16

/* ========== 俯仰角合理范围 (°) ==========
 * 超出这个范围说明 MPU 没焊好或者数据坏了, 直接拒收。 */
#define TRIM_PITCH_MIN      (-30.0f)
#define TRIM_PITCH_MAX      30.0f

/* ========== 一条标定记录 ========== */
typedef struct {
    float servo_deg;    /* 舵机逻辑角 (°), 与 servo_get_angle() 同量纲 */
    float pitch_deg;    /* 把车身扶到接近平衡时的 Pitch (°) */
} trim_point_t;

/* ========== 标定表 ========== */
typedef struct {
    uint8_t        count;                    /* 已录入的点数 */
    trim_point_t   pts[TRIM_MAX_POINTS];    /* 按 servo_deg 升序存放 */
} trim_t;

/* ================= API ================= */

/**
 * @brief 从 NVS 载入配平表
 * @return ESP_OK 成功 (无数据时用空表, 补偿量恒为 0)
 * @note  balance_init() 内部调用, 一般不直接用
 */
esp_err_t trim_load(void);

/**
 * @brief 保存配平表到 NVS
 * @return ESP_OK 成功
 * @note  网页点「保存到 NVS」时调用
 */
esp_err_t trim_save(void);

/**
 * @brief 取得配平表指针 (只读)
 * @return 指向全局表的指针, 不会为 NULL
 * @note  外部不要直接改; 要改走 trim_set_point()
 */
const trim_t *trim_get(void);

/**
 * @brief 某个舵机角对应的"平衡俯仰角" (插值后)
 * @param servo_deg 舵机逻辑角 (°)
 * @param ref_deg   参考角 (°), 通常是 servo_c —— 补偿在这个角度为 0
 * @return 补偿量 (°), = 表插值(servo_deg) − 表插值(ref_deg)
 *
 * @note  表点数 < 2 时返回 0 (不做补偿), 行为和没标定时完全一致。
 * @note  舵机角超出标定范围时**夹到端点**, 不外推:
 *         外推的话机构可能已到死点, 曲线形状完全变了, 推出来的值是编的。
 *         夹到端点则补偿幅值始终有界, 越界不会突变。
 */
float trim_pitch(float servo_deg, float ref_deg);

/**
 * @brief 录入/更新一个配定点
 * @param servo_deg 舵机逻辑角 (°)
 * @param pitch_deg 该姿态的 Pitch (°)
 * @return ESP_OK 成功; ESP_ERR_INVALID_ARG 角度超范围; ESP_ERR_NO_MEM 点数已满
 * @note  按舵机角升序插入 (不是 append)。已有同角度则覆盖。
 */
esp_err_t trim_set_point(float servo_deg, float pitch_deg);

/**
 * @brief 删除某个舵机角上的点
 * @param servo_deg 舵机逻辑角 (°)
 * @return ESP_OK 成功 (该角度本来就没有点也返回 OK)
 */
esp_err_t trim_del_point(float servo_deg);

/**
 * @brief 清空配平表 (只改内存)
 * @return ESP_OK 成功
 */
esp_err_t trim_clear(void);

/**
 * @brief 擦除 NVS 中配平表数据 (仅擦 namespace, 不改内存)
 * @return ESP_OK 成功
 * @note  用于换机械结构后丢弃旧标定数据。
 *        ⚠ 2026-10-25 起**不再在每次上电时自动调用** —— 那样配平表永远存不住。
 *        现在只有显式调用才会擦 (例如换机械结构后手动清一次)。 */
esp_err_t trim_erase_nvs(void);

/**
 * @brief 检查配平表是否可用 (点数 >= min_points 且俯仰角有合理变化)
 * @param min_points 最少点数
 * @return true=可用
 *
 * @note  "俯仰角有合理变化" 是防呆: 如果所有点记下来的 Pitch 几乎一样,
 *        说明要么车根本没扶到不同姿态, 要么 MPU 没工作。
 *        这种表查出来的补偿恒等于常数, 没有任何作用。
 * @note  变化太小**不会**判定不可用 (只是警告), 因为实际操作中
 *        换角度时 Pitch 变化确实可能不大 (比如 0.3°)。
 *        只在"完全没变化"(< 0.05°) 时才认为数据有问题。
 */
bool trim_is_usable(uint8_t min_points);

/**
 * @brief 取表内俯仰角的跨度 (最大值 − 最小值, °)
 * @return 跨度; 表空返回 0
 * @note  网页用来判断"这张表有没有实际内容"
 */
float trim_pitch_span(void);

#endif // TRIM_H
