/**
 * @file servo_height_table.h
 * @brief 舵机角度 ↔ 底盘高度对照表
 * @note  测量方法: 水平放置小车, 调整舵机到指定角度, 用尺子测量底盘到地面的垂直距离
 *        (舵机轴中心线或底盘边缘到地面均可, 选一个统一基准)
 *        测量完后填入下方表格, 未测的角度会线性插值
 */

#ifndef SERVO_HEIGHT_TABLE_H
#define SERVO_HEIGHT_TABLE_H

#include <stdint.h>

/* ========== 对照表数据结构 ========== */

/**
 * @brief 单个测量点 (角度 → 高度)
 */
typedef struct {
    uint8_t angle;      // 舵机角度 (°)
    float height_cm;    // 底盘高度 (cm, 底盘边缘到地面)
} servo_height_point_t;

/**
 * @brief 底盘高度对照表 (实测数据)
 * @note  按角度从小到大排列, 方便二分查找和插值
 *        TODO: 实测后填入真实数据, 当前为占位示例
 */
static const servo_height_point_t SERVO_HEIGHT_TABLE[] = {
    // 角度(°)  底盘高度(cm)  备注
    {  30,      17.0f },   // TODO: 实测填入
    {  35,      16.0f },   // TODO: 实测填入
    {  40,      15.6f },   // TODO: 实测填入
    {  45,      14.8f },   // TODO: 实测填入
    {  50,     14.0f },   // TODO: 实测填入
    {  55,      13.5f },   // TODO: 实测填入
    {  60,      13.0f },   // TODO: 实测填入
    {  65,     12.5f },   // TODO: 实测填入
    {  70,      12.0f },   // TODO: 实测填入 (站立/蹲下分界)
    {  75,      10.2f },   // TODO: 实测填入
    {  80,      10.0f },   // TODO: 实测填入
    {  85,      9.8f },   // TODO: 实测填入
    {  90,      8.8f },   // TODO: 实测填入
    {  95,      8.0f },   // TODO: 实测填入
    { 100,      7.8f },   // TODO: 实测填入
    { 105,      7.5f },   // TODO: 实测填入
};

#define SERVO_HEIGHT_TABLE_SIZE  (sizeof(SERVO_HEIGHT_TABLE) / sizeof(servo_height_point_t))

/* ========== 查表函数 ========== */

/**
 * @brief 根据舵机角度查询底盘高度 (线性插值)
 * @param angle 舵机角度 (°)
 * @return 底盘高度 (cm), 超出表格范围时钳位到边界值
 * @note  使用线性插值: 未在表中的角度会根据相邻两点计算
 */
static inline float servo_angle_to_height(uint8_t angle) {
    // 边界检查
    if (angle <= SERVO_HEIGHT_TABLE[0].angle) {
        return SERVO_HEIGHT_TABLE[0].height_cm;
    }
    if (angle >= SERVO_HEIGHT_TABLE[SERVO_HEIGHT_TABLE_SIZE - 1].angle) {
        return SERVO_HEIGHT_TABLE[SERVO_HEIGHT_TABLE_SIZE - 1].height_cm;
    }
    
    // 二分查找相邻区间
    for (size_t i = 0; i < SERVO_HEIGHT_TABLE_SIZE - 1; i++) {
        if (angle >= SERVO_HEIGHT_TABLE[i].angle &&
            angle <= SERVO_HEIGHT_TABLE[i + 1].angle) {
            // 线性插值
            float angle_diff = SERVO_HEIGHT_TABLE[i + 1].angle - SERVO_HEIGHT_TABLE[i].angle;
            float height_diff = SERVO_HEIGHT_TABLE[i + 1].height_cm - SERVO_HEIGHT_TABLE[i].height_cm;
            float ratio = (angle - SERVO_HEIGHT_TABLE[i].angle) / angle_diff;
            return SERVO_HEIGHT_TABLE[i].height_cm + ratio * height_diff;
        }
    }
    
    return SERVO_HEIGHT_TABLE[0].height_cm;  // 兜底 (不应到达)
}

/**
 * @brief 根据目标底盘高度查询舵机角度 (线性插值)
 * @param height_cm 目标底盘高度 (cm)
 * @return 舵机角度 (°), 超出表格范围时钳位到边界值
 * @note  使用线性插值: 未在表中的高度会根据相邻两点计算
 *        注意: 假设表格中角度↑ 对应 高度↓ (收腿变矮)
 */
static inline uint8_t servo_height_to_angle(float height_cm) {
    // 边界检查 (注意高度是反向的: 小角度→高底盘)
    if (height_cm >= SERVO_HEIGHT_TABLE[0].height_cm) {
        return SERVO_HEIGHT_TABLE[0].angle;
    }
    if (height_cm <= SERVO_HEIGHT_TABLE[SERVO_HEIGHT_TABLE_SIZE - 1].height_cm) {
        return SERVO_HEIGHT_TABLE[SERVO_HEIGHT_TABLE_SIZE - 1].angle;
    }
    
    // 查找相邻区间 (高度递减)
    for (size_t i = 0; i < SERVO_HEIGHT_TABLE_SIZE - 1; i++) {
        if (height_cm <= SERVO_HEIGHT_TABLE[i].height_cm &&
            height_cm >= SERVO_HEIGHT_TABLE[i + 1].height_cm) {
            // 线性插值
            float height_diff = SERVO_HEIGHT_TABLE[i].height_cm - SERVO_HEIGHT_TABLE[i + 1].height_cm;
            float angle_diff = SERVO_HEIGHT_TABLE[i + 1].angle - SERVO_HEIGHT_TABLE[i].angle;
            float ratio = (SERVO_HEIGHT_TABLE[i].height_cm - height_cm) / height_diff;
            return (uint8_t)(SERVO_HEIGHT_TABLE[i].angle + ratio * angle_diff + 0.5f);
        }
    }
    
    return SERVO_HEIGHT_TABLE[0].angle;  // 兜底
}

/**
 * @brief 计算高度差对应的舵机角度调整量
 * @param height_diff_cm 高度差 (cm, 正=需要升高)
 * @param current_angle 当前舵机角度 (°)
 * @return 角度调整量 (°, 正=收腿, 负=伸腿), 已限幅在合理范围
 * @note  用于补偿计算: 如左腿高 2cm, 需要收 2cm → 返回正的角度增量
 */
static inline int8_t servo_height_diff_to_angle_delta(float height_diff_cm, uint8_t current_angle) {
    float current_height = servo_angle_to_height(current_angle);
    float target_height = current_height - height_diff_cm;  // 需要升高 → 目标高度降低
    uint8_t target_angle = servo_height_to_angle(target_height);
    
    int delta = (int)target_angle - (int)current_angle;
    // 限幅 ±30° (防止查表误差导致跳变)
    if (delta > 30) delta = 30;
    if (delta < -30) delta = -30;
    
    return (int8_t)delta;
}

#endif // SERVO_HEIGHT_TABLE_H
