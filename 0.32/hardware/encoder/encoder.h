/**
 * @file encoder.h
 * @brief 增量式正交编码器驱动 (ESP32-S3 PCNT 外设)
 * @author ESP32 机械狗项目
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "esp_err.h"

/* ========== 引脚配置 ==========
 * 编号统一为"物理左=1, 物理右=2": 编码器1 (左) = GPIO45/46, 编码器2 (右) = GPIO41/42
 * 物理左轮电机正转时计数方向反, 在 init 里用 pcnt_set_mode 对调 pos_mode/neg_mode */
#define ENC1_A_N_GPIO      45      // 编码器1 A 相负 (物理左轮)
#define ENC1_A_P_GPIO      46      // 编码器1 A 相正 (物理左轮)
#define ENC2_B_N_GPIO      41      // 编码器2 B 相负 (物理右轮)
#define ENC2_B_P_GPIO      42      // 编码器2 B 相正 (物理右轮)

/* ========== 编码器参数 ========== */
#define ENCODER_PULSES_PER_REV   44   // 每转脉冲数 (11 线 * 4 倍频)
#define ENCODER_PCNT_HIGH_LIMIT  32767
#define ENCODER_PCNT_LOW_LIMIT   -32768

/* ========== 编码器 ID 定义 ========== */
typedef enum {
    ENCODER_1 = 0,    // 编码器1 (左轮)
    ENCODER_2 = 1     // 编码器2 (右轮)
} encoder_id_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化所有编码器
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t encoder_init(void);

/**
 * @brief 获取编码器当前计数值
 * @param id 编码器 ID
 * @return 当前计数值 (有符号)
 */
int encoder_get_count(encoder_id_t id);
/**
 * @brief 计算转速 (RPM)
 * @note  调用间隔由内部 tick 快照自动推导, 保持固定间隔调用可得稳定转速
 * @param id 编码器 ID
 * @param interval_ms 预期的调用间隔 (ms), 实际间隔偏差超过 ±50% 时本次返回 0
 * @return 转速 (RPM, 有符号, 正值正转)
 */
float encoder_get_rpm(encoder_id_t id, uint32_t interval_ms);
#endif // ENCODER_H