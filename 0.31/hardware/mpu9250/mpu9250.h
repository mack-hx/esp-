/**
 * @file mpu9250.h
 * @brief MPU9250 九轴惯性测量单元驱动
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

/* ========== 引脚配置 ========== */
#define MPU9250_SCL_GPIO        1       // I2C 时钟线
#define MPU9250_SDA_GPIO        2       // I2C 数据线
#define MPU9250_INT_GPIO        3       // 中断引脚

/* ========== I2C 配置 ========== */
#define MPU9250_I2C_NUM         I2C_NUM_0
#define MPU9250_I2C_FREQ        400000  // 400kHz
#define MPU9250_I2C_ADDR        0x68    // I2C 地址 (AD0=0)

/* ========== MPU9250 寄存器地址 ========== */
#define MPU9250_WHO_AM_I        0x75
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_CONFIG          0x1A
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_ACCEL_CONFIG2   0x1D
#define MPU9250_INT_PIN_CFG     0x37
#define MPU9250_INT_ENABLE      0x38
#define MPU9250_INT_STATUS      0x3A
#define MPU9250_USER_CTRL       0x6D    /* bypass 位: bit0 = 磁力计 bypass, bit1 = 陀螺/加速度 bypass */

#define MPU9250_ACCEL_XOUT_H    0x3B    // 加速度计数据起始地址
#define MPU9250_TEMP_OUT_H      0x41    // 温度数据起始地址
#define MPU9250_GYRO_XOUT_H     0x43    // 陀螺仪数据起始地址

/* ========== AK8963 磁力计 (MPU9250 内置, bypass 后独立挂在 0x0C) ==========
 * ⚠ 寄存器地址按 AK8963 原厂数据手册:
 *   0x00 WIA, 0x01 INFO, 0x02 ST1, 0x03~0x08 HXL..HZH, 0x09 ST2,
 *   0x0A CNTL1, 0x0B CNTL2, 0x0C~0x0F ASTC, 0x10 ASAX, 0x11 ASAY, 0x12 ASAZ
 *
 * ⚠ 走 bypass 模式时, AK8963 才能被当成"独立器件"用 0x0C 直接访问。
 *   MPU9250 的 INT_PIN_CFG 要置 bit1 (0x02, 让 INT 引脚直通 AK8963)
 *   和 bit6 (0x40, 允许 bypass), USER_CTRL 置 0 (退出 I2C master 模式)。 */
#define AK8963_I2C_ADDR         0x0C
#define AK8963_WHO_AM_I_VAL     0x48

#define AK8963_REG_WIA          0x00    /* 器件 ID, 期望 0x48 */
#define AK8963_REG_INFO         0x01
#define AK8963_REG_ST1          0x02    /* bit0 = DRDY (数据就绪) */
#define AK8963_REG_HXL          0x03    /* 数据起始 (HXL..HZH, 小端) */
#define AK8963_REG_ST2          0x09    /* 读完必须读一次, 清 DRDY 标志 */
#define AK8963_REG_CNTL1        0x0A    /* 模式控制 */
#define AK8963_REG_ASAX         0x10    /* 厂标敏感度系数起始 (ASAX/Y/Z) */

/* ========== 配置参数 ========== */
// 加速度计量程
typedef enum {
    ACCEL_FS_2G  = 0,   // ±2g
    ACCEL_FS_4G  = 1,   // ±4g
    ACCEL_FS_8G  = 2,   // ±8g
    ACCEL_FS_16G = 3    // ±16g
} mpu9250_accel_fs_t;

// 陀螺仪量程
typedef enum {
    GYRO_FS_250DPS  = 0,    // ±250°/s
    GYRO_FS_500DPS  = 1,    // ±500°/s
    GYRO_FS_1000DPS = 2,    // ±1000°/s
    GYRO_FS_2000DPS = 3     // ±2000°/s
} mpu9250_gyro_fs_t;

/* ========== 数据结构 ========== */
/**
 * @brief 三轴数据结构
 */
typedef struct {
    float x;    // X 轴数据
    float y;    // Y 轴数据
    float z;    // Z 轴数据
} mpu9250_axis_t;

/**
 * @brief MPU9250 数据结构
 */
typedef struct {
    mpu9250_axis_t accel;   // 加速度 (m/s²)
    mpu9250_axis_t gyro;    // 角速度 (°/s)
    float temperature;       // 温度 (°C)
} mpu9250_data_t;

/**
 * @brief 欧拉角结构
 */
typedef struct {
    float roll;     // 横滚角 (°)
    float pitch;    // 俯仰角 (°)
    float yaw;      // 航向角 (°)
} euler_angle_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 MPU9250
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_init(void);

/**
 * @brief 查询 MPU9250 是否就绪
 * @note  mpu9250_init() 成功后才为 true
 * @return true=可用
 */
bool mpu9250_is_ready(void);

/**
 * @brief 配置加速度计量程
 * @param fs 加速度计量程
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_set_accel_fs(mpu9250_accel_fs_t fs);

/**
 * @brief 配置陀螺仪量程
 * @param fs 陀螺仪量程
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_set_gyro_fs(mpu9250_gyro_fs_t fs);

/**
 * @brief 读取原始数据
 * @param data 数据结构指针
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_read_data(mpu9250_data_t *data);

/**
 * @brief 读取加速度数据
 * @param accel 加速度数据指针
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_read_accel(mpu9250_axis_t *accel);

/**
 * @brief 读取陀螺仪数据
 * @param gyro 陀螺仪数据指针
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_read_gyro(mpu9250_axis_t *gyro);

/**
 * @brief 读取温度
 * @param temp 温度指针 (°C)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_read_temperature(float *temp);

/**
 * @brief 获取欧拉角 (互补滤波)
 * @param angle 欧拉角指针
 * @return ESP_OK 成功, 其他值失败
 * @note  mpu9250_get_pose() 的兼容包装, 内部同样只读一次 I2C
 */
esp_err_t mpu9250_get_euler_angle(euler_angle_t *angle);

/**
 * @brief 一次读取, 同时得到姿态角和陀螺仪角速度
 * @param out      [出参] 欧拉角指针
 * @param gyro_out [出参] 三轴角速度指针 (°/s), 可传 NULL 只要姿态
 * @return ESP_OK 成功, 其他值失败
 * @note  控制任务请用这个接口, 不要分别调 mpu9250_get_euler_angle() +
 *        mpu9250_read_gyro(): 那会读两次 I2C, 两次数据不同时刻,
 *        姿态角与角速度错位会让平衡环的 D 项(角速度阻尼)失准。
 */
esp_err_t mpu9250_get_pose(euler_angle_t *out, mpu9250_axis_t *gyro_out);

/**
 * @brief 校准陀螺仪零点
 * @param samples 采样次数 (每 10ms 一个样本, 上限 256)
 * @return ESP_OK 成功; ESP_ERR_INVALID_STATE 采样失败或整段都没找到稳的一段
 * @note  2026-10-26 改成"最稳窗口"逻辑: 从整段采样里挑三轴标准差最小的
 *        0.25 秒窗口, 用它的中位数当零偏。所以**手持标定也能成功**
 *        (只要能拿稳 0.25 秒)。失败时不写 NVS, 由调用方退回旧零偏。
 */
esp_err_t mpu9250_calibrate_gyro(uint16_t samples);

/**
 * @brief 读磁力计 (AK8963)
 * @param out [出参] 三轴磁场 (uT), 没找到磁力计时不写
 * @return ESP_OK 读到; ESP_ERR_INVALID_STATE 这块板没有可用的 AK8963
 * @note  ⚠ 磁力计读数**不参与**任何姿态解算 (2026-10-26 用户决定: 先只观察)。
 *        量纲换算: uT = 原始值 × 厂标系数 × 0.15
 */
esp_err_t mpu9250_read_mag(mpu9250_axis_t *out);

/**
 * @brief 开/关"零偏慢漂跟随"
 * @param on true=允许零偏在设备静止时缓慢向当前读数靠拢
 * @note  为什么要这个: 陀螺零偏随温度漂 (实测冷机 0.90 °/s → 热机 0.65 °/s),
 *        而互补滤波把残余零偏放大 1/(1-alpha) ≈ 333 倍 → 姿态角差 0.83°,
 *        车会带着倾角站、慢慢往一边跑。
 * @note  为什么要"静止时"才跟: 车在动的时候读数里混着真实角速度,
 *        拿它当零偏会把真实运动也吸进零偏里, D 项阻尼会失准。
 * @note  默认关闭; 由上层 (balance.c) 在"车没在平衡状态"时打开。
 */
void mpu9250_set_bias_tracking(bool on);

/**
 * @brief 获取陀螺仪零点偏移 (校准得到的偏置值, °/s)
 * @note  mpu9250_read_data() 已把该偏移从读数中扣除,
 *        此接口用于查看偏置本身有多大 (判断校准是否正常)
 * @param offset 三轴偏移数据结构指针
 */
void mpu9250_get_gyro_offset(mpu9250_axis_t *offset);

/**
 * @brief 软件复位 MPU9250
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_reset(void);

#endif // MPU9250_H
