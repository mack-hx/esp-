/**
 * @file mpu9250.h
 * @brief MPU9250 九轴惯性测量单元驱动
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
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

#define MPU9250_ACCEL_XOUT_H    0x3B    // 加速度计数据起始地址
#define MPU9250_TEMP_OUT_H      0x41    // 温度数据起始地址
#define MPU9250_GYRO_XOUT_H     0x43    // 陀螺仪数据起始地址

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
 */
esp_err_t mpu9250_get_euler_angle(euler_angle_t *angle);

/**
 * @brief 校准陀螺仪零点
 * @param samples 采样次数
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_calibrate_gyro(uint16_t samples);

/**
 * @brief 软件复位 MPU9250
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t mpu9250_reset(void);

#endif // MPU9250_H
