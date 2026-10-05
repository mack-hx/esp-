/**
 * @file mpu9250.c
 * @brief MPU9250 九轴惯性测量单元驱动实现
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#include "mpu9250.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MPU9250";

/* 陀螺仪零点偏移 */
static mpu9250_axis_t gyro_offset = {0, 0, 0};

/* 初始化是否成功 (供外部查询, 避免反复尝试初始化失败的模块) */
static bool s_ready = false;

/* 当前量程配置 */
static mpu9250_accel_fs_t current_accel_fs = ACCEL_FS_2G;
static mpu9250_gyro_fs_t current_gyro_fs = GYRO_FS_250DPS;

/* 量程转换系数 */
static float accel_scale = 16384.0f;  // ±2g
static float gyro_scale = 131.0f;     // ±250°/s

/* 互补滤波参数
 * alpha = 陀螺仪(短期)权重, 越大越信陀螺仪、越不平滑
 * 0.98 配 100Hz 采样 ≈ 时间常数 0.5s, 兼顾响应和无噪声 */
static euler_angle_t current_angle = {0, 0, 0};
static const float alpha = 0.98f;  // 互补滤波系数

/* I2C 互斥锁
 * ⚠ 必须加: mpu9250_read_data() / mpu9250_read_gyro() 是对外独立接口,
 *   控制任务和 UI 页可能同时调用, 两条 i2c_master_* 同时操作同一总线
 *   会拿到互相穿插的数据。互补滤波还要求 accel/gyro 严格同一时刻,
 *   所以必须在最底层加锁。
 *   代价: 临界区只有一次 14 字节 I2C 读 (400kHz 下约 0.4ms), 可以接受。 */
static portMUX_TYPE s_i2c_mux = portMUX_INITIALIZER_UNLOCKED;

/* 上次调用互补滤波的时刻 (esp_timer 微秒), 首次为 0
 * ⚠ 以前 dt 硬编码 0.01f, 控制周期一改陀螺仪积分就错且无任何提示。
 *    改成自己测: 两次调用之间实际过了多久就用多久。 */
static int64_t s_filt_last_us = 0;

/**
 * @brief 写入寄存器
 * @note  调用方必须已持有 s_i2c_mux
 */
static esp_err_t mpu9250_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(MPU9250_I2C_NUM, MPU9250_I2C_ADDR,
                                       write_buf, 2, pdMS_TO_TICKS(1000));
}

/**
 * @brief 读取寄存器
 * @note  调用方必须已持有 s_i2c_mux
 */
static esp_err_t mpu9250_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(MPU9250_I2C_NUM, MPU9250_I2C_ADDR,
                                        &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

/**
 * @brief 初始化 I2C 总线
 */
static esp_err_t mpu9250_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU9250_SDA_GPIO,
        .scl_io_num = MPU9250_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU9250_I2C_FREQ,
    };
    
    esp_err_t ret = i2c_param_config(MPU9250_I2C_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 参数配置失败");
        return ret;
    }
    
    ret = i2c_driver_install(MPU9250_I2C_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 驱动安装失败");
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C 初始化成功 (SCL=%d, SDA=%d)", MPU9250_SCL_GPIO, MPU9250_SDA_GPIO);
    return ESP_OK;
}

/**
 * @brief 初始化 MPU9250
 */
esp_err_t mpu9250_init(void)
{
    esp_err_t ret;
    
    // 初始化 I2C
    ret = mpu9250_i2c_init();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 检查器件 ID
    uint8_t who_am_i;
    ret = mpu9250_read_reg(MPU9250_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取 WHO_AM_I 失败");
        return ret;
    }
    
    if (who_am_i != 0x71 && who_am_i != 0x73) {
        ESP_LOGE(TAG, "WHO_AM_I 错误: 0x%02X (期望 0x71 或 0x73)", who_am_i);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "MPU9250 检测成功, WHO_AM_I=0x%02X", who_am_i);
    
    // 复位设备
    ret = mpu9250_write_reg(MPU9250_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 唤醒设备,选择最佳时钟源
    ret = mpu9250_write_reg(MPU9250_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 配置数字低通滤波器 (DLPF_CFG=3, 带宽41Hz)
    ret = mpu9250_write_reg(MPU9250_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;
    
    // 配置默认量程
    ret = mpu9250_set_accel_fs(ACCEL_FS_2G);
    if (ret != ESP_OK) return ret;
    
    ret = mpu9250_set_gyro_fs(GYRO_FS_250DPS);
    if (ret != ESP_OK) return ret;
    
    // 配置加速度计低通滤波器
    ret = mpu9250_write_reg(MPU9250_ACCEL_CONFIG2, 0x03);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "MPU9250 初始化完成");
    s_ready = true;
    
    // 校准陀螺仪
    ESP_LOGI(TAG, "开始校准陀螺仪,请保持设备静止...");
    ret = mpu9250_calibrate_gyro(100);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "陀螺仪校准完成: offset(%.3f, %.3f, %.3f)", 
                 gyro_offset.x, gyro_offset.y, gyro_offset.z);
    }
    
    return ESP_OK;
}

/**
 * @brief 查询 MPU9250 是否就绪
 * @note  供上层模块 (比如平衡控制) 判断能不能用,
 *        避免拿到 ESP_ERR_INVALID_STATE 之类的错误码还要猜是不是没初始化
 * @return true=初始化成功且可用
 */
bool mpu9250_is_ready(void)
{
    return s_ready;
}

/**
 * @brief 配置加速度计量程
 */
esp_err_t mpu9250_set_accel_fs(mpu9250_accel_fs_t fs)
{
    esp_err_t ret = mpu9250_write_reg(MPU9250_ACCEL_CONFIG, fs << 3);
    if (ret == ESP_OK) {
        current_accel_fs = fs;
        // 更新转换系数
        accel_scale = 16384.0f / (1 << fs);
    }
    return ret;
}

/**
 * @brief 配置陀螺仪量程
 */
esp_err_t mpu9250_set_gyro_fs(mpu9250_gyro_fs_t fs)
{
    esp_err_t ret = mpu9250_write_reg(MPU9250_GYRO_CONFIG, fs << 3);
    if (ret == ESP_OK) {
        current_gyro_fs = fs;
        // 更新转换系数
        gyro_scale = 131.0f / (1 << fs);
    }
    return ret;
}

/**
 * @brief 读取原始数据
 * @param data 数据结构指针
 * @return ESP_OK 成功
 * @note  含 I2C 互斥保护, 可与控制任务并发调用
 */
esp_err_t mpu9250_read_data(mpu9250_data_t *data)
{
    uint8_t raw_data[14];

    taskENTER_CRITICAL(&s_i2c_mux);
    esp_err_t ret = mpu9250_read_reg(MPU9250_ACCEL_XOUT_H, raw_data, 14);
    taskEXIT_CRITICAL(&s_i2c_mux);
    if (ret != ESP_OK) {
        return ret;
    }

    // 解析加速度数据
    int16_t accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    
    // 解析温度数据
    int16_t temp_raw = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    
    // 解析陀螺仪数据
    int16_t gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    int16_t gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    int16_t gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    
    // 转换为物理单位
    data->accel.x = accel_x / accel_scale;
    data->accel.y = accel_y / accel_scale;
    data->accel.z = accel_z / accel_scale;
    
    data->gyro.x = gyro_x / gyro_scale - gyro_offset.x;
    data->gyro.y = gyro_y / gyro_scale - gyro_offset.y;
    data->gyro.z = gyro_z / gyro_scale - gyro_offset.z;
    
    data->temperature = temp_raw / 333.87f + 21.0f;
    
    return ESP_OK;
}

/**
 * @brief 读取加速度数据
 */
esp_err_t mpu9250_read_accel(mpu9250_axis_t *accel)
{
    mpu9250_data_t data;
    esp_err_t ret = mpu9250_read_data(&data);
    if (ret == ESP_OK) {
        *accel = data.accel;
    }
    return ret;
}

/**
 * @brief 读取陀螺仪数据
 */
esp_err_t mpu9250_read_gyro(mpu9250_axis_t *gyro)
{
    mpu9250_data_t data;
    esp_err_t ret = mpu9250_read_data(&data);
    if (ret == ESP_OK) {
        *gyro = data.gyro;
    }
    return ret;
}

/**
 * @brief 读取温度
 */
esp_err_t mpu9250_read_temperature(float *temp)
{
    mpu9250_data_t data;
    esp_err_t ret = mpu9250_read_data(&data);
    if (ret == ESP_OK) {
        *temp = data.temperature;
    }
    return ret;
}

/**
 * @brief 互补滤波解算欧拉角 (内部函数, 不碰 I2C)
 * @param data 同一时刻读到的 accel + gyro (调用方必须保证是同一次采样)
 * @param out  [出参] 欧拉角
 * @note  dt 由本函数自己测 (esp_timer), 不再假设固定 10ms
 */
static void mpu9250_fuse_euler(const mpu9250_data_t *data, euler_angle_t *out)
{
    /* 从加速度计计算角度 (只能当参考, 车一动加速度就混进了重力以外的干扰) */
    float accel_roll = atan2f(data->accel.y, data->accel.z) * 180.0f / M_PI;
    float accel_pitch = atan2f(-data->accel.x, sqrtf(data->accel.y * data->accel.y +
                               data->accel.z * data->accel.z)) * 180.0f / M_PI;

    /* 陀螺仪积分: dt 用两次调用之间的真实间隔
     * ⚠ 跨设备复位 / 长时间空档时 dt 会异常大, 积分会炸, 这里夹住上限 */
    int64_t now_us = esp_timer_get_time();
    float dt = 0.01f;                       /* 首次调用或异常时的兜底值 */
    if (s_filt_last_us > 0) {
        int64_t gap = now_us - s_filt_last_us;
        if (gap > 0) {
            dt = (float)gap / 1000000.0f;
            if (dt > 0.1f) dt = 0.1f;       /* 上限 100ms, 防止大间隔积分发散 */
        }
    }
    s_filt_last_us = now_us;

    current_angle.roll  += data->gyro.x * dt;
    current_angle.pitch += data->gyro.y * dt;
    current_angle.yaw   += data->gyro.z * dt;

    /* 互补滤波融合: 陀螺仪负责短时响应, 加速度计负责长期拉回真实水平 */
    current_angle.roll  = alpha * current_angle.roll  + (1 - alpha) * accel_roll;
    current_angle.pitch = alpha * current_angle.pitch + (1 - alpha) * accel_pitch;

    *out = current_angle;
}

/**
 * @brief 一次读取, 同时拿到姿态角和陀螺仪角速度 (推荐给控制任务用)
 * @param out        [出参] 姿态角
 * @param gyro_out   [出参] 俯仰/横滚/偏航三轴角速度 (°/s)
 * @return ESP_OK 成功
 * @note  ⚠ 控制任务必须用这个接口, 不要分别调 get_euler_angle() + read_gyro():
 *        那会读两次 I2C, 两次数据不同时刻, 姿态角和角速度错位,
 *        平衡环的 D 项(角速度阻尼)正是靠这两者的对应关系算的, 错位会让阻尼失准。
 */
esp_err_t mpu9250_get_pose(euler_angle_t *out, mpu9250_axis_t *gyro_out)
{
    mpu9250_data_t data;

    esp_err_t ret = mpu9250_read_data(&data);
    if (ret != ESP_OK) return ret;

    mpu9250_fuse_euler(&data, out);
    if (gyro_out != NULL) *gyro_out = data.gyro;
    return ESP_OK;
}

/**
 * @brief 获取欧拉角 (互补滤波)
 * @param angle 欧拉角指针
 * @return ESP_OK 成功
 * @note  只是 mpu9250_get_pose() 的兼容包装, 多传一个 NULL 而已
 */
esp_err_t mpu9250_get_euler_angle(euler_angle_t *angle)
{
    return mpu9250_get_pose(angle, NULL);
}

/**
 * @brief 校准陀螺仪零点
 */
esp_err_t mpu9250_calibrate_gyro(uint16_t samples)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        mpu9250_axis_t gyro;
        esp_err_t ret = mpu9250_read_gyro(&gyro);
        if (ret != ESP_OK) {
            return ret;
        }
        
        sum_x += gyro.x;
        sum_y += gyro.y;
        sum_z += gyro.z;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    gyro_offset.x = sum_x / samples;
    gyro_offset.y = sum_y / samples;
    gyro_offset.z = sum_z / samples;
    
    return ESP_OK;
}

/**
 * @brief 获取陀螺仪零点偏移
 * @param offset 输出三轴偏移 (°/s)
 */
void mpu9250_get_gyro_offset(mpu9250_axis_t *offset)
{
    if (offset) {
        *offset = gyro_offset;
    }
}

/**
 * @brief 软件复位 MPU9250
 */
esp_err_t mpu9250_reset(void)
{
    esp_err_t ret = mpu9250_write_reg(MPU9250_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ret;
}
