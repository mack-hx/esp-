/**
 * @file mpu9250.c
 * @brief MPU9250 / MPU6500 惯性测量单元驱动实现
 * @note  0.11 第八阶段: 修正 MPU 轴向 + 烘焙 PITCH_ZERO_REF
 *        0.13:         代码层面无改动 (mpu9250_get_pose 已使用 X 轴)
 * @author ESP32 机械狗项目
 * @date 2026-10-07
 */

#include "mpu9250.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MPU9250";

/* 陀螺仪零点偏移 */
static mpu9250_axis_t gyro_offset = {0, 0, 0};

/* ========== 陀螺零偏的 NVS 存放位置 ==========
 * namespace "imu", key "gof": 三轴零偏, 单位 0.001 °/s, 3 个 int16
 *
 * @brief 为什么必须存下来 (2026-10-26)
 *   零偏会进两个地方:
 *     1) 陀螺积分 → 互补滤波里的姿态角 (被 1/(1-alpha)≈49 倍放大的那一部分)
 *     2) 平衡环 D 项 → 一个**恒定**的力矩输出 (零偏 1°/s × Kd 1.5 = 1.5% 常数)
 *   每次上电都重新标定的话, 只要标定时手在抖 / 车没放稳, 零偏就是随机的 →
 *   每次上电的 pitch 和电机输出都不一样 (用户反馈: "pitch 每次上电重启后就不对")。
 *   存进 NVS 之后每次上电用同一个零偏 → 一致, 而且断电不丢。
 *   换 IMU 或觉得零偏不对时, 用 MPU 调试页的 K2 重新标定 (标完也会自动存盘)。 */
#define MPU_NVS_NS            "imu"
#define MPU_NVS_KEY_GYRO      "gof"
/* 零偏绝对值超过这个数就当作数据不可信 (重新标定) */
#define MPU_GYRO_OFFSET_MAX   5.0f
/* 标定时判定"设备在动"的阈值 (°/s): 超过就丢掉这个样本 */
#define MPU_GYRO_MOTION_THR   2.0f

/* 初始化是否成功 (供外部查询, 避免反复尝试初始化失败的模块) */
static bool s_ready = false;

/* ================= 陀螺零偏的持久化 ================= */

/**
 * @brief 把当前陀螺零偏写进 NVS
 * @note  存成 3 个 int16 (单位 0.001 °/s), 避免 float 的对齐问题
 */
static void mpu9250_save_gyro_offset(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(MPU_NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "陀螺零偏保存失败: NVS 打开失败 (%s)", esp_err_to_name(ret));
        return;
    }
    int16_t v[3] = {
        (int16_t)lrintf(gyro_offset.x * 1000.0f),
        (int16_t)lrintf(gyro_offset.y * 1000.0f),
        (int16_t)lrintf(gyro_offset.z * 1000.0f),
    };
    ret = nvs_set_blob(h, MPU_NVS_KEY_GYRO, v, sizeof(v));
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "陀螺零偏已保存: (%.3f, %.3f, %.3f) °/s",
                 (double)gyro_offset.x, (double)gyro_offset.y, (double)gyro_offset.z);
    } else {
        ESP_LOGE(TAG, "陀螺零偏保存失败: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 从 NVS 读回陀螺零偏
 * @return true=读到了可信的零偏 (已写进 gyro_offset); false=没有/不可信
 */
static bool mpu9250_load_gyro_offset(void)
{
    nvs_handle_t h;
    if (nvs_open(MPU_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    int16_t v[3] = {0, 0, 0};
    size_t len = sizeof(v);
    esp_err_t ret = nvs_get_blob(h, MPU_NVS_KEY_GYRO, v, &len);
    nvs_close(h);

    if (ret != ESP_OK || len != sizeof(v)) {
        return false;
    }

    mpu9250_axis_t off = {
        v[0] / 1000.0f,
        v[1] / 1000.0f,
        v[2] / 1000.0f,
    };
    /* 零偏不该有这么大, 超了说明数据是脏的 → 当作没有, 重新标定 */
    if (fabsf(off.x) > MPU_GYRO_OFFSET_MAX ||
        fabsf(off.y) > MPU_GYRO_OFFSET_MAX ||
        fabsf(off.z) > MPU_GYRO_OFFSET_MAX) {
        ESP_LOGW(TAG, "NVS 里的陀螺零偏不合理 (%.3f, %.3f, %.3f), 重新标定",
                 (double)off.x, (double)off.y, (double)off.z);
        return false;
    }

    gyro_offset = off;
    return true;
}

/* 当前量程配置 */
static mpu9250_accel_fs_t current_accel_fs = ACCEL_FS_2G;
static mpu9250_gyro_fs_t current_gyro_fs = GYRO_FS_250DPS;

/* 量程转换系数 */
static float accel_scale = 16384.0f;  // ±2g
static float gyro_scale = 131.0f;     // ±250°/s

/* 互补滤波参数
 * alpha = 陀螺仪(短期)权重, 越大越信陀螺仪、越不平滑
 *
 * ⚠⚠ 2026-10-26 大改: alpha 0.98 → 0.997
 *
 * 这个 alpha 决定的是**姿态角反馈通道的滞后**, 不是"平滑程度":
 *     姿态角 = alpha·(陀螺积分) + (1−alpha)·(加速度算出来的角)
 * 加速度那一路以 (1−alpha)/dt 的速率把角度往真实值上拉,
 * 也就是一个一阶低通, 时间常数 τ = dt/(1−alpha):
 *     alpha=0.98  → τ = 0.5s   ← 旧值
 *     alpha=0.995 → τ = 0.2s
 *     alpha=0.997 → τ = 0.03s  ← 现在
 *
 * 平衡环的穿越频率在 2~4Hz, 而 0.5s 的滞后 = 0.8 个周期 (≈290° 相位滞后)
 * → P 反馈用的是"半秒前的姿态" → 扰动一来就过冲, 越晃越大
 * (现象: 静止站着没事, 一碰/一走就剧烈抖动)
 *
 * 陀螺零偏已经在 NVS 里标定好了 (mpu9250_load_gyro_offset),
 * 所以陀螺积分的长期漂移很小, 把 alpha 调高没有副作用。
 * 代价: 陀螺残余零偏 (假设 0.1°/s) 带来的稳态角误差 = 零偏·dt/(1−alpha)
 *       = 0.33° (alpha=0.997), 可以接受 (起立时会自动微调配平零点)。 */
static euler_angle_t current_angle = {0, 0, 0};
static const float alpha = 0.997f;  // 互补滤波系数 (陀螺权重)

/* ========== 俯仰角零点基准 (°) ==========
 *
 * 俯仰角定义为"相对配平姿态往前倾多少度", 所以配平位的自然读数应该
 * 换算成 0°, 否则整条控制链要额外背 30 多度的固定偏置。
 *
 * 基准值由平衡位实测 (ax=0.03 ay=-0.85 az=0.57) 算出:
 *     atan2(az, -ay) = atan2(0.57, 0.85) = 33.85°
 *
 * ⚠ 换 MPU 或改了安装方向, 这个数就得重算。
 *   重算方法: 车扶到平衡姿态, 读 MPU 调试页的 ay / az,
 *   代入 atan2(az, -ay) 就是本常数。
 *
 * 2026-10-15 新四连杆结构上线
 * 实测: 水平位 az≈1, ay≈0 → atan2(1,0)=90°
 * 所以 PITCH_ZERO_REF = 90, 平衡时显示 pitch ≈ 0 */
#define PITCH_ZERO_REF   90.0f

/* I2C 总线互斥锁
 *
 * ⚠ 必须加: 控制任务 (10ms 周期) 和 UI 任务 (50ms 周期) 都会调
 *   mpu9250_read_data(), 两条 i2c_master_* 同时操作同一总线会拿到
 *   互相穿插的数据。互补滤波还要求 accel/gyro 严格同一时刻。
 *   虽然 I2C0 上只有 MPU9250 一个器件 (PCF8574 在 I2C1), 但**并发调用者**
 *   存在, 所以锁不能省。
 *
 * ⚠⚠ 必须是**互斥锁(semaphore)**, 不能用 taskENTER_CRITICAL():
 *   临界区会关中断, 而 i2c_master_cmd_begin() 等硬件应答是靠中断推进的。
 *   中断一关, I2C 永远等不到完成 → 中断看门狗超时 → 整机重启。
 *   (实测: 进 MPU 调试页必崩, panic = "Interrupt wdt timeout on CPU0")
 *
 *   互斥锁只阻止任务切换, 不关中断, I2C 中断照常工作, 两个需求都满足。 */
static SemaphoreHandle_t s_i2c_mutex = NULL;

/* 上次调用互补滤波的时刻 (esp_timer 微秒), 首次为 0
 * ⚠ 以前 dt 硬编码 0.01f, 控制周期一改陀螺仪积分就错且无任何提示。
 *    改成自己测: 两次调用之间实际过了多久就用多久。 */
static int64_t s_filt_last_us = 0;

/* 互补滤波是否已经出过第一帧 (第一帧直接用加速度角当初值) */
static bool s_filt_primed = false;

/**
 * @brief 获取 I2C 总线锁
 * @note  所有走 I2C 的入口 (init / read_data / calibrate / reset) 都要先调它
 */
static void mpu9250_lock(void)
{
    /* 锁没建好时不能等, 否则会永久阻塞。
     * 只在 init 之前的窗口期可能出现 (那时还不该有并发读)。 */
    if (s_i2c_mutex != NULL) {
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    }
}

/**
 * @brief 释放 I2C 总线锁
 */
static void mpu9250_unlock(void)
{
    if (s_i2c_mutex != NULL) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

/**
 * @brief 写入寄存器
 * @note  调用方必须已持有 I2C 锁 (见 mpu9250_lock)
 */
static esp_err_t mpu9250_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(MPU9250_I2C_NUM, MPU9250_I2C_ADDR,
                                       write_buf, 2, pdMS_TO_TICKS(1000));
}

/**
 * @brief 读取寄存器
 * @note  调用方必须已持有 I2C 锁 (见 mpu9250_lock)
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

    /* 先建 I2C 锁再碰总线: 后面所有 read_reg/write_reg 都假定锁已就绪。
     * 放最前面保证"init 中途失败"也不会留下半初始化状态让上层误用。 */
    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        if (s_i2c_mutex == NULL) {
            ESP_LOGE(TAG, "创建 I2C 互斥锁失败 (内存不足)");
            return ESP_ERR_NO_MEM;
        }
    }

    // 初始化 I2C
    ret = mpu9250_i2c_init();
    if (ret != ESP_OK) {
        return ret;
    }

    // 检查器件 ID
    // ⚠ 不只是 MPU9250: MPU6500 的 WHO_AM_I = 0x70, 寄存器完全兼容 MPU9250
    //   的加速度/陀螺部分 (MPU9250 实际就是 MPU6500 + 一颗 AK8963 磁力计),
    //   我们代码也没用到磁力计, 所以一并接受。三种 ID 对应的启用结果:
    //     0x71 = MPU9250 (标准 9-axis)
    //     0x73 = MPU9250 (某些批次, 同上)
    //     0x70 = MPU6500 (6-axis, 加速度+陀螺同 9250, 没磁力计 → 我们用不到)
    //   如果是 0x68/0x98 之类的值说明器件 ID 寄存器读到的根本不是 IMU,
    //   这通常是 AD0 接错 / I2C 地址冲突 / SDA/SCL 接反。
    uint8_t who_am_i;
    ret = mpu9250_read_reg(MPU9250_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取 WHO_AM_I 失败");
        return ret;
    }

    if (who_am_i != 0x71 && who_am_i != 0x73 && who_am_i != 0x70) {
        ESP_LOGE(TAG, "WHO_AM_I 错误: 0x%02X (期望 0x70/0x71/0x73)", who_am_i);
        ESP_LOGE(TAG, "如为 0x98 之类 → AD0/SDA/SCL 接错或地址冲突; 模块可能没真接好");
        return ESP_FAIL;
    }

    /* 0x70 = MPU6500 (6-axis),  0x71/0x73 = MPU9250 (9-axis)
     * 后面所有寄存器操作两种芯片一样, 不再分支 */
    if (who_am_i == 0x70) {
        ESP_LOGW(TAG, "检测到 MPU6500 (WHO_AM_I=0x70), 加速度+陀螺与 MPU9250 兼容, 不支持磁力计");
    }
    ESP_LOGI(TAG, "IMU 检测成功, WHO_AM_I=0x%02X", who_am_i);
    
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
    
    ESP_LOGI(TAG, "IMU 初始化完成 (加速度+陀螺配置就绪)");
    s_ready = true;

    /* 校准陀螺仪:
     * 优先用 NVS 里存好的零偏 (每次上电用同一个 → pitch 每次上电一致),
     * 没有才现场标定 (要求设备静止), 标定完自动存盘。 */
    if (mpu9250_load_gyro_offset()) {
        ESP_LOGI(TAG, "已载入陀螺仪零偏 (NVS): (%.3f, %.3f, %.3f)",
                 (double)gyro_offset.x, (double)gyro_offset.y, (double)gyro_offset.z);
    } else {
        ESP_LOGI(TAG, "开始校准陀螺仪,请保持设备静止...");
        ret = mpu9250_calibrate_gyro(100);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "陀螺仪校准完成: offset(%.3f, %.3f, %.3f)",
                     (double)gyro_offset.x, (double)gyro_offset.y, (double)gyro_offset.z);
        } else {
            ESP_LOGW(TAG, "陀螺仪校准未成功 (设备可能在动), 零偏暂用 0; "
                          "可在 MPU 调试页按 K2 重新标定");
        }
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
 * @brief 读取**未经零偏补偿**的原始陀螺仪读数 (°/s)
 * @param gyro [出参] 三轴原始角速度
 * @return ESP_OK 成功
 * @note  标定零偏必须用原始值: mpu9250_read_gyro() 已经把旧零偏扣掉了,
 *        再拿它去算"新零偏"会算成残差 (二次相减), 越校越歪。
 */
static esp_err_t mpu9250_read_raw_gyro(mpu9250_axis_t *gyro)
{
    uint8_t raw[6];

    mpu9250_lock();
    esp_err_t ret = mpu9250_read_reg(MPU9250_GYRO_XOUT_H, raw, 6);
    mpu9250_unlock();
    if (ret != ESP_OK) return ret;

    int16_t gx = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t gy = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t gz = (int16_t)((raw[4] << 8) | raw[5]);

    gyro->x = gx / gyro_scale;
    gyro->y = gy / gyro_scale;
    gyro->z = gz / gyro_scale;
    return ESP_OK;
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

    mpu9250_lock();
    esp_err_t ret = mpu9250_read_reg(MPU9250_ACCEL_XOUT_H, raw_data, 14);
    mpu9250_unlock();
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
    /* ---------- ⚠⚠ 本机 MPU 轴向定义 (2026-10-25 真机实测标定) ----------
     *
     * MPU 是**平装**在车体上的, 车在平衡位是**立着的**, 所以 MPU 的板面是竖直的。
     * 实测 (平衡位, 直立不倒):  ax≈+0.03  ay≈-0.10  az≈+1.03  (单位 g)
     *   az 最大且为正 -> 重力主要在 **Z 轴** -> MPU 的 **Z 轴 ≈ 车的"上"**
     *   前后倾时重力在 Y↔Z 之间转移 -> 俯仰旋转轴 = MPU 的 **X 轴**
     *
     * 由此得到车体坐标系与 MPU 坐标系的对应:
     *     车的上方向  = MPU 的  Z 轴
     *     车体前后轴  = MPU 的  Y 轴  (**车头方向 = MPU 的 -Y**)
     *     车体左右轴  = MPU 的  X 轴
     *
     * ⚠ 符号约定 (实测): **车头前倾 (车头往下) 时, pitch 与 gx 都是负数**。
     *   即前倾 = 绕 X 轴的负向旋转, 所以
     *     - 加速度算出来的 pitch = atan2(az, -ay) - 90 = -(前倾角)
     *     - 陀螺积分写成 pitch += gyro.x·dt, 两者符号自洽 (互补滤波不会互相打架)
     *   之前注释里写的 "前倾时 gx 为正" 是老装机状态的, 已作废。
     *
     * ⚠ 以前这里写的是 atan2(-accel.x, sqrt(ay²+az²)), 也就是假设
     *   "车的上 = MPU 的 Z 轴"却算错了轴 —— 在那个错误公式下, 车前后倾时 ax 恒为 0,
     *   pitch 跟不到倾斜 —— 这就是曾经"车转 30° 读数只动 2~3°"的原因
     *   (那 2~3° 是陀螺零偏漂移出来的假象)。
     *   现在这个公式 (绕 X 轴的转角) 与实际装机一致。
     */
    float accel_pitch = atan2f(data->accel.z, -data->accel.y) * 180.0f / M_PI
                       - PITCH_ZERO_REF;   /* 前倾为负 */
    /* roll 是左右倾 (绕车体前后轴 = MPU 的 Z 轴), 本车不控制, 保留一个
     * 粗略值仅供观察: 左右倾时重力在 X↔Y 之间转移 */
    float accel_roll  = atan2f(data->accel.x, -data->accel.y) * 180.0f / M_PI;

    /* ---------- 首帧: 直接用加速度算出的角度当初值 (2026-10-26) ----------
     * 以前 current_angle 从 0 开始, 要靠互补滤波慢慢收敛 (时间常数 ~0.5s),
     * 结果上电后这半秒里 pitch 读数是错的 (用户反馈"pitch 每次上电重启后就不对"),
     * 如果这时候刚好起立, 控制器会拿着一个错的姿态角输出。
     * 现在首帧直接用加速度角, 读数从第一帧就是对的。 */
    if (!s_filt_primed) {
        current_angle.pitch = accel_pitch;
        current_angle.roll  = accel_roll;
        current_angle.yaw   = 0.0f;
        s_filt_primed = true;
        *out = current_angle;
        return;
    }

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

    /* 陀螺仪积分: 俯仰是绕 MPU 的 X 轴, 所以 pitch 积分 **gyro.x**
     * (不是 gyro.y —— 那是车体左右轴, 对两轮前后走的平衡车是纯噪声)
     * 实测: 车头前倾 gx 为正, 与 accel_pitch 增大同号, 所以不取负。
     *
     * roll 绕 MPU 的 Z 轴 (车体前后轴), yaw 绕 MPU 的 Y 轴 (车体竖直轴)。
     * roll/yaw 本车不使用, 积分只为了数值连续, 不参与控制。
     * ⚠ 陀螺零偏会被 1/(1-alpha) ≈ 49 倍放大, 所以零偏校准必须做,
     *   且 accel 通道是唯一的长期纠偏来源。 */
    current_angle.roll  += data->gyro.z * dt;
    current_angle.pitch += data->gyro.x * dt;
    current_angle.yaw   += data->gyro.y * dt;

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
 * @param samples 采样次数
 * @return ESP_OK 成功; ESP_ERR_INVALID_STATE 采样期间设备在动, 标定失败
 * @note  ⚠ 2026-10-26 改动:
 *   1) 用**原始**读数 (不扣旧零偏) 计算新零偏 —— 之前用已补偿过的读数,
 *      二次标定时算出来的是残差, 越校越歪;
 *   2) 剔除采样期间"设备在动"的样本 (任一轴 > 2°/s): 标定时手在抖 /
 *      车没放稳的话, 采到的是运动而不是零偏, 直接平均会得到一个假的零偏,
 *      表现为每次上电 pitch 都不一样;
 *   3) 标定成功后**自动存进 NVS**, 断电不丢。
 */
esp_err_t mpu9250_calibrate_gyro(uint16_t samples)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t used = 0;

    for (uint16_t i = 0; i < samples; i++) {
        mpu9250_axis_t g;
        esp_err_t ret = mpu9250_read_raw_gyro(&g);
        if (ret != ESP_OK) {
            return ret;
        }

        if (fabsf(g.x) > MPU_GYRO_MOTION_THR ||
            fabsf(g.y) > MPU_GYRO_MOTION_THR ||
            fabsf(g.z) > MPU_GYRO_MOTION_THR) {
            /* 设备在动 → 这个样本丢掉 (但照常延时, 保持采样节奏) */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        sum_x += g.x;
        sum_y += g.y;
        sum_z += g.z;
        used++;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* 有效的样本太少 → 说明采样期间一直在动, 不能拿这种数据当零偏 */
    if (used < samples / 2) {
        ESP_LOGE(TAG, "陀螺仪校准失败: 采样期间检测到运动 "
                      "(%u/%u 个样本被剔除), 请把设备放稳再校准",
                 (unsigned)(samples - used), (unsigned)samples);
        return ESP_ERR_INVALID_STATE;
    }

    gyro_offset.x = sum_x / used;
    gyro_offset.y = sum_y / used;
    gyro_offset.z = sum_z / used;

    /* 存盘: 下次上电直接用这个零偏, 保证每次上电一致 */
    mpu9250_save_gyro_offset();

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
 * @note  UI 页 K3 会调, 与控制任务可能并发, 所以要持锁
 */
esp_err_t mpu9250_reset(void)
{
    mpu9250_lock();
    esp_err_t ret = mpu9250_write_reg(MPU9250_PWR_MGMT_1, 0x80);
    mpu9250_unlock();
    vTaskDelay(pdMS_TO_TICKS(100));
    /* 复位后姿态角的历史值作废: 下一帧重新用加速度角初始化 */
    s_filt_primed = false;
    s_filt_last_us = 0;
    return ret;
}
