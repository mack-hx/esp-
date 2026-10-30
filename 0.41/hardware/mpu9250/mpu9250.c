/**
 * @file mpu9250.c
 * @brief MPU9250 / MPU6500 惯性测量单元驱动实现
 * @note  0.11 第八阶段: 修正 MPU 轴向 + 烘焙 PITCH_ZERO_REF
 *        代码层面用 X 轴 (mpu9250_get_pose)
 * @author ESP32 机械狗项目
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
#include <stdio.h>

static const char *TAG = "MPU9250";

/* 陀螺仪零点偏移 */
static mpu9250_axis_t gyro_offset = {0, 0, 0};

static bool        s_mag_available = false;    /* AK8963 探测成功 */
static mpu9250_axis_t s_mag_raw    = {0, 0, 0}; /* 最近一次磁力计原始值 (LSB) */
static mpu9250_axis_t s_mag_ut     = {0, 0, 0}; /* 最近一次换算后的磁场 (μT) */
static float       s_mag_adj[3]    = {1.0f, 1.0f, 1.0f}; /* 厂标敏感度系数 */
static int64_t     s_mag_log_ms    = 0;        /* 上次打印磁力计日志的时刻 */
static bool        s_mag_data_ok   = false;    /* 最近一次是否读到了新数据 */

/* ========== 陀螺零偏的 NVS 存放位置 ==========
 * namespace "imu", key "gof": 三轴零偏, 单位 0.001 °/s, 3 个 int16
 */
#define MPU_NVS_NS            "imu"
#define MPU_NVS_KEY_GYRO      "gof"
/* 零偏绝对值超过这个数就当作数据不可信 (重新标定) */
#define MPU_GYRO_OFFSET_MAX   5.0f
#define MPU_GYRO_MOTION_THR   2.5f

/* ========== 陀螺零偏标定: "最稳窗口" 参数 ==========
 * 窗口长度 25 个样本 × 10ms = 0.25s。
 * ⚠ 26 个以上样本时窗口才有意义 (要能算标准差); 少于 26 个就用整段。 */
#define MPU_GYRO_CAL_WIN      25      /* 最稳窗口长度 (样本数, ≈0.25s) */
/* 采样数组上限: 256 个样本 = 2.56s @10ms。
 * ⚠ 上限是为了不把静态数组开太大 (256×3×4 = 3KB, 直接放 .bss) */
#define MPU_GYRO_CAL_MAX      256

/* ========== 零偏慢漂跟随 (slow drift tracking) 参数 ==========
 * 详见 mpu9250_set_bias_tracking() 的注释。
 *
 * 静止判定阈值: 扣过零偏后, 三轴绝对值之**和**的 1 秒滑动平均 < 0.5 °/s
 * ⚠ 这个阈值偏保守: 残余零偏每轴 0.17 °/s 就会顶到阈值 → 跟随停住。
 *   真机发现跟不动时, 调大这个数 (比如 1.5) 即可。 */
#define MPU_BIAS_STATIC_THR   0.5f
/* 静止判定的滑动窗口长度 (样本数): 100 个样本 @10ms = 1 秒 */
#define MPU_BIAS_WIN_SAMPLES  100
/* 两次零偏调整之间的最小间隔 (毫秒) */
#define MPU_BIAS_ADJ_INTERVAL_MS  30000
/* 每次调整零偏的步长比例 (5%) */
#define MPU_BIAS_ADJ_ALPHA     0.05f

/* 16bit 连续模式下原始值 → μT 的系数 */
#define AK8963_RAW_TO_UT       0.15f
/* 磁力计日志节流周期 (毫秒): 每秒最多一条 */
#define AK8963_LOG_PERIOD_MS   1000

static bool     s_bias_track_on     = false;  /* 慢漂跟随是否打开 */
static float    s_bias_win_sum      = 0.0f;   /* 滚动窗口内"三轴绝对值之和"的累加 */
static float    s_bias_win_buf[MPU_BIAS_WIN_SAMPLES]; /* 滚动窗口的环形缓冲 (每样本一个值) */
static uint16_t s_bias_win_idx      = 0;      /* 环形缓冲写指针 */
static uint16_t s_bias_win_cnt      = 0;      /* 窗口内已累计的样本数 */
static int64_t  s_bias_last_adj_ms  = 0;      /* 上次调整零偏的时刻 (esp_timer ms) */
static int64_t  s_bias_last_log_ms  = 0;      /* 上次打印调整日志的时刻 (限流用) */
/* 标定进行中标志: 标定会写 gyro_offset, 而慢漂跟随也会改它 ——
 * 两者同时动这个变量会把刚标好的零偏又带偏。标定期间靠这个标志
 * 让慢漂跟随让路 (标志的读写都在 I2C 锁里, 不会竞态)。 */
static bool     s_calibrating       = false;

static char  bias_log_buf[128];
static bool  bias_log_pending = false;
static char  mag_log_buf[160];

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
 * 姿态角 = alpha·(陀螺积分) + (1−alpha)·(加速度算出来的角)
 * 加速度一路是一阶低通, 时间常数 τ = dt/(1−alpha); alpha=0.997 → τ = 0.03s
 * 陀螺残余零偏 0.1°/s 带来的稳态角误差 = 零偏·dt/(1−alpha) ≈ 0.33° */
static euler_angle_t current_angle = {0, 0, 0};
static const float alpha = 0.997f;  // 互补滤波系数 (陀螺权重)

/* ========== 俯仰角零点基准 (°) ==========
 * 俯仰角 = 相对配平姿态往前倾多少度, 配平位读数应换算成 0°。
 * 基准值由平衡位实测 (ax=0.03 ay=-0.85 az=0.57) 算出:
 *     atan2(az, -ay) = atan2(0.57, 0.85) = 33.85°
 * 换 MPU 或改安装方向需重算: 车扶到平衡姿态, 读 ay / az, 代入 atan2(az, -ay)。
 * 水平位 az≈1, ay≈0 → atan2(1,0)=90°, 故 PITCH_ZERO_REF = 90 */
#define PITCH_ZERO_REF   90.0f

/* I2C 总线互斥锁
 * 控制任务 (10ms) 与 UI 任务 (50ms) 都会调 mpu9250_read_data(), 并发访问同一
 * 总线会拿到互相穿插的数据, 互补滤波还要求 accel/gyro 同一时刻。
 * 必须用互斥锁 (semaphore), 不能用 taskENTER_CRITICAL(): 临界区关中断,
 * 而 i2c_master_cmd_begin() 靠中断推进, 会触发 "Interrupt wdt timeout on CPU0"。 */
static SemaphoreHandle_t s_i2c_mutex = NULL;

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
 * @brief 写入寄存器 (指定器件地址)
 * @param dev_addr 器件的 7 位 I2C 地址 (MPU9250 = 0x68, AK8963 bypass = 0x0C)
 * @param reg      寄存器地址
 * @param data     要写入的数据
 * @return ESP_OK 成功
 * @note  调用方必须已持有 I2C 锁 (见 mpu9250_lock)
 */
static esp_err_t mpu9250_write_reg_at(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(MPU9250_I2C_NUM, dev_addr,
                                       write_buf, 2, pdMS_TO_TICKS(1000));
}

/**
 * @brief 读取寄存器 (指定器件地址)
 * @param dev_addr 器件的 7 位 I2C 地址
 * @param reg      寄存器地址
 * @param data     输出缓冲区
 * @param len      读取长度
 * @return ESP_OK 成功
 * @note  调用方必须已持有 I2C 锁 (见 mpu9250_lock)
 */
static esp_err_t mpu9250_read_reg_at(uint8_t dev_addr, uint8_t reg,
                                      uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(MPU9250_I2C_NUM, dev_addr,
                                        &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

/**
 * @brief 写入寄存器 (MPU9250 本体)
 * @note  薄封装, 地址固定 MPU9250_I2C_ADDR; 磁力计用 mpu9250_write_reg_at()
 */
static esp_err_t mpu9250_write_reg(uint8_t reg, uint8_t data)
{
    return mpu9250_write_reg_at(MPU9250_I2C_ADDR, reg, data);
}

/**
 * @brief 读取寄存器 (MPU9250 本体)
 * @note  薄封装, 地址固定 MPU9250_I2C_ADDR; 磁力计用 mpu9250_read_reg_at()
 */
static esp_err_t mpu9250_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return mpu9250_read_reg_at(MPU9250_I2C_ADDR, reg, data, len);
}

/**
 * @brief 探测 AK8963 磁力计 (MPU9250 内置)
 * @return ESP_OK 找到并配置好; ESP_ERR_INVALID_STATE 没找到 (不报错退出)
 *
 * @note  流程 (bypass 模式):
 *   1) INT_PIN_CFG |= 0x42  (bit6 允许 bypass, bit1 INT 直通)
 *   2) USER_CTRL = 0x00     (退出 I2C master 模式)
 *   3) 等 10ms 让 bypass 生效
 *   4) 读 0x0C 的 WIA, 期望 0x48
 *   5) 找到的话: CNTL1=0x00 → 0x0F (ROM 模式读 ASA 厂标) → 0x12 (16bit 连续模式)
 *      → 再读一次 WIA 确认
 * @note  找不到 AK8963 只打告警, 不影响加速度/陀螺。
 */
static esp_err_t mpu9250_mag_probe(void)
{
    uint8_t val = 0;
    esp_err_t ret;

    /* 先把"可用"标志清掉: 重新 init (UI 页 K3 软复位后) 时探测可能失败,
     * 留着上一次的 true 会让上层以为磁力计还在, 去读一个状态未知的器件 */
    s_mag_available = false;

    /* 1) 打开 bypass + 关掉 MPU 的 I2C master */
    ret = mpu9250_read_reg(MPU9250_INT_PIN_CFG, &val, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "磁力计探测失败: 读 INT_PIN_CFG 失败 (%s)", esp_err_to_name(ret));
        return ret;
    }
    val |= 0x42;
    ret = mpu9250_write_reg(MPU9250_INT_PIN_CFG, val);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "磁力计探测失败: 写 INT_PIN_CFG 失败 (%s)", esp_err_to_name(ret));
        return ret;
    }
    ret = mpu9250_write_reg(MPU9250_USER_CTRL, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "磁力计探测失败: 写 USER_CTRL 失败 (%s)", esp_err_to_name(ret));
        return ret;
    }

    /* 2) 等 bypass 生效 (MPU 手册要求至少 10ms) */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 3) 读 AK8963 的 WIA (WHO_AM_I), 期望 0x48 */
    uint8_t wia = 0;
    ret = mpu9250_read_reg_at(AK8963_I2C_ADDR, AK8963_REG_WIA, &wia, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "没找到 AK8963 磁力计 (读不到 WHO_AM_I), 这块板的磁力计不是 AK8963 "
                     "(可能是换过的/盗版芯片), 磁力计不可用");
        return ESP_ERR_INVALID_STATE;
    }
    if (wia != AK8963_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "没找到 AK8963 磁力计 (WHO_AM_I=0x%02X, 期望 0x48), 这块板的磁力计"
                     "不是 AK8963 (可能是换过的/盗版芯片), 磁力计不可用",
                 (unsigned)wia);
        return ESP_ERR_INVALID_STATE;
    }

    /* 4) power down → ROM 模式读厂标系数 (ASAX/Y/Z) */
    ret = mpu9250_write_reg_at(AK8963_I2C_ADDR, AK8963_REG_CNTL1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 power down 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    ret = mpu9250_write_reg_at(AK8963_I2C_ADDR, AK8963_REG_CNTL1, 0x0F);  /* ROM 模式 */
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 进 ROM 模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t asa[3] = {0, 0, 0};
    ret = mpu9250_read_reg_at(AK8963_I2C_ADDR, AK8963_REG_ASAX, asa, 3);
    if (ret == ESP_OK) {
        for (int k = 0; k < 3; k++) {
            /* 厂标系数: adj = 1 + (asa - 128)/256
             * ⚠ asa 是**无符号** 8 位寄存器值 (0~255), 按 AK8963 手册的公式
             *   H_adj = H × ((ASAx − 128) × 2⁻⁸ + 1) 换算。
             *   所以这里不能用 (int8_t) 强转 —— asa > 127 时会被解释成负数,
             *   系数会算成 0.2x 这种离谱值 (参考工程 3.4 就是用无符号的写法)。
             * ⚠ 系数必须为正, 否则读出来的磁场是反的/离谱的 → 兜底成 1.0 */
            float adj = 1.0f + ((float)asa[k] - 128.0f) / 256.0f;
            s_mag_adj[k] = (adj > 0.0f) ? adj : 1.0f;
        }
        ESP_LOGI(TAG, "AK8963 厂标系数 ASA(%d,%d,%d) → 灵敏度(%.4f, %.4f, %.4f)",
                 (int)asa[0], (int)asa[1], (int)asa[2],
                 (double)s_mag_adj[0], (double)s_mag_adj[1], (double)s_mag_adj[2]);
    } else {
        ESP_LOGW(TAG, "读 AK8963 厂标系数失败 (%s), 用默认灵敏度 1.0",
                 esp_err_to_name(ret));
    }

    /* 5) 进 16bit 连续模式 (0x12 = 连续 100Hz + 16bit 输出) */
    ret = mpu9250_write_reg_at(AK8963_I2C_ADDR, AK8963_REG_CNTL1, 0x12);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 进 16bit 连续模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 6) 再读一次 WIA 确认还在线 (连续模式下 WIA 仍然是 0x48) */
    wia = 0;
    ret = mpu9250_read_reg_at(AK8963_I2C_ADDR, AK8963_REG_WIA, &wia, 1);
    if (ret != ESP_OK || wia != AK8963_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "AK8963 进入连续模式后失联 (WHO_AM_I=0x%02X), 磁力计不可用",
                 (unsigned)wia);
        return ESP_ERR_INVALID_STATE;
    }

    s_mag_available = true;
    ESP_LOGI(TAG, "AK8963 磁力计就绪 (WHO_AM_I=0x%02X), 16bit 连续模式",
             (unsigned)wia);
    return ESP_OK;
}

/**
 * @brief 读取磁力计数据 (内部函数, 调用方必须已持有 I2C 锁)
 * @param out [出参] 三轴磁场 (μT); 数据没就绪时沿用上一次的读数
 * @return ESP_OK 成功
 */
static void mpu9250_read_mag_locked(mpu9250_axis_t *out)
{
    uint8_t st1 = 0;

    /* ST1 bit0 = DRDY: 数据没就绪就不读, 沿用上一次的读数。
     * ⚠ 16bit 模式下数据没就绪时读 HXL 拿到的不是有效值 (会拿到 0 或旧值),
     *   所以这里必须先查 DRDY。 */
    if (mpu9250_read_reg_at(AK8963_I2C_ADDR, AK8963_REG_ST1, &st1, 1) != ESP_OK) {
        s_mag_data_ok = false;
        if (out) *out = s_mag_ut;
        return;
    }
    if ((st1 & 0x01) == 0) {
        /* 数据没就绪 → 沿用上次读数 */
        s_mag_data_ok = false;
        if (out) *out = s_mag_ut;
        return;
    }

    /* HXL..HZH (0x03~0x08, 小端) + ST2 (0x09) 一次读完:
     * ST2 必须读, 它会清掉 DRDY 标志, 否则下次永远等不到新数据。 */
    uint8_t buf[7] = {0};
    if (mpu9250_read_reg_at(AK8963_I2C_ADDR, AK8963_REG_HXL, buf, 7) != ESP_OK) {
        s_mag_data_ok = false;
        if (out) *out = s_mag_ut;
        return;
    }

    int16_t mx = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);   /* HXL 低字节在前 */
    int16_t my = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    int16_t mz = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    s_mag_raw.x = (float)mx;
    s_mag_raw.y = (float)my;
    s_mag_raw.z = (float)mz;

    /* 量纲换算: μT = 原始值 × 厂标系数 × 0.15 */
    s_mag_ut.x = (float)mx * s_mag_adj[0] * AK8963_RAW_TO_UT;
    s_mag_ut.y = (float)my * s_mag_adj[1] * AK8963_RAW_TO_UT;
    s_mag_ut.z = (float)mz * s_mag_adj[2] * AK8963_RAW_TO_UT;
    s_mag_data_ok = true;

    if (out) *out = s_mag_ut;
}
/**
 * @brief 读磁力计 + 打观察日志 (给**低频**任务用, 不要在控制任务里调)
 * @return ESP_OK 成功; ESP_ERR_INVALID_STATE 这块板没有可用的 AK8963
 */
esp_err_t mpu9250_update_mag(void)
{
    if (!s_mag_available) {
        return ESP_ERR_INVALID_STATE;
    }

    mpu9250_lock();
    mpu9250_read_mag_locked(NULL);

    /* 每秒打一条日志: 让用户看串口就能判断这个"盗版"磁力计的数据有没有用。
     * ⚠ 限流, 不刷屏。
     * ⚠⚠ 这里只把内容**填进缓冲**, 真正的打印放到解锁之后 ——
     *   ESP_LOGI 走 UART, 一条 100 多字节的日志要写 10ms 左右,
     *   在 I2C 锁里打印会把调用者卡住。 */
    int64_t now_ms = esp_timer_get_time() / 1000;
    bool log_now = false;
    if ((now_ms - s_mag_log_ms) >= AK8963_LOG_PERIOD_MS) {
        s_mag_log_ms = now_ms;
        float mag_norm = sqrtf(s_mag_ut.x * s_mag_ut.x +
                               s_mag_ut.y * s_mag_ut.y +
                               s_mag_ut.z * s_mag_ut.z);
        snprintf(mag_log_buf, sizeof(mag_log_buf),
                 "MAG raw(%.0f, %.0f, %.0f) uT(%.1f, %.1f, %.1f) "
                 "|B|=%.1f uT %s",
                 (double)s_mag_raw.x, (double)s_mag_raw.y, (double)s_mag_raw.z,
                 (double)s_mag_ut.x, (double)s_mag_ut.y, (double)s_mag_ut.z,
                 (double)mag_norm,
                 s_mag_data_ok ? "数据就绪" : "数据未就绪(沿用上次)");
        log_now = true;
    }
    mpu9250_unlock();

    if (log_now) {
        ESP_LOGI(TAG, "%s", mag_log_buf);
    }
    return ESP_OK;
}

/**
 * @brief 开/关"零偏慢漂跟随"
 * @param on true=允许零偏在设备静止时缓慢向当前读数靠拢
 */
void mpu9250_set_bias_tracking(bool on)
{
    if (s_bias_track_on == on) return;

    s_bias_track_on = on;
    /* 开关切换时把统计窗口清掉: 开着的时候攒的窗口数据是"静止"条件下
     * 攒的, 关掉期间设备可能被搬动过, 混在一起会误判。
     * ⚠ 环形缓冲也要清, 否则清零后的 sum 会跟缓冲里残留的旧值对不上账。 */
    s_bias_win_sum = 0.0f;
    s_bias_win_cnt = 0;
    s_bias_win_idx = 0;
    memset(s_bias_win_buf, 0, sizeof(s_bias_win_buf));
    ESP_LOGI(TAG, "零偏慢漂跟随: %s", on ? "开" : "关");
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

    /* ---------- 陀螺零偏 ----------
     * 每次上电都重新标定 (成功就存进 NVS), 失败才退回 NVS 里存好的那份。
     */
    mpu9250_axis_t saved_off = {0, 0, 0};
    bool have_saved = mpu9250_load_gyro_offset();
    if (have_saved) {
        saved_off = gyro_offset;
        ESP_LOGI(TAG, "NVS 里存着陀螺零偏: (%.3f, %.3f, %.3f) °/s",
                 (double)saved_off.x, (double)saved_off.y, (double)saved_off.z);
    }

    ret = ESP_ERR_INVALID_STATE;
    for (int attempt = 1; attempt <= 2; attempt++) {
        /* 采样 200 个 (2 秒), 给"挑最稳的 0.25 秒窗口"留足素材。
         * ⚠ 提示措辞按新逻辑改: 现在只要能拿稳 0.25 秒就能标成功,
         *   不再要求"全程静止" (那是手持标定永远做不到的事)。 */
        ESP_LOGI(TAG, "开始校准陀螺仪 (第 %d 次), 请拿稳设备 "
                      "(可以拿在手上, 只要 0.25 秒内不晃)...", attempt);
        ret = mpu9250_calibrate_gyro(200);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "陀螺仪校准完成: offset(%.3f, %.3f, %.3f)",
                     (double)gyro_offset.x, (double)gyro_offset.y, (double)gyro_offset.z);
            break;
        }
        if (attempt < 2) {
            ESP_LOGW(TAG, "校准未成功, 0.5s 后再试一次");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    if (ret != ESP_OK) {
        if (have_saved) {
            /* 标定失败 (设备在动) → 退回 NVS 里存好的那份 */
            gyro_offset = saved_off;
            ESP_LOGW(TAG, "标定失败, 沿用 NVS 里的零偏: (%.3f, %.3f, %.3f) °/s",
                     (double)gyro_offset.x, (double)gyro_offset.y, (double)gyro_offset.z);
        } else {
            ESP_LOGE(TAG, "陀螺仪校准最终未成功, 零偏暂用 0 —— 姿态角会有约 3° 的"
                          "恒定误差, 车起立就会剧烈抖! 请把设备放稳 (手拿也行, "
                          "只要 0.25 秒不晃) 后重新上电, "
                          "或在 MPU 调试页按 K2 重新标定");
        }
    }

    /* ---------- 磁力计 (AK8963) 探测 ----------
     * ⚠ 放在陀螺零偏标定**之后**: 这块板是"盗版"芯片, 磁力计的探测
     *   (bypass 模式切换 + 几次 I2C 事务) 有可能干扰陀螺读数。
     *   陀螺零偏是车能不能站住的关键, 磁力计只是观察用, 所以让磁力计让路。
     *
     * ⚠ 探测失败**只打告警, 不影响 IMU 其余功能** (加速度/陀螺照常用)。 */
    mpu9250_lock();
    esp_err_t mag_ret = mpu9250_mag_probe();
    mpu9250_unlock();
    if (mag_ret != ESP_OK) {
        ESP_LOGW(TAG, "磁力计不可用 (不影响加速度/陀螺), 继续运行");
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
    if (ret != ESP_OK) {
        mpu9250_unlock();
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

    /* ---------- 零偏慢漂跟随 (slow drift tracking) ----------
     * ⚠ 用的是**扣过零偏后**的角速度 (data->gyro): 只有扣完零偏还接近 0,
     *   才说明设备真的静止 (不动的陀螺读数应该只剩残余零偏)。
     *
     * 统计量: 三轴绝对值之和的滚动窗口平均 (窗口 = 100 个样本 ≈ 1 秒)。
     * 静止判定: 平均 < MPU_BIAS_STATIC_THR (0.5 °/s) → 设备静止。
     * 动作: 距离上次调整超过 30 秒 → 零偏朝当前原始读数挪 5%。
     */
    if (s_bias_track_on && !s_calibrating) {
        float abs_sum = fabsf(data->gyro.x) + fabsf(data->gyro.y) + fabsf(data->gyro.z);

        /* 滚动窗口: 每个样本的 abs_sum 存环形缓冲, 满 100 个以后
         * 把最老的换掉。
         * ⚠ 用环形缓冲而不是"攒够 100 个就清零": 后者只在窗口刚满的那
         *   一瞬间判断一次, 两次判断之间隔了整整 1 秒;
         *   滚动窗口每来一个样本就更新, 判定及时得多。 */
        s_bias_win_sum -= s_bias_win_buf[s_bias_win_idx];   /* 减去被换掉的那个 */
        s_bias_win_buf[s_bias_win_idx] = abs_sum;
        s_bias_win_sum += abs_sum;
        s_bias_win_idx = (uint16_t)((s_bias_win_idx + 1) % MPU_BIAS_WIN_SAMPLES);
        if (s_bias_win_cnt < MPU_BIAS_WIN_SAMPLES) s_bias_win_cnt++;

        /* 窗口攒满 (≈1 秒) 才做判断, 否则样本太少平均值不可信 */
        if (s_bias_win_cnt >= MPU_BIAS_WIN_SAMPLES) {
            float avg = s_bias_win_sum / (float)s_bias_win_cnt;

            int64_t now_ms = esp_timer_get_time() / 1000;
            if (avg < MPU_BIAS_STATIC_THR &&
                (now_ms - s_bias_last_adj_ms) >= MPU_BIAS_ADJ_INTERVAL_MS) {
                /* 原始读数 (°/s) —— 零偏就是往它身上靠 */
                float raw_x = (float)gyro_x / gyro_scale;
                float raw_y = (float)gyro_y / gyro_scale;
                float raw_z = (float)gyro_z / gyro_scale;

                mpu9250_axis_t old = gyro_offset;
                gyro_offset.x += MPU_BIAS_ADJ_ALPHA * (raw_x - gyro_offset.x);
                gyro_offset.y += MPU_BIAS_ADJ_ALPHA * (raw_y - gyro_offset.y);
                gyro_offset.z += MPU_BIAS_ADJ_ALPHA * (raw_z - gyro_offset.z);

                s_bias_last_adj_ms = now_ms;

                /* 日志限流: 每 30 秒最多一条 (调整本身就是 30 秒一次,
                 * 这里再兜一层, 防止以后把间隔改短了刷屏)
                 * ⚠ 同样只填缓冲, 打印放到解锁之后 (见函数末尾) */
                if ((now_ms - s_bias_last_log_ms) >= MPU_BIAS_ADJ_INTERVAL_MS) {
                    s_bias_last_log_ms = now_ms;
                    snprintf(bias_log_buf, sizeof(bias_log_buf),
                             "零偏慢漂跟随: 静止(平均|角速度| %.2f °/s), "
                             "零偏 (%.3f,%.3f,%.3f) → (%.3f,%.3f,%.3f) °/s",
                             (double)avg,
                             (double)old.x, (double)old.y, (double)old.z,
                             (double)gyro_offset.x, (double)gyro_offset.y,
                             (double)gyro_offset.z);
                    bias_log_pending = true;
                }
            }
        }
    }

    /* ⚠ 磁力计读**不在这里**
     * 磁力计只是"观察用", 不参与姿态解算 → 改到 UI 任务(50ms)里读,
     * 见 mpu9250_update_mag()。 */

    mpu9250_unlock();

    /* 解锁之后再打印 (UART 是慢速设备, 不能占着 I2C 锁写) */
    if (bias_log_pending) {
        bias_log_pending = false;
        ESP_LOGI(TAG, "%s", bias_log_buf);
    }

    return ESP_OK;
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
 * @brief 互补滤波解算欧拉角 (内部函数, 不碰 I2C)
 * @param data 同一时刻读到的 accel + gyro (调用方必须保证是同一次采样)
 * @param out  [出参] 欧拉角
 * @note  dt 由本函数自己测 (esp_timer), 不再假设固定 10ms
 */
static void mpu9250_fuse_euler(const mpu9250_data_t *data, euler_angle_t *out)
{
    /* ---------- 本机 MPU 轴向定义 (真机实测标定) ----------
     * MPU 平装在车体上, 平衡位时板面竖直。
     * 实测 (平衡位): ax≈+0.03 ay≈-0.10 az≈+1.03 (g), 重力主要在 Z 轴。
     * 车体坐标系与 MPU 坐标系对应:
     *     车的上方向 = MPU 的 Z 轴
     *     车体前后轴 = MPU 的 Y 轴 (车头方向 = MPU 的 -Y)
     *     车体左右轴 = MPU 的 X 轴
     *     俯仰旋转轴 = MPU 的 X 轴
     */
    float accel_pitch = atan2f(data->accel.z, -data->accel.y) * 180.0f / M_PI
                       - PITCH_ZERO_REF;   /* 前倾为负 */
    /* roll 是左右倾 (绕车体前后轴 = MPU 的 Z 轴), 本车不控制, 保留一个
     * 粗略值仅供观察: 左右倾时重力在 X↔Y 之间转移 */
    float accel_roll  = atan2f(data->accel.x, -data->accel.y) * 180.0f / M_PI;

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

static float s_cal_samples[MPU_GYRO_CAL_MAX][3];

/**
 * @brief 对一个小数组做升序排序 (插入排序)
 * @param a   待排序数组
 * @param n   元素个数 (很小, n ≤ 256)
 * @note  只用来算中位数, n 很小, 插入排序足够快
 */
static void mpu9250_sort_small(float *a, int n)
{
    for (int i = 1; i < n; i++) {
        float key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
}

/**
 * @brief 校准陀螺仪零点的实际实现 (见 mpu9250_calibrate_gyro 的说明)
 * @param samples 采样次数 (每 10ms 一个样本); 超过 256 按 256 处理
 * @return ESP_OK 成功; ESP_ERR_INVALID_STATE 采样失败或整段都没找到稳的一段
 * @note  单独拆成一个内部函数: 外层 mpu9250_calibrate_gyro() 负责
 *        "标定期间关掉零偏慢漂跟随" 的置位/复位, 拆开才好保证
 *        **所有**返回路径都复位了标志。
 */
static esp_err_t mpu9250_calibrate_gyro_impl(uint16_t samples)
{
    /* 采样数上限 256 (静态数组就那么大), 超出按 256 处理 */
    uint16_t want = (samples > MPU_GYRO_CAL_MAX) ? MPU_GYRO_CAL_MAX : samples;
    uint16_t got  = 0;

    /* ---------- 1) 采样 (原始读数, 不扣零偏) ---------- */
    for (uint16_t i = 0; i < want; i++) {
        mpu9250_axis_t g;
        esp_err_t ret = mpu9250_read_raw_gyro(&g);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "陀螺仪校准失败: 采样失败 (%u/%u 个样本), %s",
                     (unsigned)got, (unsigned)want, esp_err_to_name(ret));
            return ret;
        }
        s_cal_samples[got][0] = g.x;
        s_cal_samples[got][1] = g.y;
        s_cal_samples[got][2] = g.z;
        got++;

        vTaskDelay(pdMS_TO_TICKS(10));   /* 10ms 一个样本 */
    }

    if (got < 10) {
        ESP_LOGE(TAG, "陀螺仪校准失败: 有效样本太少 (%u 个)", (unsigned)got);
        return ESP_ERR_INVALID_STATE;
    }

    /* ---------- 2) 整段采样的统计 (日志用, 说明"整段有多不稳") ---------- */
    float g_sum[3]  = {0.0f, 0.0f, 0.0f};
    float g_sum2[3] = {0.0f, 0.0f, 0.0f};
    float g_mn[3]   = {1e9f, 1e9f, 1e9f};
    float g_mx[3]   = {-1e9f, -1e9f, -1e9f};
    for (uint16_t i = 0; i < got; i++) {
        for (int k = 0; k < 3; k++) {
            float v = s_cal_samples[i][k];
            g_sum[k]  += v;
            g_sum2[k] += v * v;
            if (v < g_mn[k]) g_mn[k] = v;
            if (v > g_mx[k]) g_mx[k] = v;
        }
    }
    float g_mean[3], g_sigma[3], g_spread[3];
    for (int k = 0; k < 3; k++) {
        g_mean[k]   = g_sum[k] / (float)got;
        float var   = g_sum2[k] / (float)got - g_mean[k] * g_mean[k];
        g_sigma[k]  = (var > 0.0f) ? sqrtf(var) : 0.0f;
        g_spread[k] = g_mx[k] - g_mn[k];
    }
    ESP_LOGI(TAG, "陀螺零偏采样: %u 个样本, 均值(%.3f, %.3f, %.3f) °/s, "
                  "标准差(%.3f, %.3f, %.3f), 极差(%.3f, %.3f, %.3f) °/s",
             (unsigned)got,
             (double)g_mean[0], (double)g_mean[1], (double)g_mean[2],
             (double)g_sigma[0], (double)g_sigma[1], (double)g_sigma[2],
             (double)g_spread[0], (double)g_spread[1], (double)g_spread[2]);

    /* ---------- 3) 滑动窗口, 找"最稳的一段" ----------
     * 窗口长度 MPU_GYRO_CAL_WIN (25 个样本 ≈ 0.25s)。
     * 样本不够一个窗口时, 就把整段当成唯一的窗口。 */
    int win = (int)got;
    if (win > MPU_GYRO_CAL_WIN) win = MPU_GYRO_CAL_WIN;

    int best_start = 0;
    float best_worst_sigma = 1e9f;   /* 最佳窗口里三轴 σ 的最大值 */
    float best_mean[3] = {0, 0, 0};
    float best_sigma[3] = {0, 0, 0};

    for (int start = 0; start + win <= (int)got; start++) {
        float w_sum[3]  = {0.0f, 0.0f, 0.0f};
        float w_sum2[3] = {0.0f, 0.0f, 0.0f};
        for (int i = start; i < start + win; i++) {
            for (int k = 0; k < 3; k++) {
                float v = s_cal_samples[i][k];
                w_sum[k]  += v;
                w_sum2[k] += v * v;
            }
        }
        float w_mean[3], w_sigma[3];
        float worst = 0.0f;
        for (int k = 0; k < 3; k++) {
            w_mean[k]  = w_sum[k] / (float)win;
            float var  = w_sum2[k] / (float)win - w_mean[k] * w_mean[k];
            w_sigma[k] = (var > 0.0f) ? sqrtf(var) : 0.0f;
            if (w_sigma[k] > worst) worst = w_sigma[k];
        }
        /* 取"三轴 σ 最大值最小"的窗口 = 整段里最稳的一截 */
        if (worst < best_worst_sigma) {
            best_worst_sigma = worst;
            best_start = start;
            for (int k = 0; k < 3; k++) {
                best_mean[k]  = w_mean[k];
                best_sigma[k] = w_sigma[k];
            }
        }
    }

    ESP_LOGI(TAG, "最稳窗口: 起点 #%d (共 %u 个样本), 均值(%.3f, %.3f, %.3f) °/s, "
                  "标准差(%.3f, %.3f, %.3f), 最差轴 σ=%.3f",
             best_start, (unsigned)got,
             (double)best_mean[0], (double)best_mean[1], (double)best_mean[2],
             (double)best_sigma[0], (double)best_sigma[1], (double)best_sigma[2],
             (double)best_worst_sigma);

    /* ---------- 4) 判定 ---------- */
    if (best_worst_sigma > MPU_GYRO_MOTION_THR) {
        /* 整段里最稳的一段都不够稳 → 放弃。
         * ⚠ 关键: **不写 NVS** —— 调用方会退回 NVS 里存好的旧零偏,
         *   不会把零偏标坏。 */
        ESP_LOGE(TAG, "陀螺仪校准失败: 整段采样里最稳的一段标准差仍为 %.2f°/s "
                      "(阈值 %.1f°/s) —— 手还是不够稳, 试试扶得更死一点, "
                      "或者把设备放在桌子上别碰它 (只要 0.25 秒不动就行)",
                 (double)best_worst_sigma, (double)MPU_GYRO_MOTION_THR);
        return ESP_ERR_INVALID_STATE;
    }

    /* ---------- 5) 取窗口内的中位数当零偏 ---------- */
    float median[3];
    /* ⚠ 排序用的临时数组放静态区 (1KB), 不占栈 ——
     *   标定会在 init 任务 / UI 任务里跑, 栈本来就别乱占。 */
    static float tmp[MPU_GYRO_CAL_MAX];
    for (int k = 0; k < 3; k++) {
        for (int i = 0; i < win; i++) {
            tmp[i] = s_cal_samples[best_start + i][k];
        }
        mpu9250_sort_small(tmp, win);
        /* 偶数个样本时取中间两个的平均, 奇数个直接取正中那个 */
        median[k] = (win & 1) ? tmp[win / 2]
                              : 0.5f * (tmp[win / 2 - 1] + tmp[win / 2]);
    }

    /* 零偏大到离谱 → 数据不可信, 别存 */
    if (fabsf(median[0]) > MPU_GYRO_OFFSET_MAX ||
        fabsf(median[1]) > MPU_GYRO_OFFSET_MAX ||
        fabsf(median[2]) > MPU_GYRO_OFFSET_MAX) {
        ESP_LOGE(TAG, "陀螺仪校准失败: 零偏过大 (%.3f, %.3f, %.3f) °/s",
                 (double)median[0], (double)median[1], (double)median[2]);
        return ESP_ERR_INVALID_STATE;
    }

    /* 写零偏: 持锁, 免得跟控制任务里的读/慢漂跟随撞上
     * (gyro_offset 是三个 float, 不加锁可能被读到"改了一半"的值) */
    mpu9250_lock();
    gyro_offset.x = median[0];
    gyro_offset.y = median[1];
    gyro_offset.z = median[2];

    /* 刚标完, 把慢漂跟随的统计窗口和计时器复位:
     * 标定期间攒下的窗口数据是"标定时的手抖", 拿来当静止样本会把零偏带偏。 */
    s_bias_win_sum = 0.0f;
    s_bias_win_cnt = 0;
    s_bias_win_idx = 0;
    memset(s_bias_win_buf, 0, sizeof(s_bias_win_buf));
    s_bias_last_adj_ms = esp_timer_get_time() / 1000;
    mpu9250_unlock();

    ESP_LOGI(TAG, "陀螺零偏采用值 (最稳窗口中位数): (%.3f, %.3f, %.3f) °/s",
             (double)gyro_offset.x, (double)gyro_offset.y, (double)gyro_offset.z);

    /* 存盘: 下次上电直接用这个零偏 */
    mpu9250_save_gyro_offset();

    /* 提示残余零偏在姿态角上的放大:
     *   互补滤波的稳态角误差 = **残余**零偏·dt/(1−alpha)
     *   alpha=0.997 → 1/(1−alpha) ≈ 333 → 每残余 0.1°/s = 0.33° 的姿态角误差
     *   (零偏本身已经被扣掉了, 这里说的是"扣得准不准") */
    float dt_est = 0.01f;
    float amp    = dt_est / (1.0f - alpha);
    ESP_LOGI(TAG, "零偏已补偿 (%.3f°/s): 每残余 0.1°/s 的零偏 = %.2f° 的姿态角误差",
             (double)gyro_offset.x, (double)(0.1f * amp));

    return ESP_OK;
}

/**
 * @brief 校准陀螺仪零点 (对外接口)
 * @param samples 采样次数 (每 10ms 一个样本, 上限 256)
 * @return ESP_OK 成功; ESP_ERR_INVALID_STATE 采样失败或整段都没找到稳的一段
 *
 *   采样切成 0.25 秒滑动窗口, 每个窗口算三轴 σ, 取三轴 σ 最大值最小的
 *   窗口 (整段里最稳的一小段) 参与标定。
 *   失败时不写 NVS: 调用方 (mpu9250_init) 退回 NVS 里存好的旧零偏。
 *
 * @note  标定必须用未扣零偏的原始读数 (mpu9250_read_raw_gyro)。
 * @note  标定期间关掉零偏慢漂跟随: 两者都会写 gyro_offset, 标志读写在 I2C 锁内。
 */
esp_err_t mpu9250_calibrate_gyro(uint16_t samples)
{
    mpu9250_lock();
    s_calibrating = true;      /* 让慢漂跟随让路 */
    mpu9250_unlock();

    esp_err_t ret = mpu9250_calibrate_gyro_impl(samples);

    mpu9250_lock();
    s_calibrating = false;     /* 恢复慢漂跟随 (标志复位走所有返回路径) */
    mpu9250_unlock();

    return ret;
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
    /* 复位后 MPU 芯片的寄存器全回默认值 → bypass 模式没了, 磁力计访问不到。
     * 必须把"磁力计可用"标志清掉, 否则会一直去读一个访问不到的器件。
     * 重新 init (UI 页 K3) 时会重新探测。 */
    s_mag_available = false;
    return ret;
}
