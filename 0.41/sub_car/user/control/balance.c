/**
 * @file balance.c
 * @brief 平衡控制串级控制器实现 (传感器 + 状态机 + 电机输出)
 * @author ESP32 平衡轮腿机器人
 */

#include "balance.h"
#include "esp_timer.h"
#include "param.h"
#include "balance_pid.h"
#include "speed_pid.h"
#include "steer_pid.h"
#include "position_pid.h"
#include "lqr.h"
#include "trim.h"
#include "mpu9250.h"
#include "encoder.h"
#include "odom.h"
#include "motor_driver.h"
#include "trace.h"
#include "servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "onenet_client.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include "freertos/task.h"

static const char *TAG = "BALANCE";

/* ========== 控制回路跟踪 (调试用) ==========
 * 采集 (trace_push) 只写内存环形缓冲, 打印在 trace 组件的独立任务里做 (core 0)。
 * 采样周期 10ms (100Hz); 串口波特率需 230400 (100 行/s × 约 125 字节)。
 *
 * 行格式 (固件 / 采集脚本 / 分析脚本 三方统一):
 *   TRACE t=1.234 pitch=-0.50 gyro=-12.3 out=8.0 tgt=0.20 spd=2.0
 *         yaw=0.0 pos=1.2 pos_out=0.0 kd=0.40 dt=10.0 st=2
 * CSV 表头: t_ms,pitch,gyro,out,tgt,spd,yaw,pos,pos_out,kd,dt,st */

/* 跟踪采样周期 (ms): 100Hz */
#define BAL_TRACE_PERIOD_MS 10

/* 站立控制器选择: 1 = LQR 状态反馈 (user/pid/lqr.c, 增益固定在代码里);
 * 0 = 原串级 PID (增益读参数表)。LQR 模式下 bal_kp/bal_ki/bal_kd/pos_* 滑块不生效。 */
#define BAL_USE_LQR 1

/* ================= 模块内部状态 ================= */

static bal_state_t s_state  = BAL_STATE_IDLE;
static bool        s_enabled = false;

/* 最近一周期数据快照
 * s_data     = 控制任务内部工作区, 只有控制任务写
 * s_data_pub = 对外发布的完整快照, 在临界区内一次性拷贝, 供 UI / 网页读
 * ⚠ 必须这样: bal_data_t 有多个 float, 不加锁会读到撕裂值 */
static bal_data_t s_data;
static bal_data_t s_data_pub;

static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

/* MPU9250 是否可用 (没接模块时控制任务仍要能跑, 只是没姿态数据) */
static bool s_mpu_ready = false;

/* 陀螺仪低通滤波状态 (一阶 IIR, 系数来自 param) */
static float s_gyro_lpf = 0.0f;
static bool  s_lpf_primed = false;

/* 本周期算出的配平补偿量 (°), 供 UI / 网页观察 */
static float s_trim_comp = 0.0f;

/* 已连续满足门控条件的累计时长 (ms); 任何条件不满足立即清零 */
static float s_arm_hold_ms = 0.0f;

/* ========== 速度环 / 差速环 的目标与测量 ==========
 * 全部用"**前进为正**"的轮速 (rpm):
 *   s_tgt_avg   = 两轮平均目标轮速 (前进为正)
 *   s_tgt_diff  = 左轮 − 右轮 的目标差 (前进为正, 左轮更快为正)
 *   s_speed_fwd = 两轮平均实测轮速 (前进为正)
 *   s_diff_fwd  = 左轮 − 右轮 的实测差 (前进为正)
 * 目标与实测之间用斜坡连接 (见 BAL_SPD_RAMP_*), 免得目标跳变导致猛冲。 */
static float s_tgt_avg      = 0.0f;   /* 斜坡后的实际目标 */
static float s_tgt_diff     = 0.0f;   /* 斜坡后的实际目标差 */

/* 位置保持环: 锚点位置 → 目标倾角, 与速度环按权重混合
 * s_pos_target_cm = 锚点位置 (cm, 前进为正): 斜坡后目标轮速非 0 时跟随当前位置,
 *                   目标退到 0 后冻结, 车被推离锚点时位置环把它拉回 */
static float s_pos_target_cm = 0.0f;

/* ========== 轮速统计 (300ms 滑动窗口) ==========
 * 换算公式: rpm = 脉冲数 × 60 / (窗口秒数 × 44 × gear)
 *   44 = 编码器每转脉冲数 (装在电机轴上), gear = 减速比 (默认 30)
 * 量化台阶 (1 个脉冲对应多少 rpm) 依赖 gear: 10ms 窗口 4.55 rpm,
 *   100ms 窗口 0.45 rpm, 300ms 窗口 0.15 rpm。
 * 不走 encoder_get_rpm(): 它的快照状态与 UI 编码器页共用。 */
#define SPD_RPM_WIN_MS    300
#define SPD_RPM_WIN_CYCLES (SPD_RPM_WIN_MS / CONTROL_PERIOD_MS)
static int   s_enc_last_cnt[2] = {0, 0};  /* 上个周期末的编码器计数值 */
static int   s_enc_win_cnt    = 0;        /* 轮速窗口已过的控制周期数 */
static int   s_enc_acc_pulse[2] = {0, 0}; /* 轮速窗口内累计的脉冲数 */
static float s_rpm_raw[2]      = {0.0f, 0.0f}; /* 编码器空间的轮速 (左/右) */
static float s_speed_fwd       = 0.0f;   /* 实测平均轮速 (前进为正, rpm) */
static float s_diff_fwd        = 0.0f;   /* 实测 左−右 轮速差 (前进为正, rpm) */

/* ========== 遥控指令 (App 方向键) ==========
 * 现在只用来设目标轮速 (走速度环), 平衡环全程在线。
 * 平衡没开 (车没起立) 时才退回开环直驱, 方便单独调电机/编码器。 */
static bal_move_t s_move_dir   = BAL_MOVE_STOP;
static int8_t     s_move_level = 0;

/* ========== 起立软启动 ==========
 * 输出限幅 (%), 进入平衡状态后从 BAL_RAMP_START_PCT 慢慢放开到 100% */
static float s_out_ramp = BAL_RAMP_START_PCT;

/* ========== 配平零点 (°) ==========
 * 配平 = 记录"车摆正时的 pitch", 之后平衡环以这个角度为 0。
 * 与配平表 (腿角 → pitch) 是两回事: 表管"腿的角度带来的偏移",
 * 零点管"整体安装/摆放的基准角"。 */static float s_pitch_zero = 0.0f;
/* 修正链里扣掉配平表补偿、但**还没扣零点**的 pitch (配平采样要用) */
static float s_pitch_corr = 0.0f;

/* 配平采样状态: 控制任务连续采 CALIB_SAMPLES 个点取平均 */
#define CALIB_SAMPLES   50          /* 50 个点 × 10ms = 0.5s */
static int   s_calib_count  = 0;
static float s_calib_sum    = 0.0f;
static bool  s_calib_active = false;

/* 配平零点的 NVS 存放位置 (namespace "bal", key "pz") */
#define BAL_NVS_NS        "bal"
#define BAL_NVS_KEY_ZERO  "pz"

/* ========== 起立时的自动配平零点微调 (°) ==========
 * 起立时如果车相对"上次标定的姿态"几乎没变 (|pitch| 在这个范围内),
 * 就重新采一次零点并落盘, 顺带修掉加速度计零偏 + 陀螺零偏随温度的漂移。
 * 容差就是"允许修正的最大角度", 故意取小 (1°):
 * 零点定的是**平衡点**, 差 1° 以上就意味着车会带着一个倾角站立。
 * 真正的重新定义走 App 的「配平」按钮 (手动配平永远覆盖)。
 * 偏差超过 1° 时不动零点, 只在日志里提示用户按「配平」。 */
#define BAL_AUTO_ZERO_TOL 1.0f

/* ================= 内部辅助 ================= */

/**
 * @brief 从 NVS 读配平零点
 * @note  存成整数 (角度×10), 避免 float 存 NVS 的对齐问题
 */
static void balance_load_zero(void)
{
    nvs_handle_t h;
    if (nvs_open(BAL_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        s_pitch_zero = 0.0f;
        return;
    }
    int16_t v = 0;
    esp_err_t ret = nvs_get_i16(h, BAL_NVS_KEY_ZERO, &v);
    nvs_close(h);

    if (ret != ESP_OK) {
        s_pitch_zero = 0.0f;
        return;
    }
    s_pitch_zero = (float)v / 10.0f;
    ESP_LOGI(TAG, "已载入配平零点: %.1f°", (double)s_pitch_zero);
}

/**
 * @brief 把配平零点写进 NVS
 */
static void balance_save_zero(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(BAL_NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配平零点保存失败: NVS 打开失败 (%s)", esp_err_to_name(ret));
        return;
    }
    int16_t v = (int16_t)lrintf(s_pitch_zero * 10.0f);
    ret = nvs_set_i16(h, BAL_NVS_KEY_ZERO, v);
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配平零点保存失败: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 从 param 表读出所有控制参数, 配置各控制环
 * @note  各环自己的增益/限幅常量都在 user/pid/ 各自的模块里, 这里只转调。
 */
static void balance_load_params(void)
{
    balance_pid_load_params();   /* 平衡环 */
    speed_pid_load_params();     /* 速度环 */
    steer_pid_load_params();     /* 转向环 */
    position_pid_load_params();  /* 位置保持环 */
    lqr_set_pos_gains(param_get_float(PARAM_LQR_KX),
                      param_get_float(PARAM_LQR_KV),
                      param_get_float(PARAM_LQR_AMAX));  /* LQR 位置项 */
}

/**
 * @brief 清空各控制环的积分和微分历史
 */
static void balance_reset_pid(void)
{
    balance_pid_reset();
    speed_pid_reset();
    steer_pid_reset();
    position_pid_reset();
    s_lpf_primed = false;
    /* 目标轮速也归零: 摔倒 / 停机时不能还挂着"往前走"的目标 */
    s_tgt_avg  = 0.0f;
    s_tgt_diff = 0.0f;
}

/**
 * @brief 读配平表, 算本周期要减掉的"腿弯曲带来的俯仰角" (°)
 * @param phi      当前舵机角 (°)
 * @param phi_home 舵机中心角 (°) —— 补偿在这一档为 0
 * @return 补偿量 (°)
 * @note  表里没数据 (点数 < 2) 时返回 0。
 *   两路舵机镜像安装, 逻辑角始终相同, 一次查询即可, 不分左右。
 */
static float balance_trim_compensation(float phi, float phi_home)
{
    s_trim_comp = trim_pitch(phi, phi_home);
    return s_trim_comp;
}

/**
 * @brief 读传感器, 填到 s_data 的输入段
 * @note  MPU 没接时姿态/角速度填 0, 不阻塞控制任务
 */
static void balance_read_sensors(void)
{
    /* --- MPU9250: 姿态角 + 角速度, 一次 I2C 读同时拿到 ---
     * ⚠ 不能分两次调 get_euler_angle() 和 read_gyro(): 那是两次独立的
     *   I2C 事务, 数据不同时刻。平衡环的 D 项(角速度阻尼)依赖姿态角与
     *   角速度的严格对应, 错位会让阻尼失准甚至激起振荡。 */
    if (s_mpu_ready) {
        euler_angle_t euler = {0};
        mpu9250_axis_t gyro = {0};

        if (mpu9250_get_pose(&euler, &gyro) == ESP_OK) {
            float phi = (float)servo_get_angle(SERVO_1);
            float comp = balance_trim_compensation(phi,
                                                   param_get_float(PARAM_SERVO_CENTER));

            s_data.pitch_raw = euler.pitch;
            s_pitch_corr     = euler.pitch - comp;   /* 配平采样用 (未扣零点) */
            s_data.pitch     = s_pitch_corr - s_pitch_zero;
            s_data.trim_comp = comp;

            /* 俯仰方向角速度: 取 MPU 的 X 轴 (车体俯仰的旋转轴),
             * 乘 GYRO_PITCH_SIGN 修正装配方向。详见 mpu9250.c 的
             * mpu9250_fuse_euler 注释, 以及 balance.h 顶部的方向常量说明。
             * ⚠ 本工程符号: 前倾 (车头往下) 时 pitch 与角速度**都是负数**,
             *   实测前倾时 gyro 与 pitch 同号, 所以 GYRO_PITCH_SIGN 取 +1。 */
            float rate = gyro.x * GYRO_PITCH_SIGN;

            /* 一阶低通滤高频噪声。系数越小越平滑但延迟越大,
             * 延迟大了阻尼会变弱 —— 这是个权衡, gyro_lpf 滑块就是调它 */
            float a = param_get_float(PARAM_GYRO_LPF);
            if (!s_lpf_primed) {
                s_gyro_lpf   = rate;
                s_lpf_primed = true;
            } else {
                s_gyro_lpf += a * (rate - s_gyro_lpf);
            }
            s_data.gyro = s_gyro_lpf;
        }
    } else {
        s_data.pitch     = 0.0f;
        s_data.gyro      = 0.0f;
        s_data.pitch_raw = 0.0f;
        s_data.trim_comp = 0.0f;
    }

    /* --- 编码器: 每周期取增量 (里程计) + 300ms 滑动窗口统计轮速 ---
     * 台阶大小依赖 gear 参数 (见上面那段注释)。
     * 自己记快照, 不走 encoder_get_rpm(): 它与 UI 编码器调试页共用同一份状态。
     * 计数每周期 (10ms) 取一次: 里程计速度反馈 (位置保持环的阻尼项) 需要它。 */
    float gear = param_get_float(PARAM_ENCODER_GEAR);
    if (gear < 0.01f) gear = 0.01f;      /* 防除零 */

    {
        int c1 = encoder_get_count(ENCODER_1);
        int c2 = encoder_get_count(ENCODER_2);
        /* 用 int16 取模差值: PCNT 计数到 ±32767 会回绕,
         * 直接按 int 相减会得到 65535 的假增量 (假的巨大转速) */
        int d1 = (int16_t)(c1 - s_enc_last_cnt[0]);
        int d2 = (int16_t)(c2 - s_enc_last_cnt[1]);
        s_enc_last_cnt[0] = c1;
        s_enc_last_cnt[1] = c2;

        odom_update((int)(BAL_ENC_FWD_SIGN * d1),
                    (int)(BAL_ENC_FWD_SIGN * d2));

        /* 累加脉冲数, 攒够 300ms 再换算轮速 (给速度环/UI 用) */
        s_enc_acc_pulse[0] += d1;
        s_enc_acc_pulse[1] += d2;
    }

    s_enc_win_cnt++;
    if (s_enc_win_cnt >= SPD_RPM_WIN_CYCLES) {
        s_enc_win_cnt = 0;

        {
            float win_sec = (float)(SPD_RPM_WIN_CYCLES * CONTROL_PERIOD_MS) / 1000.0f;
            /* rpm = 脉冲数 / 每转脉冲数 / 窗口秒数 × 60 / 减速比
             * ⚠ 这里的 gear 来自参数表 (默认 30): 编码器装在电机轴上,
             *   轮子转一圈对应 44×30 = 1320 个脉冲。
             *   量化台阶 = 60/(win_sec × 44 × gear) = 0.15 rpm @300ms/gear=30
             *   —— 改 gear 必须重算 (见文件头的注释)。 */
            float k = 60.0f / (win_sec * ENCODER_PULSES_PER_REV * gear);
            s_rpm_raw[0] = (float)s_enc_acc_pulse[0] * k;
            s_rpm_raw[1] = (float)s_enc_acc_pulse[1] * k;
            s_enc_acc_pulse[0] = 0;
            s_enc_acc_pulse[1] = 0;
        }
    }

    /* 换算成"前进为正": 编码器正方向 = 车尾方向, 要取反 (见 balance.h) */
    float fwd_l = BAL_ENC_FWD_SIGN * s_rpm_raw[0];
    float fwd_r = BAL_ENC_FWD_SIGN * s_rpm_raw[1];
    s_speed_fwd = (fwd_l + fwd_r) * 0.5f;    /* 两轮平均 */
    s_diff_fwd  = fwd_l - fwd_r;             /* 左 − 右 */

    s_data.speed = s_speed_fwd;
}

/**
 * @brief 摔倒检测
 * @param pitch 修正后姿态角 (°)
 * @return true=判定摔倒
 * @note  超过 param 的 ang_limit 就认为车已经倒了, 立刻停电机。
 *        宁可误停 (车还立着但先不跑) 也不能不停 (车倒了还在输出把电机烧了)
 */
static bool balance_check_fallen(float pitch)
{
    float limit = param_get_float(PARAM_ANGLE_LIMIT);
    return (pitch > limit) || (pitch < -limit);
}

/* ================= 启动门控 (arm gate) =================
 * 门控要求**连续满足** 3 个条件一段时间才放行:
 *     1) 倾角在 arm_pitch 以内
 *     2) 角速度在 arm_rate 以内
 *     3) 持续 arm_time 毫秒      (默认 500ms)
 * 期间电机保持刹车, 传感器和姿态融合照常跑。
 */

/**
 * @brief 重置门控计时 (enable / 摔倒 / 关闭控制时调用)
 */
static void balance_arm_reset(void)
{
    s_arm_hold_ms = 0.0f;
}

/**
 * @brief 开机以来的秒数
 * @return 秒
 * @note  物模型里 armed / fallen 事件的 time 参数单位是"秒" (不是毫秒),
 *        平台会按上限校验, 发毫秒会被拒收。
 */
static int64_t balance_uptime_sec(void)
{
    return (int64_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
}

/**
 * @brief 判断当前姿态是否**瞬时**满足门控条件
 * @return true=满足
 * @note  与"持续了多久"无关, 那部分由 s_arm_hold_ms 累积
 */static bool balance_arm_conditions_ok(void)
{
    float pitch_lim = param_get_float(PARAM_ARM_PITCH);
    float rate_lim  = param_get_float(PARAM_ARM_RATE);

    /* 用 !(x <= lim) 而不是 (x > lim): NaN 的比较永远返回 false,
     * 不能让"数据坏了"被当成"条件通过" */
    if (!(fabsf(s_data.pitch) <= pitch_lim)) return false;
    if (!(fabsf(s_data.gyro)  <= rate_lim))  return false;
    return true;
}

/**
 * @brief 状态机
 * @param dt 控制周期 (秒)
 */
static void balance_state_machine(float dt)
{
    bool fallen = balance_check_fallen(s_data.pitch);

    switch (s_state) {
        case BAL_STATE_IDLE:
            break;   /* 未启用, 什么都不做 */

        case BAL_STATE_STANDBY:
            if (fallen) {
                /* 已歪到摔的阈值, 门控没意义, 只等人扶正 */
                balance_arm_reset();
            } else if (balance_arm_conditions_ok()) {
                float need = param_get_float(PARAM_ARM_TIME);
                if (need <= 0.0f) {
                    s_state = BAL_STATE_BALANCING;     /* 配置为 0 = 不等待 */
                        ESP_LOGI(TAG, "姿态正常 (门控已跳过), 进入平衡状态");
                        if (mqtt_client_is_connected()) {
                            char params[64];
                            int n = snprintf(params, sizeof(params),
                                "{\"time\":{\"value\":%lld}}", (long long)balance_uptime_sec());
                            if (n > 0) mqtt_client_post_event("armed", params);
                        }
                } else {
                    s_arm_hold_ms += dt * 1000.0f;
                    if (s_arm_hold_ms >= need) {
                        s_state = BAL_STATE_BALANCING;
                        ESP_LOGI(TAG, "连续稳定 %.0fms, 进入平衡状态",
                                 (double)s_arm_hold_ms);
                        if (mqtt_client_is_connected()) {
                            char params[64];
                            int n = snprintf(params, sizeof(params),
                                "{\"time\":{\"value\":%lld}}", (long long)balance_uptime_sec());
                            if (n > 0) mqtt_client_post_event("armed", params);
                        }
                    }
                }
            } else {
                /* 任一条件不满足, 计时清零重来 */
                balance_arm_reset();
            }
            break;

        case BAL_STATE_BALANCING:
            if (fallen) {
                s_state = BAL_STATE_FALLEN;
                ESP_LOGW(TAG, "检测到摔倒 (pitch=%.1f°), 停电机", (double)s_data.pitch);
                /* 这里不能调 balance_emergency_stop(): 它会把 s_enabled 清掉,
                 * FALLEN→STANDBY→BALANCING 的自动恢复路径就走不到了。
                 * 摔倒只需要刹车 + 清积分。 */
                balance_reset_pid();
                motor_brake_all();
                s_out_ramp = BAL_RAMP_START_PCT;   /* 下次起立重新从缓启动 */
                if (mqtt_client_is_connected()) {
                    char params[128];
                    /* 物模型里 fallen 的 pitch 步长是 1 (整数), time 单位是秒,
                     * 所以 pitch 取整、time 用开机秒数, 否则平台拒收整个事件。 */
                    int n = snprintf(params, sizeof(params),
                        "{\"pitch\":{\"value\":%ld},\"time\":{\"value\":%lld}}",
                        (long)lrintf(s_data.pitch),
                        (long long)balance_uptime_sec());
                    if (n > 0) mqtt_client_post_event("fallen", params);
                }
            }
            break;

        case BAL_STATE_FALLEN:
            /* 等人把车扶正才重新进待机, 且门控重新计时, 不能一扶正就冲 */
            if (!fallen) {
                s_state = BAL_STATE_STANDBY;
                balance_arm_reset();
                ESP_LOGI(TAG, "车已扶正, 等待连续稳定后重新启动");
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 把当前值朝目标值挪一步 (加速慢、减速快)
 * @param cur  当前值
 * @param tgt  目标值
 * @param up   往正方向走的一步上限
 * @param down 往负方向走的一步上限
 * @return 挪动后的值
 */
static float balance_move_toward(float cur, float tgt, float up, float down)
{
    float d = tgt - cur;
    if (d > 0.0f) {
        float s = (d < up) ? d : up;
        return cur + s;
    }
    if (d < 0.0f) {
        float s = (-d < down) ? -d : down;
        return cur - s;
    }
    return cur;
}

/**
 * @brief 走路目标轮速的上限 (rpm)
 * @return 上限值
 * @note 优先用参数表里的 move_rpm_max (App 参数页可实时调),
 *       为 0 时退回代码里的兜底值 BAL_MOVE_RPM_MAX。
 *       目标轮速越大, 速度环要求的车身倾角越大,
 *       而倾角被 SPD_PID_WALK_PITCH_MAX (4°) 卡着。
 */
static float balance_move_rpm_max(void)
{
    float v = param_get_float(PARAM_MOVE_RPM_MAX);
    return (v > 0.1f) ? v : BAL_MOVE_RPM_MAX;
}

/**
 * @brief 遥控指令 → 目标轮速 (前进为正)
 * @param dir      方向
 * @param level    档位百分比 (20/40/60/80)
 * @param out_avg  [出参] 两轮平均目标轮速 (前进为正, rpm); 前进/后退用它
 * @param out_diff [出参] 左轮 − 右轮 的目标差 (前进为正, rpm); 转向用它
 *
 * @note  前进/后退: 两轮同向 → 平均 = ±rpm, 差 = 0
 *       左/右转: 两轮反向 (原地转) → 平均 = 0, 差 = ±2×rpm
 */
static void balance_move_targets(bal_move_t dir, int8_t level,
                                 float *out_avg, float *out_diff)
{
    /* ⚠ 上限走参数表 (move_rpm_max), 不是写死的宏 —— App 上能实时调 */
    float rpm  = (float)level * balance_move_rpm_max() / 100.0f;
    float avg  = 0.0f;
    float diff = 0.0f;

    switch (dir) {
        case BAL_MOVE_FWD:   avg  =  rpm; break;
        case BAL_MOVE_BACK:  avg  = -rpm; break;
        case BAL_MOVE_LEFT:  diff =  2.0f * rpm; break;
        case BAL_MOVE_RIGHT: diff = -2.0f * rpm; break;
        default: break;
    }
    *out_avg  = avg;
    *out_diff = diff;
}

/**
 * @brief 由遥控指令算出目标轮速, 并用斜坡逼近
 * @param dt 控制周期 (秒)
 * @note  目标轮速一阶斜坡: 起步时目标不会瞬间跳到 80 rpm;
 *        松手时目标慢慢退到 0, 车自然减速站稳。
 *
 * @note  指令 (s_move_dir / s_move_level) 由 MQTT 任务写, 这里只读。
 * @note  锚点位置 s_pos_target_cm: 斜坡后的平均目标轮速非 0 时跟随当前位置,
 *        目标退到 0 的那一周期起冻结。
 */
static void balance_update_target(float dt)
{
    /* 指令 → 目标轮速 (前进为正) */
    float cmd_avg = 0.0f;
    float cmd_diff = 0.0f;
    balance_move_targets(s_move_dir, s_move_level, &cmd_avg, &cmd_diff);

    /* 没在平衡状态时目标一律归零 (门控没过 / 摔倒 / 没起立) */
    if (s_state != BAL_STATE_BALANCING) {
        cmd_avg = 0.0f;
        cmd_diff = 0.0f;
    }

    float step_up   = BAL_SPD_RAMP_UP   * dt;
    float step_down = BAL_SPD_RAMP_DOWN * dt;
    s_tgt_avg  = balance_move_toward(s_tgt_avg,  cmd_avg,  step_up, step_down);
    s_tgt_diff = balance_move_toward(s_tgt_diff, cmd_diff, step_up, step_down);

    /* --- 锚点位置: 非平衡态或目标轮速非 0 时跟随当前位置, 否则冻结 --- */
    if (s_state != BAL_STATE_BALANCING || s_tgt_avg != 0.0f) {
        s_pos_target_cm = odom_get_position_cm();
    }
}

/**
 * @brief 位置保持环: 锚点误差 + 里程计速度 → 目标倾角
 * @param dt 控制周期 (秒)
 * @return 目标倾角 (°, 前倾为正); 非平衡态返回 0
 */
static float balance_update_position(float dt)
{
    float cur = odom_get_position_cm();
    s_data.position_cm = cur;

    if (s_state != BAL_STATE_BALANCING) {
        s_data.pos_out = 0.0f;
        position_pid_reset();
        lqr_pos_integral_reset();
        return 0.0f;
    }

#if BAL_USE_LQR
    {
        float err = s_pos_target_cm - cur;
        float vel = odom_get_velocity_cmps();
        float lean = lqr_pos_lean(err, vel) + lqr_pos_integral(dt, err, vel);
        s_data.pos_out = fminf(fmaxf(lean, -POS_PID_LEAN_MAX), POS_PID_LEAN_MAX);
    }
#else
    s_data.pos_out = position_pid_update(dt, s_pos_target_cm - cur,
                                         odom_get_velocity_cmps());
#endif
    return s_data.pos_out;
}

/**
 * @brief 跑一次串级控制运算并输出到电机
 * @param dt 控制周期 (秒)
 * @note  驱动方式: **双轮同向** (平衡环) + 可选的**差速** (转向环)。
 *        车往前倒 (pitch < 0) 时两个轮子都往"追着倒的方向"转。
 */
static void balance_control_step(float dt)
{
    balance_load_params();

    /* --- 起立软启动: 输出限幅慢慢放开到 100% ---
     * 刚起立时车还歪着, PID 输出会直接顶格 → 电机猛冲。
     * 这里先把限幅压在 BAL_RAMP_START_PCT, 再按 BAL_RAMP_TIME_MS 放开。 */
    if (s_state == BAL_STATE_BALANCING && s_out_ramp < 100.0f) {
        float rate = (100.0f - BAL_RAMP_START_PCT) / (BAL_RAMP_TIME_MS / 1000.0f);
        s_out_ramp += rate * dt;
        if (s_out_ramp > 100.0f) s_out_ramp = 100.0f;
    }

    /* --- 目标轮速斜坡 (App 方向键 → 目标轮速) --- */
    balance_update_target(dt);

    /* --- 外环: 速度环与位置环按斜坡后目标轮速加权混合 ---
     * w = min(1, |目标轮速| / BAL_POS_BLEND_RPM); 本工程 pitch 前倾为负,
     * 速度环输出已按此约定, 位置环输出前倾为正, 故取负号。
     * w=0 时位置环独占, 速度环清零 (不留积分)。 */
    float pos_lean = balance_update_position(dt);
    float w = fabsf(s_tgt_avg) / BAL_POS_BLEND_RPM;
    if (w > 1.0f) w = 1.0f;

    float spd_pitch = 0.0f;
    if (w > 0.0f) {
        spd_pitch = speed_pid_update(dt, s_speed_fwd, s_tgt_avg);
    } else {
        speed_pid_reset();
    }
    float target_pitch = w * spd_pitch - (1.0f - w) * pos_lean;

    /* 安全阀: 倾角设定点上限 = max(走路上限, 位置环上限), 防止外环把车放倒 */
    const float pitch_cap = fmaxf(SPD_PID_WALK_PITCH_MAX, POS_PID_LEAN_MAX);
    if (target_pitch >  pitch_cap) target_pitch =  pitch_cap;
    if (target_pitch < -pitch_cap) target_pitch = -pitch_cap;

    s_data.target_pitch = target_pitch;

    /* --- 差速环 (转向): 左−右 轮速差 → 两轮输出的差 ---
     * 转向时两轮反向转 (原地转), 平均轮速保持 0, 平衡环照常扶住车身。 */
    float yaw = steer_pid_update(dt, s_diff_fwd, s_tgt_diff);
    s_data.yaw_out = yaw;

    /* --- 内环: 平衡环 → 电机输出 ---
     * 误差方向、D 项用角速度反馈 (阻尼) 的细节都在
     * user/pid/balance_pid.c 里。 */
#if BAL_USE_LQR
    s_data.bal_out = lqr_balance_out(s_data.pitch - s_data.target_pitch, s_data.gyro);
#else
    s_data.bal_out = balance_pid_update(dt, s_data.pitch,
                                        s_data.target_pitch, s_data.gyro);
#endif

    /* --- 执行: 双轮同向输出 (+ 差速) ---
     * BAL_MOTOR_SIGN 烘焙在代码里 (不在网页上可调):
     * 改电机装配方向后, 要改的是 balance.h 的宏, 然后重烧固件。
     * ⚠ 实测: 这个符号必须取 +1, 取反会导致前倾时轮子往后跑,
     *   车必然往前扑倒。详见 balance.h 的方向常量说明。 */
    float out = s_data.bal_out * BAL_MOTOR_SIGN;

    float out_max = param_get_float(PARAM_MOTOR_OUT_MAX);
    if (out >  out_max) out =  out_max;
    if (out < -out_max) out = -out_max;

    /* --- 起立软启动: 输出限幅慢慢放开 ---
     * 刚进入平衡状态时车多半还歪着 (甚至被人扶着), 误差大,
     * PID 输出会直接顶到限幅 → 电机猛地冲一下, 看着就是"剧烈抖动"。
     * 所以起立后先把输出压在 BAL_RAMP_START_PCT, 再用 BAL_RAMP_TIME_MS
     * 慢慢放到全功率。 */
    if (out >  s_out_ramp) out =  s_out_ramp;
    if (out < -s_out_ramp) out = -s_out_ramp;

    /* --- 叠加差速 (转向) ---
     * 差速为正 = 左轮更快 = 左轮输出更小 (电机输出负 = 往车头 = 前进) */
    float out_a = out;
    float out_b = out;
    if (BAL_MOTOR_A_IS_LEFT) { out_a = out - yaw; out_b = out + yaw; }
    else                     { out_a = out + yaw; out_b = out - yaw; }

    if (out_a >  out_max) out_a =  out_max;
    if (out_a < -out_max) out_a = -out_max;
    if (out_b >  out_max) out_b =  out_max;
    if (out_b < -out_max) out_b = -out_max;
    if (out_a >  s_out_ramp) out_a =  s_out_ramp;
    if (out_a < -s_out_ramp) out_a = -s_out_ramp;
    if (out_b >  s_out_ramp) out_b =  s_out_ramp;
    if (out_b < -s_out_ramp) out_b = -s_out_ramp;

    s_data.motor_a = (int8_t)out_a;
    s_data.motor_b = (int8_t)out_b;

    /* 死区补偿: 每周期写入, 网页拖滑块立刻生效。
     * 作用: 让极小的非零输出也能真正推动电机, 消除平衡点附近的
     * "慢慢爬行 + 突然突跳"。0 输出和制动不受影响。 */
    motor_set_deadband((int8_t)param_get_float(PARAM_MOTOR_DEADBAND));

    if (s_state == BAL_STATE_BALANCING) {
        motor_set_speed_both(s_data.motor_a, s_data.motor_b);
    } else {
        motor_brake_all();
    }
}

/**
 * @brief 把"方向 + 档位"换算成两轮的 PWM 值
 * @param dir   方向
 * @param level 档位百分比 (正数)
 * @param out_a [出参] 电机 A (左轮) PWM
 * @param out_b [出参] 电机 B (右轮) PWM
 * @note  前进/后退 = 两轮同向; 左转/右转 = 两轮反向 (原地转向)
 */
static void balance_move_wheels(bal_move_t dir, int8_t level,
                                int8_t *out_a, int8_t *out_b)
{
    int8_t sa = 0;
    int8_t sb = 0;

    switch (dir) {
        case BAL_MOVE_FWD:   sa =  level; sb =  level; break;
        case BAL_MOVE_BACK:  sa = -level; sb = -level; break;
        case BAL_MOVE_LEFT:
            if (BAL_MOTOR_A_IS_LEFT) { sa = -level; sb =  level; }
            else                     { sa =  level; sb = -level; }
            break;
        case BAL_MOVE_RIGHT:
            if (BAL_MOTOR_A_IS_LEFT) { sa =  level; sb = -level; }
            else                     { sa = -level; sb =  level; }
            break;
        default:
            sa = 0; sb = 0;
            break;
    }

    *out_a = sa;
    *out_b = sb;
}

/**
 * @brief 开环直驱: 直接给两轮 PWM (不走平衡环)
 * @note  **只在平衡没开 (车没起立) 时用**, 方便单独调电机/编码器
 *        (跟 UI 编码器调试页 ui_encoder.c 行为一致):
 *        - 前进/后退: 两轮同向
 *        - 左转/右转: 两轮反向 (原地转向)
 *        档位取自 motor_spd_levels[], 与 ui_encoder 的调速档位同源。
 *        平衡开着的时候方向键走速度环, 不会走到这里。
 */
static void balance_apply_direct_move(void)
{
    int8_t sa = 0;
    int8_t sb = 0;

    balance_move_wheels(s_move_dir, s_move_level, &sa, &sb);

    s_data.motor_a = sa;
    s_data.motor_b = sb;
    motor_set_speed_both(sa, sb);
}

/**
 * @brief 配平采样: 控制任务里连续采 CALIB_SAMPLES 个 pitch 取平均
 * @note  采样的是**未扣零点**的 pitch (s_pitch_corr),
 *        否则第二次配平会把零点又减一遍, 越配越偏。
 */
static void balance_calib_step(void)
{
    if (!s_calib_active) return;

    s_calib_sum += s_pitch_corr;
    s_calib_count++;

    if (s_calib_count < CALIB_SAMPLES) return;

    /* 采样完成: 取平均作为新的配平零点 */
    float avg = s_calib_sum / (float)s_calib_count;
    s_pitch_zero = avg;
    s_calib_active = false;

    /* 立刻生效 (内存里的零点), 再落盘 */
    balance_save_zero();

    ESP_LOGI(TAG, "配平完成: 记录零点 pitch=%.2f° (采样 %d 个点), 已落盘",
             (double)avg, s_calib_count);
}

/**
 * @brief 控制任务: 固定周期跑串级控制
 */
static void balance_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    /* ⚠ 用 vTaskDelayUntil 而不是 vTaskDelay
     *   vTaskDelay(10ms) 的语义是"**至少**等 10ms", 实际周期 =
     *   10ms + 本次执行耗时。
     *
     *   vTaskDelayUntil 从**上次唤醒时刻**起算, 执行耗时自动扣掉,
     *   周期严格稳定在 10ms。同时下面用实测 dt 喂给 PID,
     *   万一真被长时间抢占, 积分和导数也知道真实间隔, 不会静默算错。 */
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t prev_wake = last_wake;
    const TickType_t min_period = 2;   /* 实测 dt 的下限 (tick), 防除零 */

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        /* --- 实测本周期真实间隔 (tick → 秒) --- */
        TickType_t now = xTaskGetTickCount();
        TickType_t gap = now - prev_wake;
        prev_wake = now;
        if (gap < min_period) gap = min_period;
        float dt = (float)gap / (float)configTICK_RATE_HZ;
        /* 夹上限: 被长时间抢占 (比如网页请求/Flash 写入) 后,
         * 不该把那一大段空档当成一个巨大的积分/微分区间 */
        if (dt > 0.1f) dt = 0.1f;

        /* 1) 采样 (只写 s_data 的输入段, 不需要加锁: 只有本任务写输入) */
        balance_read_sensors();

        /* 1.5) 配平采样 (只记录 pitch, 不碰电机) */
        balance_calib_step();

        balance_state_machine(dt);

        /* 2.5) 陀螺零偏慢漂跟随的开关
         * ⚠ 只有**不在平衡状态**时才允许跟随:
         *   车站着顶的时候 (BAL_STATE_BALANCING) 读数里全是真实角速度,
         *   拿它当零偏 = 把真实运动也吸进零偏里 → D 项阻尼失准 → 越晃越歪。
         *   平时 (没起立 / 摔倒 / 未启用) 车是静止的, 这时候让零偏慢慢
         *   跟着当前读数走, 就能抵消温度漂。
         *
         * mpu9250_set_bias_tracking() 内部只在状态**变化**时动作,
         * 所以每个周期调它没有开销。 */
        mpu9250_set_bias_tracking(s_state != BAL_STATE_BALANCING);

        /* 3) 运算 + 输出 */
        if (s_enabled) {
            balance_control_step(dt);
        } else if (s_move_dir != BAL_MOVE_STOP) {
            /* 平衡没开 (车没起立): 方向键走开环直驱, 方便单独调电机/编码器 */
            balance_apply_direct_move();
            s_data.target_pitch = 0.0f;
            s_data.bal_out      = 0.0f;
            s_data.yaw_out      = 0.0f;
        } else {
            /* 未启用: 只刹车, PID 不动 (enable 时会 reset) */
            s_data.target_pitch = 0.0f;
            s_data.bal_out      = 0.0f;
            s_data.yaw_out      = 0.0f;
            s_data.motor_a      = 0;
            s_data.motor_b      = 0;
            motor_brake_all();
        }

        /* 3.5) 磁力计: 只观察, 不参与姿态解算
         * ⚠ 必须放在**电机输出写完之后**:
         *   磁力计的 I2C 事务要 0.3~1ms, 如果夹在"读传感器"和"写电机"
         *   之间, 就等于给平衡内环加了一段**可变延迟**。
         *   放在输出之后: 环路延迟不受影响; 本周期多花的时间由
         *   vTaskDelayUntil 自动吸收 (实测 dt 仍稳定 10.0ms)。 */
        mpu9250_update_mag();

        /* 4) 一次性发布整帧快照, 避免外部读到撕裂的数据 */
        s_data.state = s_state;
        s_data.dt    = dt;
        taskENTER_CRITICAL(&s_data_mux);
        s_data_pub = s_data;
        taskEXIT_CRITICAL(&s_data_mux);

        /* --- 调试跟踪: 采一条样存进环形缓冲 (不碰串口) --- */
        static TickType_t s_trace_tick = 0;
        static bool s_trace_was_balancing = false;
        TickType_t now_t = xTaskGetTickCount();
        if (now_t - s_trace_tick >= pdMS_TO_TICKS(BAL_TRACE_PERIOD_MS)) {
            s_trace_tick = now_t;

            /* 不管当前是什么状态都存, 这样"起立前"也能看到 */
            trace_sample_t ts;
            ts.t       = (float)esp_timer_get_time() / 1000000.0f;
            ts.pitch   = s_data.pitch;
            ts.gyro    = s_data.gyro;
            ts.out     = s_data.bal_out;
            ts.tgt     = s_data.target_pitch;
            ts.spd     = s_data.speed;
            ts.yaw     = s_data.yaw_out;
            ts.pos     = s_data.position_cm;
            ts.pos_err = s_pos_target_cm - s_data.position_cm;
            ts.pos_out = s_data.pos_out;
            ts.pos_vel = odom_get_velocity_cmps();
            /* kd 可以热改, 必须逐条记采样时刻的值 */
            ts.kd      = param_get_float(PARAM_PID_BAL_KD);
            ts.dt      = dt * 1000.0f;
            ts.st      = (int8_t)s_state;
            trace_push(&ts);
        }

        /* 状态从"平衡中"变成别的 → 请求把最近几秒的现场回灌到串口。
         * 触发场景: 摔了 / 关了平衡 / 退出了 —— 不用卡时间点去抓。
         * 真正的打印在 trace 组件的独立任务里做, 这里只是发个信号。 */
        if (s_state != BAL_STATE_BALANCING && s_trace_was_balancing) {
            s_trace_was_balancing = false;
            trace_request_dump();
        } else if (s_state == BAL_STATE_BALANCING) {
            s_trace_was_balancing = true;
        }
    }
}

/* ================= 对外 API ================= */

esp_err_t balance_init(void)
{
    /* 各控制环初始化: 读 param 表的增益 + 清积分。
     * 位置保持环 (user/pid/position_pid.c) 接入里程计 (user/odom/):
     * 里程计用编码器脉冲 + 轮径参数算出位移, 位置反馈因此可用。 */
    balance_pid_init();
    speed_pid_init();
    steer_pid_init();
    position_pid_init();
    odom_init();
    s_lpf_primed = false;

    trim_load();

    /* 配平零点 (跟配平表分开存, 不受 trim 表的存/取影响) */
    balance_load_zero();

    s_state      = BAL_STATE_IDLE;
    s_enabled    = false;
    s_arm_hold_ms = 0.0f;
    s_mpu_ready  = mpu9250_is_ready();
    s_move_dir   = BAL_MOVE_STOP;
    s_move_level = 0;
    s_out_ramp   = BAL_RAMP_START_PCT;

    /* 轮速统计的初始快照: 第一个周期只当基准, 不算速度。
     * (否则会拿"从 0 开始"的计数差算出一个假的巨大转速) */
    s_enc_last_cnt[0] = encoder_get_count(ENCODER_1);
    s_enc_last_cnt[1] = encoder_get_count(ENCODER_2);
    s_enc_win_cnt    = 0;
    s_enc_acc_pulse[0] = 0;
    s_enc_acc_pulse[1] = 0;
    s_rpm_raw[0]      = 0.0f;
    s_rpm_raw[1]      = 0.0f;
    s_speed_fwd       = 0.0f;
    s_diff_fwd        = 0.0f;
    s_tgt_avg         = 0.0f;
    s_tgt_diff        = 0.0f;

    if (!s_mpu_ready) {
        ESP_LOGW(TAG, "MPU9250 不可用, 平衡控制无法工作 (请先接好 IMU)");
    }

    /* 启动日志: 让人一眼看出当前是什么配置 */
    const trim_t *t = trim_get();
    if (trim_is_usable(2)) {
        ESP_LOGI(TAG, "配平表: %u 个点, 俯仰角跨度 %.2f°, 补偿生效",
                 t->count, (double)trim_pitch_span());
    } else {
        ESP_LOGW(TAG, "没有配平表 —— 腿改变角度时车会往一个方向偏。"
                      "请在网页「配平标定」面板里标几个点");
    }

    if (xTaskGetHandle("bal") != NULL) {
        return ESP_OK;    /* 已经起过了 */
    }

    /* 跟踪组件要在控制任务启动**之前**初始化 (它有自己的输出任务) */
    trace_init();

    /* 任务栈放大: 平衡控制有大量浮点运算。
     * 优先级 7: 必须高于网络/上报任务 (OneNet 上报 5/4, MQTT 客户端 5),
     *   否则控制周期抖动 → 阻尼相位失配。
     * 绑到 core 1: IDF WiFi 任务 (优先级 23) 与 lwIP tcpip 任务 (优先级 18)
     *   留在 core 0, 避免抢占控制任务 (lwIP 绑 core 0 见
     *   sdkconfig 的 CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0)。 */
    BaseType_t ret = xTaskCreatePinnedToCore(balance_task, "bal", 8192, NULL, 7, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "控制任务创建失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "控制任务已启动 (周期 %dms, MPU9250 %s, BAL_MOTOR_SIGN=%.0f, GYRO_PITCH_SIGN=%.0f)",
             CONTROL_PERIOD_MS,
             s_mpu_ready ? "就绪" : "未就绪",
             (double)BAL_MOTOR_SIGN,
             (double)GYRO_PITCH_SIGN);
    return ESP_OK;
}

esp_err_t balance_set_enable(bool on)
{
    if (on) {
        /* 启动前必须确认车是扶正的, 否则一启动就是摔倒状态。
         * 读已发布的快照, 不要直接读 s_data (那是控制任务的工作区)。 */
        float pitch = 0.0f;
        taskENTER_CRITICAL(&s_data_mux);
        pitch = s_data_pub.pitch;
        taskEXIT_CRITICAL(&s_data_mux);

        if (s_mpu_ready && balance_check_fallen(pitch)) {
            ESP_LOGW(TAG, "当前倾角 %.1f° 超过限幅, 拒绝启动 (请先把车扶正)",
                     (double)pitch);
            s_state = BAL_STATE_IDLE;
            return ESP_ERR_INVALID_STATE;
        }
        balance_reset_pid();
        odom_reset();                 /* 位置基准重设: 当前位置 = 原点 */
        s_pos_target_cm = 0.0f;
        balance_arm_reset();          /* 门控从零开始计时 */
        s_state   = BAL_STATE_STANDBY;
        s_enabled = true;
        s_out_ramp = BAL_RAMP_START_PCT;   /* 起立软启动: 输出限幅从缓开始 */

        /* --- 自动微调配平零点 (断电保存) ---
         * 车相对上次标定的姿态几乎没变时, 重新采一次零点并落盘,
         * 修掉加速度计/陀螺零偏随温度的漂移。
         * 偏差超过 1° 说明零点已经不准了 (或者车没扶到平衡点),
         * 这时不动零点, 只提示用户按 App 的「配平」重新定义。 */
        if (fabsf(pitch) <= BAL_AUTO_ZERO_TOL) {
            ESP_LOGI(TAG, "起立: pitch=%.2f° 与上次标定一致, 自动微调配平零点",
                     (double)pitch);
            balance_calibrate();
        } else {
            ESP_LOGW(TAG, "起立: pitch=%.2f° 偏离上次标定的姿态 (>%.1f°), "
                          "不自动改零点 —— 请把车扶到平衡点后按 App 的「配平」按钮",
                     (double)pitch, (double)BAL_AUTO_ZERO_TOL);
        }

        ESP_LOGI(TAG, "平衡控制已启动, 等待连续稳定 %.0fms 后输出 "
                      "(需 |pitch|<%.0f° 且 |角速度|<%.0f°/s)",
                 (double)param_get_float(PARAM_ARM_TIME),
                 (double)param_get_float(PARAM_ARM_PITCH),
                 (double)param_get_float(PARAM_ARM_RATE));
    } else {
        s_enabled = false;
        s_state   = BAL_STATE_IDLE;
        balance_arm_reset();
        balance_clear_move();        /* 直驱也要停 */
        motor_brake_all();
        servo_set_both((uint8_t)param_get_float(PARAM_SERVO_CENTER));
        ESP_LOGI(TAG, "平衡控制已停止, 电机刹车, 舵机回中");
    }
    return ESP_OK;
}

bool balance_is_enabled(void)
{
    return s_enabled;
}

esp_err_t balance_emergency_stop(void)
{
    s_enabled = false;
    s_state   = BAL_STATE_IDLE;
    balance_arm_reset();
    balance_reset_pid();       /* 清积分, 避免下次启动猛冲 */
    balance_clear_move();      /* 直驱指令也要清掉 */
    motor_brake_all();
    return ESP_OK;
}

esp_err_t balance_get_data(bal_data_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    taskENTER_CRITICAL(&s_data_mux);
    *out = s_data_pub;
    taskEXIT_CRITICAL(&s_data_mux);
    return ESP_OK;
}

bal_state_t balance_get_state(void)
{
    return s_state;
}
esp_err_t balance_set_move(bal_move_t dir, int8_t level_pct)
{
    if (dir == BAL_MOVE_STOP) {
        balance_clear_move();
        return ESP_OK;
    }

    /* 档位吸附到 motor_spd_levels[] (20/40/60/80), 与 UI 编码器页同源 */
    int8_t lvl = motor_spd_level_snap(level_pct);
    if (lvl <= 0) {
        balance_clear_move();
        return ESP_OK;
    }

    /* 指令变了才清速度环/差速环的积分:
     * 积分是"上一个指令"攒的, 换了方向还留着 → 一按就猛冲一下。
     * (App 每秒会补发一次同样的指令, 指令没变时不能清) */    if (s_move_dir != dir || s_move_level != lvl) {
        speed_pid_reset();
        steer_pid_reset();
    }

    s_move_dir   = dir;
    s_move_level = lvl;

    /* 目标轮速 (前进为正, rpm) —— 真正的目标由控制任务按斜坡逼近 */
    float tgt_avg = 0.0f;
    float tgt_diff = 0.0f;
    balance_move_targets(dir, lvl, &tgt_avg, &tgt_diff);

    ESP_LOGI(TAG, "遥控指令: 方向=%d, 档位=%d%% → 目标轮速 平均 %.1f rpm / 差 %.1f rpm "
                  "(走速度环, 平衡环不撒手)",
             (int)dir, lvl, (double)tgt_avg, (double)tgt_diff);
    return ESP_OK;
}

void balance_clear_move(void)
{
    if (s_move_dir == BAL_MOVE_STOP) return;

    s_move_dir   = BAL_MOVE_STOP;
    s_move_level = 0;

    /* 目标轮速由控制任务按斜坡退到 0 (车自然减速站稳),
     * 这里只清速度环/差速环的积分, 免得积分残留让车继续歪着走 */
    speed_pid_reset();
    steer_pid_reset();

    /* 平衡没开就直接刹车; 开了则交给平衡环接管 */
    if (!s_enabled) motor_brake_all();

    ESP_LOGI(TAG, "遥控指令结束, 目标轮速归零, 平衡环接管");
}

esp_err_t balance_calibrate(void)
{
    if (!s_mpu_ready) {
        ESP_LOGE(TAG, "配平失败: MPU9250 不可用");
        return ESP_ERR_INVALID_STATE;
    }

    /* 只启动采样, 由控制任务连续采 0.5s 取平均。
     * 这里**不碰电机、不动平衡状态** —— 配平只是记录 pitch。 */
    s_calib_count  = 0;
    s_calib_sum    = 0.0f;
    s_calib_active = true;

    ESP_LOGI(TAG, "配平开始: 采样 %d 个 pitch 点取平均 (不驱动电机)", CALIB_SAMPLES);
    return ESP_OK;
}

float balance_get_pitch_zero(void)
{
    return s_pitch_zero;
}

esp_err_t balance_record_trim_point(void)
{
    if (!s_mpu_ready) {
        ESP_LOGE(TAG, "记录配平点失败: MPU9250 不可用");
        return ESP_ERR_INVALID_STATE;
    }

    /* 取舵机角和**原始** pitch。
     * ⚠ 必须用 pitch_raw (修正链之前的原始值):
     *   修正链 = 原始 pitch − 配平表补偿 − 配平零点,
     *   表里再存修正后的 pitch 就会跟修正链重复扣一次。
     *   读 s_data_pub (发布的快照) 而不是 s_data (控制任务的工作区),
     *   避免跟控制任务抢数据。 */
    bal_data_t d;
    if (balance_get_data(&d) != ESP_OK) {
        return ESP_FAIL;
    }

    float phi   = (float)servo_get_angle(SERVO_1);   /* 两路舵机逻辑角始终相同 */
    float pitch = d.pitch_raw;

    esp_err_t ret = trim_set_point(phi, pitch);
    if (ret != ESP_OK) {
        /* 错误码透出来: 表已满 (ESP_ERR_NO_MEM) / pitch 超范围 (ESP_ERR_INVALID_ARG) */
        ESP_LOGE(TAG, "记录配平点失败: 舵机角 %.0f°, 原始 pitch %.2f° (%s)",
                 (double)phi, (double)pitch, esp_err_to_name(ret));
        return ret;
    }

    const trim_t *t = trim_get();
    ESP_LOGI(TAG, "配平表已记录: 舵机角 %.0f° → 原始 pitch %.2f°, 表内共 %u 个点",
             (double)phi, (double)pitch, (unsigned)t->count);
    /* 顺便打印跨度: 用户可以据此判断曲线是不是有内容
     * (换腿高时俯仰角应该差好几度; 跨度接近 0 说明每次都扶成了同一个姿态) */
    ESP_LOGI(TAG, "配平表俯仰角跨度: %.2f° (参考角 servo_c = %.0f°)",
             (double)trim_pitch_span(),
             (double)param_get_float(PARAM_SERVO_CENTER));
    return ESP_OK;
}

int balance_get_trim_json(char *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return 0;

    const trim_t *t = trim_get();
    float ref = param_get_float(PARAM_SERVO_CENTER);   /* 配平表参考角 */
    float pz  = balance_get_pitch_zero();               /* 配平零点 */

    /* 当前原始 pitch (修正链之前的那个)。表里存的就是它, 而遥测上报的
     * pitch 是修正后的 (扣了配平表补偿和配平零点)。 */
    bal_data_t d;
    float rp = 0.0f;
    if (balance_get_data(&d) == ESP_OK) rp = d.pitch_raw;

    int n = snprintf(buf, len, "{\"n\":%u,\"ref\":%.1f,\"pz\":%.2f,\"rp\":%.2f,\"pts\":[",
                     (unsigned)t->count, (double)ref, (double)pz, (double)rp);
    if (n < 0 || (uint32_t)n >= len) return 0;

    for (uint8_t i = 0; i < t->count; i++) {
        int w = snprintf(buf + n, len - n, "%s[%.0f,%.2f]",
                         i ? "," : "",
                         (double)t->pts[i].servo_deg,
                         (double)t->pts[i].pitch_deg);
        if (w < 0 || (uint32_t)(n + w) >= len) return 0;
        n += w;
    }

    int w = snprintf(buf + n, len - n, "]}");
    if (w < 0 || (uint32_t)(n + w) >= len) return 0;
    return n + w;
}
