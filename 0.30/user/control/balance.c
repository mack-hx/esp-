/**
 * @file balance.c
 * @brief 平衡控制串级控制器实现
 *
 * @note  控制结构 (串级, 从外到内):
 *     速度环 (外环): 轮速误差 → 目标倾角 (°)
 *     平衡环 (内环): 姿态误差 → 电机输出 (%), 双轮同向
 *
 * @note  姿态读数的修正链 (采样阶段做, 在算误差之前):
 *
 *     修正后 pitch = 原始 pitch − 配平表补偿(φ) − 配平零点
 *                      └─ user/trim ─┘        └─ 配平动作记录的基准角 ┘
 *
 *   0.13 起不再有 ang_zero / bal_sign / gyro_sign 三个参数
 *   (详见 balance.h 顶部的方向常量)
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-07
 */

#include "balance.h"
#include "param.h"
#include "trim.h"
#include "mpu9250.h"
#include "encoder.h"
#include "motor_driver.h"
#include "servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "onenet_client.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "BALANCE";

/* ========== 控制回路跟踪 (调试用, 默认关闭) ==========
 * 打开后每 100ms 打印一次控制回路的关键量 (姿态角/角速度/输出/目标/周期),
 * 用来在真机上看"抖"到底是姿态读数在抖、还是控制器在抖、还是周期在抖。
 * 打开: 把下面的 0 改成 1 重新编译烧录。
 * ⚠ 开了之后串口会被刷屏, 别在正式使用时开着。 */
#define BAL_TRACE_ENABLE  0
#if BAL_TRACE_ENABLE
#define BAL_TRACE_PERIOD_MS 100
#endif

/* ================= 模块内部状态 ================= */

static bal_pid_t s_pid_bal;      /* 姿态误差 → 电机输出 (%) */
static bal_pid_t s_pid_spd;      /* 轮速误差 → 目标倾角 (°) */
static bal_pid_t s_pid_yaw;      /* 左−右 轮速差 → 差速输出 (%) */

static bal_state_t s_state  = BAL_STATE_IDLE;
static bool        s_enabled = false;

/* 最近一周期数据快照
 * s_data     = 控制任务内部工作区, 只有控制任务写
 * s_data_pub = 对外发布的完整快照, 在临界区内一次性拷贝, 供 UI / 网页读
 * ⚠ 必须这样: bal_data_t 有 10 个 float, 不加锁会读到"一半新一半旧"的撕裂值
 *   (典型现象: 网页显示 BALANCING 但电机输出还是上一帧的 0) */
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

/* ========== 轮速统计 (300ms 滑动窗口) ==========
 * 编码器 44 脉冲/转, 10ms 窗口的读数是量化跳变 (1 个脉冲 = 136 rpm),
 * 100ms 窗口也好不到哪去 (1 个脉冲 = 13.6 rpm) —— 速度环拿到这种读数,
 * 误差 2 rpm 就把目标倾角打到限幅, 平衡环随即饱和 → 电机前后猛顶。
 * 所以把 3 个 100ms 窗口的脉冲数**累加**后再算速度 (300ms 窗口):
 * 量化台阶降到 1 个脉冲 = 4.5 rpm, 读数平滑可用。
 *
 * ⚠ 不走 encoder_get_rpm(): 它的快照状态和 UI 编码器页共用, 会被抢。 */
#define SPD_WIN_MS       100                                  /* 基本窗口长度 (ms) */
#define SPD_ACC_WIN      3                                   /* 累加几个基本窗口 (300ms) */
#define SPD_WIN_CYCLES   (SPD_WIN_MS / CONTROL_PERIOD_MS)    /* 窗口内的控制周期数 */
static int   s_enc_last_cnt[2] = {0, 0};  /* 上个窗口结束时的编码器计数值 */
static int   s_enc_win_cnt    = 0;        /* 当前窗口已过的控制周期数 */
static int   s_enc_acc_pulse[2] = {0, 0}; /* 累加窗口内累计的脉冲数 */
static int   s_enc_acc_win    = 0;        /* 已累加了几个基本窗口 */
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
 * 零点管"整体安装/摆放的基准角"。 */
static float s_pitch_zero = 0.0f;
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
 * 就重新采一次零点并落盘:
 *   - 顺带修掉加速度计零偏 + 陀螺零偏随温度的漂移 (每次上电 pitch 都不一样)
 *   - 结果写进 NVS, 断电不丢 → 下次上电直接沿用
 * ⚠ 容差就是"允许修正的最大角度", 故意取小 (1°):
 *   零点定的是**平衡点**, 差 1° 以上就意味着车会带着一个倾角站立,
 *   慢慢往一边跑。所以这里只允许"微调"传感器漂移, 不允许重新定义平衡点 ——
 *   真正的重新定义走 App 的「配平」按钮 (手动配平永远覆盖, 不受此容差限制)。
 *   偏差超过 1° 时不动零点, 只在日志里提示用户按「配平」。 */
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
 * @brief 算积分累加值的上限 (i_max)
 * @param ki      积分系数
 * @param out_max 该环的输出上限
 * @return 累加值上限
 * @note  pid 里夹的是**累加值 i**, 而输出项是 Ki·i。
 *        所以要让积分项不超过输出上限的一半, 累加值上限就得是 (out_max/2)/Ki。
 *        ⚠ 之前直接写 out_max/2 是错的: 乘上 Ki 之后积分项最大能到
 *        Ki·out_max/2 (12×15 = 180%), 远超 30% 的输出上限 →
 *        起立时积分直接把输出顶死, 表现为"猛冲"。
 */
static float balance_i_max(float ki, float out_max)
{
    if (ki <= 0.01f) return out_max;    /* Ki≈0 时积分不参与运算 */
    return (out_max * 0.5f) / ki;
}

/**
 * @brief 从 param 表读出所有控制参数, 配置两个 PID
 * @note  **每个控制周期都调一次**: 网页/App 上拖滑块后, 下一次运算立刻用新值,
 *        不用重启。
 *        ⚠ 2026-10-26 修正: 之前这个函数只在启动时调一次, 而且用的是
 *          pid_config() (带 pid_reset(), 会清积分) —— 结果 App 上把
 *          bal_kp 拖到 0 也**完全不生效**, 用户误以为"降 Kp 没用"。
 *          现在改用 pid_set_gains() (只改系数, 保留积分/微分历史),
 *          并且每周期调用, 参数改动立刻生效。
 */
static void balance_load_params(void)
{
    float out_max    = param_get_float(PARAM_MOTOR_OUT_MAX);
    float spdout_max = param_get_float(PARAM_SPEED_OUT_MAX);

    float bal_kp = param_get_float(PARAM_PID_BAL_KP);
    float bal_ki = param_get_float(PARAM_PID_BAL_KI);
    float bal_kd = param_get_float(PARAM_PID_BAL_KD);

    float spd_kp = param_get_float(PARAM_PID_SPD_KP);
    float spd_ki = param_get_float(PARAM_PID_SPD_KI);
    float spd_kd = param_get_float(PARAM_PID_SPD_KD);

    pid_set_gains(&s_pid_bal,
                  bal_kp, bal_ki, bal_kd,
                  balance_i_max(bal_ki, out_max),   /* 积分项最大 = 输出一半 */
                  out_max);

    /* 速度环: 输出 = 目标倾角 (°)。
     * ⚠ 限幅夹到 BAL_WALK_PITCH_MAX: 走路时车最多前倾这么多度,
     *   目标轮速定太高时宁可走慢, 也不能倾到摔。
     *   (夹在 PID 内部而不是外面: 否则积分会继续往 40° 攒,
     *    误差反向时要很久才退得回来 → 车会先猛冲再回不来) */
    float walk_max = spdout_max;
    if (walk_max > BAL_WALK_PITCH_MAX) walk_max = BAL_WALK_PITCH_MAX;
    if (walk_max < 0.0f) walk_max = 0.0f;

    /* 速度环增益为 0 时用内置默认值 (防止旧版 App 把参数写回 0,
     * 结果方向键完全没反应 —— 2026-10-26 那次"恢复默认"事故的教训) */
    float spd_kp_eff = (spd_kp > 0.001f) ? spd_kp : BAL_SPD_KP_DEFAULT;
    float spd_ki_eff = (spd_ki > 0.001f) ? spd_ki : BAL_SPD_KI_DEFAULT;
    float spd_kd_eff = (spd_kd > 0.001f) ? spd_kd : 0.0f;

    pid_set_gains(&s_pid_spd,
                  spd_kp_eff, spd_ki_eff, spd_kd_eff,
                  balance_i_max(spd_ki_eff, walk_max),
                  walk_max);        /* 输出 = 目标倾角 (°) */

    /* 差速环 (转向): 输出 = 两轮的差 (%); 正 = 左轮更快 */
    pid_set_gains(&s_pid_yaw,
                  BAL_YAW_KP, BAL_YAW_KI, 0.0f,
                  balance_i_max(BAL_YAW_KI, BAL_YAW_OUT_MAX),
                  BAL_YAW_OUT_MAX);
}

/**
 * @brief 清空两个 PID 的积分和微分历史
 */
static void balance_reset_pid(void)
{
    pid_reset(&s_pid_bal);
    pid_reset(&s_pid_spd);
    pid_reset(&s_pid_yaw);
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
 * @note  表里没数据 (点数 < 2) 时返回 0, 行为与不标定时完全一致。
 *
 *   两路舵机现在是镜像的, 逻辑角始终相同, 所以一次查询就够。
 *   标定表是"一起扶"测出来的, 左右腿的装配差异已经包含在测出来的 Pitch 里,
 *   不需要再分左右。
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
            /* 姿态修正链: 原始 pitch − 配平表补偿 − 配平零点。
             *   - 配平表补偿: 腿角度带来的偏移 (trim_pitch)
             *   - 配平零点:   车"摆正"时的 pitch (配平动作记录的基准角)
             * 0.13 起 ang_zero 已删除: 配平表每个腿角都采, 零偏自然包含在测出的 Pitch 里 */
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

    /* --- 编码器: 300ms 滑动窗口统计两轮轮速 ---
     *
     * ⚠ 不用 encoder_get_rpm(..., 10ms) 的原因:
     *   编码器 44 脉冲/转, 10ms 窗口里轮子常常转不到 1 个脉冲,
     *   读数在 0 和 136 rpm 之间跳 (1 个脉冲 = 136 rpm @10ms 窗口)。
     *   速度环拿到这种读数, 误差 2 rpm 就把目标倾角打到限幅,
     *   平衡环随即饱和 → 电机前后猛顶 (起立剧烈抖动的直接来源)。
     *   100ms 窗口把量化误差降到 1/10 (1 脉冲 = 13.6 rpm), 还是偏粗,
     *   所以这里累加 3 个 100ms 窗口 (300ms), 量化台阶 = 4.5 rpm。
     *
     * ⚠ 自己记快照, 不走 encoder_get_rpm(): 它内部存的是"上次调用时刻",
     *   和 UI 编码器调试页 (150ms 一读) 是同一份状态, 两边互抢会算出
     *   错误的瞬时转速。 */
    float gear = param_get_float(PARAM_ENCODER_GEAR);
    if (gear < 0.01f) gear = 0.01f;      /* 防除零 */

    s_enc_win_cnt++;
    if (s_enc_win_cnt >= SPD_WIN_CYCLES) {
        s_enc_win_cnt = 0;

        int c1 = encoder_get_count(ENCODER_1);
        int c2 = encoder_get_count(ENCODER_2);
        /* 用 int16 取模差值: PCNT 计数到 ±32767 会回绕,
         * 直接按 int 相减会得到 65535 的假增量 (假的巨大转速) */
        int d1 = (int16_t)(c1 - s_enc_last_cnt[0]);
        int d2 = (int16_t)(c2 - s_enc_last_cnt[1]);
        s_enc_last_cnt[0] = c1;
        s_enc_last_cnt[1] = c2;

        /* 累加脉冲数, 攒够 SPD_ACC_WIN 个窗口再换算速度 */
        s_enc_acc_pulse[0] += d1;
        s_enc_acc_pulse[1] += d2;
        s_enc_acc_win++;

        if (s_enc_acc_win >= SPD_ACC_WIN) {
            float win_sec = (float)(SPD_WIN_MS * SPD_ACC_WIN) / 1000.0f;
            /* rpm = 脉冲数 / 每转脉冲数 / 窗口秒数 × 60 / 减速比 */
            float k = 60.0f / (win_sec * ENCODER_PULSES_PER_REV * gear);
            s_rpm_raw[0] = (float)s_enc_acc_pulse[0] * k;
            s_rpm_raw[1] = (float)s_enc_acc_pulse[1] * k;
            s_enc_acc_pulse[0] = 0;
            s_enc_acc_pulse[1] = 0;
            s_enc_acc_win    = 0;
        }
    }

    /* 换算成"前进为正": 编码器正方向 = 车尾方向, 要取反 (见 balance.h) */
    float fwd_l = BAL_ENC_FWD_SIGN * s_rpm_raw[0];
    float fwd_r = BAL_ENC_FWD_SIGN * s_rpm_raw[1];
    s_speed_fwd = (fwd_l + fwd_r) * 0.5f;    /* 两轮平均 */
    s_diff_fwd  = fwd_l - fwd_r;             /* 左 − 右 */

    s_data.speed     = s_speed_fwd;
    s_data.speed_raw = s_rpm_raw[0];         /* 调试用: 编码器空间的原始轮速 */
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
 *
 * @brief 这是干什么的
 *   如果流程是 "enable → 看一眼 pitch 没超限 → 立刻输出", 那:
 *     - 车被扶着还在小幅晃 → 立刻放行 → 电机接手 → 晃得更厉害
 *     - 推着车启动           → 照样放行 → 立刻冲出去
 *   两种情况都会直接摔, 而且摔的瞬间电机还在全力输出。
 *   "没超限"和"稳得住"完全是两回事。
 *
 *   门控要求**连续满足** 3 个条件一段时间才放行:
 *     1) 倾角在 arm_pitch 以内
 *     2) 角速度在 arm_rate 以内  (这条能抓住"正在被晃动")
 *     3) 持续 arm_time 毫秒      (默认 500ms)
 *   期间电机保持刹车, 传感器和姿态融合照常跑。
 *
 * @brief 为什么放在 STANDBY 里而不是新加一个状态
 *   状态枚举被 UI / 网页引用, 加新状态要改 4 个文件;
 *   而 STANDBY 的语义本来就是"已上电等解锁", 门控正是"解锁条件变严了"。
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
 */
static bool balance_arm_conditions_ok(void)
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
 * @brief 遥控指令 → 目标轮速 (前进为正)
 * @param dir      方向
 * @param level    档位百分比 (20/40/60/80)
 * @param out_avg  [出参] 两轮平均目标轮速 (前进为正, rpm); 前进/后退用它
 * @param out_diff [出参] 左轮 − 右轮 的目标差 (前进为正, rpm); 转向用它
 *
 * @note  前进/后退: 两轮同向 → 平均 = ±rpm, 差 = 0
 *       左/右转:  两轮反向 (原地转) → 平均 = 0, 差 = ±2×rpm
 *       (这个换算只有这一处, balance_set_move() 的日志和
 *        balance_update_target() 的斜坡都用它, 免得两处算出不一样的值)
 */
static void balance_move_targets(bal_move_t dir, int8_t level,
                                 float *out_avg, float *out_diff)
{
    float rpm  = (float)level * BAL_MOVE_RPM_MAX / 100.0f;
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
 * @note  目标轮速一阶斜坡:
 *        - 起步时目标不会瞬间跳到 48 rpm (否则速度环一上来就要求很大的
 *          前倾角 → 车猛地一冲);
 *        - 松手时目标慢慢退到 0, 车自然减速站稳 (跟人手推车的习惯一致)。
 *
 * ⚠ 指令 (s_move_dir / s_move_level) 是 MQTT 任务写的, 这里只读,
 *   不在两个任务之间传递"目标值", 免得两个任务同时改一个变量。
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
}

/**
 * @brief 跑一次串级控制运算并输出到电机
 * @param dt 控制周期 (秒)
 * @note  驱动方式: **双轮同向** (平衡环) + 可选的**差速** (转向环)。
 *        车往前倒 (pitch < 0) 时两个轮子都往"追着倒的方向"转,
 *        轮子跑出去 → 重心被甩到轮轴上方 → 车身被托回竖直。
 */
static void balance_control_step(float dt)
{
    /* --- 实时读取调参表 ---
     * App / 网页上改的 Kp/Ki/Kd/限幅/死区, 在**下一个控制周期**就生效。
     * (2026-10-26 修正: 之前只在启动时读一次, App 上改参数完全没用) */
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
    s_data.target_speed = s_tgt_avg;

    /* --- 外环: 速度环 → 目标倾角 ---
     * 误差 = 实测 − 目标 (前进为正)。
     *   实测比目标慢 → 误差为负 → 目标倾角为负 → 车前倾 (pitch 负 = 车头往下)
     *   → 轮子往车头跑 → 加速前进 ✓
     *   实测比目标快 → 误差为正 → 目标倾角为正 → 车后仰 → 减速 ✓
     *
     * ⚠ 误差死区: 编码器量化台阶是 4.5 rpm, 不设死区的话,
     *   车会为了追一个量化噪声一直歪着 (慢慢往一边跑)。 */
    float spd_err = s_speed_fwd - s_tgt_avg;
    if (fabsf(spd_err) < BAL_SPD_ERR_DEADBAND) spd_err = 0.0f;
    s_data.target_pitch = pid_update(&s_pid_spd, spd_err, dt);

    /* --- 差速环 (转向): 左−右 轮速差 → 两轮输出的差 ---
     * 转向时两轮反向转 (原地转), 平均轮速保持 0, 平衡环照常扶住车身。
     * 误差 = 目标差 − 实测差: 左轮该更快时误差为正 → 差速输出为正
     * → 左轮输出更小 (往车头 = 前进) → 左轮更快 ✓ */
    float diff_err = s_tgt_diff - s_diff_fwd;
    float yaw = pid_update(&s_pid_yaw, diff_err, dt);
    s_data.yaw_out = yaw;

    /* --- 内环: 平衡环 → 电机输出 ---
     * 误差 = 实际倾角 - 目标倾角
     * 本工程 pitch 的符号: **前倾 (车头往下) 为负** (见 balance.h 的方向常量)。
     * 所以车往前倒时误差为负 → 平衡输出为负 → 乘 BAL_MOTOR_SIGN(+1) 后
     * 电机输出仍为负 → 轮子往车头方向跑, 把重心拉回来。
     *
     * ⚠ D 项用**角速度反馈**而不是误差微分 (见 pid_update_rate 的注释),
     * 且与 P 项同号: 车往前倒得越快, 轮子追得越凶 —— 这才是阻尼。 */
    float bal_err = s_data.pitch - s_data.target_pitch;
    s_data.bal_out = pid_update_rate(&s_pid_bal, bal_err, s_data.gyro, dt);

    /* --- 执行: 双轮同向输出 (+ 差速) ---
     * BAL_MOTOR_SIGN 烘焙在代码里 (0.13 起不在网页上可调):
     * 改电机装配方向后, 要改的是 balance.h 的宏, 然后重烧固件。
     * ⚠ 2026-10-25 实测: 这个符号之前取反了 (原 -1), 导致前倾时轮子往后跑,
     *   车必然往前扑倒。现在是 +1, 详见 balance.h 的方向常量说明。 */
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
     *
     *   vTaskDelay(10ms) 的语义是"**至少**等 10ms", 实际周期 =
     *   10ms + 本次执行耗时。执行耗时里有 I2C 读(14 字节 @400kHz ≈ 0.4ms)、
     *   配平表插值、网页快照拷贝等, 累积起来真实周期可能是 11~12ms。
     *
     *   后果: PID 的 dt 如果按标称 0.01 算, 实际却是 0.012,
     *   相当于 Kd 和积分被**系统性低估**, 而且这个误差随负载浮动 ——
     *   表现为"同一组参数有时行有时不行"。
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

        /* 2) 状态机 (摔倒检测) —— **始终运行**
         * ⚠ 2026-10-26: 以前按方向键时这里被整个跳过 (直驱优先),
         *   平衡环和摔倒检测都不跑 → 车站着的时候一推, 平衡环等于被关掉,
         *   松手时车已经歪了, 平衡环接手必然剧烈纠偏。
         *   现在方向键走速度环, 平衡环全程在线。 */
        balance_state_machine(dt);

        /* 3) 运算 + 输出 */
        if (s_enabled) {
            balance_control_step(dt);
        } else if (s_move_dir != BAL_MOVE_STOP) {
            /* 平衡没开 (车没起立): 方向键走开环直驱, 方便单独调电机/编码器 */
            balance_apply_direct_move();
            s_data.target_pitch = 0.0f;
            s_data.bal_out      = 0.0f;
            s_data.spd_out      = 0.0f;
            s_data.yaw_out      = 0.0f;
        } else {
            /* 未启用: 只刹车, PID 不动 (enable 时会 reset) */
            s_data.target_pitch = 0.0f;
            s_data.bal_out      = 0.0f;
            s_data.spd_out      = 0.0f;
            s_data.yaw_out      = 0.0f;
            s_data.motor_a      = 0;
            s_data.motor_b      = 0;
            motor_brake_all();
        }

        /* 4) 一次性发布整帧快照, 避免外部读到撕裂的数据 */
        s_data.state = s_state;
        s_data.dt    = dt;
        s_data.spd_out = s_data.target_pitch;   /* 速度环输出就是目标倾角 */
        taskENTER_CRITICAL(&s_data_mux);
        s_data_pub = s_data;
        taskEXIT_CRITICAL(&s_data_mux);

#if BAL_TRACE_ENABLE
        /* 调试跟踪: 姿态角 / 角速度 / 输出 / 目标倾角 / 实测周期 */
        static TickType_t s_trace_tick = 0;
        TickType_t now_t = xTaskGetTickCount();
        if (now_t - s_trace_tick >= pdMS_TO_TICKS(BAL_TRACE_PERIOD_MS)) {
            s_trace_tick = now_t;
            ESP_LOGI(TAG, "TRACE pitch=%.2f gyro=%.1f out=%.1f tgt=%.2f "
                          "spd=%.1f/%.1f yaw=%.1f dt=%.1fms st=%d",
                     (double)s_data.pitch, (double)s_data.gyro,
                     (double)s_data.bal_out, (double)s_data.target_pitch,
                     (double)s_data.target_speed, (double)s_data.speed,
                     (double)s_data.yaw_out,
                     (double)(dt * 1000.0f), (int)s_state);
        }
#endif
    }
}

/* ================= 对外 API ================= */

esp_err_t balance_init(void)
{
    balance_load_params();
    balance_reset_pid();

    /* 载入配平表 (空表也 OK, 补偿恒为 0)
     * ⚠ 2026-10-25 修正: 原来这里每次上电都 trim_erase_nvs() + trim_load(),
     *   等于"每次上电都把标定好的配平表擦掉" —— 网页上标定半天,
     *   一断电就回到原点, 配平功能等于没有。
     *   现在只加载不擦; 换机械结构后要丢弃旧标定数据时,
     *   再显式调用 trim_erase_nvs() (见 trim.h)。 */
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

    /* 轮速统计的初始快照: 第一个 100ms 窗口只当基准, 不算速度。
     * (否则会拿"从 0 开始"的计数差算出一个假的巨大转速) */
    s_enc_last_cnt[0] = encoder_get_count(ENCODER_1);
    s_enc_last_cnt[1] = encoder_get_count(ENCODER_2);
    s_enc_win_cnt    = 0;
    s_enc_acc_pulse[0] = 0;
    s_enc_acc_pulse[1] = 0;
    s_enc_acc_win    = 0;
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

    /* 增大任务栈: 平衡控制涉及大量浮点运算, 需要足够空间避免溢出
     * ⚠ 优先级 7 (2026-10-26): 必须高于所有网络/上报任务
     *   (OneNet 高速上报 5/4、MQTT 客户端 5), 否则 WiFi 收发或 MQTT 发布会
     *   抢占控制任务 → 控制周期抖动 (dt 突然变大) → 阻尼相位失配 →
     *   表现为"站一会之后开始高频抖"。
     *
     * ⚠ 2026-10-26 追加: **绑到 core 1**
     *   IDF 的 WiFi 任务优先级 23 且固定跑在 core 0, lwIP 的 tcpip 任务
     *   优先级 18 (默认两个 core 都能跑)。它们一忙就会抢占优先级 7 的
     *   控制任务 → 控制周期从 10ms 变成 20~30ms → 反馈相位错位 →
     *   "一碰 / 一让走 (App 一发指令, MQTT 立刻高频上报) 就剧烈抖"。
     *   把控制任务绑到 core 1, 网络栈留在 core 0, 互不干扰。
     *   (lwIP 绑 core 0 见 sdkconfig 的 CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0) */
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
        balance_arm_reset();          /* 门控从零开始计时 */
        s_state   = BAL_STATE_STANDBY;
        s_enabled = true;
        s_out_ramp = BAL_RAMP_START_PCT;   /* 起立软启动: 输出限幅从缓开始 */

        /* --- 自动微调配平零点 (断电保存) ---
         * 车相对上次标定的姿态几乎没变时, 重新采一次零点并落盘:
         *   - 修掉加速度计/陀螺零偏随温度的漂移 → 每次上电 pitch 一致
         *   - 零点写进 NVS → 断电不丢
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

float balance_get_arm_ms(void)
{
    return s_arm_hold_ms;
}

esp_err_t balance_reload_params(void)
{
    balance_load_params();
    balance_reset_pid();
    ESP_LOGI(TAG, "控制参数已从 param 表重新读取");
    return ESP_OK;
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
     * (App 每秒会补发一次同样的指令, 指令没变时不能清) */
    if (s_move_dir != dir || s_move_level != lvl) {
        pid_reset(&s_pid_spd);
        pid_reset(&s_pid_yaw);
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
    pid_reset(&s_pid_spd);
    pid_reset(&s_pid_yaw);

    /* 平衡没开就直接刹车; 开了则交给平衡环接管 */
    if (!s_enabled) motor_brake_all();

    ESP_LOGI(TAG, "遥控指令结束, 目标轮速归零, 平衡环接管");
}

void balance_get_move(bal_move_t *out_dir, int8_t *out_level)
{
    if (out_dir)   *out_dir   = s_move_dir;
    if (out_level) *out_level = s_move_level;
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
