/**
 * @file balance.c
 * @brief 平衡控制串级控制器实现 (骨架版)
 * @note  ⚠ 本版是骨架: 接口、状态机、参数读取、传感器采样都已就位,
 *        但**三个 PID 的输出还没有接到电机上**, 控制任务只维持刹车。
 *        下一步填算法时改 control_step() 里的输出段即可, 其余不用动。
 *
 * @note  为什么骨架版先不驱动电机:
 *        平衡参数还没在真车上试过, 直接接上电机的风险是车立刻飞出去。
 *        骨架版保证上电后电机一定是停的, 可以安全地烧录验证。
 *
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#include "balance.h"
#include "param.h"
#include "legmap.h"
#include "mpu9250.h"
#include "encoder.h"
#include "motor_driver.h"
#include "servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BALANCE";

/* ================= 模块内部状态 ================= */

/* 三个环的 PID 实例 */
static bal_pid_t s_pid_bal;      /* 平衡环: 姿态误差 → 电机差速 */
static bal_pid_t s_pid_spd;      /* 速度环: 轮速误差 → 目标倾角 */
static bal_pid_t s_pid_steer;    /* 转向环: 转向误差 → 左右差速修正 */

/* 状态机 + 使能标志 */
static bal_state_t s_state     = BAL_STATE_IDLE;
static bool        s_enabled    = false;

/* 最近一周期数据快照 (UI / 网页读)
 * s_data    = 控制任务内部的工作区, 只有控制任务写
 * s_data_pub = 对外发布的完整快照, 在临界区内一次性拷贝, 供 UI / 网页读 */
static bal_data_t  s_data;
static bal_data_t  s_data_pub;

/* 保护 s_data 的互斥锁
 * ⚠ 必须加锁: 控制任务 100Hz 写, UI 主循环和 httpd 任务同时读,
 *   bal_data_t 有 10 个 float, 不加锁会读到"一半新一半旧"的撕裂值
 *   (典型现象: 状态显示 BALANCING 但 motor 输出还是上一帧的 0)。
 *   读接口用 portMUX (临界区极短, 不影响控制周期)。 */
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

/* MPU9250 是否可用 (没接模块时控制任务仍要能跑, 只是没姿态数据) */
static bool s_mpu_ready = false;

/* 陀螺仪低通滤波状态 (一阶 IIR, 系数来自 param) */
static float s_gyro_lpf = 0.0f;
static bool  s_lpf_primed = false;

/* ================= 腿角 → 俯仰角 几何补偿 =================
 *
 * @brief 这是什么问题
 *   MPU 装在**车身**上, 它报的是"车身相对铅垂线"的角度。但车站在地上时
 *   轮轴被地面强制水平, 真正决定会不会倒的是"**车身相对轮轴**"的角度,
 *   而后者完全由**腿弯曲多少**决定 —— 腿一弯, 车身就跟着歪。
 *
 *   控制器把"车身 pitch=0"当成站立目标, 但真实的站立目标是
 *   "车身相对轮轴 = 某个随腿弯曲而变的角度"。腿一弯, 目标就偏了。
 *
 * @brief 两级方案 (优先用查表, 没有才退回线性)
 *
 *   【方案1】查表 (user/legmap) —— 首选
 *     用户实测每个舵机角下的: 轮心高度(mm) 和 车身 Pitch(°),
 *     存在 legmap 里做插值。补偿 = pitch(当前角) - pitch(参考角)。
 *     优点: 直接量真实世界, 含杆长误差/销孔间隙/装配偏差/打印形变,
 *           不依赖任何模型精度; 且补偿曲线可以是任意形状 (实测是 U 型,
 *           用一次斜率近似会丢掉近一半补偿量)。
 *
 *   【方案2】线性 leg_k (param) —— 查表不可用时的兜底
 *     pitch_修正 = pitch_原始 + ang_zero - leg_k * (φ - φ_center)
 *     优点: 不需要量尺, 拨几个腿角读 Pitch 就能定; 缺点: 只能拟合直线。
 *
 *   两级都不可用 (legmap 空 + leg_k=0) → 不补偿, 与最初版本行为一致。
 *
 * @brief 为什么放信号侧而不是加在目标倾角上
 *   1) 摔倒判定 balance_check_fallen() 判的是 |pitch|, 修正后它也跟着对,
 *      否则腿弯多了会误判摔倒。放目标值上则判定不会跟着变。
 *   2) 目标倾角就干干净净只剩"想让它倾斜多少"(速度环输出), 语义清晰。
 *
 * @brief 局限 (务必知道)
 *   舵机是**开环**的 (servo_set_angle 只写 PWM, 没有位置回读),
 *   查表的横轴 φ 是"指令角"不是"实际角", 腿受力变形会造成误差。
 *   查表方案因为纵轴也是实测的, 这部分误差已在标定数据里部分体现;
 *   但常值残差仍需平衡环的 Ki 兜住。
 */

/* 本周期的中间量, 供 UI / 网页观察补偿是否在工作 */
static float s_leg_comp   = 0.0f;   /* 姿态补偿量 (°) */
static float s_leg_height = 0.0f;   /* 推算腿高 (mm) */
static float s_kp_eff     = 0.0f;   /* 实际生效的平衡环 Kp */


/* ================= 内部辅助 ================= */

/**
 * @brief 从 param 表读出所有控制参数, 配置三个 PID
 * @note  每次控制周期都调一次: 网页拖滑块后下一次运算立刻用新值, 无需重启
 */
static void balance_load_params(void)
{
    /* 积分限幅: 取各自输出限幅的一半, 保证 Ki 有效的同时不会积分饱和 */
    float out_max   = param_get_float(PARAM_MOTOR_OUT_MAX);
    float spdout_max = param_get_float(PARAM_SPEED_OUT_MAX);

    pid_config(&s_pid_bal,
               param_get_float(PARAM_PID_BAL_KP),
               param_get_float(PARAM_PID_BAL_KI),
               param_get_float(PARAM_PID_BAL_KD),
               out_max / 2.0f,      /* 平衡环积分限幅 */
               out_max);            /* 平衡环输出 = 电机差速 (%) */

    pid_config(&s_pid_spd,
               param_get_float(PARAM_PID_SPD_KP),
               param_get_float(PARAM_PID_SPD_KI),
               param_get_float(PARAM_PID_SPD_KD),
               spdout_max / 2.0f,   /* 速度环积分限幅 */
               spdout_max);         /* 速度环输出 = 目标倾角 (°) */

    pid_config(&s_pid_steer,
               param_get_float(PARAM_PID_STEER_KP),
               param_get_float(PARAM_PID_STEER_KI),
               param_get_float(PARAM_PID_STEER_KD),
               0.0f,                /* 转向环暂不积分 */
               out_max);            /* 转向环输出 = 差速修正 (%) */
}

/**
 * @brief 清空三个 PID 的积分和微分历史
 */
static void balance_reset_pid(void)
{
    pid_reset(&s_pid_bal);
    pid_reset(&s_pid_spd);
    pid_reset(&s_pid_steer);
    s_lpf_primed = false;
}

/**
 * @brief 查表 + 线性兜底, 算本周期要减掉的"腿弯曲带来的俯仰角" (°)
 * @return 补偿量 (°), 已写入 s_leg_comp / s_leg_height
 * @note  优先级: legmap 查表 (>=2 点且单调) > leg_k 线性 > 都不补偿
 *        任何一级都拿不到数据时返回 0, 行为退回到最初版本, 不会误动作。
 */
static float balance_leg_compensation(void)
{
    /* 两路舵机是镜像的 (视觉对称), 共用一个"逻辑角", 所以只有一个腿角自由度。
     * 取 SERVO_1 的逻辑角: 它是内部维护的逻辑量, 不受物理镜像映射影响。 */
    float phi = (float)servo_get_angle(SERVO_1);

    /* --- 方案1: 查表 (首选) --- */
    if (legmap_is_usable(2)) {
        /* 参考角取舵机中心角。查表里没有精确这个角也没关系,
         * legmap_pitch_comp() 内部会做插值。
         * 为什么要减掉参考角的 Pitch: 因为 ang_zero 只配平了"中心角这一档",
         * 其他腿角相对中心角的差值就是腿带来的额外部分。 */
        float comp = legmap_pitch_comp(phi, param_get_float(PARAM_SERVO_CENTER));

        float h = legmap_height(phi);
        s_leg_height = (h == LEGMAP_HEIGHT_UNKNOWN) ? 0.0f : h;
        s_leg_comp   = comp;
        return comp;
    }

    /* --- 方案2: 线性 leg_k 兜底 --- */
    s_leg_height = 0.0f;   /* 没查表就推不出腿高, Kp 调度自动失效 */
    float k = param_get_float(PARAM_LEG_PITCH_K);
    if (k == 0.0f) {
        s_leg_comp = 0.0f;
        return 0.0f;
    }
    s_leg_comp = k * (phi - param_get_float(PARAM_SERVO_CENTER));
    return s_leg_comp;
}

/**
 * @brief 按当前腿高算出平衡环实际生效的 Kp
 * @param base_kp 基础 Kp (param 里的 bal_kp)
 * @return 调度后的 Kp
 * @note  物理依据: 腿长 → 重心高 → 绕轮的转动惯量大。
 *        同样的 Kp 在高腿位下阻尼不足, 车会发软、振荡;
 *        低腿位下 Kp 太高又容易抖。所以 Kp 要随腿高线性升降。
 *
 * ⚠ 斜率必须自己试车标定, 没有通用值。
 *   调法: 站不起来的那个腿高, 把 Kp 加到不晃为止, 记下; 换腿高再记一次,
 *   两次差值除以高度差 = kp_hslope。方向反了就取负。
 */
static float balance_effective_kp(float base_kp)
{
    float slope = param_get_float(PARAM_KP_HEIGHT_SLOPE);
    if (slope == 0.0f || s_leg_height <= 0.0f) {
        s_kp_eff = base_kp;
        return base_kp;
    }

    float ref  = param_get_float(PARAM_KP_HEIGHT_REF);
    float kp   = base_kp + slope * (s_leg_height - ref);

    /* 限幅: 调度出来的 Kp 必须落在合理区间, 否则腿高跳变时会直接把车搞飞。
     * 上限用滑块允许的最大 Kp, 下限为 0 (不做负 Kp, 那等于反作用力)。 */
    float kp_max = 200.0f;
    if (kp > kp_max) kp = kp_max;
    if (kp < 0.0f)   kp = 0.0f;

    s_kp_eff = kp;
    return kp;
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
            /* 姿态修正链:
             *   1) 加零偏 (ang_zero), 配平"参考腿角"这一档的固定歪角
             *   2) 减腿弯曲带来的部分 (查表优先, 退回 leg_k 线性) */
            s_data.pitch = euler.pitch
                         + param_get_float(PARAM_ANGLE_ZERO)
                         - balance_leg_compensation();
            /* 留档, 供 UI/网页观察补偿与腿高是否在动 */
            s_data.leg_comp   = s_leg_comp;
            s_data.leg_height = s_leg_height;

            /* 一阶低通滤掉高频噪声, 系数越小越平滑但延迟越大 */
            float a = param_get_float(PARAM_GYRO_LPF);
            if (!s_lpf_primed) {
                s_gyro_lpf = gyro.x;
                s_lpf_primed = true;
            } else {
                s_gyro_lpf += a * (gyro.x - s_gyro_lpf);
            }
            s_data.gyro_x = s_gyro_lpf;
        }
    } else {
        s_data.pitch      = 0.0f;
        s_data.gyro_x     = 0.0f;
        s_data.leg_comp   = 0.0f;
        s_data.leg_height = 0.0f;
    }

    /* --- 编码器: 两轮平均轮速 (去掉减速比, 得到轮子真实转速) --- */
    float gear = param_get_float(PARAM_ENCODER_GEAR);
    if (gear < 0.01f) gear = 0.01f;      /* 防除零 */
    float rpm1 = encoder_get_rpm(ENCODER_1, CONTROL_PERIOD_MS) / gear;
    float rpm2 = encoder_get_rpm(ENCODER_2, CONTROL_PERIOD_MS) / gear;
    s_data.speed = (rpm1 + rpm2) / 2.0f;
}

/**
 * @brief 摔倒检测
 * @param pitch 姿态角 (°)
 * @return true=判定摔倒
 * @note  超过 param 的 ang_limit 就认为车已经倒了, 立刻停电机。
 *        宁可误停 (车还立着但先不跑) 也不能不停 (车倒了还在输出把电机烧了)
 */
static bool balance_check_fallen(float pitch)
{
    float limit = param_get_float(PARAM_ANGLE_LIMIT);
    return (pitch > limit) || (pitch < -limit);
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
            /* 未启用, 什么都不做 */
            break;

        case BAL_STATE_STANDBY:
            /* 已上电等解锁: 倾角在范围内才允许进平衡, 否则保持待机 */
            if (!fallen) {
                s_state = BAL_STATE_BALANCING;
                ESP_LOGI(TAG, "姿态正常, 进入平衡状态");
            }
            break;

        case BAL_STATE_BALANCING:
            if (fallen) {
                s_state = BAL_STATE_FALLEN;
                ESP_LOGW(TAG, "检测到摔倒 (pitch=%.1f°), 停电机", (double)s_data.pitch);
                /* 注意: 这里不能调 balance_emergency_stop(),
                 * 它会把 s_enabled 清掉, FALLEN→STANDBY→BALANCING 的
                 * 自动恢复路径就走不到了。摔倒只需要刹车 + 清积分。 */
                balance_reset_pid();
                motor_brake_all();
            }
            break;

        case BAL_STATE_FALLEN:
            /* 摔倒后: 等人把车扶正 (倾角回到范围内) 才重新进待机 */
            if (!fallen) {
                s_state = BAL_STATE_STANDBY;
                ESP_LOGI(TAG, "车已扶正, 等待重新启动");
            }
            break;

        default:
            break;
    }
    (void)dt;
}

/**
 * @brief 跑一次串级控制运算并输出到电机
 * @param dt 控制周期 (秒)
 * @note  ⚠ 骨架版: 三个 PID 的输出**故意不接到电机**, 只记录中间量。
 *        下一步填算法时, 打开下面注释掉的输出段即可。
 */
static void balance_control_step(float dt)
{
    /* --- 外环: 速度环 → 目标倾角 ---
     * 车速要往前, 车就得先往前倾一点, 所以目标倾角 = 速度环输出 */
    float spd_err = 0.0f - s_data.speed;
    s_data.target_pitch = pid_update(&s_pid_spd, spd_err, dt);

    /* --- 内环: 平衡环 → 电机差速 ---
     * 误差 = 车实际倾角 - 目标倾角
     * 车往前倒 (pitch > 0) 时误差为正, 输出为正 → 电机往"追着倒的方向"转,
     * 把重心拉回来。符号约定下一步真机试车时再确认。
     *
     * ⚠ Kp 按当前腿高动态调度 (见 balance_effective_kp 的物理依据)。
     *   必须放在误差计算之前, 因为误差要先乘 Kp。 */
    s_pid_bal.kp = balance_effective_kp(param_get_float(PARAM_PID_BAL_KP));

    float bal_err = s_data.pitch - s_data.target_pitch;
    s_data.bal_out = pid_update(&s_pid_bal, bal_err, dt);
    s_data.kp_eff  = s_kp_eff;

    /* --- 转向环: 本版没有转向目标, 输出恒为 0 --- */
    s_data.steer_out = 0.0f;

    /* --- 执行: 骨架版保持刹车 ---
     * 下一步算法写好后, 把下面这段的 motor_a / motor_b 计算打开,
     * 并改用 motor_set_speed_both() 输出。
     *
     * float base = s_data.bal_out;                 // 差速驱动 (坦克转向)
     * float diff = s_data.steer_out;               // 转向修正
     * float out_max = param_get_float(PARAM_MOTOR_OUT_MAX);
     * float la = base + diff;
     * float lb = base - diff;
     * s_data.motor_a = (int8_t)CLAMP_F(la, -out_max, out_max);
     * s_data.motor_b = (int8_t)CLAMP_F(lb, -out_max, out_max);
     */
    s_data.motor_a = 0;
    s_data.motor_b = 0;

    /* 骨架版: 只有在平衡状态才输出, 且当前恒为 0 → 实际效果是刹车 */
    if (s_state == BAL_STATE_BALANCING) {
        motor_set_speed_both(s_data.motor_a, s_data.motor_b);
    } else {
        motor_brake_all();
    }
}

/**
 * @brief 控制任务: 固定周期跑串级控制
 */
static void balance_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    for (;;) {
        /* 1) 采样 (只写 s_data 的输入段, 不需要加锁: 只有本任务写输入) */
        balance_read_sensors();

        /* 2) 状态机 (摔倒检测) */
        balance_state_machine(CONTROL_DT);

        /* 3) 运算 + 输出 */
        if (s_enabled) {
            balance_control_step(CONTROL_DT);
        } else {
            /* 未启用: 只刹车, PID 不动 (保留积分, 但 enable 时会 reset) */
            s_data.target_pitch = 0.0f;
            s_data.bal_out      = 0.0f;
            s_data.spd_out      = 0.0f;
            s_data.steer_out    = 0.0f;
            s_data.motor_a      = 0;
            s_data.motor_b      = 0;
            motor_brake_all();
        }

        /* 4) 一次性发布整帧快照, 避免外部读到撕裂的数据 */
        s_data.state = s_state;
        taskENTER_CRITICAL(&s_data_mux);
        s_data_pub = s_data;
        taskEXIT_CRITICAL(&s_data_mux);

        vTaskDelay(period);
    }
}

/* ================= 对外 API ================= */

esp_err_t balance_init(void)
{
    balance_load_params();
    balance_reset_pid();

    /* 载入腿高标定表 (没有就退回 leg_k 线性, 都没有则不补偿) */
    legmap_init();

    s_state  = BAL_STATE_IDLE;
    s_enabled = false;
    s_mpu_ready = mpu9250_is_ready();

    if (!s_mpu_ready) {
        ESP_LOGW(TAG, "MPU9250 不可用, 平衡控制无法工作 (请先接好 IMU)");
    }

    const legmap_t *lm = legmap_get();
    if (legmap_is_usable(2)) {
        ESP_LOGI(TAG, "腿高标定: %u 个点, 补偿与 Kp 调度生效", lm->count);
    } else if (param_get_float(PARAM_LEG_PITCH_K) != 0.0f) {
        ESP_LOGI(TAG, "腿高未标定, 退回 leg_k 线性补偿 (%.3f)",
                 (double)param_get_float(PARAM_LEG_PITCH_K));
    } else {
        ESP_LOGW(TAG, "腿角未标定 (leg_k=0), 不做几何补偿 —— "
                     "腿改变角度时车会往一个方向偏");
    }

    if (xTaskGetHandle("bal") != NULL) {
        return ESP_OK;    /* 已经起过了 */
    }

    BaseType_t ret = xTaskCreate(balance_task, "bal", 4096, NULL, 6, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "控制任务创建失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "控制任务已启动 (周期 %dms, MPU9250 %s)",
             CONTROL_PERIOD_MS, s_mpu_ready ? "就绪" : "未就绪");
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
        s_state   = BAL_STATE_STANDBY;
        s_enabled = true;
        ESP_LOGI(TAG, "平衡控制已启动");
    } else {
        s_enabled = false;
        s_state   = BAL_STATE_IDLE;
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
    balance_reset_pid();       /* 清积分, 避免下次启动猛冲 */
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

esp_err_t balance_reload_params(void)
{
    balance_load_params();
    balance_reset_pid();
    ESP_LOGI(TAG, "控制参数已从 param 表重新读取");
    return ESP_OK;
}
