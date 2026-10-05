/**
 * @file balance.c
 * @brief 平衡控制串级控制器实现
 * @note  三个 PID 的输出**已接到电机** (2026-10-05)。
 *        驱动方式 = 双轮同向 (标准 inverted-pendulum-on-wheels 平衡车)。
 *        方向符号由 param 的 bal_sign 控制, 搞反了网页上改成 -1 即可。
 *
 * @note  2026-10-05 第二轮修改 (参照 lk888l/wheeled-legged_Robot-WL1):
 *   1) 陀螺仪取 **Y 轴** —— 之前误取 X 轴, 阻尼项一直在取噪声 (真 bug)
 *   2) D 项改**角速度反馈** (pid_update_rate), 不再用误差微分
 *   3) vTaskDelay → vTaskDelayUntil + 实测 dt, 消除周期漂移
 *   4) 新增**启动门控**: 连续稳定 500ms 才输出 (原来一扶正就冲)
 *   5) 新增**电机死区补偿** (motor_set_deadband)
 *   6) motor_max 默认 100 → 30 (默认值不该是全功率)
 *
 * @note  真车调参的起点 (顺序不要改):
 *   1) 确认 gyro_sign: 抬轮子前后晃, Pitch 与角速度应同向; 反了改 -1
 *   2) 扶正车, 启动后观察它往哪边冲: 往倒的方向冲 → bal_sign=+1 正确
 *   3) 再逐步加 bal_kp, 最后调 bal_kd
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
#include <math.h>

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
 *   而后者由左右两腿弯曲共同决定 —— 任一条腿弯了, 车身都跟着歪。
 *
 *   控制器把"车身 pitch=0"当成站立目标, 但真实的站立目标是
 *   "车身相对轮轴 = 某个随腿弯曲而变的角度"。腿一弯, 目标就偏了。
 *
 * @brief 两级方案 (查表 + 分腿)
 *
 *   【方案1】查表 (user/legmap) —— 首选
 *     左右两条腿**各有一张独立的实测表** (legmap_all_t.legs[LEFT/RIGHT])。
 *     每张表里存: { 舵机角, 电机中心↔舵机轴心直距, 该姿态 Pitch }。
 *     补偿 = 左表.pitch(φ_L) − 左表.pitch(φ_home) + 右表.pitch(φ_R) − 右表.pitch(φ_home)。
 *     为什么要分两腿查: 装配偏差/销孔间隙/打印形变左右不一样, 共用一张表
 *     会有一边欠补另一边过补, 表现就是车往一边歪 —— 这是用户实测抓的难题。
 *
 *   【方案2】线性 leg_k (param) —— 查表不可用时的兜底
 *     pitch_修正 = pitch_原始 + ang_zero - leg_k * (φ - φ_center)
 *     优点: 不需要量尺, 拨几个腿角读 Pitch 就能定; 缺点: 只能拟合直线。
 *
 *   两级都不可用 (legmap 双表都空 + leg_k=0) → 不补偿, 与最初版本行为一致。
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

    /* 电机死区补偿在 init 时就先写一次, 避免第一次 enable 前是旧值。
     * 控制任务每周期也会重写, 这里只是让初值就是对的。 */
    motor_set_deadband((int8_t)param_get_float(PARAM_MOTOR_DEADBAND));
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
 * @note  优先级: 左右两腿都查表 (>=2 点且曲线正常) > leg_k 线性 > 都不补偿
 *        任一路查表不可用就退回 leg_k; 都拿不到数据时返回 0,
 *        行为退回到最初版本, 不会误动作。
 *
 *   现在两路舵机是镜像的, SERVO_1 与 SERVO_2 的逻辑角始终相同,
 *   但**查表仍分两路走** —— 这样未来舵机如果改成独立驱动, 代码不用动。
 */
static float balance_leg_compensation(void)
{
    float phi_l = (float)servo_get_angle(SERVO_1);
    float phi_r = (float)servo_get_angle(SERVO_2);
    float phi_home = param_get_float(PARAM_SERVO_CENTER);

    /* --- 方案1: 双路查表 (首选) ---
     * 任一路查表不可用就整体退回 leg_k, 不要"半查表半线性", 那样补偿项会跳。 */
    if (legmap_is_usable(LEG_LEFT, 2) && legmap_is_usable(LEG_RIGHT, 2)) {
        float comp_l = legmap_pitch_comp(LEG_LEFT,  phi_l, phi_home);
        float comp_r = legmap_pitch_comp(LEG_RIGHT, phi_r, phi_home);

        /* 腿高只显示一边 (左右对称场景下数值一样; 不一样时说明装歪了,
         * 也只能给一个参考, 这里取左) */
        float h = legmap_height(LEG_LEFT, phi_l);
        s_leg_height = (h == LEGMAP_HEIGHT_UNKNOWN) ? 0.0f : h;

        s_leg_comp   = comp_l + comp_r;
        return s_leg_comp;
    }

    /* --- 方案2: 线性 leg_k 兜底 ---
     * 现在两路同角, 用一次查询就够了 (语义上仍按单腿算)。 */
    s_leg_height = 0.0f;   /* 没查表就推不出腿高, Kp 调度自动失效 */
    float k = param_get_float(PARAM_LEG_PITCH_K);
    if (k == 0.0f) {
        s_leg_comp = 0.0f;
        return 0.0f;
    }
    s_leg_comp = k * (phi_l - phi_home);
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

            /* ⚠⚠ 俯仰方向的角速度必须取 **gyro.y**, 不是 gyro.x (2026-10-05 修)
             *
             * 之前这里是 gyro.x —— 是个真 bug, 不是笔误:
             *   - mpu9250.c 融合 pitch 时积分的是 gyro.y (所以姿态角是对的)
             *   - 但平衡环的 D 项(阻尼)却取 gyro.x
             *   两者轴不一致 → 阻尼项在"车前后摆"的自由度上取的是 roll 方向
             *   的角速度, 对两轮前后走的平衡车来说那基本是**纯噪声**。
             *   表现: Kd 怎么调都不对, 车发软、抖、参数毫无规律。
             *
             * gyro_sign: 即便轴选对了, 陀螺仪读数方向仍可能与 pitch 符号相反
             * (取决于 MPU 装配朝向)。这个我无法从代码判断, 必须真机抬轮子测。
             * 测法: 网页看 Pitch 与 Gyro Sign 对应的角速度读数,
             *       车头朝前倾 → 两者应同向; 反了就把它改成 -1。
             * 取 0 或非 ±1 → 强制按 +1 并告警 (和 bal_sign 同样的处理)。 */
            float gsign = param_get_float(PARAM_GYRO_SIGN);
            if (gsign > 0.0f) {
                gsign = 1.0f;
            } else if (gsign < 0.0f) {
                gsign = -1.0f;
            } else {
                ESP_LOGW(TAG, "gyro_sign=%.1f 非法, 按 +1 处理", (double)param_get_float(PARAM_GYRO_SIGN));
                gsign = 1.0f;
            }
            float rate_raw = gyro.y * gsign;

            /* 一阶低通滤掉高频噪声, 系数越小越平滑但延迟越大 */
            float a = param_get_float(PARAM_GYRO_LPF);
            float g;
            if (!s_lpf_primed) {
                g = rate_raw;
                s_lpf_primed = true;
            } else {
                g = s_gyro_lpf + a * (rate_raw - s_gyro_lpf);
            }
            s_gyro_lpf = g;
            s_data.gyro_x = g;
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

/* ================= 启动门控 (arm gate) =================
 *
 * @brief 这是干什么的 (2026-10-05, 参照 WL1 BalanceStartupGate)
 *   以前的流程是: enable → 看一眼 pitch 没超限 → 立刻进 BALANCING 出电机。
 *   问题是"没超限"和"稳得住"完全是两回事:
 *     - 车被扶着、还在小幅晃动 → 旧逻辑立刻放行 → 电机接手 → 晃得更厉害
 *     - 推着车启动           → 旧逻辑照样放行 → 立刻冲出去
 *   两种情况都会直接摔, 而且摔的瞬间电机还在全力输出。
 *
 *   门控要求**连续满足** 3 个条件一段时间才放行:
 *     1) 倾角在 arm_pitch 以内
 *     2) 角速度在 arm_rate 以内  (这条能抓住"正在被晃动")
 *     3) 持续 arm_time 毫秒      (默认 500ms)
 *   期间电机保持刹车, 传感器和姿态融合照常跑。
 *
 * @brief 为什么放在 STANDBY 里而不是新加一个状态
 *   状态机对外暴露的状态枚举被 UI / 网页引用, 加新状态要改 4 个文件;
 *   而 STANDBY 的语义本来就是"已上电等解锁", 门控正是"解锁条件变严了",
 *   语义吻合且改动面小。
 */

/* 已连续满足门控条件的累计时长 (ms); 任何条件不满足立即清零 */
static float s_arm_hold_ms = 0.0f;

/**
 * @brief 重置门控计时 (enable / 摔倒 / 关闭控制时调用)
 */
static void balance_arm_reset(void)
{
    s_arm_hold_ms = 0.0f;
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

    /* 姿态或角速度是 NaN 时也判为不满足 —— NaN 比较永远返回 false,
     * 不能让"数据坏了"被当成"条件通过"。 */
    if (!(fabsf(s_data.pitch) <= pitch_lim)) return false;
    if (!(fabsf(s_data.gyro_x) <= rate_lim))  return false;
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
            /* 未启用, 什么都不做 */
            break;

        case BAL_STATE_STANDBY:
            /* 门控: 连续稳定 arm_time 毫秒才进平衡, 期间电机保持刹车。
             * 之前这里只判 "没摔倒" 就放行, 扶着车晃两下就会启动。 */
            if (fallen) {
                /* 已经歪到摔的阈值了, 门控没有意义, 只等人扶正 */
                balance_arm_reset();
            } else if (balance_arm_conditions_ok()) {
                float need = param_get_float(PARAM_ARM_TIME);
                if (need <= 0.0f) {
                    s_state = BAL_STATE_BALANCING;     /* 配置为 0 = 不等待 */
                    ESP_LOGI(TAG, "姿态正常 (门控已跳过), 进入平衡状态");
                } else {
                    s_arm_hold_ms += dt * 1000.0f;
                    if (s_arm_hold_ms >= need) {
                        s_state = BAL_STATE_BALANCING;
                        ESP_LOGI(TAG, "连续稳定 %.0fms, 进入平衡状态", (double)s_arm_hold_ms);
                    }
                }
            } else {
                /* 一旦有任一条件不满足, 计时清零重来 */
                balance_arm_reset();
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
            /* 摔倒后: 等人把车扶正 (倾角回到范围内) 才重新进待机,
             * 且门控重新计时, 不能一扶正就立刻冲 */
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
 * @brief 跑一次串级控制运算并输出到电机
 * @param dt 控制周期 (秒)
 * @note  驱动方式: **双轮同向**。
 *        车往前倒 (pitch > 0) 时两个轮子都往"追着倒的方向"转,
 *        轮子跑出去 → 重心被甩到轮轴上方 → 车身被托回竖直。
 *        这是普通平衡车 (inverted pendulum on wheels) 的标准做法。
 *
 *        ⚠ 早先 balance.h 的注释写的是"差速/坦克转向", 那个是错的:
 *          差速只能原地转, 产生不了把重心托回来需要的**平移**加速度。
 *
 * @note  ⚠ 转向环: 输出恒为 0 (本版没有转向目标)。
 *        保留字段是为了以后加"转向"时结构不用动, 现在不参与电机输出。
 */
static void balance_control_step(float dt)
{
    /* --- 外环: 速度环 → 目标倾角 ---
     * 车速要往前, 车就得先往前倾一点, 所以目标倾角 = 速度环输出 */
    float spd_err = 0.0f - s_data.speed;
    s_data.target_pitch = pid_update(&s_pid_spd, spd_err, dt);

    /* --- 内环: 平衡环 → 电机输出 ---
     * 误差 = 车实际倾角 - 目标倾角
     * 车往前倒 (pitch > 0) 时误差为正, 输出为正 → 电机往"追着倒的方向"转,
     * 把重心拉回来。
     *
     * ⚠ Kp 按当前腿高动态调度 (见 balance_effective_kp 的物理依据)。
     *   必须放在误差计算之前, 因为误差要先乘 Kp。 */
    s_pid_bal.kp = balance_effective_kp(param_get_float(PARAM_PID_BAL_KP));

    float bal_err = s_data.pitch - s_data.target_pitch;

    /* D 项用**角速度反馈**而不是误差微分 (2026-10-05, 参照 WL1)
     *   平衡环要阻尼的是"车摆得多快", 那就是陀螺仪读数本身。
     *   误差微分 (pid_update) 相当于对姿态再求一次导: 目标阶跃时出尖峰,
     *   而且把姿态量化噪声二次放大 —— 对平衡环是负作用。
     *   s_data.gyro_x 已在采样阶段取自 gyro.y 并按 gyro_sign 调正符号。 */
    s_data.bal_out = pid_update_rate(&s_pid_bal, bal_err, s_data.gyro_x, dt);
    s_data.kp_eff  = s_kp_eff;

    /* --- 转向环: 本版没有转向目标, 输出恒为 0 --- */
    s_data.steer_out = 0.0f;

    /* --- 执行: 双轮同向输出 ---
     *
     * bal_sign: 电机装配/接线的方向符号。搞反了车会往倒的反方向冲,
     * 一启动就摔。这个用参数而不是写死常量, 就是为了让方向错了
     * 能在网页上直接改, 不用重新烧录固件。
     *
     *   取 0 或非 ±1 → 强制按 +1 处理, 避免"参数没设好"导致输出恒为 0
     *   (那样会静默退化成刹车, 看不出是符号问题还是没输出)。 */
    float sign = param_get_float(PARAM_BAL_SIGN);
    if (sign > 0.0f) {
        sign = 1.0f;
    } else if (sign < 0.0f) {
        sign = -1.0f;
    } else {
        ESP_LOGW(TAG, "bal_sign=%.1f 非法, 按 +1 处理", (double)param_get_float(PARAM_BAL_SIGN));
        sign = 1.0f;
    }

    float out = s_data.bal_out * sign;
    float out_max = param_get_float(PARAM_MOTOR_OUT_MAX);
    if (out >  out_max) out =  out_max;
    if (out < -out_max) out = -out_max;

    s_data.motor_a = (int8_t)out;
    s_data.motor_b = (int8_t)out;   /* 同向: 两轮一起转, 不做差速 */

    /* 死区补偿 (2026-10-05 新增, 参照 WL1)
     *   每周期写入, 网页拖滑块立刻生效。
     *   作用: 让极小的非零输出也能真正推动电机, 消除平衡点附近的
     *   "慢慢爬行 + 突然突跳"。0 输出和制动不受影响 (见 motor_apply_deadzone)。 */
    motor_set_deadband((int8_t)param_get_float(PARAM_MOTOR_DEADBAND));

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

    /* ⚠ 用 vTaskDelayUntil 而不是 vTaskDelay (2026-10-05 修)
     *
     *   vTaskDelay(10ms) 的语义是"**至少**等 10ms", 实际周期 =
     *   10ms + 本次执行耗时。执行耗时里有 I2C 读(14 字节 @400kHz ≈ 0.4ms)、
     *   legmap 插值、网页快照拷贝等, 累积起来真实周期可能是 11~12ms。
     *
     *   后果: PID 的 dt 参数写死 CONTROL_DT=0.01, 实际却是 0.012。
     *   对平衡环是实打实的增益误差 —— 相当于 Kd/积分都被系统性低估,
     *   而且这个误差随负载/网络负载浮动, 表现为"同一组参数有时行有时不行"。
     *
     *   vTaskDelayUntil(&last_wake, period) 从**上次唤醒时刻**起算,
     *   执行耗时会被自动扣掉, 周期严格稳定在 10ms。
     *   同时下面用实测 dt 喂给 PID, 万一真的超时了 (调度被抢占),
     *   积分和导数也知道真实的间隔是多少, 不会静默算错。 */
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
        /* 夹上限: 任务被长时间抢占 (比如网页请求/Flash 写入) 后,
         * 不该把那一大段空档当成一个巨大的积分/微分区间。 */
        if (dt > 0.1f) dt = 0.1f;

        /* 1) 采样 (只写 s_data 的输入段, 不需要加锁: 只有本任务写输入) */
        balance_read_sensors();

        /* 2) 状态机 (摔倒检测) */
        balance_state_machine(dt);

        /* 3) 运算 + 输出 */
        if (s_enabled) {
            balance_control_step(dt);
        } else {
            /* 未启用: 只刹车, PID 不动 (保留积分, 但 enable 时会 reset) */
            s_data.target_pitch = 0.0f;
            s_data.bal_out      = 0.0f;
            s_data.spd_out      = 0.0f;
            s_data.steer_out    = 0.0f;
            s_data.motor_a      = 0;
            s_data.motor_b      = 0;
            s_data.kp_eff       = 0.0f;
            motor_brake_all();
        }

        /* 4) 一次性发布整帧快照, 避免外部读到撕裂的数据 */
        s_data.state = s_state;
        s_data.dt    = dt;
        taskENTER_CRITICAL(&s_data_mux);
        s_data_pub = s_data;
        taskEXIT_CRITICAL(&s_data_mux);
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
    s_arm_hold_ms = 0.0f;
    s_mpu_ready = mpu9250_is_ready();

    if (!s_mpu_ready) {
        ESP_LOGW(TAG, "MPU9250 不可用, 平衡控制无法工作 (请先接好 IMU)");
    }

    const legmap_all_t *lm = legmap_get();
    if (legmap_is_usable(LEG_LEFT, 2) && legmap_is_usable(LEG_RIGHT, 2)) {
        ESP_LOGI(TAG, "腿高标定: L:%u 点, R:%u 点, 补偿与 Kp 调度生效",
                 lm->legs[LEG_LEFT].count, lm->legs[LEG_RIGHT].count);
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
        balance_arm_reset();          /* 门控从零开始计时 */
        s_state   = BAL_STATE_STANDBY;
        s_enabled = true;
        ESP_LOGI(TAG, "平衡控制已启动, 等待连续稳定 %.0fms 后输出 "
                      "(需 |pitch|<%.0f° 且 |角速度|<%.0f°/s)",
                 (double)param_get_float(PARAM_ARM_TIME),
                 (double)param_get_float(PARAM_ARM_PITCH),
                 (double)param_get_float(PARAM_ARM_RATE));
    } else {
        s_enabled = false;
        s_state   = BAL_STATE_IDLE;
        balance_arm_reset();
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

/**
 * @brief 读取启动门控的已累计稳定时长
 * @return 毫秒; 0=当前不满足门控条件(或还没 enable)
 * @note  与 PARAM_ARM_TIME 比较即可知道还要等多久。
 *        网页显示这个值, 便于判断"是没满足条件"还是"条件满足了在计时"。
 */
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
