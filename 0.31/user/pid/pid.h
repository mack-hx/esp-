/**
 * @file pid.h
 * @brief 通用 PID 控制器 (位置式)
 * @note  平衡车用到的三个环都用这个结构:
 *        - 平衡环 (balance_pid.c): 输入 = 姿态角误差 (°),   输出 = 电机输出 (%)
 *        - 速度环 (speed_pid.c):   输入 = 轮速误差 (rpm),  输出 = 目标倾角 (°)
 *        - 转向环 (steer_pid.c):   输入 = 转向误差,        输出 = 左右差速修正 (%)
 *
 * @note  dt 由调用方传入 (秒)。不要在 pid_update() 内部取时间:
 *        控制周期是固定的 (control.c 里 CONTROL_PERIOD_MS),
 *        传固定 dt 能保证同一组参数在离线计算和真机运行时表现一致。
 *
 * @note  ⚠ 类型名必须叫 bal_pid_t, 不能叫 pid_t:
 *        newlib 的 <sys/types.h> 已经 typedef 了 pid_t (进程 id, int),
 *        重名会直接编译报错 "conflicting types for 'pid_t'"。
 *
 * @note  平衡环**不要**用 pid_update() 的误差微分, 要用 pid_update_rate()
 *        的角速度反馈型 D 项。理由见 pid_update_rate() 的注释。
 *
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

/**
 * @brief PID 控制器状态结构
 * @note  各个环各持有一个实例, 互不干扰
 */
typedef struct {
    float kp;        /* 比例系数 */
    float ki;        /* 积分系数 */
    float kd;        /* 微分系数 */
    float i_max;     /* 积分累积限幅 (防积分饱和) */
    float out_max;   /* 输出限幅 (正负对称) */

    /* ---- 以下为运行状态, 由 pid_update() 维护, 不要手动改 ---- */
    float i;         /* 积分累积值 */
    float prev_err;  /* 上一次的误差 (微分用) */
    bool  primed;    /* 是否已经执行过至少一次 pid_update (false 时跳过微分) */
} bal_pid_t;

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
 * @note  ki≈0 时积分不参与运算, 直接返回 out_max (不起作用)。
 */
float pid_i_max(float ki, float out_max);

/**
 * @brief 初始化 PID 系数
 * @param pid     PID 结构指针
 * @param kp      比例系数
 * @param ki      积分系数
 * @param kd      微分系数
 * @param i_max   积分累积限幅 (绝对值, <=0 表示不限制)
 * @param out_max 输出限幅 (绝对值, <=0 表示不限制)
 */
void pid_config(bal_pid_t *pid, float kp, float ki, float kd,
                float i_max, float out_max);

/**
 * @brief 只改系数, **保留**积分和微分历史
 * @param pid     PID 结构指针
 * @param kp      比例系数
 * @param ki      积分系数
 * @param kd      微分系数
 * @param i_max   积分累积限幅 (绝对值, <=0 表示不限制)
 * @param out_max 输出限幅 (绝对值, <=0 表示不限制)
 *
 * @brief 为什么需要它 (2026-10-26)
 *   App / 网页上拖滑块改 Kp 之后, 必须**立刻**作用到正在跑的 PID 上。
 *   之前 balance.c 只在启动时调 pid_config() 一次, 而 pid_config() 里
 *   带着 pid_reset() (清积分), 所以要么用 pid_config (积分被清, 每周期
 *   调用会让积分永远为 0), 要么参数改了不生效 (App 上把 Kp 拖到 0 也没用)。
 *   → 分成两个函数: pid_config   = 改系数 + 清历史 (启动/切换模式用)
 *                    pid_set_gains = 只改系数       (运行中改参数用)
 */
void pid_set_gains(bal_pid_t *pid, float kp, float ki, float kd,
                   float i_max, float out_max);

/**
 * @brief 清空积分和微分历史 (上电 / 摔倒恢复 / 切模式时调用)
 * @note  不清积分的话, 上一次摔倒前累积的积分会让电机一启动就猛冲一下
 * @param pid PID 结构指针
 */
void pid_reset(bal_pid_t *pid);

/**
 * @brief 让积分项按系数衰减 (向 0 靠拢)
 * @param pid       PID 结构指针
 * @param decay     衰减系数 (0~1): 每次调用后 i *= decay
 * @param dt        距上次调用的时间间隔 (秒, 用于按时间常数折算)
 * @param time_const 衰减时间常数 (秒): 越小衰减越快; <=0 表示不衰减
 * @return 无
 *
 * @brief 为什么需要它 (2026-10-27, 修"车一直往前跑")
 *   速度环的误差死区是在 pid_update() **之前** 把小误差归零的。
 *   这样做的副作用是: 误差一旦进死区, 传给 PID 的 err 恒为 0,
 *   积分 i 变成 i += 0*dt —— **冻结**, 既不增长也不减少。
 *   → 走路时攒下的积分残留会**永久冻结**在非零值上,
 *     目标倾角恒定非零 → 车持续往前跑。
 *   而且积分只在"换指令/摔倒/上电"这些边界才清, 平衡期间永不清理。
 *
 * @note  与"直接清零"的区别:
 *   - 直接清零: 误差进死区的一瞬间积分归零, 输出会有台阶跳变 → 抖动
 *   - 指数衰减: 积分平滑地滑向 0, 输出连续 → 不抖
 *   @note  时间常数公式: decay = exp(-dt / time_const)
 *         这样 time_const 的物理含义是"积分衰减到 37% 需要的时间",
 *         与调用频率无关。
 */
void pid_decay_integral(bal_pid_t *pid, float dt, float time_const);

/**
 * @brief 执行一次 PID 运算 (误差微分型)
 * @param pid PID 结构指针
 * @param err 当前误差 (目标值 - 反馈值)
 * @param dt  距上次调用的时间间隔 (秒, 必须 > 0)
 * @return 控制输出 (已按 out_max 限幅)
 *
 * @note  ⚠ 平衡环**不要**用这个函数 (见 pid_update_rate)。
 *        这里的 D 项是"误差变化率", 对平衡环来说是二次微分,
 *        而且会放大编码器/姿态的量化噪声。
 *        本函数保留给速度环这类反馈本身干净的场合。
 */
float pid_update(bal_pid_t *pid, float err, float dt);

/**
 * @brief 执行一次 PID 运算, D 项用**角速度反馈** (平衡环专用)
 * @param pid   PID 结构指针
 * @param err   当前误差 (目标值 - 反馈值)
 * @param rate  该误差对应方向上的角速度 (°/s), 已按正确符号
 * @param dt    距上次调用的时间间隔 (秒, 必须 > 0)
 * @return 控制输出 (已按 out_max 限幅)
 *
 * @brief 为什么平衡环必须用这个 (2026-10-05, 参照 WL1)
 *   平衡环要抑制的是**车体在摆**, 而"摆得多快"直接就是陀螺仪读数。
 *   所以 D 项应该直接取角速度, 而不是对误差再求一次导:
 *
 *     误差微分型: d = (err - prev_err)/dt
 *                 = 对 (目标 - 姿态) 再求导 = 姿态的二阶导 + 目标的二阶导
 *                 → 目标阶跃时会产生**尖峰** (微分冲击)
 *                 → 姿态量化噪声被二次放大
 *
 *     角速度反馈型: d = rate
 *                 = 姿态的一阶导, 正好是我们要阻尼的物理量
 *                 → 目标阶跃不产生尖峰
 *                 → 噪声只被放一次
 *
 *   这也是 WL1 的做法: `float even_pwm = angle_pid.update(angle_target, angle.Pitch + bias)`
 *   其内部 D 项写成 `-kd * (measured - prev_actual)`, measured 就是被修正后的姿态角。
 *
 * @note  rate 必须是**俯仰方向**的角速度, 且符号与 err **同向**。
 *        本工程的符号约定: 前倾 (车头往下) 时 pitch 与角速度**都是负数**
 *        (实测: 车头下压时 pitch 从 -1.4° 走到 -22°, gyro 同为负)。
 *        符号由 balance.c 的 GYRO_PITCH_SIGN 参数负责, 本函数不判断方向。
 *
 * @note  ⚠ D 项符号 (2026-10-25 修正): 与 P 项**同号**, 即 out = kp·err + ki·i + **+** kd·rate。
 *        之前的负号是**负阻尼**, 车必然站不住 (详见 pid.c 实现处的推导):
 *          车前倾时轮子要往倒的方向追才接得住 → 轮加速度 a = K1·θ + K2·θ̇,
 *          闭环稳定要求 K2 > 0; D 项与 P 同号才有 K2 = k·kd > 0。
 *        直觉: 往前倒得越快, 轮子要追得越凶; "往回收"是帮着车继续倒。
 *
 * @note  ⚠ 本函数只管 D 与 P 的相对符号; "输出正 = 轮子往倒的方向追"
 *        由 balance.h 的 BAL_MOTOR_SIGN 决定 (实测为 +1)。
 *
 * @note  首次调用 (primed=false) 时跳过 D 项, 避免上电瞬间拿未初始化的
 *        历史值算出一个假的大阻尼冲量。
 */
float pid_update_rate(bal_pid_t *pid, float err, float rate, float dt);

#endif // PID_H
