/**
 * @file lqr.h
 * @brief LQR 状态反馈: u = -K·x, x = [位置, 平移速度, 倾角, 角速度]
 * @author ESP32 平衡轮腿机器人
 */

#ifndef LQR_H
#define LQR_H

/* 增益 K (u = K·x, x 的位置/速度为"当前位置 − 锚点") */
#define LQR_KX  (-5.0f)    /* %/cm        位置偏差 */
#define LQR_KV  (-2.25f)   /* %/(cm/s)    平移速度 */
#define LQR_AMAX 8.0f      /* cm/s²       回位减速度上限: 期望回位速度 = sqrt(2·AMAX·偏差) */
#define LQR_VMAX 30.0f     /* cm/s        期望回位速度上限 */
#define LQR_KP  18.0f      /* %/°         倾角 */
#define LQR_KD  0.9f       /* %/(°/s)     角速度默认值 (运行时用参数表 lqr_kd, 两处默认值须一致)
                            * 1.5 在 8~11Hz 会自激 (陀螺串陷波时表现为陷波裙边上的等幅抖),
                            * 0.5 在站姿明显欠阻尼 (2Hz 大晃动); 1.1 是两者的折中 */
#define LQR_VEL_LO 2.0f    /* cm/s        |速度| 小于此值时速度项不参与 */
#define LQR_VEL_HI 5.0f    /* cm/s        |速度| 大于此值时速度项完全参与, 两者之间平滑过渡 */
#define LQR_ERR_DEADBAND  0.267f  /* cm   位置偏差死区 */
#define LQR_ERR_SOFT_BAND 0.533f  /* cm   死区软化过渡带宽度 */
/* 目标倾角上限。别调大: 车速高时速度项要求的刹车倾角一旦超过车身能摆到的角度,
 * 内环会朝反方向饱和 (想后仰 → 向前驱动) → 车越跑越快 → 摔。 */
#define LQR_LEAN_MAX      8.0f    /* °    位置环给平衡环的最大倾角目标 (含死区补偿) */
/* 常规回位/刹车时的倾角需求上限。实测这台车腿部舵机会吃掉轮子的反作用扭矩:
 * out < 50% 时车身几乎不后仰, 于是"要求 5~6° 后仰"只会变成持续的向前驱动,
 * 车速被自己推到 130~230rpm 后摔车。封在 3.5° 以内, 输出停在 ~25%, 不会失控。 */
#define LQR_LEAN_SOFT_MAX 8.0f    /* °    常规倾角需求上限 (静摩擦补偿可以突破) */

/**
 * @brief 设置位置项增益 (覆盖 LQR_KX / LQR_KV / LQR_AMAX 的默认值)
 * @param kx   位置偏差增益 (%/cm, 正值, 小于 0 按 0)
 * @param kv   平移速度增益 (%/(cm/s), 正值, 小于 0 按 0)
 * @param amax 回位减速度上限 (cm/s², 正值, 小于等于 0 表示不限制回位速度)
 */
void lqr_set_pos_gains(float kx, float kv, float amax);

/**
 * @brief 位置项折算成等效倾角
 * @param err_cm  锚点位置 − 当前位置 (cm, 前进为正)
 * @param v_cmps  平移速度 (cm/s, 前进为正)
 * @return 等效倾角 (°, 前倾为正), 不超过 ±LQR_LEAN_MAX
 * @note  期望回位速度 v_des: 偏差小时 = (KX/KV)·偏差 (线性), 偏差大时按
 *        sqrt(2·AMAX·偏差) 增长并限到 LQR_VMAX, 让车提前减速; 倾角 = (KX·偏差_等效 − KV·速度)/KP,
 *        其中 偏差_等效 = KV·v_des/KX; 速度先乘平滑增益 (|v| < LQR_VEL_LO 为 0, > LQR_VEL_HI 为 1)
 */
float lqr_pos_lean(float err_cm, float v_cmps);
float lqr_desired_speed(float err_cm);

/* 轮速阻尼: 位置环的刹车不再走倾角 (摆倾角追不上, 车速反而被推高),
 * 超出期望回位速度的部分直接在电机输出上反向拉 */
#define LQR_KW            1.0f    /* %/(cm/s) */
#define LQR_POS_KI        0.5625f  /* °/(cm·s)  静止时位置偏差的积分速率 */
#define LQR_POS_I_MAX     5.0f   /* °         积分项上限 (和比例项加起来不超过 LEAN_MAX) */
#define LQR_POS_I_VEL     4.0f   /* cm/s      |速度| 小于此值才按静止处理 */
/* |偏差| 小于此值才累积。必须覆盖"回位后停住"的偏差范围:
 * 比例项在 30cm 偏差时只给约 3° 倾角, 正好落在轮胎平斑的静稳区里,
 * 车会刹在原地不动 (实测 err=29cm、out≈0、电机进 BRAKE)。积分要把
 * 目标倾角顶到 5° 以上才能推出去, 所以这里不能设小。 */
#define LQR_POS_I_BAND    6.0f  /* cm        |偏差| 小于此值才累积 */
#define LQR_POS_I_DECAY   0.3f   /* s         远离锚点或车在动时的衰减时间常数 */
#define LQR_POS_I_RELEASE 0.5f   /* s         锚点附近 (死区内/偏差变号) 且车慢时的衰减时间常数 */

/* ========== 静摩擦死区补偿 ==========
 * 轮胎静摩擦 + 平斑让 |目标倾角| 小于约 3.5° 时车推不动: 车会"斜着站住",
 * 比例项(1~3°)和积分项都在这个死区里, 表现就是回不到位 / 停顿后才动。
 * 有偏差且车基本静止时, 把目标倾角直接顶到 LQR_BREAK_LEAN, 一动起来立刻撤回。 */
/* ========== 位置直接反馈 (不经过倾角目标, 见 balance_update_position) ==========
 * 输出 = DX·软化偏差 + DV·(v_des - vel), v_des = DV_SLOPE·|偏差| (限 VMAX)。
 * 封顶 ±DIRECT_MAX, 剩余输出留给直立环, 保证车身姿态永远优先于位置。 */
#define BAL_POS_DX          1.0f    /* %/cm      位置偏差直接增益 */
#define BAL_POS_DV          0.5f    /* %/(cm/s)  速度误差直接增益 (刹车主力) */
#define BAL_POS_DIRECT_MAX  15.0f   /* %         直接分量上限 (直立环还有 ±100%) */
#define LQR_DV_SLOPE        0.5f    /* 1/s       期望回位速度斜率 */

#define LQR_BREAK_ERR     2.5f   /* cm    偏差超过它才补 */
#define LQR_BREAK_VEL     3.0f   /* cm/s  速度低于它才算卡住 */
#define LQR_BREAK_LEAN    5.0f   /* °     补到的目标倾角 (必须大于静摩擦死区) */

/**
 * @brief 位置积分项: 车几乎不动但仍有位置偏差时, 逐渐加大倾角把车推出静摩擦
 * @param dt      控制周期 (秒)
 * @param err_cm  锚点位置 − 当前位置 (cm, 前进为正)
 * @param v_cmps  平移速度 (cm/s, 前进为正)
 * @return 附加倾角 (°, 前倾为正, 与 lqr_pos_lean 同号约定), 范围 ±LQR_POS_I_MAX
 * @note  仅当 LQR_ERR_DEADBAND < |偏差| < LQR_POS_I_BAND、|速度| < LQR_POS_I_VEL 且偏差与积分同号时累积;
 *       其余情况按指数衰减 (锚点附近 LQR_POS_I_RELEASE, 其他 LQR_POS_I_DECAY), 不瞬间清零
 */
float lqr_pos_integral(float dt, float err_cm, float v_cmps);

/** @brief 清零位置积分项 */
void lqr_pos_integral_reset(void);

/**
 * @brief 倾角项与角速度项
 * @param pitch_err 倾角 − 目标倾角 (°)
 * @param gyro      角速度 (°/s)
 * @param kp_scale  P 通道倍率 (蹲姿调度用, 站姿传 1.0)
 * @param kd_scale  D 通道倍率 (蹲姿调度用, 站姿传 1.0)
 * @return 电机输出 (%), 未限幅
 * @note  P / D 分开传倍率: 蹲姿要保刚度 (P 少缩) 又要防自激 (D 多缩)
 */
float lqr_balance_out(float pitch_err, float gyro, float kp_scale, float kd_scale);

#endif /* LQR_H */
