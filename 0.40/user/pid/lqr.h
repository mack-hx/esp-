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
#define LQR_VMAX 20.0f     /* cm/s        期望回位速度上限 */
#define LQR_KP  15.0f      /* %/°         倾角 */
#define LQR_KD  0.4f       /* %/(°/s)     角速度 */
#define LQR_VEL_LO 2.0f    /* cm/s        |速度| 小于此值时速度项不参与 */
#define LQR_VEL_HI 5.0f    /* cm/s        |速度| 大于此值时速度项完全参与, 两者之间平滑过渡 */
#define LQR_ERR_DEADBAND  0.267f  /* cm   位置偏差死区 */
#define LQR_ERR_SOFT_BAND 0.533f  /* cm   死区软化过渡带宽度 */

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
 * @return 等效倾角 (°, 前倾为正), 不超过 ±POS_PID_LEAN_MAX
 * @note  期望回位速度 v_des: 偏差小时 = (KX/KV)·偏差 (线性), 偏差大时按
 *        sqrt(2·AMAX·偏差) 增长并限到 LQR_VMAX, 让车提前减速; 倾角 = (KX·偏差_等效 − KV·速度)/KP,
 *        其中 偏差_等效 = KV·v_des/KX; 速度先乘平滑增益 (|v| < LQR_VEL_LO 为 0, > LQR_VEL_HI 为 1)
 */
float lqr_pos_lean(float err_cm, float v_cmps);

#define LQR_POS_KI        0.5625f  /* °/(cm·s)  静止时位置偏差的积分速率 */
#define LQR_POS_I_MAX     3.0f   /* °         积分项上限 */
#define LQR_POS_I_VEL     4.0f   /* cm/s      |速度| 小于此值才累积 */
#define LQR_POS_I_BAND    6.0f   /* cm        |偏差| 小于此值才累积 */
#define LQR_POS_I_DECAY   0.3f   /* s         远离锚点或车在动时的衰减时间常数 */
#define LQR_POS_I_RELEASE 0.5f   /* s         锚点附近 (死区内/偏差变号) 且车慢时的衰减时间常数 */

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
 * @return 电机输出 (%), 未限幅
 */
float lqr_balance_out(float pitch_err, float gyro);

#endif /* LQR_H */
