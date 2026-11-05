/**
 * @file paramDef.js
 * @brief 1:1 镜像 ESP32 端 param.c 的参数定义
 *
 * 数据来源: main_car/user/param/param.c (PARAM_DATA_VERSION 39)
 * 改了 param.c 就要同步这个文件, 否则 App 显示的滑块范围/默认值会跟车端对不上.
 *
 * 约定:
 *   1) 本文件的默认值必须跟 param.c 的 g_param_items[] 完全一致
 *   2) "恢复默认"**不下发本地默认值**, 而是让车端自己恢复
 *      (param_set 下发 {"_reset":true}, 车端调用 param_reset_default())
 *   3) 拉参数时会带回车端的 "_v" (参数表版本号), 跟 PARAM_VER 不一致就提示用户
 *   4) step 只影响"怎么吸附", 不改变参数表布局和默认值;
 *      App 和车端必须同步改 step, 否则 App 拖 0.06、车端按 0.5 吸附 → 两边对不上
 *
 * 版本说明:
 * 39: 删除 bal_kp/bal_ki/bal_kd 与 pos_kp/pos_kd/pos_ki (直立环由 LQR 接管, 这些滑块不生效)
 */

/** App 内置的参数表版本号, 必须跟 param.c 的 PARAM_DATA_VERSION 一致 */
export const PARAM_VER = 39

export const PARAM_GROUPS = {
  0: { key: 'speed', label: '速度环 / 位置环' },
  1: { key: 'limit', label: '限幅 / 安全' }
}

export const PARAM_DEFS = [
  // 速度环 (App 方向键就走这个环: 目标轮速 → 目标倾角 → 平衡环)
  //    车端 param_set_float() 会把下发值吸附到 step 的整数倍, step=0.5 时
  //    想设 0.06 → 0.0, 想设 0.3 → 0.5 —— App 上拖出来的中间值一个都设不进去
  //    (静默失效: 车端有兜底常量 0.06, 看起来"还能用", 但用户调不动)。
  { id: 0,  key: 'spd_kp',      label: 'Speed Kp',   unit: '',  def: 0.08,  min: 0,  max: 200,  step: 0.01, dec: 2, grp: 0,
    hint: '走多快 (°/rpm)。App 方向键走的就是这个环: 目标轮速 → 目标倾角 → 平衡环 (平衡环全程不撒手)。调大走得快, 但目标倾角被限制在 10° 以内 (安全阀), 调太大只会让车左右猛摇。步进 0.01 —— 步进太粗的话 App 拖出来的中间值会被吸附掉' },
  { id: 1,  key: 'spd_ki',      label: 'Speed Ki',   unit: '',  def: 0.04,  min: 0,  max: 300,  step: 0.01, dec: 2, grp: 0,
    hint: '消除稳态速度误差 (走不到目标速度时慢慢补倾角)。太大容易积分饱和, 也会让车越走越歪。步进 0.01 (步进太粗的话中间值会被吸附掉)' },
  { id: 2,  key: 'spd_kd',      label: 'Speed Kd',   unit: '',  def: 0.0,   min: 0,  max: 30,   step: 0.01, dec: 2, grp: 0,
    hint: '速度变化阻尼。默认 0: 编码器量化台阶会让 D 项噪声很大。步进 0.01' },

  // 限幅 / 安全
  { id: 3,  key: 'motor_max',   label: 'Motor Out Max', unit: '%',  def: 95, min: 0, max: 100, step: 1, dec: 0, grp: 1,
    hint: '电机总输出上限。默认 95%: 大角度纠偏需要全力输出, 留 5% 给转向差速' },
  { id: 4,  key: 'spdout_max',  label: 'Speed Out Max', unit: '°', def: 40, min: 0, max: 100, step: 1, dec: 0, grp: 1,
    hint: '速度环能给出的最大倾角目标 (°)。车本来就有配平偏差时别调太大' },
  { id: 5,  key: 'ang_limit',   label: 'Angle Limit',   unit: '°', def: 35, min: 5, max: 60,  step: 1, dec: 0, grp: 1,
    hint: '超过这个倾角判定摔倒, 立刻停电机。大角度纠偏时放宽到 35°, 留住电机出力机会' },
  { id: 6,  key: 'gyro_lpf',    label: 'Gyro LPF',      unit: '',  def: 0.8, min: 0, max: 1,  step: 0.01, dec: 2, grp: 1,
    hint: '陀螺仪低通系数 (D 项用的角速度)。**越小越平滑但延迟越大, 延迟大了阻尼会变弱 → 一碰就剧烈抖**。默认 0.8 (时间常数 12ms): MPU 自带的 41Hz 数字低通已经够用, 这里再滤只是白白加延迟' },
  { id: 7,  key: 'gear',        label: 'Gear Ratio',    unit: '',  def: 30.0, min: 0.1, max: 50, step: 0.1, dec: 1, grp: 1,
    hint: '电机转速 / 轮转速。本机编码器装在**电机轴**上, 减速箱 30:1, 所以填 30。填错会让轮速算错 30 倍 (速度环直接饱和, 车自己跑偏)' },
  { id: 8,  key: 'deadzone',    label: 'Motor Deadband',unit: '%', def: 4,   min: 0, max: 30,   step: 1,   dec: 0, grp: 1,
    hint: '电机最小启动 PWM (硬件死区)。**要填电机真正能转起来的最小占空比**, 填小了补偿不够 (小输出时电机不转, 车会突然猛冲), 填大了会一直有底噪扭矩。补偿方式是"加上"死区, 所以这个值填得越准越好' },
  { id: 9,  key: 'arm_pitch',   label: 'Arm Pitch',     unit: '°', def: 12,  min: 1, max: 30,   step: 1,   dec: 0, grp: 1,
    hint: '启动门控: 倾角小于这个值才允许输出。大角度扶正后能直接接住, 不用扶到很正' },
  { id: 10, key: 'arm_rate',    label: 'Arm Rate',      unit: '°/s',def: 20, min: 2, max: 100,  step: 1,   dec: 0, grp: 1,
    hint: '启动门控: 角速度小于这个值才允许输出。防止推着车启动' },
  { id: 11, key: 'arm_time',    label: 'Arm Time',      unit: 'ms',def: 500, min: 0, max: 3000, step: 100, dec: 0, grp: 1,
    hint: '启动门控: 需要连续稳定这么久才真正输出。0=不等(不推荐)' },
  { id: 12, key: 'servo_c',     label: 'Servo Center',  unit: '°', def: 100, min: 30, max: 120, step: 1,   dec: 0, grp: 1,
    hint: '舵机中心角。也是配平表的参考角 (补偿在这一档为 0)' },
  { id: 13, key: 'move_rpm_max',label: 'Move RPM Max',  unit: 'rpm', def: 120, min: 0, max: 333, step: 1, dec: 1, grp: 1,
    hint: 'App 方向键 100% 档位对应的目标轮速 (rpm)。本机轮子最高转速 333 rpm, 默认取 120 (约 36%); 再大目标倾角会顶到 10° 安全阀, 车会左右猛摇。0 = 用代码里的默认值' },

  // 里程计 / 位置环
  { id: 14, key: 'wheel_dia',   label: 'Wheel Diameter', unit: 'cm', def: 6.5, min: 1, max: 30, step: 0.1, dec: 1, grp: 1,
    hint: '**轮子直径** (cm)。里程计靠它把编码器脉冲换算成位移: 距离 = 脉冲数 / (44 × gear) × π × 轮径。本机实测 6.5 cm。填错会让位置反馈整体缩放错误 (位置环纠偏力度不对)' },
  { id: 15, key: 'lqr_kx',      label: 'LQR Kx',         unit: '%/cm', def: 5, min: 0, max: 20, step: 0.25, dec: 2, grp: 0,
    hint: 'LQR 位置偏差增益 (%/cm): 站立时位置偏差 1 cm 对应的回位力度, 越大回位越硬。回位冲过头 → 减小; 回位太慢 → 加大。步进 0.25' },
  { id: 16, key: 'lqr_kv',      label: 'LQR Kv',         unit: '%/(cm/s)', def: 2.25, min: 0, max: 10, step: 0.125, dec: 3, grp: 0,
    hint: 'LQR 平移速度增益 (%/(cm/s)): 回位时的速度阻尼(刹车)。推完来回摆多次 → 加大; 车身高频抖或自己振起来 → 减小。步进 0.125' },
  { id: 17, key: 'lqr_amax',    label: 'LQR 回位减速度', unit: 'cm/s²', def: 8, min: 0, max: 150, step: 1, dec: 0, grp: 0,
    hint: '回位减速度上限 (cm/s²): 偏差大时期望回位速度 = sqrt(2·值·偏差), 车会提前减速, 到位时速度接近 0。回位冲过头 → 减小; 回位太慢 → 加大。0 = 不限制回位速度 (退回纯线性)。步进 1' }
]

/** 用 key 找 def */
export function getDefByKey(key) {
  return PARAM_DEFS.find(d => d.key === key)
}

/** 初始化默认值 (在 App 启动时调) */
export function getDefaultValues() {
  const obj = {}
  PARAM_DEFS.forEach(d => { obj[d.key] = d.def })
  return obj
}
