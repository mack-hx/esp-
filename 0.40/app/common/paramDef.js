/**
 * @file paramDef.js
 * @brief 1:1 镜像 ESP32 端 param.c 的参数定义
 *
 * 数据来源: 0.40/user/param/param.c (PARAM_DATA_VERSION 24)
 * 改了 param.c 就要同步这个文件, 否则 App 显示的滑块范围/默认值会跟车端对不上.
 *
 * 约定:
 *   1) 本文件的默认值必须跟 param.c 的 g_param_items[] 完全一致
 *   2) "恢复默认"**不下发本地默认值**, 而是让车端自己恢复
 *      (param_set 下发 {"_reset":true}, 车端调用 param_reset_default())
 *   3) 拉参数时会带回车端的 "_v" (参数表版本号), 跟 PARAM_VER 不一致就提示用户
 *   4) step 只影响"怎么吸附", 不改变参数表布局和默认值;
 *      App 和车端必须同步改 step, 否则 App 拖 0.06、车端按 0.5 吸附 → 两边对不上
 */

/** App 内置的参数表版本号, 必须跟 param.c 的 PARAM_DATA_VERSION 一致 */
export const PARAM_VER = 24

export const PARAM_GROUPS = {
  0: { key: 'balance', label: '平衡环' },
  1: { key: 'speed',   label: '速度环' },
  2: { key: 'limit',   label: '限幅 / 安全' }
}

export const PARAM_DEFS = [
  // 平衡环
  { id: 0,  key: 'bal_kp',      label: 'Balance Kp', unit: '',  def: 12.0,  min: 0,  max: 200,  step: 0.5,  dec: 1, grp: 0,
    hint: '越大越硬。太小扶不倒, 太大高频抖。这是**最需要真机试出来的**参数' },
  { id: 1,  key: 'bal_ki',      label: 'Balance Ki', unit: '',  def: 0.5,   min: 0,  max: 100,  step: 0.5,  dec: 1, grp: 0,
    hint: '消除配平残余偏差。太大容易积分饱和 (车会先猛冲再回不来)' },
  { id: 2,  key: 'bal_kd',      label: 'Balance Kd', unit: '',  def: 0.4,   min: 0,  max: 30,   step: 0.1,  dec: 1, grp: 0,
    hint: '角速度阻尼。⚠ 默认 0.4, **不要再加大**: >1 会激起 8~12Hz 满幅打摆自激 (真机实测: kd=1.5 时输出常驻 ±60% 打摆, kd=0.4 时输出标准差 0.12%, 死稳)。想找回阻尼余量要给 D 路加陷波, 不是加大 kd' },

  // 速度环 (App 方向键就走这个环: 目标轮速 → 目标倾角 → 平衡环)
  //    车端 param_set_float() 会把下发值吸附到 step 的整数倍, step=0.5 时
  //    想设 0.06 → 0.0, 想设 0.3 → 0.5 —— App 上拖出来的中间值一个都设不进去
  //    (静默失效: 车端有兜底常量 0.06, 看起来"还能用", 但用户调不动)。
  //    step 只是"怎么吸附", 不影响参数表布局 → PARAM_VER 仍然是 14, 不用动。
  { id: 3,  key: 'spd_kp',      label: 'Speed Kp',   unit: '',  def: 0.06,  min: 0,  max: 200,  step: 0.01, dec: 2, grp: 1,
    hint: '走多快 (°/rpm)。App 方向键走的就是这个环: 目标轮速 → 目标倾角 → 平衡环 (平衡环全程不撒手)。调大走得快, 但目标倾角被限制在 4° 以内 (安全阀), 调太大只会让车左右猛摇。步进 0.01 —— 步进太粗的话 App 拖出来的中间值会被吸附掉' },
  { id: 4,  key: 'spd_ki',      label: 'Speed Ki',   unit: '',  def: 0.01,  min: 0,  max: 300,  step: 0.01, dec: 2, grp: 1,
    hint: '消除稳态速度误差 (走不到目标速度时慢慢补倾角)。太大容易积分饱和, 也会让车越走越歪。步进 0.01 (步进太粗的话中间值会被吸附掉)' },
  { id: 5,  key: 'spd_kd',      label: 'Speed Kd',   unit: '',  def: 0.0,   min: 0,  max: 30,   step: 0.01, dec: 2, grp: 1,
    hint: '速度变化阻尼。默认 0: 编码器量化台阶会让 D 项噪声很大。步进 0.01' },

  // 限幅 / 安全
  { id: 6,  key: 'motor_max',   label: 'Motor Out Max', unit: '%',  def: 60, min: 0, max: 100, step: 1, dec: 0, grp: 2,
    hint: '电机总输出上限。默认 60%: 方向修正后实测能自己站住, 但倾角大一点就要更大出力才接得住 (Kp=12 时 5° 就顶格)' },
  { id: 7,  key: 'spdout_max',  label: 'Speed Out Max', unit: '°', def: 40, min: 0, max: 100, step: 1, dec: 0, grp: 2,
    hint: '速度环能给出的最大倾角目标 (°)。车本来就有配平偏差时别调太大' },
  { id: 8,  key: 'ang_limit',   label: 'Angle Limit',   unit: '°', def: 25, min: 5, max: 60,  step: 1, dec: 0, grp: 2,
    hint: '超过这个倾角判定摔倒, 立刻停电机' },
  { id: 9,  key: 'gyro_lpf',    label: 'Gyro LPF',      unit: '',  def: 0.8, min: 0, max: 1,  step: 0.01, dec: 2, grp: 2,
    hint: '陀螺仪低通系数 (D 项用的角速度)。**越小越平滑但延迟越大, 延迟大了阻尼会变弱 → 一碰就剧烈抖**。默认 0.8 (时间常数 12ms): MPU 自带的 41Hz 数字低通已经够用, 这里再滤只是白白加延迟' },
  { id: 10, key: 'gear',        label: 'Gear Ratio',    unit: '',  def: 30.0, min: 0.1, max: 50, step: 0.1, dec: 1, grp: 2,
    hint: '电机转速 / 轮转速。本机编码器装在**电机轴**上, 减速箱 30:1, 所以填 30。填错会让轮速算错 30 倍 (速度环直接饱和, 车自己跑偏)' },
  { id: 11, key: 'deadzone',    label: 'Motor Deadband',unit: '%', def: 4,   min: 0, max: 30,   step: 1,   dec: 0, grp: 2,
    hint: '电机最小启动 PWM (硬件死区)。**要填电机真正能转起来的最小占空比**, 填小了补偿不够 (小输出时电机不转, 车会突然猛冲), 填大了会一直有底噪扭矩。补偿方式是"加上"死区, 所以这个值填得越准越好' },
  { id: 12, key: 'arm_pitch',   label: 'Arm Pitch',     unit: '°', def: 8,   min: 1, max: 30,   step: 1,   dec: 0, grp: 2,
    hint: '启动门控: 倾角小于这个值才允许输出。防止歪着启动直接摔' },
  { id: 13, key: 'arm_rate',    label: 'Arm Rate',      unit: '°/s',def: 20, min: 2, max: 100,  step: 1,   dec: 0, grp: 2,
    hint: '启动门控: 角速度小于这个值才允许输出。防止推着车启动' },
  { id: 14, key: 'arm_time',    label: 'Arm Time',      unit: 'ms',def: 500, min: 0, max: 3000, step: 100, dec: 0, grp: 2,
    hint: '启动门控: 需要连续稳定这么久才真正输出。0=不等(不推荐)' },
  { id: 15, key: 'servo_c',     label: 'Servo Center',  unit: '°', def: 80, min: 30, max: 120, step: 1,   dec: 0, grp: 2,
    hint: '舵机中心角。也是配平表的参考角 (补偿在这一档为 0)' },
  { id: 16, key: 'move_rpm_max',label: 'Move RPM Max',  unit: 'rpm', def: 100, min: 0, max: 333, step: 1, dec: 1, grp: 2,
    hint: 'App 方向键 100% 档位对应的目标轮速 (rpm)。本机轮子最高转速 333 rpm, 默认取 100 (约 30%); 再大目标倾角会顶到 4° 安全阀, 车会左右猛摇。0 = 用代码里的默认值' },

  // 里程计 / 位置环
  { id: 17, key: 'wheel_dia',   label: 'Wheel Diameter', unit: 'cm', def: 6.5, min: 1, max: 30, step: 0.1, dec: 1, grp: 2,
    hint: '**轮子直径** (cm)。里程计靠它把编码器脉冲换算成位移: 距离 = 脉冲数 / (44 × gear) × π × 轮径。本机实测 6.5 cm。填错会让位置反馈整体缩放错误 (位置环纠偏力度不对)' },
  { id: 18, key: 'pos_kp',      label: 'Position Kp',    unit: '(cm/s)/cm', def: 2.0, min: 0, max: 3, step: 0.05, dec: 2, grp: 1,
    hint: '位置环比例增益 ((cm/s)/cm): 位置偏差 1 cm 时, 要求车**走多快** (cm/s)。位置环输出的是**目标速度**, 不是倾角 —— 速度环 (hold_kp) 再把速度误差换成目标倾角, 平衡环只管跟住倾角。三个环各管一件事, 不会互相拖。**默认 2.0** (纯 P)。这个增益决定"推了有多大的阻力": 太小 → 偏差 2 cm 也只换 0.6° 倾角, 低于电机死区, 轮子根本不转 (车回不来); 太大 → 回位太快冲过头。一次加 0.05, 推一下车看效果' },
  { id: 19, key: 'pos_ki',      label: 'Position Ki',    unit: '(cm/s)/(cm·s)', def: 0,  min: 0, max: 5, step: 0.01, dec: 2, grp: 1,
    hint: '位置环积分增益: 消除站住之后慢慢偏这种位置漂移。**默认 0, 而且建议一直保持 0**: 位置环再带积分, 加上车本身的偏置, 很容易攒出一个常驻速度 → 车一直往一个方向走。步进 0.01' },
  { id: 20, key: 'hold_kp',     label: 'Hold Kp',        unit: '°/(cm/s)', def: 0.5, min: 0, max: 3, step: 0.05, dec: 2, grp: 1,
    hint: '锁位速度环比例增益 (°/(cm/s)): 平移速度差 1 cm/s 时, 要求车身**前倾多少度**。这是位置保持的主力环 —— 它给平衡环的倾角越小, 车越稳; 太大就冲过头再倒回来。**默认 0.5**。反馈用的是**里程计速度** (60ms 滑动窗口, 每 10ms 更新), 不是编码器轮速 (300ms 窗口): 后者滞后太多, 阻尼相位落后 → 等价负阻尼 → 前后大幅晃。步进 0.05' },
  { id: 21, key: 'hold_ki',     label: 'Hold Ki',        unit: '°/cm', def: 0,  min: 0, max: 2, step: 0.01, dec: 2, grp: 1,
    hint: '锁位速度环积分增益 (°/cm): 攒出站住不动时的**常驻倾角** (抗摩擦/抗轮子阻力), 顺带消掉稳态速度误差。**默认 0**: 配大了会攒出一个长驻倾角 → 车一直往一个方向走; 真机上确认有稳态速度误差, 再从 0 往上加。步进 0.01' }
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
