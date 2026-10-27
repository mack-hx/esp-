/**
 * @file paramDef.js
 * @brief 1:1 镜像 ESP32 端 param.c 的参数定义 (17 项)
 * @date 2026-10-26
 *
 * 数据来源: 0.31/user/param/param.c (PARAM_DATA_VERSION 14)
 * 改了 param.c 就要同步这个文件, 否则 App 显示的滑块范围/默认值会跟车端对不上.
 *
 * ⚠ 2026-10-26 踩过的坑 (不要再犯):
 *   这份表曾经停在 v7 的默认值 (bal_kp 40 / bal_ki 12 / spd_kp 20 / spd_ki 30 /
 *   motor_max 30 / deadzone 8), 而车端早已改成 v9/v10 的新默认值
 *   (bal_kp 12 / bal_ki 0.5 / 速度环全 0 / motor_max 60 / deadzone 4)。
 *   用户在 App 上点"恢复默认"→"保存", 把这份旧默认值整份下发到车端:
 *   速度环被意外打开 (spd_kp=20/spd_ki=30, 站立时目标轮速却是 0),
 *   车被速度环一直往前推 → 只朝一个方向转, 根本稳不住。
 *
 *   所以:
 *   1) 本文件的默认值必须跟 param.c 的 g_param_items[] 完全一致
 *   2) "恢复默认"现在**不再下发本地默认值**, 而是让车端自己恢复
 *      (param_set 下发 {"_reset":true}, 车端调用 param_reset_default())
 *   3) 拉参数时会带回车端的 "_v" (参数表版本号), 跟 PARAM_VER 不一致就提示用户
 *
 * @note v14 (0.31) 的变更:
 *   - 减速比 gear 默认值 1.0 → 30 (编码器装在**电机轴**上, 减速箱 30:1;
 *     之前按 1.0 算, 轮速被算大 30 倍, 速度环一开就饱和, 车自己跑偏)
 *   - 新增 "走路目标轮速上限" move_rpm_max (id 16), 默认 100 rpm
 *   - 参数总数 16 → 17 项
 *   ⚠ NVS blob 的布局变了, 车端会自动作废旧数据 (param.c 的 PARAM_DATA_VERSION 已 +1)
 *
 * @note 0.31 收尾 (2026-10-26, 同一版内改动, 版本号不变):
 *   - spd_kp / spd_ki / spd_kd 的 step 从 0.5/0.5/0.1 改成 **0.01**
 *     (车端 param_set_float() 按 step 吸附, step 太粗会把中间值吸掉, App 上调不动)
 *   - step 只影响"怎么吸附", **不改变参数表布局和默认值** → PARAM_VER 仍然是 14,
 *     车端 PARAM_DATA_VERSION 也不动。App 和车端必须同步改 step, 否则
 *     App 拖 0.06、车端按 0.5 吸附 → 两边对不上。
 */

/** App 内置的参数表版本号, 必须跟 param.c 的 PARAM_DATA_VERSION 一致 */
export const PARAM_VER = 14

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
  { id: 2,  key: 'bal_kd',      label: 'Balance Kd', unit: '',  def: 1.5,   min: 0,  max: 30,   step: 0.1,  dec: 1, grp: 0,
    hint: '角速度阻尼。车晃得停不下来就加大, 出现高频嗡嗡声就减小' },

  // 速度环 (App 方向键就走这个环: 目标轮速 → 目标倾角 → 平衡环)
  // ⚠ 2026-10-26: 这三项的 step 从 0.5/0.5/0.1 改成 0.01。
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
    hint: '电机总输出上限。2026-10-25 起默认 60%: 方向修正后实测能自己站住, 但倾角大一点就要更大出力才接得住 (Kp=12 时 5° 就顶格)' },
  { id: 7,  key: 'spdout_max',  label: 'Speed Out Max', unit: '°', def: 40, min: 0, max: 100, step: 1, dec: 0, grp: 2,
    hint: '速度环能给出的最大倾角目标 (°)。车本来就有配平偏差时别调太大' },
  { id: 8,  key: 'ang_limit',   label: 'Angle Limit',   unit: '°', def: 25, min: 5, max: 60,  step: 1, dec: 0, grp: 2,
    hint: '超过这个倾角判定摔倒, 立刻停电机' },
  { id: 9,  key: 'gyro_lpf',    label: 'Gyro LPF',      unit: '',  def: 0.8, min: 0, max: 1,  step: 0.01, dec: 2, grp: 2,
    hint: '陀螺仪低通系数 (D 项用的角速度)。**越小越平滑但延迟越大, 延迟大了阻尼会变弱 → 一碰就剧烈抖**。默认 0.8 (时间常数 12ms): MPU 自带的 41Hz 数字低通已经够用, 这里再滤只是白白加延迟' },
  { id: 10, key: 'gear',        label: 'Gear Ratio',    unit: '',  def: 30.0, min: 0.1, max: 50, step: 0.1, dec: 1, grp: 2,
    hint: '电机转速 / 轮转速。本机编码器装在**电机轴**上, 减速箱 30:1, 所以填 30。填错会让轮速算错 30 倍 (速度环直接饱和, 车自己跑偏)' },
  { id: 11, key: 'deadzone',    label: 'Motor Deadband',unit: '%', def: 4,   min: 0, max: 30,   step: 1,   dec: 0, grp: 2,
    hint: '电机最小启动 PWM (硬件死区)。**要填电机真正能转起来的最小占空比**, 填小了补偿不够 (小输出时电机不转, 车会突然猛冲), 填大了会一直有底噪扭矩。2026-10-26 起补偿改成"加上"死区, 所以这个值填得越准越好' },
  { id: 12, key: 'arm_pitch',   label: 'Arm Pitch',     unit: '°', def: 8,   min: 1, max: 30,   step: 1,   dec: 0, grp: 2,
    hint: '启动门控: 倾角小于这个值才允许输出。防止歪着启动直接摔' },
  { id: 13, key: 'arm_rate',    label: 'Arm Rate',      unit: '°/s',def: 20, min: 2, max: 100,  step: 1,   dec: 0, grp: 2,
    hint: '启动门控: 角速度小于这个值才允许输出。防止推着车启动' },
  { id: 14, key: 'arm_time',    label: 'Arm Time',      unit: 'ms',def: 500, min: 0, max: 3000, step: 100, dec: 0, grp: 2,
    hint: '启动门控: 需要连续稳定这么久才真正输出。0=不等(不推荐)' },
  { id: 15, key: 'servo_c',     label: 'Servo Center',  unit: '°', def: 80, min: 30, max: 120, step: 1,   dec: 0, grp: 2,
    hint: '舵机中心角。也是配平表的参考角 (补偿在这一档为 0)' },
  { id: 16, key: 'move_rpm_max',label: 'Move RPM Max',  unit: 'rpm', def: 100, min: 0, max: 333, step: 1, dec: 1, grp: 2,
    hint: 'App 方向键 100% 档位对应的目标轮速 (rpm)。本机轮子最高转速 333 rpm, 默认取 100 (约 30%); 再大目标倾角会顶到 4° 安全阀, 车会左右猛摇。0 = 用代码里的默认值' }
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
