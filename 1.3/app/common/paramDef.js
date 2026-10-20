/**
 * @file paramDef.js
 * @brief 1:1 镜像 ESP32 端 param.c 的参数定义 (16 项)
 * @date 2026-10-09
 *
 * 数据来源: 1.3/user/param/param.c (PARAM_DATA_VERSION 7)
 * 改了 param.c 就要同步这个文件, 否则 App 显示的滑块范围/默认值会跟车端对不上.
 */

export const PARAM_GROUPS = {
  0: { key: 'balance', label: '平衡环' },
  1: { key: 'speed',   label: '速度环' },
  2: { key: 'limit',   label: '限幅 / 安全' }
}

export const PARAM_DEFS = [
  // 平衡环
  { id: 0,  key: 'bal_kp',      label: 'Balance Kp', unit: '',  def: 40.0,  min: 0,  max: 200,  step: 0.5,  dec: 1, grp: 0,
    hint: '越大越硬。太小扶不倒, 太大高频抖' },
  { id: 1,  key: 'bal_ki',      label: 'Balance Ki', unit: '',  def: 12.0,  min: 0,  max: 100,  step: 0.5,  dec: 1, grp: 0,
    hint: '消除配平残余偏差' },
  { id: 2,  key: 'bal_kd',      label: 'Balance Kd', unit: '',  def: 2.0,   min: 0,  max: 30,   step: 0.1,  dec: 1, grp: 0,
    hint: '角速度阻尼' },

  // 速度环
  { id: 3,  key: 'spd_kp',      label: 'Speed Kp',   unit: '',  def: 20.0,  min: 0,  max: 200,  step: 0.5,  dec: 1, grp: 1,
    hint: '走多快' },
  { id: 4,  key: 'spd_ki',      label: 'Speed Ki',   unit: '',  def: 30.0,  min: 0,  max: 300,  step: 0.5,  dec: 1, grp: 1,
    hint: '消除稳态速度误差' },
  { id: 5,  key: 'spd_kd',      label: 'Speed Kd',   unit: '',  def: 0.0,   min: 0,  max: 30,   step: 0.1,  dec: 1, grp: 1,
    hint: '速度变化阻尼' },

  // 限幅 / 安全
  { id: 6,  key: 'motor_max',   label: 'Motor Out Max', unit: '%',  def: 30, min: 0, max: 100, step: 1, dec: 0, grp: 2,
    hint: '电机总输出上限。**调试时宁小勿大**' },
  { id: 7,  key: 'spdout_max',  label: 'Speed Out Max', unit: '°', def: 40, min: 0, max: 100, step: 1, dec: 0, grp: 2,
    hint: '速度环能给出的最大倾角目标' },
  { id: 8,  key: 'ang_limit',   label: 'Angle Limit',   unit: '°', def: 25, min: 5, max: 60,  step: 1, dec: 0, grp: 2,
    hint: '超过这个倾角判定摔倒, 立刻停电机' },
  { id: 9,  key: 'gyro_lpf',    label: 'Gyro LPF',      unit: '',  def: 0.2, min: 0, max: 1,  step: 0.01, dec: 2, grp: 2,
    hint: '陀螺仪低通滤波' },
  { id: 10, key: 'encoder_gear',label: 'Encoder Gear',  unit: '',  def: 30,  min: 1, max: 200, step: 1,  dec: 0, grp: 2,
    hint: '电机转速/轮转速减速比' },
  { id: 11, key: 'motor_db',    label: 'Motor Deadband',unit: '%', def: 5,   min: 0, max: 30,  step: 0.5, dec: 1, grp: 2,
    hint: '电机最小启动 PWM' },
  { id: 12, key: 'arm_pitch',   label: 'Arm Pitch',     unit: '°', def: 5,   min: 1, max: 30,  step: 0.5, dec: 1, grp: 2,
    hint: '允许的最大倾角' },
  { id: 13, key: 'arm_rate',    label: 'Arm Rate',      unit: '°/s',def: 30, min: 5, max: 200, step: 1,  dec: 0, grp: 2,
    hint: '允许的最大角速度' },
  { id: 14, key: 'arm_time',    label: 'Arm Time',      unit: 'ms',def: 500, min: 100, max: 5000, step: 50, dec: 0, grp: 2,
    hint: '需要连续稳定的时长' },
  { id: 15, key: 'servo_ctr',   label: 'Servo Center',  unit: '°', def: 90,  min: 0, max: 180, step: 1, dec: 0, grp: 2,
    hint: '舵机中心角' }
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
