/**
 * @file onenet_model.js
 * @brief OneNet 物模型定义 + 工具函数
 * @date 2026-10-10
 *
 * 物模型来源: OneNet Studio (Masa 平台)
 *   - 文档: https://open.iot.10086.cn/doc/iot_platform/book/device-connect&manager/thing-model/
 *   - 数据点 topic:  $sys/{pid}/{device-name}/thing/property/post
 *   - 属性设置 topic: $sys/{pid}/{device-name}/thing/property/set
 *   - 属性获取 topic: $sys/{pid}/{device-name}/thing/property/get
 *   - 服务调用 topic: $sys/{pid}/{device-name}/thing/service/{identifier}/invoke
 *   - 服务应答 topic: $sys/{pid}/{device-name}/thing/service/{identifier}/invoke/reply
 *   - 事件上报 topic: $sys/{pid}/{device-name}/thing/event/post
 *
 * 配套文件:
 *   - common/onenet.js      token 计算 + HTTP API 封装
 *   - common/topic.js       topic 字符串集中出口
 *   - app/onenet_物模型.md  物模型在 OneNet 后台的手工建法
 */

/**
 * @brief 物模型属性定义 (云端和设备端共用的字段表)
 *
 * OneNet 物模型属性 JSON 格式:
 *   {
 *     "id": "1",              // 消息 id, 13 位数字字符串
 *     "version": "1.0",
 *     "params": {
 *       "pitch":   { "value": 0.5,   "time": 1700000000000 },
 *       "voltage": { "value": 7.32,  "time": 1700000000000 }
 *     }
 *   }
 */
export const PROPERTIES = {
  // ---- 姿态 (设备 → 云, 只读属性) ----
  pitch:    { type: 'float', access: 'r', desc: '俯仰角 (°)' },
  roll:     { type: 'float', access: 'r', desc: '横滚角 (°)' },
  yaw:      { type: 'float', access: 'r', desc: '偏航角 (°)' },
  gz:       { type: 'float', access: 'r', desc: 'Z 轴角速度 (°/s)' },

  // ---- 电机 ----
  motor_l:  { type: 'int',   access: 'r', desc: '左轮电机输出 (-100~100, %)' },
  motor_r:  { type: 'int',   access: 'r', desc: '右轮电机输出 (-100~100, %)' },
  speed:    { type: 'float', access: 'r', desc: '车体速度 (m/s)' },
  enc_l:    { type: 'int',   access: 'r', desc: '左编码器原始计数' },
  enc_r:    { type: 'int',   access: 'r', desc: '右编码器原始计数' },

  // ---- 电池 ----
  voltage:  { type: 'float', access: 'r', desc: '电池电压 (V)' },
  percent:  { type: 'int',   access: 'r', desc: '电池电量百分比 (0~100)' },

  // ---- 设备状态 ----
  state:    { type: 'string',access: 'r', desc: '状态机: IDLE/STANDBY/BALANCING/FALLEN' },
  uptime:   { type: 'int',   access: 'r', desc: '设备启动时长 (秒)' },

  // ---- 移动指令 (云 → 设备, 高频写入) ----
  vx:       { type: 'float', access: 'rw',desc: '前后速度目标 (-1.0~1.0, 标幺)' },
  vy:       { type: 'float', access: 'rw',desc: '左右平移目标 (预留, 当前未用)' },
  w:        { type: 'float', access: 'rw',desc: '转向角速度目标 (-1.0~1.0, 标幺)' }
}

/**
 * @brief 物模型服务定义 (云主动调用设备, 设备应答)
 */
export const SERVICES = {
  // ---- 单次动作类 ----
  stop: {
    desc: '急停 (清 PID 积分 + 电机刹车)',
    in:  [],
    out: [{ key: 'ok',     type: 'bool'  }]
  },
  stand: {
    desc: '起立 (让车从倒下恢复到 STANDBY)',
    in:  [],
    out: [{ key: 'ok',     type: 'bool'  }]
  },
  calibrate: {
    desc: '配平标定 (用当前姿态作新平衡零点)',
    in:  [],
    out: [{ key: 'ok',     type: 'bool' }]
  },

  // ---- 参数读写 ----
  param_get: {
    desc: '读全部 PID 参数',
    in:  [],
    out: [{ key: 'values', type: 'object' }]
  },
  param_set: {
    desc: '写全部 PID 参数',
    in:  [{ key: 'values', type: 'object' }],
    out: [{ key: 'ok',     type: 'bool' }]
  },
  param_save: {
    desc: '参数落盘 (NVS)',
    in:  [],
    out: [{ key: 'ok',     type: 'bool' }]
  }
}

/**
 * @brief 物模型事件定义 (设备主动上报的离散事件)
 */
export const EVENTS = {
  fallen: {
    desc: '车摔倒了',
    out: [{ key: 'pitch', type: 'float' }, { key: 'time', type: 'int' }]
  },
  armed: {
    desc: '从 STANDBY 进入 BALANCING',
    out: [{ key: 'time', type: 'int' }]
  },
  error: {
    desc: '设备异常 (IMU 失联 / 编码器异常等)',
    out: [{ key: 'code', type: 'int' }, { key: 'msg', type: 'string' }]
  }
}

/**
 * @brief 构造属性上报 payload
 * @param {Object} values 形如 { pitch: 0.5, voltage: 7.32, ... }
 * @returns {Object} OneNet 物模型属性上报 JSON
 */
export function buildPropertyPost(values) {
  const now = Date.now()
  const params = {}
  for (const key in values) {
    if (!(key in PROPERTIES)) continue
    const v = values[key]
    if (v === null || v === undefined) continue
    params[key] = { value: v, time: now }
  }
  return {
    id: String(Date.now() % 10000000000000).padStart(13, '0'),
    version: '1.0',
    params
  }
}

/**
 * @brief 构造事件上报 payload
 * @param {string} name 事件名 (在 EVENTS 里)
 * @param {Object} outData 事件输出字段
 */
export function buildEventPost(name, outData = {}) {
  const ev = EVENTS[name]
  if (!ev) throw new Error(`Unknown event: ${name}`)
  const params = {}
  ev.out.forEach(f => {
    params[f.key] = { value: outData[f.key], time: Date.now() }
  })
  return {
    id: String(Date.now() % 10000000000000).padStart(13, '0'),
    version: '1.0',
    name,
    params
  }
}

/**
 * @brief 构造属性期望值 payload (用于 SetDeviceDesiredProperty)
 */
export function buildDesiredParams(values) {
  // desired 的格式: { "switch": true } 一层 key/value, 不带 time
  const out = {}
  for (const key in values) {
    if (key in PROPERTIES) out[key] = values[key]
  }
  return out
}
