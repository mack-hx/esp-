/**
 * @file ble.js
 * @brief 蓝牙直控 (BLE, 无配对码), 主车/副车各一条连接
 *
 * 协议与车端 ble_ctrl.h 一致:
 *   服务 0xFFE0, 特征 0xFFE1 (写 + 通知), ASCII 文本
 *   App → 车: "M:<v>" 移动 / "STOP" 急停 / "STAND" 起立
 *             "PROP:<json>\n" 灯/舵机/电磁铁属性 (无应答)
 *             "SV:<name>[:<入参>]\n" 服务调用 (参数/配平/标定), 见 request()
 *   车 → App: "P:<pitch>" 俯仰角 (度), 约 20Hz
 *             "A:<name>:<1|0>:<返回文本>\n" SV 应答 (按 MTU 分片, 到 '\n' 结束)
 *
 * 车端非零移动指令 600ms 不刷新会自动归零, 所以按住方向键时要持续补发。
 */

const SERVICE_ID = '0000FFE0-0000-1000-8000-00805F9B34FB'
const CHAR_ID    = '0000FFE1-0000-1000-8000-00805F9B34FB'

/** 广播名 → 车 ('main' | 'sub') */
export const BLE_NAMES = { BalanceDog: 'main', BalanceDogSub: 'sub' }

/** 上次连过的设备 ID 存储键 */
const STORE_KEY = 'ebr_ble_ids_v1'

/** 每辆车的连接: { deviceId, connected, chain (串行写队列) } */
const links = {
  main: { deviceId: '', connected: false, chain: Promise.resolve(), rx: '', pending: null, reqChain: Promise.resolve() },
  sub:  { deviceId: '', connected: false, chain: Promise.resolve(), rx: '', pending: null, reqChain: Promise.resolve() }
}

const stateListeners = new Set()   // (which, connected) => void
const pitchListeners = new Set()   // (which, pitch) => void
let adapterOpen = false
let hooksInstalled = false

/**
 * @brief uni 回调式接口 → Promise
 * @param fn   uni 接口名
 * @param opts 接口参数
 * @param ms   超时 (毫秒), 超时 reject 并带上接口名; 0 = 不限
 */
function call(fn, opts = {}, ms = 0) {
  return new Promise((resolve, reject) => {
    const t = ms ? setTimeout(() => reject(new Error(fn + ' 超时')), ms) : null
    uni[fn]({
      ...opts,
      success: r => { clearTimeout(t); resolve(r) },
      fail: e => { clearTimeout(t); reject(new Error(fn + ': ' + (e.errMsg || e.errCode || e))) }
    })
  })
}

const sleep = ms => new Promise(r => setTimeout(r, ms))

/** 单次写入的最大字节数 (默认 ATT MTU 23 - 3) */
const BLE_CHUNK = 20

function str2buf(s) {
  const u = new Uint8Array(s.length)
  for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i) & 0xff
  return u.buffer
}

function buf2str(b) {
  return String.fromCharCode.apply(null, new Uint8Array(b))
}

function whichOf(deviceId) {
  if (links.main.deviceId === deviceId) return 'main'
  if (links.sub.deviceId === deviceId) return 'sub'
  return null
}

function setConnected(which, v) {
  links[which].connected = v
  if (!v) failPending(which)
  stateListeners.forEach(cb => cb(which, v))
}

/** 打开蓝牙适配器 + 安装全局回调 (只做一次) */
async function ensureAdapter() {
  if (!adapterOpen) {
    await call('openBluetoothAdapter')
    adapterOpen = true
  }
  if (hooksInstalled) return
  hooksInstalled = true
  uni.onBLEConnectionStateChange(res => {
    const w = whichOf(res.deviceId)
    if (w && !res.connected && links[w].connected) setConnected(w, false)
  })
  uni.onBLECharacteristicValueChange(res => {
    const w = whichOf(res.deviceId)
    if (!w) return
    const s = buf2str(res.value)
    const l = links[w]
    if (s.startsWith('A:')) l.rx = ''                 /* 新应答开头 */
    if (l.rx || s.startsWith('A:')) {
      l.rx += s
      if (l.rx.endsWith('\n')) {
        const frame = l.rx.slice(2, -1)
        l.rx = ''
        const m = frame.match(/^([^:]*):([01]):([\s\S]*)$/)
        if (m && l.pending && l.pending.name === m[1]) {
          const p = l.pending
          l.pending = null
          clearTimeout(p.timer)
          if (m[2] === '1') p.resolve(m[3])
          else p.reject(new Error('车端执行失败: ' + m[1]))
        }
      }
    } else if (s.startsWith('P:')) {
      const v = Number(s.slice(2))
      if (!isNaN(v)) pitchListeners.forEach(cb => cb(w, v))
    }
  })
}

/** 上次连过的设备 ID: { main, sub } */
export function getSavedIds() {
  try { return uni.getStorageSync(STORE_KEY) || {} } catch (e) { return {} }
}

/**
 * @brief 扫描车 (只回调广播名在 BLE_NAMES 里的设备)
 * @param onFound (dev) => void, dev = { deviceId, name, which, RSSI }
 * @param ms      扫描时长 (毫秒), 到时自动停止
 * @return Promise, 扫描结束时 resolve
 */
export async function scan(onFound, ms = 6000) {
  await ensureAdapter()
  const seen = new Set()
  uni.onBluetoothDeviceFound(res => {
    (res.devices || []).forEach(d => {
      const name = d.localName || d.name || ''
      const which = BLE_NAMES[name]
      if (!which || seen.has(d.deviceId)) return
      seen.add(d.deviceId)
      onFound({ deviceId: d.deviceId, name, which, RSSI: d.RSSI })
    })
  })
  await call('startBluetoothDevicesDiscovery', { allowDuplicatesKey: false })
  await new Promise(r => setTimeout(r, ms))
  await call('stopBluetoothDevicesDiscovery').catch(() => {})
}

/**
 * @brief 连接一辆车并订阅 pitch 通知
 * @param which    'main' | 'sub'
 * @param deviceId 扫描得到的设备 ID
 */
export async function connect(which, deviceId) {
  await ensureAdapter()
  await call('stopBluetoothDevicesDiscovery', {}, 3000).catch(() => {})
  if (links[which].connected) await disconnect(which)
  links[which].deviceId = deviceId
  await call('createBLEConnection', { deviceId, timeout: 8000 }, 10000)
  try {
    /* Android 要先拿到服务/特征列表才能读写; 刚连上时服务列表可能为空, 重试 */
    let found = false
    for (let i = 0; i < 6 && !found; i++) {
      await sleep(i ? 500 : 300)
      const r = await call('getBLEDeviceServices', { deviceId }, 5000)
      found = (r.services || []).some(s => s.uuid.toUpperCase() === SERVICE_ID)
    }
    if (!found) throw new Error('车端没有 FFE0 服务')
    await call('getBLEDeviceCharacteristics', { deviceId, serviceId: SERVICE_ID }, 5000)
    await call('notifyBLECharacteristicValueChange', {
      deviceId, serviceId: SERVICE_ID, characteristicId: CHAR_ID, state: true
    }, 5000)
  } catch (e) {
    await call('closeBLEConnection', { deviceId }).catch(() => {})
    throw e
  }
  const ids = getSavedIds()
  ids[which] = deviceId
  uni.setStorageSync(STORE_KEY, ids)
  setConnected(which, true)
}

/** @brief 断开一辆车 */
export async function disconnect(which) {
  const l = links[which]
  if (l.deviceId) await call('closeBLEConnection', { deviceId: l.deviceId }, 3000).catch(() => {})
  if (l.connected) setConnected(which, false)
}

/** 连接断开时清掉未完成的请求 */
function failPending(which) {
  const l = links[which]
  l.rx = ''
  if (l.pending) {
    clearTimeout(l.pending.timer)
    l.pending.reject(new Error('蓝牙已断开'))
    l.pending = null
  }
}

/** @return 该车蓝牙是否已连接 */
export function isConnected(which) {
  return links[which].connected
}

/**
 * @brief 发一条指令 (同一辆车的写操作串行执行，超过20字节自动分包)
 * @param which 'main' | 'sub'
 * @param text  指令文本
 * @return Promise, 写失败时 reject
 */
export function send(which, text) {
  const l = links[which]
  const parts = []
  for (let i = 0; i < text.length; i += BLE_CHUNK) parts.push(text.slice(i, i + BLE_CHUNK))
  const job = parts.reduce((p, part) => p.then(() => call('writeBLECharacteristicValue', {
    deviceId: l.deviceId, serviceId: SERVICE_ID, characteristicId: CHAR_ID,
    value: str2buf(part), writeType: 'writeNoResponse'
  })), l.chain)
  l.chain = job.catch(() => {})
  return job
}

/** @brief 订阅连接状态变化, 返回取消函数 */
export function onBleState(cb) {
  stateListeners.add(cb)
  return () => stateListeners.delete(cb)
}

/** @brief 订阅 pitch 通知, 返回取消函数 */
export function onPitch(cb) {
  pitchListeners.add(cb)
  return () => pitchListeners.delete(cb)
}

/**
 * @brief 向车发一个服务请求并等应答 (同一辆车的请求串行执行)
 * @param which 'main' | 'sub'
 * @param name  服务名: calibrate / param_get / param_set / param_save /
 *              trim_record / trim_save / trim_clear / trim_get
 * @param input 入参文本 (param_set 传参数 JSON), 可省略
 * @param ms    等应答超时 (毫秒)
 * @return Promise<string> 车端返回文本; 车端执行失败 / 超时 / 断开时 reject
 */
export function request(which, name, input = '', ms = 6000) {
  const l = links[which]
  const run = () => new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      if (l.pending && l.pending.name === name) { l.pending = null; l.rx = '' }
      reject(new Error('车端无应答: ' + name))
    }, ms)
    l.pending = { name, resolve, reject, timer }
    send(which, 'SV:' + name + (input ? ':' + input : '') + '\n').catch(e => {
      clearTimeout(timer)
      l.pending = null
      reject(e)
    })
  })
  const job = l.reqChain.then(run, run)
  l.reqChain = job.catch(() => {})
  return job
}

/**
 * @brief 设置灯 / 舵机 / 电磁铁属性 (无应答)
 * @param which   'main' | 'sub'
 * @param payload 扁平对象, 键同 OneNet 物模型属性 (led_on / servo1_angle / magnet_on ...)
 */
export function setProps(which, payload) {
  return send(which, 'PROP:' + JSON.stringify(payload) + '\n')
}
