# OneNet 物模型接入指南 (0.2)

> App 和 ESP32 都走 OneNet Studio 物模型平台 (HTTPS 开放接口 + MQTT)。
> 本文件是**给操作员看的**: 平台上的项目长什么样、怎么配 App 和固件、出问题怎么查。
>
> 本表内容已于 2026-10-21 用 `GET /thingmodel/query-thing-model` 实测核对,
> 与平台后台一致。改动物模型后请同步更新本文件。

## 1. 已建好的项目

| 字段 | 值 |
|---|---|
| 平台地址 | https://open.iot.10086.cn |
| 产品 ID (product_id) | `6YlUqr9lue` |
| 设备名 (device_name) | `balancing_dog` |
| 设备级 AccessKey | `YlpFVXlONlMzTUNnMHRocTRXSG04SkZMbmVrdEdmQTI=` |
| 项目 ID (project_id) | **选填**, 填不填平台都收 |
| 接入协议 | MQTT |

> AccessKey 用**设备级**那把 (跟 ESP32 端同一把)。开放接口
> (查属性 / 下发期望值 / 服务调用) 都能通。后台还有一把产品级的 key,
> 拿它调开放接口会报 `10403 invalid authorization`, 不要用。

## 2. 物模型现状

### 2.1 属性 (设备 → 云上报)

| 标识符 | 类型 | 访问 | 范围 / 步长 | 说明 |
|---|---|---|---|---|
| `pitch` | float | 读写 | -90 ~ 90, step 0.1 | 俯仰角 (°) |
| `roll` | float | 读写 | -90 ~ 90, step 0.1 | 横滚角 (°), 本机无此传感器, 固定上报 0 |
| `yaw` | float | 读写 | -360 ~ 360, step 0.1 | 偏航角 (°), 本机无航向传感器, 固定上报 0 |
| `yaw` | float | 读写 | -360 ~ 360, step 0.1 | 偏航角 (°) |
| `gz` | float | 读写 | -2000 ~ 2000, step 0.01 | Z 轴角速度 (°/s) |
| `motor_l` | int32 | 只读 | -100 ~ 100 | 左轮电机输出 (%) |
| `motor_r` | int32 | 读写 | -100 ~ 100 | 右轮电机输出 (%) |
| `speed` | float | 读写 | -10 ~ 10, step 0.01 | **移动指令通道** (打包指令, 见 2.4) |
| `enc_l` | int32 | 读写 | int32 范围 | 左编码器计数 |
| `enc_r` | int32 | 读写 | int32 范围 | 右编码器计数 |
| `uptime` | int32 | 读写 | 0 ~ 2147483647 | 设备运行时间 (秒) |
| `state` | string | 读写 | 长度 16 | 状态机: IDLE / STANDBY / BALANCING / FALLEN |
| `led_on` | int32 | 读写 | 0 ~ 1 | 灯开关 |
| `led_color` | int32 | 读写 | 0 ~ 16777215 | 灯颜色 (0x00RRGGBB) |
| `led_brightness` | int32 | 读写 | 0 ~ 255 | 灯亮度 |
| `magnet_on` | int32 | 读写 | 0 ~ 1 | 电磁铁 |
| `servo1_angle` | float | 读写 | 0 ~ 180, step 1 | 舵机 1 角度 (°) |
| `servo2_angle` | float | 读写 | 0 ~ 180, step 1 | 舵机 2 角度 (°) |
| `sw1` | int32 | 只读 | 0 ~ 1 | 物理开关 1 |
| `sw2` | int32 | 只读 | 0 ~ 1 | 物理开关 2 |

> **移动指令只有 `speed` 一个通道**, 没有 `vx` / `vy` / `w`。
> 平台只认物模型里存在的标识符, 往不存在的标识符下发会回 `10412 identifier not exist`。
>
> `speed` 现在装的是**打包指令**(方向 + 档位), 编码规则见 2.4。
> 后台单位写的是 m/s, 属于显示不准, 不影响功能。

### 2.4 `speed` 属性的指令编码 (打包)

App 的方向键要同时表达"方向"和"档位", 但物模型里能写的属性只有 `speed`
一个(`motor_l` 是只读, 写不进去), 所以方向和档位**打包进一个数**:

| 指令值 | 含义 | 目标轮速 / 目标差 |
|---|---|---|
| `0` | 停止 | 目标轮速归零 (斜坡退到 0) |
| `+0.2` / `+0.4` / `+0.6` / `+0.8` | 前进 20 / 40 / 60 / 80 % | 两轮平均 +20 / +40 / +60 / +80 rpm |
| `-0.2` / `-0.4` / `-0.6` / `-0.8` | 后退 20 / 40 / 60 / 80 % | 两轮平均 −20 / −40 / −60 / −80 rpm |
| `+1.2` / `+1.4` / `+1.6` / `+1.8` | 左转 20 / 40 / 60 / 80 % | 左−右 差 +40 / +80 / +120 / +160 rpm (原地转) |
| `-1.2` / `-1.4` / `-1.6` / `-1.8` | 右转 20 / 40 / 60 / 80 % | 左−右 差 −40 / −80 / −120 / −160 rpm (原地转) |

> 上表的 rpm 按 `move_rpm_max = 100` (参数表默认值) 算: 目标轮速 = 档位百分比 ×
> `move_rpm_max` / 100; 转向的差是轮速的 2 倍 (两轮反向)。
> ⚠ 2026-10-26: `move_rpm_max` 从 60 改成 100 (0.31), 上表随之从
> 12/24/36/48 rpm 变成 20/40/60/80 rpm; 改了这个参数, 上表要跟着重算。

- 档位表 (`20/40/60/80`) 与车上"编码器调试"页 (`ui_encoder.c`) 同源,
  都取自 `hardware/motor_driver/motor_driver.c` 的 `motor_spd_levels[]`。
- 设备端解码在 `0.41/user/onenet_client/onenet_client.c` 的 `apply_move_cmd()`;
  App 端编码在 `0.41/app/utils/control.js` 的 `_dirToCmd()`, 两边一一对应。
- ⚠ 2026-10-26: 设备收到后**不再直驱**, 而是走**速度环**:
  目标轮速 → 目标倾角 → 平衡环, 平衡环全程在线。
  (以前是直接给 PWM, 平衡环整个暂停 —— 车站着的时候一推, 松手时车已经歪了,
  平衡环接手必然剧烈纠偏 → "一让走就剧烈抖"。)
  只有车**没起立** (平衡没开) 时才退回开环直驱, 方便单独调电机/编码器。
- 目标倾角上限 4° (安全阀), 速度环增益 `spd_kp=0.06 / spd_ki=0.01` ——
  仿真 (`test/bal_sim.py --walk`) 里 27 组植物参数的平均结果:
  车身倾角波动中位 3.5°, 走着被推 25°/s 也不摔。
  调大增益车会走得更快, 但倾角波动会成倍变大 (走起来就是"剧烈抖")。
- 设备上报的 `speed` 是**当前指令原值**(即上表里的数), 后台能直接看出在遥控什么。

### 2.2 服务 (云 → 设备, 单次动作)

| 标识符 | 输入参数 | 输出参数 | 说明 |
|---|---|---|---|
| `stand` | (无) | `ok: bool` | 起立: 启动平衡 |
| `stop` | (无) | `ok: bool` | 急停: 清 PID 积分 + 电机刹车 |
| `calibrate` | (无) | `ok: bool` | 配平: 重新标定当前姿态 |
| `param_get` | (无) | `values: string` (512) | 读全部参数 (JSON 字符串) |
| `param_set` | `values: string` (512) | **无** | 写全部参数 |
| `param_save` | (无) | `ok: bool` | 参数落盘 (NVS) |
| `trim_record` | (无) | `values: string` (256) | 记一个配平表标定点 |
| `trim_save` | (无) | `values: string` (64) | 配平表落盘 (NVS) |
| `trim_clear` | (无) | `values: string` (64) | 清空配平表并落盘 |
| `trim_get` | (无) | `values: string` (1024) | 读回配平表 (JSON 字符串) |

> **输出参数一览 (现在一共 10 个服务)**: `stand` / `stop` / `calibrate` / `param_save`
> 这 4 个回 `ok: bool`; `param_get` 和 4 个 trim 服务
> (`trim_record` / `trim_save` / `trim_clear` / `trim_get`) 回的是 **`values: string`**,
> 不是 `ok`; `param_set` 的输出参数是空的 (没配任何输出)。
> 平台会校验应答里的标识符, 多发一个它不认识的字段就报 `10415 response invalid`,
> 所以设备现在对 `param_set` 回空对象 `"data":{}`。
> 以后给 `param_set` 补上 `ok` (bool) 输出后, 改设备端一行即可:
> `user/onenet_client/onenet_client.c` 里 `param_set` 分支,
> `service_reply(name, req_id, "{}")` → `service_reply(name, req_id, "{\"ok\":true}")`。
>
> ⚠ **后台配 4 个 trim 服务时, 输出参数一律配 `values` (string), 不要配 `ok`**。
> 设备端这 4 个服务回的字段名就是 `values`, 配成 `ok` 的话平台认不出应答里的
> `values`, 会判成 `10415 response invalid`, 服务调用全部失败。

#### 2.2.1 参数 JSON 的三个特殊键 (2026-10-26)

`values` 里的 JSON 除了 17 个参数键, 还认两个特殊键:

| 键 | 方向 | 作用 |
|---|---|---|
| `_v` | App → 车端 | 参数表版本号。设备比对不一致就在串口打告警 (多半是 App 没重新打包) |
| `_reset` | App → 车端 | 值为 `true`/`1` 时, **车端用自己 `param.c` 的默认值整表重置** |
| `_w` | 车端 → App | `param_get` 返回。值为 `1` 表示"最近一次写参数来自旧版 App", App 弹提示条提醒用户点"恢复默认" |

> ⚠ App 的"恢复默认"必须走 `_reset`, **不能**把 App 内置的默认值表整份下发。
> 2026-10-26 出过事故: App 的默认值表停在 v7 (旧值), 一次"恢复默认"就把
> `spd_kp=20/spd_ki=30` 下发到车端 → 速度环被意外打开 → 车只朝一个方向转。
> `_reset` 把"默认值"这个概念收回到车端唯一一份, 从根上避免这类不一致。

> `param_get` 返回的 JSON 里也带 `_v`, App 拿它跟自己的版本比, 不一致就在页面上提示用户。

#### 2.2.2 配平表标定服务 (2026-10-26 新增, 4 个)

机器人的腿高由舵机角决定, 腿一换高度, "平衡姿态"的车身俯仰角就跟着变
(实测从最低到最高差约 10°)。车端有一张**配平表** (舵机角 → 平衡俯仰角),
控制链每周期自动扣掉这个偏差: `修正后 pitch = 原始 pitch − 配平表补偿 − 配平零点`。

这 4 个服务就是给 App 的「配平标定」页用的:

| 标识符 | 输入参数 | 输出参数 | 说明 |
|---|---|---|---|
| `trim_record` | (无) | `values: string` (256) | 记一个点: 当前舵机角 → 当前**原始** pitch |
| `trim_save` | (无) | `values: string` (64) | 配平表落盘 (NVS), 断电不丢 |
| `trim_clear` | (无) | `values: string` (64) | 清空配平表并落盘 (换机械结构后用) |
| `trim_get` | (无) | `values: string` (1024) | 读回整张配平表 |

- 4 个服务的**输出参数都只有一个 `values` (string) 字段**, 内容是一段文本/JSON 字符串,
  跟 `param_get` 一个格式 (App 侧从 `res.data.values` 取)。
- `trim_record` 成功时 `values` 形如 `ok, phi=60, pitch=-1.23, n=3`
  (φ = 舵机角, pitch = 记进去的原始 pitch, n = 表内点数);
  失败时形如 `fail: ESP_ERR_NO_MEM` (表已满) / `fail: ESP_ERR_INVALID_STATE` (MPU 不可用)。
- `trim_get` 的 `values` 是一段 JSON 字符串, 解开后是:
  ```json
  { "n": 3, "ref": 80, "pz": 0.0, "rp": -1.2, "pts": [[40,-2.1],[60,-0.4],[80,0.0]] }
  ```
  `n` = 点数, `ref` = 参考角 (param 里的 `servo_c`), `pz` = 当前配平零点 (°),
  `rp` = 当前**原始** pitch (°, 标定时盯着它扶车), `pts` = 每个点的 `[舵机角, 原始 pitch]`。

> ⚠ **必须去 OneNet 平台后台把这 4 个服务加进去**, 否则 App 点按钮会报
> `10412 identifier not exist` (平台只认物模型里声明过的标识符)。
> 4 个服务都不用配输入参数, 输出参数都配一个 `values`, 类型 **string**,
> 长度给够 (`trim_get` 建议 1024, 其余 256 足够)。

> ⚠ 表里存的是**原始 pitch** (没扣配平表补偿、也没扣配平零点),
> 因为修正链里还会再扣一次 `trim_pitch()`。存修正后的 pitch 会重复扣一次, 补偿就废了。

### 2.3 事件 (设备 → 云, 离散事件)

| 标识符 | 输出参数 | 说明 |
|---|---|---|
| `armed` | `time: int64` (秒) | 进入 BALANCING |
| `fallen` | `pitch: float` (取整), `time: int64` (秒) | 车摔倒 |

> 事件参数单位是**秒** (不是毫秒), `fallen.pitch` 的 step 是 1 (整数)。

## 3. 烧录配置 (ESP32 端)

1. 确认平台上的产品/设备已建好, 物模型已按第 2 节配好
2. `0.2/user/onenet_client/onenet_client.h` 顶部三个宏已填好 (本项目默认):
   - `ONENET_PRODUCT_ID = "6YlUqr9lue"`
   - `ONENET_DEVICE_NAME = "balancing_dog"`
   - `ONENET_DEVICE_AK = "YlpFVXlONlMzTUNnMHRocTRXSG04SkZMbmVrdEdmQTI="`
3. 换了**新设备**或**新 AccessKey** 就改这三行, 重烧固件
4. `cd 0.2 && pio run -t upload`

## 4. App 端配置

1. 打开 App → "连接"页
2. 填:
   - 产品 ID: `6YlUqr9lue`
   - 设备名: `balancing_dog`
   - AccessKey: 设备级那把 (见第 1 节)
   - 项目 ID: 留空即可
3. 点 "验证配置" → 算 token → 真拉一次设备属性, 通了才算就绪
4. 浏览器刷新后 App 会自动重连 (配置存在 localStorage), 不用每次手点

## 5. Topic 清单 (跟 `0.2/user/onenet_client/onenet_client.c` 1:1 对齐)

```
# ESP32 → OneNet
$sys/{pid}/{dev}/thing/property/post              属性上报
$sys/{pid}/{dev}/thing/property/post/reply        上报应答
$sys/{pid}/{dev}/thing/event/post                 事件上报
$sys/{pid}/{dev}/thing/property/desired/get/reply 期望值应答

# OneNet → ESP32
$sys/{pid}/{dev}/thing/property/set               期望值下发
$sys/{pid}/{dev}/thing/property/set_reply         下发应答
$sys/{pid}/{dev}/thing/service/{name}/invoke      服务调用
$sys/{pid}/{dev}/thing/service/#                  服务调用订阅
$sys/{pid}/{dev}/thing/service/{name}/invoke_reply 服务应答
```

## 6. 调试点 (常见问题)

| 现象 | 原因 / 排查 |
|---|---|
| 平台回 `10403 invalid authorization` | AccessKey 用错了。必须是**设备级**那把, 且末尾 `=` 不能丢 |
| 平台回 `10412 identifier not exist` | 下发了物模型里没有的标识符 (比如 `vx` / `w`) |
| 平台回 `10415 response invalid` | 设备应答里多了物模型没声明的输出字段 (见 2.2 的 `param_set`) |
| 平台回 `2306` | 期望值应答里问了不存在的标识符, 整条 reply 被拒 |
| 设备在线但属性不更新 | 一帧里只要有一个属性不合规 (超范围 / step 对不上), **整帧被拒收**。上报已按 step 分帧 |
| 服务调用随机超时 | 设备端解析 topic 时没补 `'\0'`, 应答发到了错的 topic。见 0.2 变更记录 (Bug 1) |
| App 点一下就变成"未连接" | 单条指令失败不该改连接状态。只有鉴权类错误 (401/403 / 10403) 才算连接失败 |
| App 移动指令没反应 | 看串口有没有收到 `speed`; 平台每下发一次期望值都会 push 一条 `property/set`, 松手必须下发 0 |
