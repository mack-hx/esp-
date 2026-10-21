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
| `yaw` | float | 读写 | -360 ~ 360, step 0.1 | 偏航角 (°) |
| `gz` | float | 读写 | -2000 ~ 2000, step 0.01 | Z 轴角速度 (°/s) |
| `motor_l` | int32 | 只读 | -100 ~ 100 | 左轮电机输出 (%) |
| `motor_r` | int32 | 读写 | -100 ~ 100 | 右轮电机输出 (%) |
| `speed` | float | 读写 | -10 ~ 10, step 0.01 | **移动指令通道**, 标幺 -1 ~ 1 |
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
> `speed` 设备端按标幺 -1 ~ 1 理解 (0.6 = 目标轮速 36 rpm), 后台单位写的是
> m/s, 属于显示不准, 不影响功能。

### 2.2 服务 (云 → 设备, 单次动作)

| 标识符 | 输入参数 | 输出参数 | 说明 |
|---|---|---|---|
| `stand` | (无) | `ok: bool` | 起立: 启动平衡 |
| `stop` | (无) | `ok: bool` | 急停: 清 PID 积分 + 电机刹车 |
| `calibrate` | (无) | `ok: bool` | 配平: 重新标定当前姿态 |
| `param_get` | (无) | `values: string` (512) | 读全部参数 (JSON 字符串) |
| `param_set` | `values: string` (512) | **无** | 写全部参数 |
| `param_save` | (无) | `ok: bool` | 参数落盘 (NVS) |

> `param_set` 的输出参数是空的 (其它 5 个服务都有 `ok`)。
> 平台会校验应答里的标识符, 多发一个它不认识的字段就报 `10415 response invalid`,
> 所以设备现在对 `param_set` 回空对象 `"data":{}`。
> 以后给 `param_set` 补上 `ok` (bool) 输出后, 改设备端一行即可:
> `user/onenet_client/onenet_client.c` 里 `param_set` 分支,
> `service_reply(name, req_id, "{}")` → `service_reply(name, req_id, "{\"ok\":true}")`。

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
