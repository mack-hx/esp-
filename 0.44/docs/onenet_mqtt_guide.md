# OneNET MQTT 物模型接入教程

> 从零在 OneNET 建产品/物模型/设备，到 ESP32 用 MQTT 收发物模型报文，再到 App 通过开放接口读写的完整流程。
> 本文对应 `0.44` 版本固件（`main_car/user/onenet_client/`）与 `0.44/app`。
>
> 平台控制台的菜单名称随改版会变，按"产品 → 物模型 → 设备"这条主线找即可。

---

## 1. 三个概念先分清

| 名词 | 含义 | 本项目取值 |
|---|---|---|
| 产品 (product) | 一类设备的模板，物模型挂在产品上，同一产品下所有设备共用一套功能点 | 产品 ID 形如 `6YlUqr9lue` |
| 设备 (device) | 一台具体的机器，是产品的一个实例 | `balancing_dog` / `balancing_dog_sub` |
| 物模型 (thing model) | 属性 / 事件 / 服务三类功能点的声明，平台按它校验收发报文 | 见第 4 节 |

**设备三元组** = 产品 ID + 设备名 + 设备级 AccessKey。填齐这三个才能算出连接 token。

- 属性 (property)：有当前值的状态量，如 `pitch`、`led_on`。设备上报，云端可下发期望值。
- 事件 (event)：一次性发生的事，如 `fallen`。只上报，不带"当前值"。
- 服务 (service)：云端让设备做一件事，如 `stand`。有请求和应答，可带输入/输出参数。

---

## 2. 在控制台建项目

平台地址：https://open.iot.10086.cn （注册并实名后可用）。

### 2.1 建产品

「产品开发 → 创建产品」：

| 选项 | 填什么 |
|---|---|
| 产品名称 | 随意，如 `esp_balance_robot` |
| 智能化方式 | 设备接入 |
| 协议 | **MQTT** |
| 数据格式 | **物模型**（OneJSON，别名"标准物模型"） |
| 联网方式 | WiFi |
| 操作系统 | 无 / 其他 |

建成后产品详情页能看到**产品 ID**，记下来。

> 数据格式必须选"物模型"。选"透传/自定义"的话，报文不会被 `/thing/...` 系列 topic 解析，本教程全部不适用。

### 2.2 建设备

「设备管理 → 添加设备」：填设备名（如 `balancing_dog`）。平台会自动生成**设备级 AccessKey**（设备详情 → 鉴权信息 → 访问密钥）。

把 AccessKey 复制出来时要带上末尾的 `=`，它是标准 Base64 字符串，缺一个字符就解不开。

> 同一个产品下每台车一个设备，各自一把 AccessKey。别用产品详情页里那把产品级 key——调开放接口会回 `10403`。

### 2.3 密钥怎么存

**不要**把 AccessKey 写进要公开的仓库。

- 固件：`0.44/main_car/user/onenet_client/onenet_secret.h`（已加入 `.gitignore`），模板是同目录的 `onenet_secret.example.h`。新克隆仓库后先复制一份再填。
- App：`0.44/app/common/config.js` 的 `DEFAULT_CONFIG` 三元组为空，在 App 的"连接"页填一次，存在本地 storage，以后自动带上。

---

## 3. MQTT 连接

| 项 | 值 |
|---|---|
| Broker（TCP） | `studio-mqtt.heclouds.com:1883` |
| Broker（TLS） | `studio-mqtts.heclouds.com:8883`，或 1883 端口配 `mqtts://` 走 TLS |
| clientId | **设备名**（必须和设备名完全一致） |
| username | **产品 ID** |
| password | 下面算出来的 token |
| keepalive | 60 s |
| 协议版本 | MQTT 3.1.1 |

### 3.1 token 算法

```
res      = products/{pid}/devices/{dev}
et       = 过期时间（Unix 秒）
method   = sha1
version  = 2018-10-31
待签名串  = "{et}\n{method}\n{res}\n{version}"
sign     = base64( HMAC-SHA1( base64decode(AccessKey), 待签名串 ) )
token    = version=2018-10-31&res={urlencode(res)}&et={et}&method=sha1&sign={urlencode(sign)}
```

要点：

1. **et 必须写长期**。本项目固定用 `1924833600`（2030-12-30 12:00:00 UTC）。用"当前时间 + 24h"这种短期 et，同一个 key 走 HTTP 开放接口能通，但 MQTT broker 会直接拒绝，报 `Bad username or password`。
2. **res 要 URL 编码**，`/` 变成 `%2F`。
3. **sign 里的 `+` `/` `=` 要 URL 编码**成 `%2B` `%2F` `%3D`。
4. AccessKey 是 Base64，签名密钥是**解码后的原始字节**，不是那串字符本身。

Python 快速验证（连不上时先用它确认 key 和算法没问题）：

```python
import base64, hmac, hashlib, urllib.parse
PID, DEV = "你的产品ID", "你的设备名"
AK  = "你的设备级AccessKey"
ET  = 1924833600
res = f"products/{PID}/devices/{DEV}"
s   = f"{ET}\nsha1\n{res}\n2018-10-31"
sign = base64.b64encode(hmac.new(base64.b64decode(AK), s.encode(), hashlib.sha1).digest()).decode()
print(f"version=2018-10-31&res={urllib.parse.quote(res, safe='')}&et={ET}&method=sha1&sign={urllib.parse.quote(sign, safe='')}")
```

固件实现见 `0.44/main_car/user/onenet_client/onenet_client.c` 的 `compute_token()`。

---

## 4. 物模型（完整表格）

控制台「产品开发 → 物模型 → 添加功能点」，按下表逐个加。**标识符必须逐字一致**，大小写敏感；设备上报/云端下发都会按它校验。

### 4.1 属性（19 个）

| 标识符 | 类型 | 访问 | 范围 / 步长 | 说明 |
|---|---|---|---|---|
| `pitch` | float | 读写 | -90 ~ 90，step 0.1 | 俯仰角 (°)，20 Hz 高频上报 |
| `roll` | float | 读写 | -90 ~ 90，step 0.1 | 横滚角 (°)，本机无此传感器，固定 0 |
| `yaw` | float | 读写 | -360 ~ 360，step 0.1 | 偏航角 (°)，无航向传感器，固定 0 |
| `gz` | float | 读写 | -2000 ~ 2000，step 0.01 | Z 轴角速度 (°/s) |
| `speed` | float | 读写 | -10 ~ 10，step 0.01 | **移动指令通道**，编码见 4.4 |
| `motor_l` | int32 | 只读 | -100 ~ 100 | 左轮输出 (%) |
| `motor_r` | int32 | 只读 | -100 ~ 100 | 右轮输出 (%) |
| `enc_l` | int32 | 只读 | 全 int32 范围 | 左编码器计数 |
| `enc_r` | int32 | 只读 | 全 int32 范围 | 右编码器计数 |
| `uptime` | int32 | 只读 | 0 ~ 2147483647 | 运行时间 (s) |
| `state` | string | 只读 | 长度 16 | `IDLE` / `STANDBY` / `BALANCING` / `FALLEN` |
| `sw1` | int32 | 只读 | 0 ~ 1 | 对接微动开关 1 |
| `sw2` | int32 | 只读 | 0 ~ 1 | 对接微动开关 2 |
| `led_on` | int32 | 读写 | 0 ~ 1 | 灯开关 |
| `led_color` | int32 | 读写 | 0 ~ 16777215 | 灯颜色 `0xRRGGBB` |
| `led_brightness` | int32 | 读写 | 0 ~ 255 | 灯亮度 |
| `magnet_on` | int32 | 读写 | 0 ~ 1 | 电磁铁 |
| `servo1_angle` | float | 读写 | 0 ~ 180，step 1 | 舵机 1 角度 (°) |
| `servo2_angle` | float | 读写 | 0 ~ 180，step 1 | 舵机 2 角度 (°) |

范围、类型、step 三项必须和表里完全一致。设备上报的值如果超出范围或不在 step 的整数倍上，**整帧被拒收**（不是只丢那一个字段），所以固件按类型和 step 拆成多帧发。

### 4.2 事件（2 个）

| 标识符 | 输出参数 | 说明 |
|---|---|---|
| `armed` | `time` int64 (s) | 进入平衡状态 |
| `fallen` | `pitch` float，step 1；`time` int64 (s) | 车摔倒 |

事件参数的时间单位是**秒**，而属性里的 `time` 是**毫秒**，别混。

### 4.3 服务（10 个）

| 标识符 | 输入参数 | 输出参数 | 说明 |
|---|---|---|---|
| `stand` | 无 | `ok` bool | 起立，开启平衡 |
| `stop` | 无 | `ok` bool | 急停，清积分并刹车 |
| `calibrate` | 无 | `ok` bool | 配平当前姿态 |
| `param_get` | 无 | `values` string (512) | 读全部参数（JSON 字符串） |
| `param_set` | `values` string (512) | **无** | 写全部参数 |
| `param_save` | 无 | `ok` bool | 参数落盘 NVS |
| `trim_record` | 无 | `values` string (256) | 记一个配平标定点 |
| `trim_save` | 无 | `values` string (64) | 配平表落盘 |
| `trim_clear` | 无 | `values` string (64) | 清空配平表 |
| `trim_get` | 无 | `values` string (1024) | 读回配平表（JSON 字符串） |

调用方式全部选「同步」。输出参数是平台校验服务应答的凭据：应答里多一个没声明的字段就回 `10415`，所以 `param_set` 没配输出，设备就回空对象 `"data":{}`。

### 4.4 `speed` 属性的指令编码

物模型能下发的属性里只有 `speed` 是"移动指令通道"（`motor_l` 只读，写不进去），所以方向和档位打包进一个数：

| 指令值 | 含义 |
|---|---|
| `0` | 停止 |
| `±0.2 / ±0.4 / ±0.6 / ±0.8` | 前进 / 后退，档位 20 / 40 / 60 / 80 % |
| `±1.2 / ±1.4 / ±1.6 / ±1.8` | 左转 / 右转，档位 20 / 40 / 60 / 80 %（原地转） |

正数前进/左转，负数后退/右转。设备收到后走速度环，平衡环全程在线；没起立时才退回开环直驱。

⚠ **松手必须下发 `0`**。平台每下发一次就推一条 `property/set`，不发 0 车就一直按上一条指令走。

---

## 5. Topic 与报文

### 5.1 Topic 清单

```
# 设备 → 云（设备发布，也可订阅其 reply）
$sys/{pid}/{dev}/thing/property/post               属性上报
$sys/{pid}/{dev}/thing/property/post/reply         上报应答（订阅）
$sys/{pid}/{dev}/thing/property/set_reply          属性设置应答（发布）
$sys/{pid}/{dev}/thing/event/post                  事件上报
$sys/{pid}/{dev}/thing/property/desired/get        拉取期望值（发布）
$sys/{pid}/{dev}/thing/property/desired/get/reply  期望值应答（订阅）
$sys/{pid}/{dev}/thing/service/{name}/invoke_reply 服务应答（发布）

# 云 → 设备（设备订阅）
$sys/{pid}/{dev}/thing/property/set                属性设置下发
$sys/{pid}/{dev}/thing/property/desired/get/reply  期望值应答
$sys/{pid}/{dev}/thing/service/#                   服务调用（通配订阅）
```

QoS 用 0，不用 retained。服务调用必须用 `#` 通配订阅，平台下发的具体 topic 是按服务名拼的，逐个订阅会漏。

### 5.2 属性上报

```json
{
  "id": "0000000000123",
  "version": "1.0",
  "params": {
    "pitch": { "value": 1.2, "time": 1730000000000 }
  }
}
```

- `id`：消息 ID，平台在 `post/reply` 里原样回传，用于对账。
- `time`：属性采样时间，**Unix 毫秒**。可以省略；不能填开机以来的毫秒数（会回 `2415 invalid time`）。
- 一个 `params` 里放多个属性时，按本节末尾的规则分组。

### 5.3 属性设置（云 → 设备）

```json
{ "id": "123", "version": "1.0", "params": { "led_on": { "value": 1, "time": 1730000000000 } } }
```

也有裸值写法 `{"led_on":1}`，固件两种都认。设备必须回：

```json
{ "id": "123", "code": 200, "msg": "success" }
```

`id` 要原样回传，否则云端认为下发失败。

### 5.4 事件上报

```json
{ "id": "0000000000124", "version": "1.0", "name": "fallen",
  "params": { "pitch": { "value": 25, "time": 1730000000 } } }
```

### 5.5 服务调用与应答

下发（设备从 `.../service/{name}/invoke` 收）：

```json
{ "id": "456", "version": "1.0", "params": { "values": "{\"kp\":1.2}" } }
```

应答（设备发到 `.../service/{name}/invoke_reply`）：

```json
{ "id": "456", "code": 200, "msg": "success", "data": { "ok": true } }
```

⚠ topic 结尾是 `invoke_reply`（下划线），不是 `invoke/reply`。写成后者平台会直接断开设备连接。

### 5.6 期望值（desired）

设备主动拉：

```json
{ "id": "0000000000001", "version": "1.0", "params": ["speed"] }
```

平台回到 `.../desired/get/reply`，`params` 里带每个标识符的期望值。**数组里只要有一个物模型里不存在的标识符，整条应答失败**（`2306 not find identify`）。

### 5.7 报文分帧规则

一帧里任何一个属性不合规（超范围 / step 不对 / 类型不对），**整帧被丢弃**。所以按下面的组合分别发帧（见 `publish_task()`）：

| 帧 | 内容 | 周期 |
|---|---|---|
| float, step 0.1 | `pitch` | 50 ms（20 Hz，独立高速任务） |
| float, step 0.1 | `roll`, `yaw` | 200 ms |
| float, step 0.01 | `gz`, `speed` | 200 ms |
| float, step 1 | `servo1_angle`, `servo2_angle` | 1 s |
| int32 | `motor_l`, `motor_r`, `enc_l`, `enc_r`, `uptime` | 200 ms |
| int32 | `sw1`, `sw2` | 1 s |
| int32 | `led_on`, `led_color`, `led_brightness`, `magnet_on` | 1 s |
| string | `state` | 200 ms |

---

## 6. 固件侧对接

文件：`0.44/main_car/user/onenet_client/`

1. `onenet_client.h` 顶部：产品 ID、设备名（明文）；`#include "onenet_secret.h"` 提供 AccessKey。
2. 复制 `onenet_secret.example.h` 为 `onenet_secret.h`，填入设备级 AccessKey。这个文件不入库。
3. `compute_token()` 用长期 et 算 token，`mqtt_client_start()` 里填 clientId/username/password。
4. 订阅 4 个 topic：`property/set`、`desired/get/reply`、`property/post/reply`、`service/#`。
5. 三个后台任务：`publish_task`（200 ms 常规上报）、`publish_fast_task`（50 ms 只发 pitch）、`desired_pull_task`（每 5 s 拉一次期望值兜底）。
6. 上报任务优先级必须低于平衡控制任务，否则 WiFi/MQTT 抢占会让控制周期抖动。

副车 `0.44/sub_car/` 结构相同，只是设备名换成 `balancing_dog_sub`，密钥用副车那把。

---

## 7. App 侧对接

App 走的是 **HTTPS 开放接口**，不是 MQTT：

| 用途 | 接口 |
|---|---|
| 读当前属性 | `GET https://iot-api.heclouds.com/thingmodel/query-device-property` |
| 读历史曲线 | `GET .../query-device-property-history` |
| 下发期望值 | `POST .../set-device-desired-property` |
| 调用服务 | `POST .../call-service` |

请求头 `Authorization` 用同一套 token 算法（`0.44/app/common/onenet.js`）。三元组在 App 的"连接"页填，存本地 storage，不写进代码。

---

## 8. 出错怎么查

| 现象 / 错误码 | 原因 |
|---|---|
| `Bad username or password` | et 不是长期；或签名串顺序错；或 sign 没 URL 编码 |
| `10403 invalid authorization` | 用了产品级 AccessKey；或 Base64 末尾 `=` 丢了 |
| `10412 identifier not exist` | 下发了物模型没声明的标识符（如 `vx` / `w`） |
| `10415 response invalid` | 应答里多了未声明的输出字段，或服务没在平台建 |
| `2254 float not conform step` | 值不是 step 的整数倍（`pitch` 0.1 / `speed` 0.01） |
| `2211 int32 type error` | int32 属性带了小数 |
| `2415 invalid time` | `time` 用了开机毫秒数；或 32 位溢出被截断成 2019 年 |
| `2306 not find identify` | `desired/get` 里问了不存在的标识符，整条应答被拒 |
| 设备在线但属性不动 | 某一帧里有属性超范围，整帧被丢；对照第 4 节核对范围 |
| 设备莫名断线 | 服务应答 topic 写成了 `invoke/reply` |
| 服务调用总超时 | 没订阅 `.../service/#`；或解析 topic 时字符串没补 `'\0'` |

排查顺序：先跑第 3.1 节的 Python 脚本确认 token → 用 MQTTX / mosquitto 订阅 `$sys/#` 看平台发下来的原始报文 → 最后才怀疑固件。

---

## 9. 新建一套项目的最小步骤

1. 建产品（MQTT + 物模型），记下产品 ID。
2. 按第 4 节加 19 个属性、2 个事件、10 个服务。
3. 加设备，复制设备级 AccessKey。
4. 固件：填 `onenet_secret.h`、改 `onenet_client.h` 里的产品 ID 和设备名，重新编译烧录。
5. App：在"连接"页填三元组，点验证。
6. 控制台看设备是否在线、`pitch` 是否在跳；不顺就按第 8 节查。
