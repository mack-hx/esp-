# OneNet 物模型接入指南

> 1.3 起, App 和 ESP32 都不再走通用 MQTT broker, 全部接入 OneNet Studio 物模型平台.
> 本文件是**给操作员看的**: 在 OneNet 后台怎么建产品 / 创建设备 / 配物模型 / 拿到 AccessKey.

## 1. 已建好的项目 (操作员只需确认)

| 字段 | 值 |
|---|---|
| 平台地址 | https://open.iot.10086.cn |
| 项目 ID (project_id) | 你在 OneNet 项目概览页能看到, App "连接"页要填这个 |
| 产品 ID (product_id) | `6YlUqr9lue` |
| 设备名 (device_name) | `balancing_dog` |
| 设备级 AccessKey | `y1uJyXF9uU8jo9Wl21tRdeHX69eNx6Gti21Z85udgZk=` |
| 接入协议 | MQTT |

如果 OneNet 后台有, 跳到第 2 步建物模型.

## 2. 在 OneNet 后台建物模型

**路径**: 产品详情 → 物模型定义 → 自定义功能

### 2.1 属性 (设备 → 云)

| 标识符 (identifier) | 类型 | 访问 | 范围 | 说明 |
|---|---|---|---|---|
| `pitch`    | float (单精度) | 只读 | -90 ~ 90 | 俯仰角 (°) |
| `roll`     | float | 只读 | -90 ~ 90 | 横滚角 (°) |
| `yaw`      | float | 只读 | -360 ~ 360 | 偏航角 (°) |
| `gz`       | float | 只读 | -2000 ~ 2000 | Z 轴角速度 (°/s) |
| `motor_l`  | int32 | 只读 | -100 ~ 100 | 左轮电机输出 (%) |
| `motor_r`  | int32 | 只读 | -100 ~ 100 | 右轮电机输出 (%) |
| `speed`    | float | 只读 | -10 ~ 10 | 车体速度 (m/s) |
| `enc_l`    | int32 | 只读 | -2147483648 ~ 2147483647 | 左编码器计数 |
| `enc_r`    | int32 | 只读 | 同上 | 右编码器计数 |
| `voltage`  | float | 只读 | 0 ~ 30 | 电池电压 (V) |
| `percent`  | int32 | 只读 | 0 ~ 100 | 电池电量百分比 (%) |
| `state`    | string | 只读 | 长度 16 | 状态机: IDLE/STANDBY/BALANCING/FALLEN |
| `uptime`   | int32 | 只读 | 0 ~ 2147483647 | 设备启动时长 (秒) |

### 2.2 属性 (云 → 设备, 移动指令)

| 标识符 | 类型 | 访问 | 范围 | 说明 |
|---|---|---|---|---|
| `vx` | float | 读写 | -1.0 ~ 1.0 | 前后速度目标 (标幺) |
| `vy` | float | 读写 | -1.0 ~ 1.0 | 左右平移 (预留) |
| `w`  | float | 读写 | -1.0 ~ 1.0 | 转向角速度 (标幺) |

> ⚠ **App 走 HTTP API `SetDeviceDesiredProperty` 写这三个值**, 设备订阅
> `$sys/.../thing/property/desired/get/reply` 拉到本地使用. 不直接用 property/set topic,
> 因为 set 是同步 (设备必须在线), 期望值是异步 (设备离线时缓存).

### 2.3 服务 (云 → 设备, 单次动作)

| 标识符 (identifier) | 输入参数 | 输出参数 | 说明 |
|---|---|---|---|
| `stop`      | (无) | `ok: bool` | 急停: 清 PID 积分 + 电机刹车 |
| `stand`     | (无) | `ok: bool` | 起立: 启动平衡 |
| `calibrate` | (无) | `ok: bool` | 配平: 重新标定当前姿态 |
| `param_get` | (无) | `values: object` | 读全部 PID 参数 |
| `param_set` | `values: object` | `ok: bool` | 写全部 PID 参数 |
| `param_save`| (无) | `ok: bool` | 参数落盘 (NVS) |

### 2.4 事件 (设备 → 云, 离散事件)

| 标识符 | 输出参数 | 说明 |
|---|---|---|
| `fallen` | `pitch: float, time: int64` | 车摔倒了 |
| `armed`  | `time: int64`              | 从 STANDBY 进入 BALANCING |
| `error`  | `code: int32, msg: string` | 设备异常 (IMU 失联 / 编码器异常) |

## 3. 烧录配置 (ESP32 端)

1. 烧录前**先确认**产品/设备已建好, AccessKey 已生成 (在设备详情页)
2. `1.3/user/mqtt_client/mqtt_client.h` 顶部三个宏已填好 (本项目默认):
   - `ONENET_PRODUCT_ID = "6YlUqr9lue"`
   - `ONENET_DEVICE_NAME = "balancing_dog"`
   - `ONENET_DEVICE_AK = "y1uJyXF9uU8jo9Wl21tRdeHX69eNx6Gti21Z85udgZk="`
3. 如果你换了**新设备**或**新 AccessKey**, 改这三行, 重烧固件
4. `cd 1.3 && pio run -t upload`

## 4. App 端配置

1. 打开 App → "连接"页
2. 填:
   - 产品 ID: `6YlUqr9lue`
   - 设备名: `balancing_dog`
   - 设备级 AccessKey: `y1uJyXF9uU8jo9Wl21tRdeHX69eNx6Gti21Z85udgZk=`
   - 项目 ID: OneNet Studio 项目概览里能看到的 project_id
3. 点 "配置就绪" → 算 token 验证 → 跳到遥控页

## 5. Topic 完整清单 (跟 `1.3/app/common/topic.js` 1:1 对齐)

```
# ESP32 → OneNet
$sys/{pid}/{dev}/thing/property/post         属性上报
$sys/{pid}/{dev}/thing/event/post            事件上报

# OneNet → ESP32
$sys/{pid}/{dev}/thing/property/desired/get/reply     期望值应答
$sys/{pid}/{dev}/thing/service/{name}/invoke           服务调用
$sys/{pid}/{dev}/thing/service/{name}/invoke/reply     服务应答
```

## 6. 调试点 (常见问题)

| 现象 | 排查 |
|---|---|
| ESP32 串口打印 "token 计算失败" | AccessKey 字符串不对, 注意末尾 `=` 不要丢 |
| ESP32 串口打印 "MQTT 错误" | 看具体 error code; 多数是产品没开 MQTT 协议 / 设备没启用 |
| App 提示 "token 计算失败" | 浏览器控制台看完整错误; 大概率是 AccessKey 复制时多了空格 |
| App 轮询状态一直空 | ESP32 没上报; 看 ESP32 串口, 确认属性 post 是不是每 100ms 都在发 |
| App 移动指令发了没反应 | 1) 看 ESP32 串口 "desired reply" 是不是收到了 vx/vy/w 2) 看 balance 模块日志 |
