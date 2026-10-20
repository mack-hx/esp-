# 1.3/app

ESP32-S3R8N8 平衡轮腿机械狗 **配套 App**（uniapp 项目）。

## 技术栈

- **uniapp**（Vue3 + JS，主打 H5 全平台发行）
- **mqtt**（npm 包，H5 走 WebSocket，App/小程序按需换插件）
- **uni-ui**（可选，UI 套件；当前手写样式够用，暂未引入）

## 目录结构

```
1.3/app/
├── App.vue                # 根组件，启动时恢复配置 + 初始化 MQTT
├── main.js                # Vue3 入口
├── manifest.json          # uniapp 应用清单（多端配置）
├── pages.json             # 路由 + tabBar
├── package.json           # npm 依赖
├── common/
│   ├── config.js          # 本地连接配置（broker url / 设备 ID / 凭据）
│   ├── topic.js           # MQTT topic 统一出口（与 ESP32 端同步）
│   └── paramDef.js        # 参数表定义（镜像 1.3/user/param/param.c v7）
├── utils/
│   ├── mqtt.js            # MQTT 客户端封装（单例 + 状态机）
│   └── control.js         # 控制指令封装（持续 publish + 急停 + 起立 + 配平）
├── pages/
│   ├── connect/connect.vue   # 设备连接页（broker url、设备 ID）
│   ├── control/control.vue   # 遥控页（前后左右 + 急停 + 起立 + 配平）
│   ├── params/params.vue     # 参数页（16 个滑块，镜像 NVS 参数表）
│   └── status/status.vue     # 状态页（姿态/电机/电池/心跳）
└── static/                # tabBar 图标（需手动补 6 个 PNG）
```

## MQTT Topic 规范

> 改了一边要同步另一边，否则 App 和 ESP32 互相找不到对方。

```
esp32/{deviceId}/cmd/move         App → ESP32   {vx, vy, w}  20Hz 心跳
esp32/{deviceId}/cmd/stop         App → ESP32   急停
esp32/{deviceId}/cmd/stand        App → ESP32   起立
esp32/{deviceId}/cmd/calibrate    App → ESP32   配平标定

esp32/{deviceId}/state/pose       ESP32 → App   {pitch, roll, yaw, gz}
esp32/{deviceId}/state/motor      ESP32 → App   {left, right, speed, enc_l, enc_r}
esp32/{deviceId}/state/battery    ESP32 → App   {voltage, percent}
esp32/{deviceId}/state/heartbeat  ESP32 → App   周期心跳

esp32/{deviceId}/param/get        App → ESP32   拉取参数表
esp32/{deviceId}/param/set        App → ESP32   写入参数表 {values: {...}}
esp32/{deviceId}/param/resp       ESP32 → App   参数表应答

esp32/{deviceId}/ack              ESP32 → App   命令应答
```

## 开发运行

### H5（最简单）

```bash
cd 1.3/app
npm install
npx uni-build --watch          # 或用 HBuilderX 直接打开 1.3/app 目录
```

浏览器访问 `http://localhost:8080`。

### 多端发行

打开 **HBuilderX** → 文件 → 导入 → 从本地目录导入 `1.3/app` → 发行菜单选目标平台。

## 当前状态（2026-10-09）

✅ 已完成：项目骨架 + 4 个页面 + MQTT 封装 + 参数表镜像
⏳ 待做：**ESP32 端的 MQTT 客户端**（`user/mqtt_client/mqtt_client.c/h`），目前只有 sdkconfig 里编译进了 esp-mqtt 库，业务代码还没接

## 调试用 Broker

默认 broker 走 `wss://broker.emqx.io:8084`（EMQX 公共测试服），**匿名可用**。
国内不稳的话可以在连接页改成：
- `wss://test.mosquitto.org:8081`
- 或自建 EMQX / Mosquitto

> ⚠ EMQX 公共服**只适合调试**，不要拿来做生产。所有设备用同一个 topic 前缀会互相串扰，建议给每辆车起不同的 `deviceId`。

## 协议版本

- App 版本：1.3.0（跟 ESP32 固件版本号对齐）
- 参数表版本：v7（与 `1.3/user/param/param.c` 的 `PARAM_DATA_VERSION` 一致）
