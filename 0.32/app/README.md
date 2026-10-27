# 0.2/app

ESP32-S3R8N8 平衡轮腿机械狗 **配套 App**（uni-app 项目，H5 为主）。

## 技术栈

- **uni-app**（Vue3 + JS，主打 H5 发行）
- **通信**: OneNet 开放接口 (HTTPS) + 物模型
  - 查询属性 / 下发期望值 / 服务调用 → `https://iot-api.heclouds.com`
  - ESP32 侧走 MQTT 订阅 `$sys/.../thing/...`
- **无需 broker**: 平台自带物模型和鉴权

## 目录结构

```
0.2/app/
├── App.vue                 # 根组件: 启动时恢复配置 + 自动连通性检查
├── main.js                 # Vue3 入口
├── manifest.json           # uni-app 清单
├── pages.json              # 路由 + tabBar
├── common/
│   ├── config.js           # 本地连接配置 (productId / deviceName / AccessKey)
│   ├── onenet.js           # OneNet 开放接口封装 (token / 查属性 / 下发 / 服务调用)
│   ├── oneapi.js           # 连接状态机 (ensureReady / markError / 统一错误出口)
│   └── paramDef.js         # 参数表定义 (镜像 user/param/param.c)
├── utils/
│   └── control.js          # 移动/起立/急停/配平指令封装
├── pages/
│   ├── connect/connect.vue # 连接页 (填配置 + 验证)
│   ├── control/control.vue # 遥控页 (前后 + 急停/起立/配平 + 灯/电磁铁/舵机)
│   ├── params/params.vue   # 参数页 (拉取/保存参数)
│   └── status/status.vue   # 状态页 (实时属性 + 去调参入口)
├── dev/
│   └── onenet_proxy.js     # 本地开发用代理 (跨域时用)
└── onenet_物模型.md        # 物模型接入指南 (给操作员看)
```

## 通信方式

| 动作 | 走哪个接口 | 说明 |
|---|---|---|
| 读设备状态 | `GET /thingmodel/query-device-property` | 状态页轮询 |
| 下发移动指令 | `POST /thingmodel/set-device-desired-property` | 期望值 `speed` (-1 ~ 1) |
| 起立/急停/配平 | `POST /thingmodel/call-service` | `stand` / `stop` / `calibrate` |
| 读参数 | 服务调用 `param_get` | 返回 JSON 字符串 |
| 写参数 | 服务调用 `param_set` + `param_save` | 写完要再调 `param_save` 才落盘 |

**鉴权**: 用**设备级** AccessKey 算 token (HMAC-SHA1, 跟 ESP32 端同一把)。
`project_id` 选填。

详见 `onet_物模型.md` 和 `../docs/物模型待补清单.md`。

## 开发运行

### H5

```bash
cd 0.2/app
npm install
npm run dev:h5          # 浏览器访问 http://localhost:8080
```

### 多端发行

**HBuilderX** → 文件 → 导入 → 从本地目录导入 `0.2/app` → 发行菜单选目标平台。

## 当前状态 (2026-10-21)

✅ 已完成: 4 个页面 + OneNet 开放接口封装 + 参数表镜像 + 与 ESP32 固件全链路联调通过
- 移动指令 (前进/后退/松开)、起立/急停/配平、参数读写、状态实时显示
- 灯 / 电磁铁 / 舵机角度下发与状态回显
- 刷新页面自动重连 (配置存 localStorage)

⏳ 待补: 平台物模型里 `param_set` 的输出参数还是空的 (设备端已适配, 平台补上 `ok` 后改一行即可)

## 版本

- App 版本: 0.31.0 (跟 ESP32 固件版本号对齐)
- 参数表版本: 与 `0.31/user/param/param.c` 的 `PARAM_DATA_VERSION` 一致 (当前 **14**, 共 17 项)
