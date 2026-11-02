# OneNet 物模型补充：配平标定 4 个服务

> 补充 `onenet_物模型.md`。2026-10-31 用 `GET /thingmodel/query-thing-model` 实测，
> 平台物模型里只有 6 个服务（`calibrate` / `param_get` / `param_set` / `param_save` / `stand` / `stop`），
> **缺少配平标定页用的 4 个 `trim_*` 服务**。

## 现象与原因

- 标定页提示"调用服务失败"，并且每秒弹一次（页面每秒调一次 `trim_get`）。
- 对平台直接调用 `trim_get`，返回 `10415 设备服务调用失败: identifier not exist`。
- 原因是平台后台没有声明这 4 个服务。**固件和 App 的代码已经就绪，不需要改，也不需要新增 MQTT 属性或 topic**
  （服务走已订阅的 `.../thing/service/#`，应答走 `.../invoke_reply`，其它 6 个服务已验证可用）。

## 需要在平台后台添加的 4 个服务

位置：OneNET Studio → 产品开发 → 物模型 → 添加自定义功能点 → 功能类型选"服务"。

| 标识符 | 名称（随意） | 调用方式 | 输入参数 | 输出参数 |
|---|---|---|---|---|
| `trim_record` | 记录标定点 | 同步 | 无 | `values`，string，长度 256 |
| `trim_save` | 保存配平表 | 同步 | 无 | `values`，string，长度 64 |
| `trim_clear` | 清空配平表 | 同步 | 无 | `values`，string，长度 64 |
| `trim_get` | 读取配平表 | 同步 | 无 | `values`，string，长度 1024 |

注意：

- 输出参数的标识符必须是 `values`，类型 string。配成 `ok` 或 bool，设备应答会被判 `10415 response invalid`。
- 标识符区分大小写，必须与上表完全一致（小写加下划线）。
- 车端实际应答内容：`trim_record` 约 40 字符，`trim_save` / `trim_clear` 约 20 字符，`trim_get` 最长约 500 字符，上表长度都够用。

## 添加后的验证

1. 车上电并联网（平台显示设备在线）。
2. 打开 App 标定页，不再弹"调用服务失败"，"已录点"和"配平零点"开始显示数值。
3. 点"记下这个点"，提示形如 `ok, phi=60, pitch=-1.23, n=3`。
