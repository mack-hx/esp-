# ESP32 平衡轮腿机械狗项目规则

## 工作流程
- 唯一开发目标是当前版本目录（见 `HANDOFF.md`）；历史版本只作参考和回滚，不修改。
- 开工先读 `HANDOFF.md` 和最近 1–2 天 `logs/YYYY-MM-DD.md`；收工更新当天日志和 `HANDOFF.md`（≤100 行，只写结论）。
- 只做用户明确要求的任务；不擅自加功能、改无关文件、做范围外"优化"。发现顺带问题先问用户。
- 需求不清楚就用朴素的话问清楚，不要反复推测。

## 版本与 Git
- 版本号数字递增；仅用户明确要求整体迭代时才建新版本。
- 积极 git 提交便于回滚；版本迭代完成后推 GitHub。
- 仓库 https://github.com/mack-hx/esp- ；认证、代理与推送细节见 `.workbuddy-ai/rules/github.md`。

## 代码组织与注释
- 外设代码按模块放 `hardware/<模块>/`，如 `hardware/motor/motor.c` 和 `.h`。
- 注释只写三类：变量含义、接口契约、技术事实；函数注释写功能、参数、返回值。
- 不写设计推导、修改历史、日期戳、长篇图表说明。代码和注释都以简洁为准。

## 编译、烧录、调试
- 可直接执行 `pio run` 和 `pio run --target upload`，不必逐次询问。
- 串口通常是 `/dev/cu.usbmodem*`；可主动读串口、引导用户按键、捕获日志，不必等用户贴日志。
- 硬件接线、按键、外设插拔、断电必须由用户操作。

## 上下文控制（硬性）
- 正文只写结论，不写推演；单段正文 ≤ 8 KB，要写长内容就写进文件。
- 查代码优先派子代理（`scripts/subagent.sh`），拿到结论后直接改，不要回头重读文件。
- 同一问题不重复推理、不重复读文件；禁止读旧会话记录，跨对话信息只看 `HANDOFF.md`。
- 大文件不整篇读：源码 >1000 行或 50 KB、日志/文档 >800 行或 50 KB 时先 Grep 定位再分段读。
- 日志按需读：只看最近 1–2 天；本地只留 3 天，更早的需用户指定日期后从 git 取回。
- 自查：`python3 scripts/ctx_check.py`（当前会话）、`--daily`（按天趋势）。
- 自动兜底：`.workbuddy-ai/hooks/` 的 hook 在本轮或会话累计超预算时拦下查询动作，要求落盘收尾；
  紧急开关 `touch /tmp/esp_balance_robot_ctx_guard_off`（30 分钟自动过期）；改 hook 后跑 `python3 scripts/test_hooks.py`。

## 参考文档（按需读，不要预加载）
- 外设配置与驱动要求：`.workbuddy-ai/rules/peripheral-config.md`
- ESP32-S3R8N8 引脚分配与硬件变更：`.workbuddy-ai/rules/pin-config.md`
- GitHub 认证与推送：`.workbuddy-ai/rules/github.md`
- H5 调试流程：`.workbuddy-ai/rules/h5-debug.md`
- 涉及引脚或外设时以规则文件为参考，并与实际源码核对。

## 日志与记忆
- 工程日志统一写 `logs/YYYY-MM-DD.md`（时间、改动、问题、解法）；只保留最近 3 天，超期先推送 GitHub 再删本地。
- `.workbuddy-ai/memory/` 只放长期约定与结论，不重复记工程日志。
- 主控资料优先查 https://wiki.lckfb.com/zh-hans/esp32s3r8n8/ ；网络不通走代理 7890。
- 不随意改本规则文件，除非用户明确要求。
