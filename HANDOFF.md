# HANDOFF.md

## 当前状态
- **2026-11-01 起最新版本是 `0.43/`**（0.42 的副本）；0.42 已发布（v0.42 tag + GitHub Release），不再修改。HBuilderX 要打开 `0.43/app`。
- **0.43 改动**: 当前为空版本（0.42 副本），等待新功能开发。
- **0.42 改动（22:30 已完成，已发布）**：三问题联动修复（站立晃动 + 蓝牙连接 + LCD刷新），参数版本 37。
  - 待烧录验证: 蓝牙连接 (开机 10s 后连是否超时), LCD 刷新流畅度, 站立移动晃动 + 回位速度
- 脱机测试：flash 记录已加 `tavg/tdiff/diff` 三个字段（68 字节/条）。用户脱机跑完要**按急停或摔倒**数据才落盘，再插线 `python3.11 scripts/read_flash_log.py /tmp/flog` 读出；待分析：转弯后位置不稳 / 向一侧走 / 摔倒，直线起步松手是否仍慢。
- 最新版本工程：`0.41/`（2026-10-30 从 0.40 迭代，已推 GitHub）。`0.40/` 是上一版回滚基线，**不再修改**。
- 用户确认 0.41 的位置保持（回位速度、回位过冲、静止晃动）“没有问题了”，真车验证通过。
- 编译：`cd 0.41/main_car && pio run` 通过；`scripts/run_tests.sh` 默认跑 0.41，仅 lqr 的 40° 扰动 1 项失败（原有，40° 模型可信度低）。
- 板子（`/dev/cu.usbmodem21133401`）上烧的是 0.40 同源代码，0.41 与其源码一致，未再重烧。

## 2026-10-31 进展（0.41，已编译，未烧录/未上车）
- LCD 同步：舵机页、主页的舵机角随手机操作刷新；App `stand` 成功后 LCD 自动切到平衡页（`lcd_ui_request_page`）；平衡页进入时不再强制停机。
- 标定页"调用服务失败"的根因：平台物模型缺 4 个 `trim_*` 服务（固件/App 无需改）。**用户需在 OneNET 后台按 `0.41/app/onenet_物模型_补充.md` 添加**，添加后设备上线时验证。
- 待用户验证：烧录后手机起立→LCD 是否进平衡页；手机拨舵机→LCD 舵机页是否跟随。

## 2026-10-31 双车（0.41，已编译，未烧录/未上车）
- 目录：`0.41/main_car`（主车固件）、`0.41/sub_car`（副车固件，删了电磁铁/微动开关；原代码无 K230）、`0.41/app`、`0.41/test`。编译要进对应子目录 `pio run`。
- 副车 OneNET 设备：同产品 `6YlUqr9lue` 下新建 `balancing_dog_sub`（设备 ID 2688649600，物模型产品级共享）。设备已建，AccessKey 已填入 `sub_car/.../onenet_client.h` 和 App `config.js`；状态"未激活"，烧录副车上线后激活。
- App 控制方式：合体（对接成功自动进入，两车同发指令，先急停关平衡后开环直驱，暂无转向）/ 主车 / 副车。转向、合体时前后车方向是否需要反向，待真车确认。

## 2026-10-31 BLE 直控 + 急停退页（0.41，已编译，未烧录/未真机）
- App 急停 → 车端 `remote_stop()`，LCD 在平衡页时退回主页。
- BLE：车端 `user/ble_ctrl`（主车广播名 BalanceDog，副车 BalanceDogSub，无配对码，协议见 `ble_ctrl.h`）；App `common/ble.js`。连着蓝牙的车：移动/急停/起立 + pitch 曲线走蓝牙，其余走 OneNET，pitch 仍上传 OneNET。
- 连接页入口改到状态页右上角头像（tabBar 只剩遥控、状态）。
- 状态页（11-01）：两车同屏，Pitch 双曲线（主绿副蓝）+ 主车/副车两列对照表；App 需重新打包才能看到。
- 11-01 主车因内部 RAM 不足（BLE_INIT Malloc failed）一直重启，已修（BLE 先于 WiFi 启动 + NimBLE/mbedTLS 放 PSRAM），主车已烧录验证不再重启。`sub_car` 同样配置未改，烧录前需同样处理。
- 11-01 手机连蓝牙超时：同为内部 RAM 不足，WiFi 静态收发缓冲 10/16→6/6 后 Mac(bleak) 连车正常。BLE 连上后 MQTT 偶发掉线一次，共存影响待观察。手机 App 需重新打包验证。
- 待验证：手机蓝牙连上后 WiFi+BLE 共存是否影响 10ms 控制周期；安卓 APK 打包后扫描/连接/直控延迟。

## 位置保持（LQR）现状
- 控制律：`user/pid/lqr.c`，`BAL_USE_LQR=1`（balance.c）；0 切回原串级 PID。LQR 模式下 `bal_*`/`pos_*` 滑块不生效。
- 增益（0.42 22:30）：`LQR_KX -5 / KV -2.25 / KP 22 / KD 0.6 / VMAX 35`（lqr.h），单位已按真实 cm（编码器两轮都四倍频）。
- 回位曲线：期望回位速度 = `sqrt(2·lqr_amax·偏差)`，限 `LQR_VMAX`=35 cm/s；`lqr_amax` 默认 8，App 可在线调，不用重烧。
- 位置积分：仅在 0.27~6 cm 偏差、速度 < 4 cm/s 时累积；其余按指数衰减（0.3 s，锚点附近 0.5 s），不清零。积分上限 5°。
- 速度项渐隐：|v| ≤ 2 cm/s 不参与，≥ 5 cm/s 完全参与（`vel_gain`）。
- 参数表 `PARAM_DATA_VERSION`=37，车端 24 项，App `PARAM_VER`=37，改参数必须两边同步（`scripts/check_param_sync.py`）。
- 倾角限幅：位置环 8°、速度环 7°、平衡环 motor_max 95% / ang_limit 35°。起立软启动 30%/800ms。

## 工具
- 脱机 flash 记录：车离开平衡态时把最近 40 s 写进 `storage` 分区，断电不丢。读取：`python3 scripts/read_flash_log.py /tmp/flog`（约 26 s，读完板子复位）。
  下一次离开平衡态会覆盖旧记录，所以先读再测下一轮。
- 在线抓包：`python3 scripts/capture_trace.py <秒> <前缀> <串口>`。
- 真车数据：`logs/data/*.csv`（`lqr_amax8/amax30/integral_band2/band6_vmax20` 等推车记录）。

## 已否决（不要重试）
- 用辨识模型/仿真整定增益：模型整定失败多次，仿真与真车不符，别再靠仿真调增益。
- 超速增益刹车（仿真 G≥0.25 失稳）、叠加速度环（等于再加一份滞后速度反馈）。
- 加大 `LQR_KV` 调过冲（≥8 失稳，KV=5 出现 4 Hz 极限环）；位置偏差限幅 `ERR_SAT`（回位变慢、停在 1~5 cm 处）。
- 内置 `Task` 子代理不可用（Cursor 侧 bug），外部 CLI 402 欠费；不要往委派方向排障。

## 待办
1. **发布规则（用户已明确，不用每次交代）**：发布的是“改好的上一版”，不是刚新建、还没写内容的新版。
   - 用户说“迭代到 X”：把当前版本（改好的那版）的 tag、压缩包、发布说明、GitHub Release 做完；新建的 X 目录只提交，**不打包、不打 tag、不建 Release**，等 X 里有了实际改动、用户再次要求迭代时才发布。
   - 流程：提交 → 推 main + tag → `git archive` 打包到 `releases/`（只留最新两版）→ 更新 RELEASE_NOTES 与 GITHUB_PUBLISH_GUIDE → 建 GitHub Release。
   - 已授权使用 git 已存的 GitHub 凭据：`gh` 没登录，用 `GH_TOKEN="$(printf 'protocol=https\nhost=github.com\n\n' | git credential fill | sed -n 's/^password=//p')"` 仅在当条命令的环境变量里传给 `gh release create --repo mack-hx/esp-`，不打印、不落盘。
   - 现状：v0.40 已完整发布。v0.41 的 tag、压缩包、Release 是误发的（0.41 还没有新内容），用户说这次先不处理；0.41 有实际改动后再迭代到 0.42 时，一并补 0.41 的发布说明。
2. App 需要重新构建打包（`paramDef.js` 已对齐 PARAM_VER 35），否则“恢复默认”会下发旧参数。
3. 遥控速度环与 `BAL_POS_BLEND_RPM` 因轮速放大 2.7 倍尚未重调。
4. 静止时若再出现晃动：先查电机死区/静摩擦（静止时 out 仅 ±6%，死区实测约 2.2%，参数 `deadzone` 默认 4%），不要再动位置环。

## 关键约定
- `scripts/run_tests.sh` 与 `scripts/test_hooks.py` 改完必须跑。
- 版本号只在用户明确要求时递增；本地只保留最新两版（0.41、0.40），更早的在 git 历史和 tag（v0.30/v0.31/v0.32）里。
- 本地 `logs/` 只保留最近 3 天，更早的已在 git 历史里。
- 不要读自己旧会话的 transcript；跨对话结论只写本文件。
- 上下文护栏：`.cursor/hooks/context_budget.py`、`limit_verbose_output.py`；监控 `python3 scripts/ctx_check.py --daily`。
- 11-01 提速：斜坡 150 rpm/s、spd_kp 0.08/ki 0.04、move_rpm_max 120、参数版本 36（参数已重置）。待真机验证：中/快/极速是否跟手、高档超调与松手回收；转向环（目标 120 实测约 70 rpm）尚未调。已知：test lqr 的 40deg 项在改动前就失败。
- 11-01 平衡环放开（0.42，未烧录）：LQR KP 15→18、KD 0.4→0.5、VMAX 20→30、位置积分上限 3→5°；位置环倾角上限 5→8°、速度环 4→7°；起立软启动 15%/1500ms → 30%/800ms；motor_max 60→95%、ang_limit 25→35°、arm_pitch 8→12°；参数版本 36→37（App 已同步）。`run_tests.sh` 全过、`check_param_sync.py` 一致、`pio run` 成功。烧录失败：串口 `/dev/cu.usbmodem21133401` 不存在，板子没连。躺倒无法自立起，本次是把可控倾角放到约 35°。
