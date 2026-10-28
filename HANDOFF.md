# 工程状态交接 (HANDOFF)

> **这个文件的作用**: 换新对话时，助手读这一份就能接上之前的工作，不用从头复述。
> **维护方式**: 每次对话结束前（或每完成一个里程碑），助手更新本文件。
> **写法要求**: 只写"结论"，不写过程；控制在 100 行以内；旧内容该删就删。

---

## 1. 当前在做什么

**状态**: **0.32 已完成并发布**（tag `v0.32`，GitHub release 已建）。
0.32 做了三件事：让 `bal_kd` 默认 0.4 **真正生效**、把控制回路跟踪拆成独立组件、
修掉三个追踪脚本里的真 bug。

🔶 **在 0.32 之上还有一批未发布的改动**（位置环 + 速度环串级，功能已写完、
离线测试全过、**还没上过真车**）：

- 方向键改成"调目标位置"：按住前进/后退时目标位置按前馈速度往前累积，
  松手后目标位置锁死，位置环一直接管（`user/control/balance.c`）
- 位置环输出加了整形：限速 12 rpm/s + 一阶低通 τ=0.2 s（`user/pid/position_pid.h`），
  防止位置误差一变就跳变 → 目标倾角跳变 → 车前后摆
- `pos_kp` 默认 0 → **0.5**（位置环默认打开，**纯 P**），`pos_ki` 保持 0
- 参数表版本 17 → **18**（车端 + App 同步）

**当前版本目录**：`0.32/`（版本号 `0.32.0`，见 `0.32/version.txt`）
**上一版本**：`0.31/`（位置环接入但默认关闭）

---

## 2. 0.32 改了什么（已完成）

| 改动 | 位置 | 说明 |
|---|---|---|
| `bal_kd` 默认 **1.5 → 0.4**（车端 + App 两侧同步） | `user/param/param.c`、`app/common/paramDef.js` | 0.31 只改了车端，App 还在推 1.5 → 车照旧打摆 |
| 参数表版本 16 → **17** | `user/param/param.c` | 作废旧 NVS blob，否则旧 kd=1.5 仍生效 |
| App `PARAM_VER` 16 → 17 | `app/common/paramDef.js` | 跟车端同步 |
| 控制回路跟踪拆成独立组件 | `user/trace/trace.{c,h}` | 采集（控制任务，几百纳秒，不碰串口）/ 输出（独立任务，钉 core 0）分离 |
| TRACE 行格式统一 | 固件 + 三个脚本 | CSV 表头 `t_ms,pitch,gyro,out,tgt,spd,yaw,pos,pos_out,kd,dt,st` |
| 控制台波特率 115200 → **230400** | `sdkconfig.defaults`、`sdkconfig.esp32-s3-devkitc-1`、`platformio.ini` | 见下面第 5 节的坑 |
| `pos_kp` 显示小数位 2 → 1 | `user/param/param.c` | step 是 0.1，App 侧本来就是 1 |
| 版本号 0.31.0 → 0.32.0 | `version.txt`、`app/manifest.json`、`app/package.json` | versionCode 141 → 142 |

**新增工具**：
- `scripts/check_param_sync.py` — 自动比对车端 `param.c` 与 App `paramDef.js`
  的 def/min/max/step/dec/grp + 版本号。**这类脱节出过两次事故**，改参数表先跑它
- `scripts/run_tests.sh` — 一条命令跑完 5 个离线测试 + 参数表一致性 + App JS 语法

---

## 3. 验证状态（已跑通，别重复跑）

| 项目 | 结果 |
|---|---|
| `pio run` (0.32) | SUCCESS，RAM 25.1% (82,204 / 327,680)，Flash 28.3% (1,039,289 / 3,670,016) |
| `test/test_param_pc.c` | **123 / 123** 通过 |
| `test/test_pid_pc.c` | **29 / 29** 通过 |
| `test/test_speed_pid_pc.c` | **14 / 14** 通过 |
| `test/test_position_pid_pc.c` | **10 / 10** 通过（0.32 之上那批改动后重跑） |
| `test/test_trim_pc.c` | **67 / 67** 通过 |
| 参数表一致性 (`check_param_sync.py 0.32`) | 20 项逐项一致，版本号 **18 = 18** ✓ |
| 同上跑 `0.31` | **2 处不一致**（bal_kd def、pos_kp dec）—— 证明脚本能抓到真问题 |
| App JS 语法 + `manifest.json` / `pages.json` JSON 解析 | 通过 |
| `fft_trace.py` / `analyze_trace.py` | 用合成 CSV 验过（11.5Hz 振荡能正确识别） |

一条命令全跑：`sh scripts/run_tests.sh 0.32`

---

## 4. 🔴 真机待验证（0.32 还没上过车）

- [ ] `pio run --target upload` 烧录 0.32
- [ ] **必须重新配平**（参数表作废 → 配平表 / 配平零点也跟着作废）
- [ ] App 参数页确认显示 `bal_kd = 0.4`（不是 1.5）
- [ ] 定轴静置 30 秒，确认不再一起立就满幅打摆
- [ ] 采样率验证：新 TRACE 格式下 `capture_trace.py` 抓的行数应接近 100 行/秒
- [ ] **位置环默认打开了**（`pos_kp = 0.5` / `pos_ki = 0`）：
  - [ ] 站住不动 30 秒，看会不会慢慢前后挪（里程计漂移 → 位置环去追）
  - [ ] 推它一把，确认能走回推之前的位置
  - [ ] 按 App 方向键，确认走完后松手能站住不回跑
- [ ] ⚠ 板子接上后才有 `/dev/cu.usbmodem*`；没接设备时 `pio run --target upload`
  会一直等端口（`upload.wait_for_upload_port = true`）

---

## 5. 踩过的坑（别再踩）

- **`bal_kd` 默认值两边脱节**：车端改了 App 没改 → App 下发就把车打回打摆状态。
  **改参数默认值必须两边都改**，改完跑 `scripts/check_param_sync.py`
- **只改默认值不升 `PARAM_DATA_VERSION` 等于没改**：NVS 里的旧 blob 仍然有效
- **kconfig 会重置"不可见符号"**：`ESP_CONSOLE_UART_BAUDRATE` 只有开了
  `ESP_CONSOLE_UART_CUSTOM` 才有 prompt，否则**每次编译都被重置回 115200**
  （直接改 `sdkconfig.esp32-s3-devkitc-1` 白改）
- **在控制任务里 printf 会污染测量**：一次写一百多字节要几毫秒，等于给平衡内环
  加几十毫秒延迟 → 内环本来就在相位裕度边缘 → 足以把车顶出振荡。
  打印必须放到独立任务里（`user/trace/` 就是干这个的）
- **`fft_trace.py` 原来"找到 0 段抖动"**：列错位一列 + 段切太碎，**恰好在真抖的时候
  什么都不输出**。已修（按状态切连续段 + 取列跳过 t_ms）

---

## 6. 关键约定（别再踩）

- **配平**：`trim_record` 存表 → `trim_save` 存 NVS → `trim_get` 读回
- **参数实时生效**：改参数不需要重启车
- **`bal_kd` 默认 0.4，不要再加**：>1 会激起 8~12Hz 满幅打摆自激（真机实测）。
  想加阻尼余量要给 D 路加 8~20Hz 陷波，不是加大 kd
- **磁力计读必须放在电机输出写完之后**（0.3~1ms 的 I2C 事务不能夹在内环里）
- **控制任务钉 core 1，trace 任务钉 core 0**，网络栈留 core 0
- **`user/pid/` 各环接口统一**：`init` / `load_params` / `reset` / `update`；
  增益从参数表实时读；每周期调 `load_params()`（用 `pid_set_gains`，不清积分）
- **位置环已接入串级（最外层），默认打开**（`pos_kp = 0.5`，**纯 P，`pos_ki` 保持 0**）
  - 方向键 = 调目标位置：按住时目标位置按前馈速度累积，松手锁死，位置环一直接管
  - 位置环输出必须经整形（限速 + 低通，常量在 `user/pid/position_pid.h`）才进速度环
  - ⚠ `pos_ki` 保持 0：速度环本身带积分，位置环再带积分 = 双重积分 → 站直时前后摆
  - ⚠ 里程计漂移：站住不动时轮子微小位移会累积成位置误差，位置环会去追 → 车可能慢慢挪。
    死区 1 cm + 小 kp 只能压住，根治要外部参照
- **位置环输出的整形常量放在 `user/pid/position_pid.h`**（不是 balance.c）→ 能离线单测
  - 单测断言要"跑到稳态"再比对：整形后单拍输出只有稳态的千分之几
- **OneNet 物模型后台配置**：4 个配平服务需要在
  [产品页面](https://open.iot.10086.cn/console/product/6YlUqr9lue/model) 配置，
  状态表见 `0.32/docs/物模型待补清单.md`
- **产品 ID** `6YlUqr9lue`，设备名 `balancing_dog`
- **10 个云端服务**（车端 `strcmp` 分支 与 App `invokeService()` 名字完全一致）：
  `stop` / `stand` / `calibrate` / `param_get` / `param_set` / `param_save` /
  `trim_record` / `trim_save` / `trim_clear` / `trim_get`

---

## 7. 追踪工具怎么用

```bash
# 抓串口 (波特率必须 230400; 设备路径可自动挑或用 TRACE_PORT 指定)
python3 scripts/capture_trace.py 60 /tmp/trace
# 分段 + 抖动统计
python3 scripts/analyze_trace.py /tmp/trace.csv
# 频谱 (找真实振荡频率)
python3 scripts/fft_trace.py /tmp/trace.csv
```

固件侧：环形缓冲 600 条 × 10ms = 6 秒历史；状态从"平衡中"离开时自动回灌串口。

---

## 8. 仓库状态

- 本地保留 `0.32/`（当前）+ `0.31/`（上一版）两版，`0.30/` 已删
  - 取回：`git checkout v0.30 -- 0.30`（tag `v0.30` = commit `82c3d47`）
  - ⚠ 真正的 0.30 发布基线是 commit `b5afccb`（`v0.30` 目录曾在提交 `eaece89`
    被误回滚成旧状态），取回时用 `git checkout b5afccb -- 0.30`
  - GitHub release `v0.30` 的源码包附件也在，内容是发布出去的那一版
- `releases/` 只放最新两版的压缩包（0.31 / 0.32）；版本级变更说明在 `releases/RELEASE_NOTES.md`
- GitHub: https://github.com/mack-hx/esp- ，tag `v0.31` / `v0.32` 已推，
  两个 release 都已建好（带源码包附件）
- release notes 用 `sh scripts/release_notes.sh <版本号>` 从 `RELEASE_NOTES.md` 抽

---

## 9. 上下文自测（对话长了先跑）

```bash
python3 scripts/ctx_check.py          # 当前对话占用，红/黄/绿灯
```

规则全文见 `.cursor/rules/project-workflow.mdc` 第 14 条。要点：
每轮正文控制在 2~3 段、只写结论；查代码前派子代理
（`scripts/subagent.sh`，⚠ 外部 CLI 额度用完时会静默产出 0 字节报告，
用之前先 `ls -l` 看大小）；大文件先 Grep 再分段 Read。
