# 工程状态交接 (HANDOFF)

> **这个文件的作用**: 换新对话时，助手读这一份就能接上之前的工作，不用从头复述。
> **维护方式**: 每次对话结束前（或每完成一个里程碑），助手更新本文件。
> **写法要求**: 只写"结论"，不写过程；控制在 100 行以内；旧内容该删就删。

---

## 1. 当前在做什么

**状态**：0.31 位置环**已接入并跑通**（里程计 + 位置 PID + 接入串级 + 参数表/App 同步），
代码已入库。**待真机验证**。上一轮修的"一直往前跑"和"高频抖动"两个控制环问题也还没上过真机。

**当前版本目录**：`0.31/`（版本号 `0.31.0`，见 `0.31/version.txt`）
**上一版本**：`0.30/`（已发布到 GitHub，勿修改）

---

## 2. 0.31 相比 0.30 改了什么

### 2.1 位置环（2026-10-27 完成，未上真机）
| 改动 | 位置 | 状态 |
|---|---|---|
| 新增里程计组件 `user/odom/`（脉冲→cm 换算，1320 脉冲/圈 × π×6.5cm 自检通过） | `0.31/user/odom/` | 完成 |
| 位置 PID 单位从"圈数"改为 **cm**，死区 1.0cm + 过渡带 1.0cm，死区内积分指数衰减(tau=3s)，输出限幅 ±20 rpm | `0.31/user/pid/position_pid.{c,h}` | 完成 |
| 接入串级控制（最外层，在速度环外）：位置修正叠加到 `s_tgt_avg`；里程挂在 **100ms 窗口脉冲增量**上（不是 rpm 换算后，否则 300ms 才动一次） | `0.31/user/control/balance.c` | 完成 |
| 行走时位置环让位：有方向指令/速度目标未退完时位置输出恒为 0，目标位置跟着走；站住不动时锁死目标位置 | `0.31/user/control/balance.c` | 完成 |
| 参数表 17 → **20 项**（新增 `wheel_dia=6.5cm`、`pos_kp=1.0`、`pos_ki=0.05`），`PARAM_DATA_VERSION` 14 → **15** | `0.31/user/param/param.c` | 完成 |
| App 侧同步：PARAM_VER 15 + 3 个新条目 (id 17/18/19) | `0.31/app/common/paramDef.js` | 完成 |
| 根 `CMakeLists.txt` 的 `EXTRA_COMPONENT_DIRS` 补 `user/odom`（白名单式，新组件漏了会编译失败） | `0.31/CMakeLists.txt` | 完成 |
| 新增 `test/test_position_pid_pc.c` | `0.31/test/` | 完成 |

### 2.2 控制环修复（早前完成，同样未上真机）

| 改动 | 位置 | 状态 |
|---|---|---|
| 编码器减速比 `gear` 1.0 → 30（编码器在电机轴上，减速箱 30:1） | `user/param/param.c` | 完成 || 参数表 v14，共 17 项（新增 `move_rpm_max`） | `user/param/param.c` | 完成 |
| 速度死区 3 → 5 rpm 改回 1.0（按 gear=30 重算） | `user/param/param.c` | 完成 |
| 磁力计 AK8963 bypass 探测（找不到只告警不报错） | `hardware/mpu9250/mpu9250.c` | 完成 |
| 配平标定入口：4 个云端服务 + App 页面 | `user/onenet_client/onenet_client.c`、`app/pages/trim/trim.vue` | 完成 |
| 陀螺零偏标定修复（此前因恒差 3° 永远失败，导致起立剧烈抖动） | `hardware/mpu9250/mpu9250.c` | 完成 |
| App 方向键改走速度环（平衡环全程不撒手） | `app/utils/control.js` | 完成 |
| 配平零点断电保存 / 配平表不再每次上电被擦除 | `user/param/param.c` | 完成 |
| **控制环 PID 拆成独立组件** `user/pid/`（balance/speed/steer/position 四个环各一个 .c，通用 `pid.c` 一并搬过去） | `user/pid/`、`user/control/balance.c` | 完成，`balance.c` 1313 → 1256 行 |
| 位置环只建文件 + 接口占位，**默认不接入控制流程**（工程里没有里程计，编码器只能给瞬时转速） | `user/pid/position_pid.c` | 完成，当前车行为不变 |

**10 个云端服务**（车端 `strcmp` 分支 与 App `invokeService()` 名字完全一致）：
`stop` / `stand` / `calibrate` / `param_get` / `param_set` / `param_save` / `trim_record` / `trim_save` / `trim_clear` / `trim_get`

---

## 3. 验证状态（已跑通，别重复跑）

| 项目 | 结果 |
|---|---|
| `pio run` | SUCCESS，RAM 16.3% (53308/327680)，Flash 28.2% (1034457/3670016) |
| `test/test_param_pc.c` | **119 / 119** 通过（原 110，新增 3 个位置环参数） |
| `test/test_pid_pc.c` | **29 / 29** 通过（新增 8 条积分衰减断言） |
| `test/test_speed_pid_pc.c` | **14 / 14** 通过（死区软化 + 积分衰减） |
| `test/test_position_pid_pc.c` | **9 / 9** 通过（新建） |
| `test/test_trim_pc.c` | 67 / 67 通过 |
| 里程计换算自检 | 1320 脉冲/圈 = 20.4204 cm = π×6.5 ✓ |
| `node --check app/utils/control.js` | 通过 |
| `node --check app/common/paramDef.js` | 通过，PARAM_VER=15，20 项，id 连续 |
| `app/pages.json` JSON.parse | 通过 |
| OneNet map JSON | 299 字节（平台限 512）✓ |
| 车端 vs App 参数表逐项比对 | 20 项全一致 |

---

## 4. 待办 / 待用户确认

### 🔴 最优先：真机验证今天修的两个问题
用户连提 4 次的问题，今天已定位根因并改完代码，**但还没上真机**。

**症状与对应修复**：

| 症状 | 根因 | 修复 | 位置 |
|---|---|---|---|
| 会一直往前跑 | 速度环死区在 PID 前归零 → 积分**冻结** → 残留积分永久非零 → 目标倾角恒定非零 | 新增 `pid_decay_integral()`，死区内积分按 `exp(-dt/3s)` 衰减到 0 | `user/pid/pid.{c,h}`、`user/pid/speed_pid.c` |
| 一直在高频抖动 | 死区**硬归零** → 实测轮速跨量化台阶 (0.90↔1.05 rpm) 时目标倾角每 300ms 跳一次 → 平衡环反复纠偏 | 死区加线性过渡带 `SPD_PID_ERR_SOFT_BAND = 0.5 rpm`，误差连续 | `user/pid/speed_pid.{c,h}` |

**验证状态**：固件编译 SUCCESS；离线测试 pid 29/29、speed_pid 14/14、
param 110/110、trim 67/67，全绿无回归。

**需要用户做的**：
- [ ] 烧录 0.31（`pio run --target upload`）
- [ ] **重新配平**（旧配平表作废）
- [ ] 定轴静置 30 秒，看**还往不往前跑**（修复 1 的直接判据）
- [ ] 静置时听电机声音，看**还有没有高频嗡嗡声**（修复 2 的直接判据）
- [ ] App 方向键走一段再松手，看停不停止（验证积分能退干净）

**如果还是往前跑**，可能的追加方向（按优先级）：
1. `SPD_PID_I_DECAY_TAU` 调小（3.0 → 1.5）
2. 死区 `SPD_PID_ERR_DEADBAND` 偏小，量化噪声进了积分 → 调大到 2.0
3. 编码器方向符号 `BAL_ENC_FWD_SIGN`（balance.h）反了 → 车速反馈符号错

**位置环：已接入，未上真机**（轮径已确认为 6.5 cm）
- 真机验证顺序：`pos_kp` 先拖到 **0**（关掉位置环）→ 确认车站得稳、前面两个修复没被破坏
  → `pos_kp` 慢慢加（0.5 → 1 → 2），每次推一下车看回位 → 最后加 `pos_ki`（0.05 → 0.1）消掉"站住后慢慢偏"
- ⚠ 位置环积分项调太大，车会突然窜出去找位置
- ⚠ 位置环增益 1.0/0.05 是**猜的**，真机大概率要调
- 判据：推一下车看是否走回原位；走一段松手看是否停在原地（不回起点）

**子代理怎么派（2026-10-27 起）**：
Cursor 内置 `Task` 报 `message_id` 错误时，用外部脚本（走本机 claude CLI）：
```bash
cat > /tmp/q.txt <<'EOF'
工程根: /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_robot
目标文件: 0.31/<具体路径>
要找什么: [明确的问题]
返回格式: 只回结论 + 文件:行号, 40 行以内, 不贴大段源码
EOF
scripts/subagent.sh /tmp/q.txt <任务名>
```
报告落在 `logs/subagents/<任务名>-<时间戳>.md`，主对话只回 20 行摘要。

⚠ **2026-10-27 17:29 起 claude CLI 返回 402 Insufficient Balance**，
脚本会**静默产出 0 字节报告**（退出码 0 但内容空）——不是"没查到"，
是额度用完了。用之前先 `ls -l` 看报告大小。
额度恢复前只能自己用 Grep/分段 Read 查（hook 会强制控制次数）。

⚠ 别用 `nohup scripts/subagent.sh ... &` 后台跑：脚本 5 秒内随父 shell 结束，
报告写一半（实测 0 字节）。前台同步等它跑完。

**`.cursor/hooks/` 装了三个强制钩子**（真实环境已验证生效）：
- `require_subagent.py`：自己查文件满 6 次就拦截，强制先派子代理
- `guard_reads.py`：拦重复读同一位置、拦大文件整读（>800 行）
- `limit_verbose_output.py`：单轮正文超 12000 字符时提醒精简
维护类命令（git/python/编译/写文件）不计数，不会被误拦。
紧急开关（子代理不可用时 hook 会把自己锁死）：
```bash
touch /tmp/esp_balance_robot_ctx_guard_off   # 全部放行
rm   /tmp/esp_balance_robot_ctx_guard_off   # 恢复正常拦截
```
⚠ **在独立的终端窗口执行**，不要在被拦的那次调用里执行 —— hook 已经
决定 deny，命令根本跑不到（2026-10-27 踩过）。

### 需要用户动手的（必须等硬件）
- [ ] 烧录 0.31 后**必须重新配平**（旧配平表 + 旧配平零点作废）
- [ ] 旧参数表也作废（会回到 `gear=30` / `move_rpm_max=100` / 速度环 0.06~0.01 的出厂值）
- [ ] 真机走一圈验证：定轴静止、推它不倒、App 方向键走路
- [ ] 位置环真机调参（顺序见上）

### 助手可以自己做的
- [x] `0.31/` 整个目录已进 Git（135 文件，提交 `700d122`）
- [x] 位置环改动已入库（`user/odom/` + 位置 PID + balance 接入 + 参数表 20 项）
- [ ] `0.30/` 的 18 个文件改动也还没提交
- [ ] 0.31 发布到 GitHub（tag `v0.31` + release + 源码包）


### 已知的文档脱节（0.30 遗留，0.31 已修好，0.30 待修）
- `0.30/README.md` 断言数写的 89（实为 110）
- `0.30/test/README.md` 第 15 行写 `cd 0.30`（0.31 里已是 `cd 0.31`）
- `0.30/app/onenet_物模型.md` 写"其它 5 个服务都有 ok"（现共 10 个服务）

---

## 5. 关键约定（别再踩）

- **配平**：`trim_record` 存表 → `trim_save` 存 NVS → `trim_get` 读回
- **参数实时生效**：改参数不需要重启车
- **速度环**：`spd_kp` 建议 0.06~0.1；`move_rpm_max` = 100 约为最高转速的 30%，再往上目标倾角会顶到 4° 安全阀，表现为一直倾着追速度、左右猛摇
- **OneNet 物模型后台配置**：4 个配平服务需要在 [产品页面](https://open.iot.10086.cn/console/product/6YlUqr9lue/model) 配置，状态表见 `0.31/docs/物模型待补清单.md`
- **产品 ID** `6YlUqr9lue`，设备名 `balancing_dog`
- **`user/pid/` 各环接口统一**：`init` / `load_params` / `reset` / `update`；增益从参数表实时读；每周期调 `load_params()`（用 `pid_set_gains`，不清积分）
- **`balance.c` 现在只管**：传感器采样 / 状态机 / 摔倒检测 / 电机输出 / 对外 API
- **位置环接口已留但未接入** —— 等里程计 (轮径参数) 做好直接接

---

## 6. 上下文自测

对话较长时先跑：

```bash
python3 scripts/ctx_check.py          # 当前对话占用，判定红/黄/绿灯
python3 scripts/ctx_check.py --all    # 最近 10 个会话的历史规律
```

**实测结论**：触发"输入内容过长"的元凶是**助手自己的输出文字**（占上下文 75%~99%），
不是读进来的大文件（工具 IO 只占 1%~30%）。所以：

- 每轮输出控制在 2~3 段以内，推演过程不要全部留在主对话里
- 大范围比对/通读 → 派子代理，只带结论回来
- 黄灯时把结论写进 `logs/` 和本文件；红灯时提醒用户新开对话

规则全文见 `.cursor/rules/project-workflow.mdc` 第 14 条。

## 7. 上下文上限的实测硬数字（2026-10-27 19:40 查清）

用户以为模型有 1M 上下文所以"不该报错"——**实测这条链路的请求体上限只有约 50 万字符
（约 15 万 token），是标称窗口的 1/6~1/7**。

| 会话 | 报错时请求体 | 估算 token | 结果 |
|---|---|---|---|
| `8cbd14b0` | 516,996 字符 | 152,057 | ✅ 成功 |
| `63abc1ff` | 517,539 字符 | 152,217 | ❌ 报错 |
| `01ced01f` | 551,609 字符 | 162,237 | ❌ 报错 |
| `189a583b` | 735,149 字符 | 216,220 | ❌ 报错 |

**边界卡在 517k 字符**（成功 516,996 / 失败 517,539，差 543 字符）。

报错原文是 Cursor 服务端发的（本地 `Cursor.app` bundle 里 grep 不到这个字符串，
带 trace id，注入在 assistant 消息里），**不是模型窗口耗尽**，提示里的
"请检查是否引入超大文件"是误判——今天三次失败的工具 IO 只有 7~8 KB。

**两种触发方式**（31 个会话统计）：
1. **会话累积**：全是助手自己的输出文字。Cursor 把 reasoning 当普通 assistant 文本存进
   transcript，每轮重发一次，于是每轮净增 ~18k token 纯开销。
2. **单个超大件**：截图 PNG 转 base64（`e83f91ad`）、浏览器 DOM 全量 dump
   （`bc2a7163` 87k / `e262b76f` 98k 字符）、超大 StrReplace 输入（`ae7238aa` 142k）。

**所以真正的规则是**：请求体（我的输出文字 + 工具输入 + 工具结果）总和控制在 **40 万字符 / 12 万 token** 以内，
且**永远不要 Read 截图或把整页 DOM 灌进上下文**。`ctx_check.py` 的 90k/140k 红黄线可以再收紧。

**这个上限在本地改不了**（已查证，别再重复查）：
- `Cursor.app/Contents/Resources/app` bundle 里没有报错字符串
- `~/Library/Application Support/Cursor/User/globalStorage/state.vscdb` 里没有
  `maxInputTokens` / `contextWindow` 之类的模型上下文配置（只有 `cursor/initialModelState` 等 UI 状态）
- Cursor 全部日志里没有 context length 类服务端记录
→ 限制在 Cursor 服务端或上游代理，本地无配置可调，只能靠"少写、多落盘"自律

## 8. 日志保留策略（2026-10-27 起，用户明确要求）

- **本地 `logs/` 只保留最近 3 天**：当前是 `2026-10-25.md` / `2026-10-26.md` / `2026-10-27.md`
- 更早的 16 个日志文件**已推送到 GitHub 后从本地删除**（提交 `40e5b10`，内容仍在 git 历史里）
- 清理命令：`scripts/log_prune.sh`（默认 3 天，删除前会检查"有没有未提交/未推送"，不通过就中止）
- **日常读日志只看最近 1~2 天**；需要更早的内容，等用户指定日期再取（`git show <commit>:logs/YYYY-MM-DD.md`）

## 9. 仓库状态（2026-10-27 19:50）

- 本地只保留 `0.31/`（当前）+ `0.30/`（冻结稳定板）两版，`0.22/` 已删
- `logs/change_log.txt` 已删，版本级变更说明改放 `releases/RELEASE_NOTES.md`
- 工作区**干净**，全部已推 GitHub（`origin/main` = `40e5b10`）
- ⚠ **git 配了代理 `http://127.0.0.1:7897` 但代理没在跑**，`git push` 会报
  `Failed to connect to 127.0.0.1 port 7897`。直连是通的，用
  `git -c http.proxy= -c https.proxy= push origin main` 绕过；或把代理开起来
