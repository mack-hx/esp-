#!/usr/bin/env python3
"""
preToolUse hook: 强制在"自己动手读"之前先派子代理。

历史教训 (2026-10-27): 连续 3 个会话, 43/38/52 次工具调用全部是自己 Read/Grep,
Task(子代理) 调用 = 0 次, 结果 0 次写入就把上下文耗光。
根本原因是: 派子代理前要先"想清楚要它干什么", 而模型在反复推演阶段就已经烧完预算。

本 hook 的作用: 把"派子代理"从需要想清楚的动作, 变成不需要想清楚的动作。
 —— 当本会话已经读了 N 个文件 (默认 6) 且还没派过子代理时, 拦截 Read/Grep/Shell,
    强制先派一个 explore 子代理去查, 把查询工作转移出主对话。

状态文件: /tmp/esp_balance_robot_ctx_guard.json (与 guard_reads.py 共用)
"""

import json
import os
import sys
import time

STATE_FILE = "/tmp/esp_balance_robot_ctx_guard.json"
STATE_TTL = 1800

# 紧急开关: 存在这个文件就完全放行 (hook 自锁或子代理不可用时用)
# 用法: touch /tmp/esp_balance_robot_ctx_guard_off
KILL_SWITCH = "/tmp/esp_balance_robot_ctx_guard_off"

# 读满这么多文件后, 强制先派子代理
READS_BEFORE_DELEGATE = 6
# 派过子代理之后, 重新计数, 允许再读这么多
READS_AFTER_DELEGATE = 12

# 会被拦截的"自己动手查"类工具
QUERY_TOOLS = {"Read", "Grep", "Shell", "Glob"}

# 外部子代理 (Claude Code CLI) 的调用特征。
# 2026-10-27 起 Cursor 内置 Task (子代理) 报
# "[invalid_argument] Cursor user message action has no message_id" 全面失败,
# 所以改用 scripts/subagent.sh (本机 Claude Code CLI) 派子代理。
EXTERNAL_SUBAGENT_PATTERNS = (
    "scripts/subagent.sh",
    "subagent.sh",
)

# 维护类命令特征: 命中就不计数, 直接放行
# ⚠ 教训: 最初把所有 Shell 都算作"查代码", 结果 hook 把自己的维护能力也锁死了
#   (连写日志的 Shell 都被拦, 计数只增不减 -> 永久锁死)。
MAINTENANCE_PATTERNS = (
    "git ", "python3 ", "python ", "mkdir ", "cp ", "mv ", "rm ",
    "chmod ", "ln ", "make ", "pio ", "npm ", "node ",
    "cat >", "echo ", "sed -i", "tee ",
)

# 查代码类命令特征: 命中就计数 (优先级高于维护类)
QUERY_PATTERNS = (
    "grep", "cat ", "ls ", "find ", "wc ", "head ", "tail ",
    "sed ", "awk ",
)


def is_maintenance(command):
    """判断 Shell 命令是不是维护类操作 (不该计入自查次数)。

    规则: 命中查代码特征 -> 算自查; 否则命中维护特征 -> 维护类, 放行。
    """
    if not command:
        return False
    cmd = command.strip()
    if any(p in cmd for p in QUERY_PATTERNS):
        return False
    return any(p in cmd for p in MAINTENANCE_PATTERNS)


def is_external_subagent(command):
    """判断 Shell 命令是不是在派外部子代理 (scripts/subagent.sh)。"""
    if not command:
        return False
    return any(p in command for p in EXTERNAL_SUBAGENT_PATTERNS)


def load_state():
    try:
        with open(STATE_FILE) as fh:
            data = json.load(fh)
        if time.time() - data.get("ts", 0) > STATE_TTL:
            return {}
        return data.get("sessions", {})
    except Exception:
        return {}


def save_state(sessions):
    try:
        with open(STATE_FILE, "w") as fh:
            json.dump({"ts": time.time(), "sessions": sessions}, fh)
    except Exception:
        pass


def respond(payload):
    # 用 json.dumps 保证转义正确 —— agent_message 里有换行, 直接拼字符串会
    # 产生非法 JSON (控制字符), Cursor 解析失败。
    sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    sys.stdout.flush()
    sys.exit(0)


def main():
    try:
        payload = json.load(sys.stdin)
    except Exception:
        respond({"permission": "allow"})

    # Cursor 的 preToolUse payload 结构:
    #   { "tool_name": "Read", "tool_input": { "path": "..." }, ... }
    # tool_name 与 tool_input 是顶层并列字段 (不是嵌在 tool_input 里)。
    # 两种结构都兼容, 防止 Cursor 版本差异。
    # 紧急开关: 直接放行一切
    if os.path.exists(KILL_SWITCH):
        respond({"permission": "allow"})

    tool_input = payload.get("tool_input") or {}
    if not isinstance(tool_input, dict):
        tool_input = {}
    tool_name = (
        payload.get("tool_name")
        or tool_input.get("tool_name")
        or ""
    )

    session = (
        payload.get("session_id")
        or payload.get("conversation_id")
        or "default"
    )
    sessions = load_state()
    sess = sessions.setdefault(
        session, {"reads": 0, "seen": [], "delegated": 0}
    )

    # 子代理调用: 放行, 并重置计数 (说明已经用对方法了)
    if tool_name == "Task":
        sess["delegated"] += 1
        sess["reads"] = 0
        save_state(sessions)
        respond({"permission": "allow"})

    # 外部子代理 (scripts/subagent.sh): 放行并清零计数 (等价于 Task)
    if tool_name == "Shell" and is_external_subagent(tool_input.get("command") or ""):
        sess["delegated"] += 1
        sess["reads"] = 0
        save_state(sessions)
        respond({"permission": "allow"})

    # 写操作不算"查", 直接放行且清零计数 (助手在动手做事)
    if tool_name in ("Write", "Edit", "MultiEdit", "Delete"):
        sess["reads"] = 0
        save_state(sessions)
        respond({"permission": "allow"})

    if tool_name not in QUERY_TOOLS:
        respond({"permission": "allow"})

    # 维护类 Shell (git / python 脚本 / 编译 / 写文件) 不计数, 直接放行
    if tool_name == "Shell":
        command = tool_input.get("command") or ""
        if is_maintenance(command):
            save_state(sessions)
            respond({"permission": "allow"})

    # 写文件 / 编辑文件 说明助手在动手做事, 不是在查 —— 计数清零
    if tool_name in ("Write", "Edit", "MultiEdit", "Delete"):
        sess["reads"] = 0
        save_state(sessions)
        respond({"permission": "allow"})

    sess["reads"] += 1
    save_state(sessions)

    limit = READS_BEFORE_DELEGATE if sess["delegated"] == 0 else READS_AFTER_DELEGATE

    if sess["reads"] <= limit:
        respond({"permission": "allow"})

    # 到这里: 读太多了, 还没派子代理
    if sess["delegated"] == 0:
        agent_msg = (
            f"本会话已自己查了 {sess['reads']} 次文件, 但一次子代理都没派过。"
            "**现在停下, 先派一个子代理。**\n\n"
            "⚠ Cursor 内置 `Task` 子代理目前是坏的 (报 "
            "`Cursor user message action has no message_id`), 不要用 Task。\n"
            "改用本工程的外部子代理脚本 (走本机 Claude Code CLI, 绕开 Cursor 的故障点):\n\n"
            "```bash\n"
            "# 1) 把自包含的提问写进 prompt 文件 (子代理看不到本对话历史)\n"
            "cat > /tmp/q.txt <<'EOF'\n"
            "工程根: /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_robot\n"
            "目标文件: 0.31/<具体路径>\n"
            "要找什么: [明确的问题]\n"
            "返回格式: 只回结论 + 文件:行号, 40 行以内, 不贴大段源码\n"
            "EOF\n\n"
            "# 2) 派子代理 (完整报告落到 logs/subagents/, 只回 20 行摘要)\n"
            "scripts/subagent.sh /tmp/q.txt <任务名>\n"
            "```\n\n"
            "拿到结论后**直接动手改代码**, 不要再回头把文件读一遍。"
        )
    else:
        agent_msg = (
            f"本会话已自己查了 {sess['reads']} 次文件 (已派过 {sess['delegated']} 次子代理)。"
            "再查下去会重蹈覆辙: 再派一个子代理做交叉核对 (scripts/subagent.sh), "
            "或者**现在就开始动手改代码**。\n"
            "历史数据: 3 个会话因为反复读文件, 0 次写入就耗尽了上下文。"
        )

    respond(
        {
            "permission": "deny",
            "user_message": "拦截: 自查次数过多, 要求先派子代理",
            "agent_message": agent_msg,
        }
    )


if __name__ == "__main__":
    main()
