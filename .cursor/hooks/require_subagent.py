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

# 读满这么多文件后, 强制先派子代理
READS_BEFORE_DELEGATE = 6
# 派过子代理之后, 重新计数, 允许再读这么多
READS_AFTER_DELEGATE = 12

# 会被拦截的"自己动手查"类工具
QUERY_TOOLS = {"Read", "Grep", "Shell", "Glob"}


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

    if tool_name not in QUERY_TOOLS:
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
            "**现在停下, 先派一个 explore 子代理。**\n\n"
            "照抄下面这个模板派 (自行把 [要找什么] 换成实际问题):\n\n"
            "```\n"
            "Task(subagent_type=\"explore\", description=\"<一句话说明查什么>\",\n"
            "     prompt=\"工程根: /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_robot\n"
            "目标文件: 0.31/<具体路径>\n"
            "要找什么: [明确的问题, 如 '速度环的误差符号与限幅在哪里计算, 用到哪些变量']\n"
            "返回格式: 只回结论 + 关键行号 (file:line), 不要贴大段原文, 控制在 30 行以内\")\n"
            "```\n\n"
            "子代理看不到本对话历史, 所以 prompt 必须自包含 —— 把背景和目标写清楚。\n"
            "子代理的结论会以摘要形式回到主对话, 原文不进上下文, 这正是我们要的效果。\n"
            "派完子代理拿到结论后**直接动手改代码**, 不要再回头把文件读一遍。"
        )
    else:
        agent_msg = (
            f"本会话已自己查了 {sess['reads']} 次文件 (已派过 {sess['delegated']} 次子代理)。"
            "再查下去会重蹈覆辙: 派一个 explore 子代理做交叉核对, "
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
