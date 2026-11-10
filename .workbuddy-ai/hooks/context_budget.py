#!/usr/bin/env python3
"""PreToolUse hook: 上下文预算刹车。

背景: 上下文被撑爆的主因是助手自己写出来的正文, 不是读进来的文件。每轮正文都会
在下一轮重新发送, 累计输出 ≈ 上下文占用。失败会话的典型形态是几百段重复结论、
几乎零次编辑。

做法: 每次查询类工具调用前量一遍当前会话:
    1. 当前轮正文超 TURN_HARD   -> 拦下, 要求停止分析
    2. 会话累计超 SESSION_HARD  -> 拦下, 要求把结论落盘收尾
    3. 超软上限                 -> 放行但提醒
只拦"继续查"的动作, 不拦 Write/Edit, 结论必须还能写出去。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (  # noqa: E402
    allow,
    deny,
    find_transcript,
    in_project,
    kill_switch_on,
    measure,
    notice,
    session_load,
)

# 当前轮正文上限 (字符)
TURN_WARN = 8000
TURN_HARD = 30000
# 会话累计负载上限 (正文全额 + 推理折算)
SESSION_WARN = 100000
SESSION_HARD = 200000

# 只拦这些"继续查"的工具
QUERY_TOOLS = {
    "Read",
    "Grep",
    "Glob",
    "Bash",
    "Shell",
    "Agent",
    "Task",
    "WebFetch",
    "WebSearch",
    "ToolSearch",
}


def read_payload():
    import json

    try:
        return json.load(sys.stdin)
    except Exception:
        return {}


def main():
    payload = read_payload()
    event = payload.get("hook_event_name") or payload.get("hookEventName") or "PreToolUse"

    if kill_switch_on():
        allow(event)

    tool = payload.get("tool_name") or payload.get("toolName") or ""
    if tool not in QUERY_TOOLS:
        allow(event)

    transcript = find_transcript(payload)
    if not in_project(payload, transcript):
        allow(event)

    stats = measure(transcript)
    load = session_load(stats)

    if load >= SESSION_HARD:
        deny(
            event,
            f"本会话累计输出已折算 {load} 字符 (上限 {SESSION_HARD}), "
            "其中正文 "
            f"{stats['text_total']}、推理 {stats['reasoning_total']}。"
            "上下文就是这样被撑爆的。\n\n"
            "现在停止一切查询, 立刻做三件事:\n"
            "1) 把已得的结论写进 HANDOFF.md 和 logs/今天的日志 (只写结论);\n"
            "2) 正文只回结论和下一步, 不要再读文件;\n"
            "3) 建议用户新开一个对话继续。",
            "拦截: 会话累计输出已超上下文预算",
        )

    if stats["text_turn"] >= TURN_HARD:
        deny(
            event,
            f"你这一轮已经写了 {stats['text_turn']} 字符 (上限 {TURN_HARD})。"
            "长推演留在正文里是上下文被撑爆的头号原因。\n\n"
            "现在停止继续分析:\n"
            "1) 要写的分析写进文件 (HANDOFF.md 或 logs/今天的日志);\n"
            "2) 正文只留结论和下一步动作;\n"
            "3) 需要查的东西派子代理, 只带回结论。",
            "拦截: 本轮助手文字过长",
        )

    if stats["text_turn"] >= TURN_WARN or load >= SESSION_WARN:
        notice(
            event,
            f"【提醒】本轮正文 {stats['text_turn']} 字符, 会话累计负载 {load} 字符。"
            "接下来长分析写进文件, 正文只留结论。",
        )

    allow(event)


if __name__ == "__main__":
    main()
