#!/usr/bin/env python3
"""
preToolUse hook: 累计上下文预算刹车。

为什么需要这个 (2026-10-30 实测 119+8 个会话):
  Cursor 报 "输入内容过长" 的真凶是助手自己写出来的推演文字, 不是读进来的文件。
  实测分段:
      10-01 ~ 10-21 (59 个会话): 助手文字 886K, 单段最长 32K, 每次提问 0.6K
      10-24 ~ 10-30 (55 个会话): 助手文字 17,686K, 单段最长 644K, 每次提问 100K
  失败会话的典型形态: 几百个 assistant block, 990 万字符输出, 0 次编辑。
  Cursor 把每轮 assistant 正文原样重发一遍, 所以累计输出 = 上下文占用。

本 hook 的做法:
  在每次 Read/Grep/Shell/Glob/Task 之前, 量一遍 transcript:
    1. 本轮助手文字超 HARD_CHARS -> 拦下, 要求本轮停止分析
    2. 累计助手文字超 SESSION_HARD -> 拦下, 要求把结论落盘并收尾
  两种情况都只拦"继续查"的动作, 不拦 Write/Edit —— 必须还能把结论写出去。

路径全部从 payload 取, 不写死绝对路径 (工程改名后旧路径会失效)。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import kill_switch_on, measure_transcript, respond  # noqa: E402

# 本轮助手文字上限 (字符)。超过就要求停止继续分析。
HARD_CHARS = 20000
# 本轮助手文字软上限。超过就提醒。
WARN_CHARS = 8000
# 整个会话累计助手文字上限 (字符)。超过就要求落盘收尾。
# 实测失败线在 50 万字符附近, 这里留出余量。
SESSION_HARD = 150000


def main():
    try:
        import json
        payload = json.load(sys.stdin)
    except Exception:
        respond({"permission": "allow"})

    if kill_switch_on():
        respond({"permission": "allow"})

    tool_input = payload.get("tool_input") or {}
    if not isinstance(tool_input, dict):
        tool_input = {}
    tool_name = payload.get("tool_name") or tool_input.get("tool_name") or ""

    # 只拦"继续查"的动作。写操作必须放行, 否则结论落不了盘。
    if tool_name not in ("Read", "Grep", "Glob", "Shell", "Task"):
        respond({"permission": "allow"})

    total, last, _longest = measure_transcript(
        payload.get("transcript_path") or ""
    )

    if total >= SESSION_HARD:
        respond(
            {
                "permission": "deny",
                "user_message": "拦截: 会话累计输出已超上下文预算",
                "agent_message": (
                    f"本会话累计已输出 {total} 字符 (上限 {SESSION_HARD})。"
                    "Cursor 会把每一轮 assistant 正文原样重发, 所以累计输出就是上下文占用, "
                    "历史上 12 个会话就是这样撞的 '输入内容过长'。\n\n"
                    "现在停止一切查询。立刻做这三件事:\n"
                    "1) 把已经得到的结论写进 HANDOFF.md (只写结论, 不写推演);\n"
                    "2) 告诉用户结论和下一步, 不要再读文件;\n"
                    "3) 建议用户新开一个对话继续。\n\n"
                    "**不要再读任何文件, 不要再做长篇分析。**"
                ),
            }
        )

    if last >= HARD_CHARS:
        respond(
            {
                "permission": "deny",
                "user_message": "拦截: 本轮助手文字过长",
                "agent_message": (
                    f"你这一轮已经写了 {last} 字符 (上限 {HARD_CHARS})。"
                    "长推演留在正文里是上下文被撑爆的头号原因"
                    "(实测失败会话里助手文字占 98%, 单段最长 64 万字符)。\n\n"
                    "现在停止继续分析:\n"
                    "1) 要写的分析写进文件 (HANDOFF.md 或 logs/今天的日志);\n"
                    "2) 正文只留结论和下一步动作;\n"
                    "3) 需要查的东西派子代理, 只带回结论。"
                ),
            }
        )

    if last >= WARN_CHARS:
        msg = (
            f"【提醒】你这一轮已经写了 {last} 字符 (软上限 {WARN_CHARS})。"
            "接下来: 长分析写进文件, 正文只留结论。"
        )
        respond(
            {
                "permission": "allow",
                "additional_context": msg,
                "agent_message": msg,
            }
        )

    respond({"permission": "allow"})


if __name__ == "__main__":
    main()
