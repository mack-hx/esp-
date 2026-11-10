#!/usr/bin/env python3
"""PostToolUse hook: 单轮正文过长时把提醒塞回上下文。

只提醒不拦截 —— 工具已经执行完了, 拦不住; 目的是让模型把剩余分析挪进文件
或派子代理, 不要在正文里继续长篇推演。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (  # noqa: E402
    allow,
    find_transcript,
    in_project,
    kill_switch_on,
    measure,
    notice,
)

WARN_CHARS = 8000
HARD_CHARS = 20000


def read_payload():
    import json

    try:
        return json.load(sys.stdin)
    except Exception:
        return {}


def main():
    payload = read_payload()
    event = payload.get("hook_event_name") or payload.get("hookEventName") or "PostToolUse"

    if kill_switch_on():
        allow(event)

    transcript = find_transcript(payload)
    if not in_project(payload, transcript):
        allow(event)

    turn = measure(transcript)["text_turn"]

    if turn >= HARD_CHARS:
        notice(
            event,
            f"【强制提醒】本轮正文已写 {turn} 字符 (硬上限 {HARD_CHARS})。"
            "停止在正文里推演: 分析写进 HANDOFF.md 或 logs/今天的日志, "
            "需要查的东西派子代理, 正文只回结论。",
        )
    elif turn >= WARN_CHARS:
        notice(
            event,
            f"【提醒】本轮正文已写 {turn} 字符 (软上限 {WARN_CHARS})。"
            "剩下的分析挪进文件, 正文只留结论。",
        )

    allow(event)


if __name__ == "__main__":
    main()
