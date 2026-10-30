#!/usr/bin/env python3
"""
preToolUse hook: 拦截 Read 工具，防止上下文被重复读取 / 大文件整读撑爆。

拦下的情况:
  1. 同一文件在本会话已被读过 -> 提示用 Grep 定位, 或派子代理
  2. 目标文件过大 (> 大文件阈值) 且本次读取不带 offset/limit -> 提示分段读或派子代理
  3. 本会话读取次数已经超预算 -> 提示派子代理

放行时输出 {"permission":"allow"}, 不改变任何行为。
拦下时输出 {"permission":"deny", "user_message":..., "agent_message":...}。

状态文件: /tmp/esp_balance_robot_ctx_guard.json
  记录每个会话已读过的文件与读取次数。30 分钟无活动自动清理。
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import kill_switch_on  # noqa: E402

STATE_FILE = "/tmp/esp_balance_robot_ctx_guard.json"
STATE_TTL = 1800  # 30 分钟

# 紧急开关: 存在且未超过 30 分钟就完全放行 (自动过期见 _common.py)
# 用法: touch /tmp/esp_balance_robot_ctx_guard_off

# 大文件阈值 (行数)。超过就必须带 offset/limit 分段读。
BIG_FILE_LINES = 800
# 单个会话允许的 Read 次数。超过就要求派子代理。
MAX_READS_PER_SESSION = 25


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


def line_count(path):
    """数文件行数。读不出来就返回 -1 (不拦)。"""
    try:
        with open(path, "rb") as fh:
            return sum(1 for _ in fh)
    except Exception:
        return -1


def respond(payload):
    # 用 json.dumps 保证转义正确 —— agent_message 里有换行, 直接拼字符串会
    # 产生非法 JSON (控制字符), Cursor 解析失败。
    sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    sys.stdout.flush()
    sys.exit(0)


def deny(agent_msg, user_msg):
    respond(
        {
            "permission": "deny",
            "user_message": user_msg,
            "agent_message": agent_msg,
        }
    )


def main():
    try:
        payload = json.load(sys.stdin)
    except Exception:
        respond({"permission": "allow"})

    # Cursor 的 preToolUse payload 结构:
    #   { "tool_name": "Read", "tool_input": { "path": "..." }, ... }
    # tool_name 与 tool_input 是顶层并列字段 (不是嵌在 tool_input 里)。
    # 两种结构都兼容, 防止 Cursor 版本差异。
    # 紧急开关: 直接放行一切 (30 分钟自动过期)
    if kill_switch_on():
        respond({"permission": "allow"})

    tool_input = payload.get("tool_input") or {}
    if not isinstance(tool_input, dict):
        tool_input = {}
    tool_name = (
        payload.get("tool_name")
        or tool_input.get("tool_name")
        or ""
    )
    if tool_name != "Read":
        respond({"permission": "allow"})

    # Cursor 传的是 path (不是 file_path), 两种都兼容
    path = tool_input.get("path") or tool_input.get("file_path") or ""
    if not path:
        respond({"permission": "allow"})

    session = (
        payload.get("session_id")
        or payload.get("conversation_id")
        or "default"
    )
    sessions = load_state()
    sess = sessions.setdefault(session, {"reads": 0, "seen": []})

    limit = tool_input.get("limit")
    offset = tool_input.get("offset")
    # 精确读取位置。同一文件读不同片段是合法的, 只有完全重复才拦。
    spot = f"{path}@{offset}/{limit}"

    # 情况 0: 读自己旧会话的 transcript
    # 实测: 10-29 21:09 那个会话为了找回"上一轮定好的方案",
    # 读自己的旧 transcript 129 次 (Shell 109 + Read 20), 之后又写了 344 段
    # 重复陈述同一结论, 全程 0 次编辑。结论应该落盘在 HANDOFF.md, 不该回头翻 transcript。
    if "/agent-transcripts/" in path or path.endswith(".jsonl"):
        sess["reads"] += 1
        save_state(sessions)
        deny(
            f"不要读自己旧会话的 transcript: {path}。\n"
            "跨对话的结论只有一个存放处: 根目录 HANDOFF.md。"
            "如果 HANDOFF.md 里没有你要的结论, 说明上一轮没写进去 —— "
            "那就自己重新查一遍代码, 或者直接问用户, 不要翻 transcript。"
            "翻 transcript 是上下文爆炸的头号原因 (单次会话实测 129 次)。",
            f"拦截: 读取自己旧会话的 transcript ({os.path.basename(path)})",
        )

    # 情况 1: 完全相同的读取已经做过 (同文件 + 同 offset + 同 limit)
    if spot in sess["seen"]:
        sess["seen"].remove(spot)
        sess["seen"].append(spot)
        sess["reads"] += 1
        save_state(sessions)
        deny(
            f"这个位置已经读过了: {path} (offset={offset}, limit={limit})。"
            "不要再重复读同一段。"
            "改用 Grep 精确定位需要的行 (带 -C/-A/-B 上下文), "
            "或者派一个 explore 子代理去读并把结论带回来。"
            "重复读同一段是本项目上下文爆炸的头号原因 "
            "(历史上有 3 个会话 12~13 次重复读同一处, 最终 0 次写入就耗尽上下文)。",
            f"拦截: 重复读取 {os.path.basename(path)} (offset={offset})",
        )

    # 情况 2: 大文件整读
    n = line_count(path)
    if n > BIG_FILE_LINES and limit is None:
        sess["reads"] += 1
        save_state(sessions)
        deny(
            f"文件 {path} 有 {n} 行, 超过大文件阈值 {BIG_FILE_LINES} 行, 不允许整读。"
            "请用 Read 带 offset/limit 分段读 (例如 offset=1 limit=120), "
            "或者派 explore 子代理通读后只把结论带回来。"
            "整读大文件会一次性吃掉大量上下文。",
            f"拦截: {os.path.basename(path)} 大文件整读 ({n} 行)",
        )

    # 情况 3: 读取次数超预算
    sess["reads"] += 1
    sess["seen"].append(spot)
    if len(sess["seen"]) > 40:
        sess["seen"] = sess["seen"][-40:]
    if sess["reads"] > MAX_READS_PER_SESSION:
        save_state(sessions)
        deny(
            f"本会话已 Read {sess['reads']} 次, 超过预算 {MAX_READS_PER_SESSION} 次。"
            "停止继续读文件: 现在就把已获得的信息派给子代理去交叉核对, "
            "或者直接动手改代码。历史上 3 个会话因为反复读取而全部耗尽上下文, 0 次写入。",
            f"拦截: 本会话读取次数已达 {sess['reads']} 次",
        )

    save_state(sessions)
    respond({"permission": "allow"})



if __name__ == "__main__":
    main()
