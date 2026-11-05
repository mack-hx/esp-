#!/usr/bin/env python3
"""hook 公共工具: 紧急开关判断 + transcript 统计。

紧急开关 (2026-10-30 加自动过期):
  /tmp/esp_balance_robot_ctx_guard_off 存在且未超过 30 分钟 -> 放行一切。
  超过 30 分钟自动失效并删除。
  原因: 10-29 17:22 手动打开后一直没删, 三个 hook 静默失效了整整一天,
  重复读和长输出都没被拦。
"""

import json
import os
import time

KILL_SWITCH = "/tmp/esp_balance_robot_ctx_guard_off"
KILL_SWITCH_TTL = 1800  # 30 分钟


def kill_switch_on():
    """紧急开关是否生效 (存在且未过期)。过期则自动删除并返回 False。"""
    if not os.path.exists(KILL_SWITCH):
        return False
    try:
        age = time.time() - os.path.getmtime(KILL_SWITCH)
    except OSError:
        return False
    if age > KILL_SWITCH_TTL:
        try:
            os.remove(KILL_SWITCH)
        except OSError:
            pass
        return False
    return True


def respond(payload):
    """输出 hook 响应并退出。用 json.dumps 保证换行被正确转义。"""
    import sys
    sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    sys.stdout.flush()
    sys.exit(0)


def measure_transcript(path):
    """统计 transcript 助手文字: (累计字符, 当前轮字符, 最长单段字符)。"""
    if not path or not os.path.exists(path):
        return 0, 0, 0
    total = 0
    longest = 0
    last = 0
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                if obj.get("role") != "assistant":
                    continue
                content = (obj.get("message") or {}).get("content")
                if not isinstance(content, list):
                    continue
                block_total = 0
                for b in content:
                    if isinstance(b, dict) and b.get("type") == "text":
                        block_total += len(b.get("text") or "")
                total += block_total
                longest = max(longest, block_total)
                last = block_total
    except Exception:
        return 0, 0, 0
    return total, last, longest
