#!/usr/bin/env python3
"""
preToolUse / postToolUse hook: 限制"助手自己写的文字"长度。

为什么需要这个 (2026-10-27 实测):
  Cursor 报 "输入内容过长" 的真凶不是读进来的大文件, 而是助手每轮输出的
  内部推演文字 (Cursor 把模型的 reasoning 当作正文 text 存进 transcript,
  每一轮都重新发一遍)。三个失败会话实测:

      会话        助手文字    工具IO   助手占比   最长单段
      01ced01f   551,422    7,127   98.7%     69,598
      63abc1ff   517,173    7,407   98.6%     45,532
      130f3ab1   504,316    8,071   98.4%     57,826
      4a8a42d5   102,907   47,322   68.5%     15,882   <- 10-26 成功那次

  单轮就写 4~7 万字符, 30 轮累积 50 万字符 ≈ 15 万 token, 加上系统提示和
  规则文件就超了上下文上限 -> Cursor 报 "输入内容过长"。

本 hook 的做法:
  postToolUse 时读 transcript_path, 统计**当前这一轮**助手写出的文字量,
  超过阈值就往上下文里塞一句提醒, 让模型把剩余分析挪进文件或派子代理。

阈值: 单轮助手文字 > WARN_CHARS 时提醒; > HARD_CHARS 时额外要求先派子代理。
"""

import json
import os
import sys

# 单轮助手文字超过这个字符数就提醒 (约 3~4k token)
WARN_CHARS = 12000
# 单轮超过这个数, 措辞更强硬 (约 8k token)
HARD_CHARS = 30000

# 紧急开关: 存在就完全放行
KILL_SWITCH = "/tmp/esp_balance_robot_ctx_guard_off"


def respond(payload):
    sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    sys.stdout.flush()
    sys.exit(0)


def read_payload():
    try:
        return json.load(sys.stdin)
    except Exception:
        return {}


def current_turn_text_len(transcript_path):
    """统计 transcript 最后一条 assistant 记录里的文字量 (即当前这一轮)。"""
    if not transcript_path or not os.path.exists(transcript_path):
        return 0
    last_assistant = None
    try:
        with open(transcript_path, encoding="utf-8", errors="replace") as fh:
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
                if isinstance(content, list):
                    total = 0
                    for block in content:
                        if isinstance(block, dict) and block.get("type") == "text":
                            total += len(block.get("text") or "")
                    last_assistant = total
    except Exception:
        return 0
    return last_assistant or 0


def main():
    payload = read_payload()
    if os.path.exists(KILL_SWITCH):
        respond({"permission": "allow"})

    transcript_path = payload.get("transcript_path") or ""
    written = current_turn_text_len(transcript_path)

    if written < WARN_CHARS:
        respond({"permission": "allow"})

    if written >= HARD_CHARS:
        msg = (
            f"【刹车】本轮你已经写了 {written} 字符的正文, 这是上下文被撑爆的头号原因"
            "(实测: 失败会话的上下文 98% 是助手自己的输出文字, 工具 IO 只占 1~3%)。\n"
            "现在立刻停止继续分析:\n"
            "1) 已有结论写进文件 (如 logs/ 或 HANDOFF.md), 不要留在正文里;\n"
            "2) 还需要查的东西派外部子代理 (Cursor 的 Task 目前是坏的):\n"
            "   scripts/subagent.sh /tmp/q.txt <任务名>\n"
            "3) 下一条消息只写结论和下一步动作, 不超过 800 字符。"
        )
    else:
        msg = (
            f"【提醒】本轮已写了 {written} 字符正文。上下文会被自己的输出撑爆"
            "(失败会话里助手文字占 98%)。\n"
            "接下来: 长分析写进文件, 查代码派外部子代理 "
            "(scripts/subagent.sh /tmp/q.txt <任务名>), 正文只留结论。"
        )

    respond(
        {
            "permission": "allow",
            "additional_context": msg,
            "agent_message": msg,
        }
    )


if __name__ == "__main__":
    main()
