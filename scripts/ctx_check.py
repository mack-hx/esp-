#!/usr/bin/env python3
"""
上下文体积自测脚本 (ctx_check)

用途: 量化"当前对话"已经占用了多少上下文, 在触发
      "输入内容过长, 请精简输入内容或新开对话" 之前提前预警。

数据来源: Cursor 会话记录目录下的 .jsonl (每个会话一个文件)。
          取"最近修改"的那个文件作为当前会话。

统计口径 (与 Cursor 实际计入上下文的内容一致):
  - assistant 文本块        → 计入 (本对话里占比最大的一项)
  - tool_use 的 input      → 计入 (读文件/写文件的参数)
  - tool_result 的 content → 计入 (读回来的文件内容)
  - user 文本块            → 计入

用法:
  python3 scripts/ctx_check.py            # 只看当前会话
  python3 scripts/ctx_check.py --all      # 看最近 10 个会话, 找出历史规律
  python3 scripts/ctx_check.py --json     # 机器可读输出
"""

import json
import os
import sys
import glob
import time

# 本工程的工作区绝对路径 (用于推导 Cursor 会话记录目录)
WS = "/Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_robot"

# Cursor 会话记录目录, 在 ~/.cursor/projects/ 下按工作区路径生成一个 slug 目录。
# 规则: 绝对路径去掉开头的 '/', 其中的 '/' 换成 '-', 并且 '_' 也会被换成 '-'。
# 这里不写死, 运行时从 ~/.cursor/projects/ 下自动匹配, 避免路径规则变动就失效。
PROJECTS_DIR = os.path.expanduser("~/.cursor/projects")


def find_transcripts_dir():
    """定位当前工作区对应的会话记录目录。

    先按 slug 规则猜, 猜不中就在 ~/.cursor/projects/ 下按工作区目录名
    模糊匹配 (例如 esp_balance_robot -> *esp-balance-robot*)。
    """
    slug = WS.strip("/").replace("/", "-").replace("_", "-")
    guess = os.path.join(PROJECTS_DIR, slug, "agent-transcripts")
    if os.path.isdir(guess):
        return guess

    if os.path.isdir(PROJECTS_DIR):
        leaf = os.path.basename(WS).replace("_", "-")
        for entry in os.listdir(PROJECTS_DIR):
            if leaf in entry and os.path.isdir(
                os.path.join(PROJECTS_DIR, entry, "agent-transcripts")
            ):
                return os.path.join(
                    PROJECTS_DIR, entry, "agent-transcripts"
                )
    return guess

# 估算阈值。Cursor 报"输入内容过长"时, 累计上下文大致在 80k~200k token。
# 保守起见, 用 3.4 字符/token 换算 (中英混排的经验值)。
CHARS_PER_TOKEN = 3.4
WARN_TOKENS = 90_000     # 黄灯: 该考虑收敛输出 / 派子代理
DANGER_TOKENS = 140_000  # 红灯: 立刻把结论落盘, 准备新开对话


def measure(path):
    """统计单个会话文件的上下文占用。"""
    total = 0
    assistant_text = 0
    tool_io = 0
    user_text = 0
    longest = 0
    turns = 0

    with open(path, errors="replace") as fh:
        for line in fh:
            try:
                d = json.loads(line)
            except Exception:
                continue

            role = d.get("role")
            msg = d.get("message") or {}
            content = msg.get("content")
            if isinstance(content, str):
                content = [{"type": "text", "text": content}]
            if not isinstance(content, list):
                continue

            for block in content:
                btype = block.get("type")
                if btype == "text":
                    n = len(block.get("text", ""))
                    total += n
                    if role == "assistant":
                        assistant_text += n
                        longest = max(longest, n)
                        turns += 1
                    elif role == "user":
                        user_text += n
                elif btype == "tool_use":
                    n = len(json.dumps(block.get("input", {}), ensure_ascii=False))
                    total += n
                    tool_io += n
                elif btype == "tool_result":
                    c = block.get("content")
                    if isinstance(c, str):
                        n = len(c)
                    elif isinstance(c, list):
                        n = sum(
                            len(x.get("text", ""))
                            for x in c
                            if isinstance(x, dict)
                        )
                    else:
                        n = 0
                    total += n
                    tool_io += n

    return {
        "file": os.path.basename(path),
        "mtime": os.path.getmtime(path),
        "total_chars": total,
        "assistant_text": assistant_text,
        "tool_io": tool_io,
        "user_text": user_text,
        "longest_assistant_block": longest,
        "assistant_turns": turns,
    }


def to_tokens(chars):
    return chars / CHARS_PER_TOKEN


def fmt(n):
    return f"{n:,}"


def report_current(stats):
    tok = to_tokens(stats["total_chars"])
    at_pct = stats["assistant_text"] / max(
        stats["assistant_text"] + stats["tool_io"], 1
    ) * 100

    print("=" * 62)
    print("当前对话上下文占用")
    print("=" * 62)
    print(f"  累计字符      {fmt(stats['total_chars']):>12}")
    print(f"  估算 token    {fmt(int(tok)):>12}")
    print()
    print(f"  助手输出文字  {fmt(stats['assistant_text']):>12}  ({at_pct:.1f}%)")
    print(f"  工具输入输出  {fmt(stats['tool_io']):>12}  ({100-at_pct:.1f}%)")
    print(f"  用户输入      {fmt(stats['user_text']):>12}")
    print()
    print(f"  最长单段助手文字  {fmt(stats['longest_assistant_block']):>8} 字符")
    print(f"  助手发言段数       {stats['assistant_turns']:>8}")
    print()

    if tok >= DANGER_TOKENS:
        verdict = "红灯 —— 快要报错了"
        action = "立刻把结论写进 logs/ 与 HANDOFF.md，然后请用户新开对话"
    elif tok >= WARN_TOKENS:
        verdict = "黄灯 —— 该收敛了"
        action = "停止在主对话里长篇推演；派子代理查，只带回结论"
    else:
        verdict = "绿灯"
        action = "正常，但每轮输出控制在 2~3 段以内"

    print(f"  判定: {verdict}")
    print(f"  建议: {action}")
    print("=" * 62)


def report_all(files):
    print("=" * 78)
    print(f"最近 {len(files)} 个会话的上下文占用 (按修改时间倒序)")
    print("=" * 78)
    print(
        f"{'会话':<10}{'累计token':>12}{'助手文字':>12}{'工具IO':>10}"
        f"{'助手占比':>10}{'最长单段':>10}"
    )
    print("-" * 78)
    for f in files:
        s = measure(f)
        tok = to_tokens(s["total_chars"])
        pct = s["assistant_text"] / max(
            s["assistant_text"] + s["tool_io"], 1
        ) * 100
        mark = "!" if tok >= DANGER_TOKENS else ("-" if tok >= WARN_TOKENS else " ")
        print(
            f"{s['file'][:8]:<10}{fmt(int(tok)):>12}"
            f"{fmt(s['assistant_text']):>12}{fmt(s['tool_io']):>10}"
            f"{pct:>9.1f}%{fmt(s['longest_assistant_block']):>10} {mark}"
        )
    print("-" * 78)
    print("! = 已达红灯线    - = 黄灯    空格 = 正常")
    print(
        "结论: 触发上限的是【助手自己的输出文字】, 不是读进来的大文件。"
        " 工具 IO 普遍只占 1~30%。"
    )


def main():
    args = sys.argv[1:]

    global TRANSCRIPTS
    TRANSCRIPTS = find_transcripts_dir()

    if not os.path.isdir(TRANSCRIPTS):
        print(f"找不到会话记录目录: {TRANSCRIPTS}")
        print("若路径不同, 请改脚本里的 TRANSCRIPTS 常量。")
        sys.exit(1)

    files = glob.glob(os.path.join(TRANSCRIPTS, "*", "*.jsonl"))
    if not files:
        print("会话记录目录为空, 还没有可统计的对话。")
        sys.exit(0)

    files.sort(key=os.path.getmtime, reverse=True)

    if "--all" in args:
        report_all(files[:10])
        return

    current = measure(files[0])

    if "--json" in args:
        current["est_tokens"] = int(to_tokens(current["total_chars"]))
        print(json.dumps(current, ensure_ascii=False, indent=2))
        return

    report_current(current)


if __name__ == "__main__":
    main()
