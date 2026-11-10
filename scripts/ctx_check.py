#!/usr/bin/env python3
"""上下文体积自测脚本 (ctx_check)

用途: 量化"当前对话"占用了多少上下文, 在触发上限之前提前预警。

数据来源: WorkBuddy 会话记录
    ~/.workbuddy-ai/projects/<workspace-slug>/<session-id>.jsonl
    每个会话一个文件, 取修改时间最新的那个作为当前会话。

统计口径:
    - assistant 正文 (type=message, role=assistant)  → 计入 (占比最大的一项)
    - reasoning 推理文字 (type=reasoning)            → 单列
    - 工具输入输出 (function_call / function_call_result) → 计入
    - 用户输入 (type=message, role=user)             → 计入

用法:
    python3 scripts/ctx_check.py            # 只看当前会话
    python3 scripts/ctx_check.py --all      # 最近 10 个会话 + 按天汇总
    python3 scripts/ctx_check.py --daily    # 只按天汇总
    python3 scripts/ctx_check.py --json     # 机器可读输出
"""

import datetime
import glob
import json
import os
import sys

# 本工程根目录 (从脚本位置推导, 工程改名后不会失效)
WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECTS_DIR = os.path.expanduser("~/.workbuddy-ai/projects")

# 估算阈值。3.4 字符/token 是中英混排的经验值。
CHARS_PER_TOKEN = 3.4
WARN_TOKENS = 90_000
DANGER_TOKENS = 140_000


def _belongs(path):
    """会话文件是否属于本工程 (读前若干条记录的 cwd 判断)。"""
    try:
        with open(path, errors="replace") as fh:
            for i, line in enumerate(fh):
                if i > 20:
                    break
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                cwd = rec.get("cwd")
                if cwd:
                    return os.path.abspath(cwd) == WS
    except Exception:
        return False
    return False


def find_sessions():
    """本工程的全部会话文件, 按修改时间倒序。"""
    slug = WS.strip(os.sep).replace(os.sep, "-")
    files = glob.glob(os.path.join(PROJECTS_DIR, slug, "*.jsonl"))
    if not files:
        files = [
            f
            for f in glob.glob(os.path.join(PROJECTS_DIR, "*", "*.jsonl"))
            if _belongs(f)
        ]
    files.sort(key=os.path.getmtime, reverse=True)
    return files


def measure(path):
    """统计单个会话文件的上下文占用。"""
    text = reasoning = tool_io = user_text = 0
    longest = 0
    blocks = 0

    with open(path, errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except Exception:
                continue
            kind = rec.get("type")

            if kind == "message":
                role = rec.get("role")
                n = 0
                for b in rec.get("content") or []:
                    if isinstance(b, dict):
                        n += len(b.get("text") or "")
                if role == "assistant":
                    text += n
                    blocks += 1
                    longest = max(longest, n)
                else:
                    user_text += n

            elif kind == "reasoning":
                for b in rec.get("rawContent") or []:
                    if isinstance(b, dict):
                        reasoning += len(b.get("text") or "")

            elif kind == "function_call":
                tool_io += len(json.dumps(rec.get("arguments"), ensure_ascii=False))

            elif kind == "function_call_result":
                out = rec.get("output")
                if isinstance(out, dict):
                    tool_io += len(out.get("text") or "")
                elif isinstance(out, str):
                    tool_io += len(out)

    total = text + reasoning + tool_io + user_text
    return {
        "file": os.path.basename(path),
        "mtime": os.path.getmtime(path),
        "total_chars": total,
        "assistant_text": text,
        "reasoning": reasoning,
        "tool_io": tool_io,
        "user_text": user_text,
        "longest_assistant_block": longest,
        "assistant_blocks": blocks,
    }


def to_tokens(chars):
    return chars / CHARS_PER_TOKEN


def fmt(n):
    return f"{n:,}"


def report_current(s):
    tok = to_tokens(s["total_chars"])
    own = s["assistant_text"] + s["reasoning"]
    own_pct = own / max(s["total_chars"], 1) * 100

    print("=" * 62)
    print("当前对话上下文占用")
    print("=" * 62)
    print(f"  累计字符      {fmt(s['total_chars']):>12}")
    print(f"  估算 token    {fmt(int(tok)):>12}")
    print()
    print(f"  助手正文      {fmt(s['assistant_text']):>12}")
    print(f"  推理文字      {fmt(s['reasoning']):>12}")
    print(f"  工具输入输出  {fmt(s['tool_io']):>12}")
    print(f"  用户输入      {fmt(s['user_text']):>12}")
    print()
    print(f"  助手自己产出占比  {own_pct:.1f}%")
    print(f"  最长单段正文      {fmt(s['longest_assistant_block']):>8} 字符")
    print(f"  正文段数          {s['assistant_blocks']:>8}")
    print()

    if tok >= DANGER_TOKENS:
        verdict = "红灯 —— 快要报错了"
        action = "立刻把结论写进 logs/ 与 HANDOFF.md, 然后请用户新开对话"
    elif tok >= WARN_TOKENS:
        verdict = "黄灯 —— 该收敛了"
        action = "停止在主对话里长篇推演; 派子代理查, 只带回结论"
    else:
        verdict = "绿灯"
        action = "正常, 但每轮正文控制在 2~3 段以内"

    print(f"  判定: {verdict}")
    print(f"  建议: {action}")
    print("=" * 62)


def report_all(files):
    print("=" * 78)
    print(f"最近 {len(files)} 个会话的上下文占用 (按修改时间倒序)")
    print("=" * 78)
    print(
        f"{'会话':<10}{'累计token':>12}{'正文':>10}{'推理':>10}"
        f"{'工具IO':>10}{'助手占比':>10}"
    )
    print("-" * 78)
    for f in files:
        s = measure(f)
        tok = to_tokens(s["total_chars"])
        own = s["assistant_text"] + s["reasoning"]
        pct = own / max(s["total_chars"], 1) * 100
        mark = "!" if tok >= DANGER_TOKENS else ("-" if tok >= WARN_TOKENS else " ")
        print(
            f"{s['file'][:8]:<10}{fmt(int(tok)):>12}"
            f"{fmt(s['assistant_text']):>10}{fmt(s['reasoning']):>10}"
            f"{fmt(s['tool_io']):>10}{pct:>9.1f}% {mark}"
        )
    print("-" * 78)
    print("! = 已达红灯线    - = 黄灯    空格 = 正常")


def report_daily(files):
    """按天统计助手产出量。这是上下文失控的最早信号。"""
    agg = {}
    for f in files:
        s = measure(f)
        day = datetime.datetime.fromtimestamp(s["mtime"]).strftime("%m-%d")
        a = agg.setdefault(
            day, {"sessions": 0, "text": 0, "reasoning": 0, "blocks": 0, "longest": 0}
        )
        a["sessions"] += 1
        a["text"] += s["assistant_text"]
        a["reasoning"] += s["reasoning"]
        a["blocks"] += s["assistant_blocks"]
        a["longest"] = max(a["longest"], s["longest_assistant_block"])

    print("=" * 78)
    print("按天统计 (助手产出量)")
    print("=" * 78)
    print(f"{'日期':<8}{'会话':>6}{'正文':>12}{'推理':>12}{'段数':>8}{'最长单段':>12}")
    print("-" * 78)
    for day in sorted(agg):
        a = agg[day]
        print(
            f"{day:<8}{a['sessions']:>6}{fmt(a['text']):>12}"
            f"{fmt(a['reasoning']):>12}{fmt(a['blocks']):>8}{fmt(a['longest']):>12}"
        )
    print("-" * 78)


def main():
    args = sys.argv[1:]

    if not os.path.isdir(PROJECTS_DIR):
        print(f"找不到会话记录目录: {PROJECTS_DIR}")
        sys.exit(1)

    files = find_sessions()
    if not files:
        print("还没有可统计的会话记录。")
        sys.exit(0)

    if "--all" in args:
        report_all(files[:10])
        report_daily(files)
        return

    if "--daily" in args:
        report_daily(files)
        return

    current = measure(files[0])

    if "--json" in args:
        current["est_tokens"] = int(to_tokens(current["total_chars"]))
        print(json.dumps(current, ensure_ascii=False, indent=2))
        return

    report_current(current)


if __name__ == "__main__":
    main()
