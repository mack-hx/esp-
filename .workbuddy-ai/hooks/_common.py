#!/usr/bin/env python3
"""hook 公共工具: 项目守卫、紧急开关、会话记录统计、响应输出。

项目守卫
    hook 注册在用户级设置里, 对所有工程生效; 本文件只对 PROJECT_ROOT 下的
    会话生效, 其他工程一律放行, 避免影响别的项目。

紧急开关
    /tmp/esp_balance_robot_ctx_guard_off 存在且未超过 30 分钟则放行一切,
    超时自动删除。手动 touch 一次最多挡 30 分钟。

会话记录
    WorkBuddy 每个会话一个 jsonl:
      ~/.workbuddy-ai/projects/<workspace-slug>/<session-id>.jsonl
    优先用 payload 给出的路径, 取不到就按修改时间找本工程最新的文件。

统计口径
    text:      type=message 且 role=assistant 的正文文字
    reasoning: type=reasoning 的推理文字
    两者都会在下一轮被重新发送, 都计入上下文占用。
"""

import glob
import json
import os
import sys
import time

KILL_SWITCH = "/tmp/esp_balance_robot_ctx_guard_off"
KILL_SWITCH_TTL = 1800

# 本文件在 <项目根>/.workbuddy-ai/hooks/_common.py
PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
PROJECTS_DIR = os.path.expanduser("~/.workbuddy-ai/projects")
TRANSCRIPT_KEYS = ("transcript_path", "transcriptPath", "session_file", "sessionFile")


def kill_switch_on():
    """紧急开关是否生效 (存在且未过期)。过期则删除并返回 False。"""
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


def _belongs(path):
    """会话文件是否属于本工程 (读前若干条记录的 cwd 判断)。"""
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for i, line in enumerate(fh):
                if i > 20:
                    break
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                cwd = rec.get("cwd")
                if cwd:
                    return os.path.abspath(cwd) == PROJECT_ROOT
    except Exception:
        return False
    return False


def find_transcript(payload):
    """定位当前会话的 jsonl 路径, 找不到返回空串。"""
    for key in TRANSCRIPT_KEYS:
        p = payload.get(key)
        if isinstance(p, str) and p and os.path.exists(p):
            return p
    slug = PROJECT_ROOT.strip(os.sep).replace(os.sep, "-")
    files = glob.glob(os.path.join(PROJECTS_DIR, slug, "*.jsonl"))
    if not files:
        files = glob.glob(os.path.join(PROJECTS_DIR, "*", "*.jsonl"))
        files = [f for f in files if _belongs(f)]
    if not files:
        return ""
    return max(files, key=os.path.getmtime)


def in_project(payload, transcript=""):
    """当前调用是否发生在本工程内。"""
    for cand in (payload.get("cwd"), os.environ.get("PWD"), os.getcwd()):
        if not cand:
            continue
        try:
            if os.path.abspath(cand).startswith(PROJECT_ROOT + os.sep):
                return True
            if os.path.abspath(cand) == PROJECT_ROOT:
                return True
        except Exception:
            pass
    return bool(transcript) and _belongs(transcript)


def measure(path):
    """统计会话文字量。

    返回 dict:
        text_total      累计助手正文 (可见回复)
        text_turn       当前轮助手正文
        reasoning_total 累计推理文字
        longest         最长单段正文
    """
    stats = {"text_total": 0, "text_turn": 0, "reasoning_total": 0, "longest": 0}
    if not path or not os.path.exists(path):
        return stats
    turn = 0
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                kind = rec.get("type")
                n = 0
                if kind == "message":
                    if rec.get("role") != "assistant":
                        turn = 0
                        stats["text_turn"] = 0
                        continue
                    for b in rec.get("content") or []:
                        if isinstance(b, dict):
                            n += len(b.get("text") or "")
                    stats["text_total"] += n
                    turn += n
                    stats["longest"] = max(stats["longest"], n)
                    stats["text_turn"] = turn
                elif kind == "reasoning":
                    for b in rec.get("rawContent") or []:
                        if isinstance(b, dict):
                            n += len(b.get("text") or "")
                    stats["reasoning_total"] += n
    except Exception:
        return stats
    return stats


def session_load(stats, reasoning_weight=0.3):
    """会话累计负载: 正文全额计入, 推理按权重折算 (推理不一定整段回传)。"""
    return int(stats["text_total"] + stats["reasoning_total"] * reasoning_weight)



def respond(payload):
    """输出 hook 响应并退出。同时给出多种字段名, 兼容不同 harness 的读取方式。"""
    sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    sys.stdout.flush()
    sys.exit(0)


def allow(event, context=""):
    """放行; context 非空时作为附加上下文注入。"""
    out = {
        "permission": "allow",
        "hookSpecificOutput": {"hookEventName": event, "permissionDecision": "allow"},
    }
    if context:
        out["additional_context"] = context
        out["agent_message"] = context
        out["hookSpecificOutput"]["additionalContext"] = context
    respond(out)


def deny(event, message, user_message=""):
    """拦下本次工具调用, 并把原因回给模型。"""
    respond(
        {
            "permission": "deny",
            "permissionDecision": "deny",
            "permissionDecisionReason": message,
            "user_message": user_message or message,
            "agent_message": message,
            "hookSpecificOutput": {
                "hookEventName": event,
                "permissionDecision": "deny",
                "permissionDecisionReason": message,
            },
        }
    )


def notice(event, message):
    """不拦截, 只把提醒塞进上下文。"""
    allow(event, message)
