#!/usr/bin/env python3
"""hook 自测脚本。跑法: python3 scripts/test_hooks.py

覆盖:
  1. context_budget: 单轮超限 / 累计超预算 / 正常放行 / 写操作不拦 / 软上限只提醒
  2. 项目守卫: 非本工程 cwd 一律放行
  3. _common: 紧急开关 30 分钟自动过期
"""

import json
import os
import subprocess
import sys
import tempfile
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOOKS = os.path.join(WS, ".workbuddy-ai", "hooks")
KILL = "/tmp/esp_balance_robot_ctx_guard_off"

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


def run_hook(script, payload, cwd=None, env=None):
    p = subprocess.run(
        [sys.executable, os.path.join(HOOKS, script)],
        input=json.dumps(payload),
        capture_output=True,
        text=True,
        cwd=cwd or WS,
        env=env,
    )
    try:
        return json.loads(p.stdout.strip().splitlines()[-1])
    except Exception:
        return {"permission": "allow", "_raw": p.stdout, "_err": p.stderr}


def make_transcript(assistant_texts, reasoning_texts=(), cwd=None):
    """造一个临时会话文件, 参数是每条 assistant 正文 / reasoning 的字符数。"""
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    with os.fdopen(fd, "w") as fh:
        fh.write(json.dumps({
            "type": "message",
            "role": "user",
            "content": [{"type": "input_text", "text": "hi"}],
            "cwd": cwd or WS,
        }) + "\n")
        for n in reasoning_texts:
            fh.write(json.dumps({
                "type": "reasoning",
                "rawContent": [{"type": "reasoning_text", "text": "r" * n}],
                "cwd": cwd or WS,
            }) + "\n")
        for n in assistant_texts:
            fh.write(json.dumps({
                "type": "message",
                "role": "assistant",
                "content": [{"type": "output_text", "text": "x" * n}],
                "cwd": cwd or WS,
            }) + "\n")
    return path


def base_payload(tool, tool_input, transcript=""):
    return {
        "session_id": "test-session",
        "cwd": WS,
        "tool_name": tool,
        "tool_input": tool_input,
        "transcript_path": transcript,
        "hook_event_name": "PreToolUse",
    }


def clear_state():
    try:
        os.remove(KILL)
    except OSError:
        pass


# ---------------------------------------------------------------- context_budget
print("context_budget.py")
clear_state()
tp = make_transcript([100, 100])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("小输出放行", r.get("permission") == "allow", r)

tp = make_transcript([100, 35000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("单轮超 30000 拦截", r.get("permission") == "deny", r)
check("拦截原因回给模型", "permissionDecisionReason" in r, r)

tp = make_transcript([10000, 10000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("累计未到线放行", r.get("permission") == "allow", r)

tp = make_transcript([30000] * 4 + [25000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("累计负载超线拦截", r.get("permission") == "deny", r)

tp = make_transcript([30000] * 8)
r = run_hook("context_budget.py", base_payload("Write", {"path": "/tmp/x.c"}, tp))
check("写操作不被拦 (结论要能落盘)", r.get("permission") == "allow", r)

tp = make_transcript([9000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("软上限只提醒不拦",
      r.get("permission") == "allow" and "additional_context" in r, r)

tp = make_transcript([40000], cwd="/tmp")
p = base_payload("Read", {"path": "/tmp/x.c"}, tp)
p.pop("cwd")
env = dict(os.environ, PWD="/tmp")
r = run_hook("context_budget.py", p, cwd="/tmp", env=env)
check("非本工程 cwd 放行 (项目守卫)", r.get("permission") == "allow", r)

# ---------------------------------------------------------------- 紧急开关
print("_common.py 紧急开关")
clear_state()
open(KILL, "w").close()
tp = make_transcript([40000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("开关开着时全部放行", r.get("permission") == "allow", r)

old_ts = time.time() - 4000
os.utime(KILL, (old_ts, old_ts))
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("开关过期后自动失效并恢复拦截", r.get("permission") == "deny", r)
check("过期开关被自动删除", not os.path.exists(KILL))

# ---------------------------------------------------------------- verbose
print("limit_verbose_output.py")
clear_state()
tp = make_transcript([100])
r = run_hook("limit_verbose_output.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("短输出不提醒", "additional_context" not in r, r)

tp = make_transcript([9000])
r = run_hook("limit_verbose_output.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("超软上限提醒", "additional_context" in r, r)

clear_state()
print(f"\nPASS={PASS}  FAIL={FAIL}")
sys.exit(1 if FAIL else 0)
