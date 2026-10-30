#!/usr/bin/env python3
"""hook 自测脚本。跑法: python3 scripts/test_hooks.py

覆盖:
  1. context_budget: 累计超预算 / 单轮超限 / 正常放行 / 写操作不拦
  2. _common: 紧急开关 30 分钟自动过期
"""

import json
import os
import subprocess
import sys
import tempfile
import time

HOOKS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".cursor", "hooks"
)
KILL = "/tmp/esp_balance_robot_ctx_guard_off"
STATE = "/tmp/esp_balance_robot_ctx_guard.json"
SUBSTATUS = "/tmp/esp_balance_robot_subagent_status"

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


def run_hook(script, payload):
    p = subprocess.run(
        [sys.executable, os.path.join(HOOKS, script)],
        input=json.dumps(payload),
        capture_output=True,
        text=True,
    )
    try:
        return json.loads(p.stdout.strip().splitlines()[-1])
    except Exception:
        return {"permission": "allow", "_raw": p.stdout, "_err": p.stderr}


def make_transcript(assistant_texts):
    """造一个临时 transcript 文件, assistant_texts 是每条 assistant 记录的正文长度。"""
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    with os.fdopen(fd, "w") as fh:
        for n in assistant_texts:
            fh.write(json.dumps({
                "role": "assistant",
                "message": {"content": [{"type": "text", "text": "x" * n}]},
            }) + "\n")
    return path


def base_payload(tool, tool_input, transcript=""):
    return {
        "session_id": "test-session",
        "conversation_id": "test-session",
        "tool_name": tool,
        "tool_input": tool_input,
        "transcript_path": transcript,
        "hook_event_name": "preToolUse",
    }


def clear_state():
    for f in (STATE, KILL, SUBSTATUS):
        try:
            os.remove(f)
        except OSError:
            pass


# ---------------------------------------------------------------- context_budget
print("context_budget.py")
clear_state()
tp = make_transcript([100, 100])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("小输出放行", r.get("permission") == "allow", r)

tp = make_transcript([100, 25000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("单轮超 20000 拦截", r.get("permission") == "deny", r)

tp = make_transcript([190000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("累计超 150000 拦截", r.get("permission") == "deny", r)

tp = make_transcript([190000])
r = run_hook("context_budget.py", base_payload("Write", {"path": "/tmp/x.c"}, tp))
check("写操作不被拦 (结论要能落盘)", r.get("permission") == "allow", r)

tp = make_transcript([9000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("软上限只提醒不拦",
      r.get("permission") == "allow" and "additional_context" in r, r)

# ---------------------------------------------------------------- 紧急开关
print("_common.py 紧急开关")
clear_state()
open(KILL, "w").close()
tp = make_transcript([190000])
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("开关开着时全部放行", r.get("permission") == "allow", r)

old_ts = time.time() - 4000
os.utime(KILL, (old_ts, old_ts))
r = run_hook("context_budget.py", base_payload("Read", {"path": "/tmp/x.c"}, tp))
check("开关过期后自动失效并恢复拦截", r.get("permission") == "deny", r)
check("过期开关被自动删除", not os.path.exists(KILL))

clear_state()
print(f"\nPASS={PASS}  FAIL={FAIL}")
sys.exit(1 if FAIL else 0)
