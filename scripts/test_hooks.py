#!/usr/bin/env python3
"""hook 自测脚本。跑法: python3 scripts/test_hooks.py

覆盖:
  1. context_budget: 累计超预算 / 单轮超限 / 正常放行 / 写操作不拦
  2. guard_reads: 重复读拦截 / 大文件整读拦截 / 旧 transcript 拦截
  3. require_subagent: 计数拦截 / 子代理 402 时降级放行
  4. _common: 紧急开关 30 分钟自动过期
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

# ---------------------------------------------------------------- guard_reads
print("guard_reads.py")
clear_state()
target = tempfile.NamedTemporaryFile(suffix=".c", delete=False)
target.write(b"line\n" * 10)
target.close()
p = base_payload("Read", {"path": target.name})
r1 = run_hook("guard_reads.py", p)
r2 = run_hook("guard_reads.py", p)
check("第一次放行", r1.get("permission") == "allow", r1)
check("完全重复读被拦", r2.get("permission") == "deny", r2)

clear_state()
old = os.path.join(os.path.expanduser("~"), ".cursor", "projects", "x",
                   "agent-transcripts", "abc", "abc.jsonl")
r = run_hook("guard_reads.py", base_payload("Read", {"path": old}))
check("读自己旧 transcript 被拦", r.get("permission") == "deny", r)

clear_state()
big = tempfile.NamedTemporaryFile(suffix=".c", delete=False)
big.write(b"line\n" * 900)
big.close()
r = run_hook("guard_reads.py", base_payload("Read", {"path": big.name}))
check("大文件整读被拦", r.get("permission") == "deny", r)

# ---------------------------------------------------------------- require_subagent
print("require_subagent.py")
clear_state()
for i in range(7):
    r = run_hook("require_subagent.py",
                 base_payload("Read", {"path": f"/tmp/f{i}.c"}))
check("读到第 7 次被拦 (要求派子代理)", r.get("permission") == "deny", r)

# 维护类命令不该计入自查次数 (2026-10-30 踩过: 混合命令被误判成查询)
clear_state()
for i in range(7):
    r = run_hook("require_subagent.py", base_payload(
        "Shell", {"command": f"cd /tmp && python3 -c 'print({i})'"}))
check("维护类命令不计入自查次数", r.get("permission") == "allow", r)

clear_state()
for i in range(7):
    r = run_hook("require_subagent.py", base_payload(
        "Shell", {"command": f"cd /tmp && grep -o x file{i}"}))
check("复合命令里的查询类仍被计数", r.get("permission") == "deny", r)

clear_state()
with open(SUBSTATUS, "w") as fh:
    fh.write("402")
r = None
for i in range(9):
    r = run_hook("require_subagent.py",
                 base_payload("Read", {"path": f"/tmp/g{i}.c"}))
check("子代理 402 时降级放行 (不锁死)",
      r.get("permission") == "allow" and "additional_context" in r, r)
clear_state()

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
