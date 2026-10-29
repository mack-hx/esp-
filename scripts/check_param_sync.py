#!/usr/bin/env python3
"""核对车端 param.c 与 App paramDef.js 的参数表是否逐项一致。

不一致的后果 (踩过的坑): App 的"恢复默认"曾经把 App 内置的默认值整份下发到车端,
而那份表跟车端脱节 → 速度环被意外打开 → 车只朝一个方向转。
0.32 又踩了一次类似的: bal_kd 车端 0.4 / App 1.5 → App 一下发就把车打回打摆状态。

用法: python3 scripts/check_param_sync.py [版本目录]
"""
import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VER = sys.argv[1] if len(sys.argv) > 1 else "0.40"
BASE = os.path.join(ROOT, VER)

# ---------- 车端: user/param/param.c ----------
pc = io.open(os.path.join(BASE, "user/param/param.c"), encoding="utf-8").read()

# 版本号
m = re.search(r"#define\s+PARAM_DATA_VERSION\s+(\d+)", pc)
fw_ver = int(m.group(1)) if m else None

# 条目: { PARAM_XXX, "key", "label", "unit", def, min, max, step, dec, grp, "hint" }
# ⚠ grp 字段写的是枚举名 (PARAM_GRP_BALANCE), 不是数字 —— 两种都要能匹配
pat = re.compile(
    r"\{\s*(PARAM_[A-Z0-9_]+)\s*,\s*\"(\w+)\"\s*,\s*\"[^\"]*\"\s*,\s*\"[^\"]*\"\s*,"
    r"\s*([-\d.]+)f\s*,\s*([-\d.]+)f\s*,\s*([-\d.]+)f\s*,\s*([-\d.]+)f\s*,\s*(\d+)\s*,"
    r"\s*(PARAM_GRP_[A-Z]+|\d+)\s*,",
    re.M)
# 枚举名 → 数字 (跟 param.h 的定义顺序一致)
GRP = {"PARAM_GRP_BALANCE": 0, "PARAM_GRP_SPEED": 1, "PARAM_GRP_LIMIT": 2}
fw = {}
order = []
for mm in pat.finditer(pc):
    key = mm.group(2)
    g = mm.group(8)
    fw[key] = {
        "def": float(mm.group(3)),
        "min": float(mm.group(4)),
        "max": float(mm.group(5)),
        "step": float(mm.group(6)),
        "dec": int(mm.group(7)),
        "grp": GRP.get(g, int(g) if g.isdigit() else -1),
    }
    order.append(key)

# ---------- App: app/common/paramDef.js ----------
js = io.open(os.path.join(BASE, "app/common/paramDef.js"), encoding="utf-8").read()
m = re.search(r"export const PARAM_VER\s*=\s*(\d+)", js)
app_ver = int(m.group(1)) if m else None

apat = re.compile(
    r"\{\s*id:\s*(\d+)\s*,\s*key:\s*'(\w+)'\s*,[^{}]*?def:\s*([-\d.]+)\s*,"
    r"\s*min:\s*([-\d.]+)\s*,\s*max:\s*([-\d.]+)\s*,\s*step:\s*([-\d.]+)\s*,\s*dec:\s*(\d+)\s*,\s*grp:\s*(\d+)\s*,",
    re.S)
app = {}
for mm in apat.finditer(js):
    app[mm.group(2)] = {
        "id": int(mm.group(1)),
        "def": float(mm.group(3)),
        "min": float(mm.group(4)),
        "max": float(mm.group(5)),
        "step": float(mm.group(6)),
        "dec": int(mm.group(7)),
        "grp": int(mm.group(8)),
    }

fail = 0
print("参数表版本: 车端 %s / App %s" % (fw_ver, app_ver))
if fw_ver != app_ver:
    print("  [FAIL] 版本号不一致")
    fail += 1

if set(fw) != set(app):
    print("  [FAIL] 参数集合不一致")
    print("        车端独有:", sorted(set(fw) - set(app)))
    print("        App 独有 :", sorted(set(app) - set(fw)))
    fail += 1

print("参数数量: 车端 %d / App %d" % (len(fw), len(app)))
if len(fw) != len(app):
    print("  [FAIL] 数量不一致")
    fail += 1

for key in order:
    if key not in app:
        continue
    a, b = fw[key], app[key]
    for field in ("def", "min", "max", "step"):
        if abs(a[field] - b[field]) > 1e-9:
            print("  [FAIL] %-12s %s: 车端 %s / App %s" % (key, field, a[field], b[field]))
            fail += 1
    for field in ("dec", "grp"):
        if a[field] != b[field]:
            print("  [FAIL] %-12s %s: 车端 %s / App %s" % (key, field, a[field], b[field]))
            fail += 1

if fail == 0:
    print("  [OK] %d 项参数逐项一致 (def/min/max/step/dec/grp)" % len(fw))
else:
    print("  [FAIL] 共 %d 处不一致" % fail)

sys.exit(1 if fail else 0)
