#!/bin/sh
# 跑完 0.41 的全部离线单元测试 + 参数表一致性检查
#
# 用法: scripts/run_tests.sh [版本目录]   (默认 0.41)
#
# 全部通过时退出码 0。任何一个失败就退出码非 0。
set -e

ROOT=$(cd "$(dirname "$0")/.." && pwd)
VER=${1:-0.41}
cd "$ROOT/$VER"

FAILED=0

run() {
    name=$1
    shift
    printf '\n===== %s =====\n' "$name"
    if "$@"; then
        :
    else
        echo "  ^^^ $name 失败"
        FAILED=$((FAILED + 1))
    fi
}

# --- 离线单元测试 (PC 上 gcc 直接跑, 不需要硬件) ---
run "trim" sh -c "gcc -std=c99 -Wall -Wextra test/test_trim_pc.c user/trim/trim.c \
    -o /tmp/trim_test -I test -I user/trim -DTRIM_OFFLINE_TEST && /tmp/trim_test"

run "pid" sh -c "gcc -std=c99 -Wall -Wextra test/test_pid_pc.c user/pid/pid.c \
    -o /tmp/pid_test -I test -I user/pid && /tmp/pid_test"

run "param" sh -c "gcc -std=c99 -Wall -Wextra test/test_param_pc.c user/param/param.c \
    -o /tmp/param_test -I test -I user/param -DPARAM_OFFLINE_TEST && /tmp/param_test"

run "speed_pid" sh -c "gcc -std=c99 -Wall -Wextra -DPARAM_OFFLINE_TEST \
    test/test_speed_pid_pc.c user/pid/speed_pid.c user/pid/pid.c \
    -o /tmp/speed_pid_test -I test -I user/pid -I user/param -lm && /tmp/speed_pid_test"

run "position_pid" sh -c "gcc -std=c99 -Wall -Wextra -DPARAM_OFFLINE_TEST \
    test/test_position_pid_pc.c user/pid/position_pid.c user/pid/pid.c \
    -o /tmp/position_pid_test -I test -I user/pid -I user/param -lm && /tmp/position_pid_test"

run "lqr" sh -c "gcc -std=c99 -Wall -Wextra -DPARAM_OFFLINE_TEST \
    test/test_lqr_pc.c user/pid/lqr.c user/pid/pid.c \
    -o /tmp/lqr_test -I test -I user/pid -I user/param -lm && /tmp/lqr_test"

# --- 参数表一致性: 车端 param.c vs App paramDef.js ---
printf '\n===== 参数表一致性 (车端 vs App) =====\n'
if python3 "$ROOT/scripts/check_param_sync.py" "$VER"; then
    :
else
    echo "  ^^^ 参数表不一致"
    FAILED=$((FAILED + 1))
fi

# --- App 端 JS 语法 ---
printf '\n===== App JS 语法 =====\n'
for f in app/common/paramDef.js app/utils/control.js; do
    if node --check "$f"; then
        echo "  [OK] $f"
    else
        echo "  [FAIL] $f"
        FAILED=$((FAILED + 1))
    fi
done

printf '\n==========================================\n'
if [ "$FAILED" -eq 0 ]; then
    echo "全部通过 ✓"
    exit 0
fi
echo "$FAILED 项失败 ✗"
exit 1
