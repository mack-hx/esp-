#!/bin/bash
# 外部子代理: 用 claude CLI 跑一个独立会话, 把结论写到文件, 主对话只读摘要。
#
# 用法:
#   scripts/subagent.sh <prompt文件> [输出名]
# 例:
#   scripts/subagent.sh /tmp/p.txt speed_loop
#
# 产出:
#   logs/subagents/<输出名>-<时间戳>.md   完整报告 (主对话不要整读)
#   标准输出                            只回 20 行摘要 + 文件路径
set -euo pipefail

PROMPT_FILE="${1:?用法: scripts/subagent.sh <prompt文件> [输出名]}"
NAME="${2:-subagent}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLAUDE_BIN="/Volumes/Moving/app/npm-global/bin/claude"
OUT_DIR="$ROOT/logs/subagents"
mkdir -p "$OUT_DIR"
STAMP="$(date +%m%d-%H%M%S)"
OUT="$OUT_DIR/${NAME}-${STAMP}.md"

STATUS_FILE="/tmp/esp_balance_robot_subagent_status"

cd "$ROOT"
# --permission-mode acceptEdits: 只读任务里不会改文件, 但允许它读文件/跑 grep
# --allowedTools 限制在只读工具上, 防止误改
"$CLAUDE_BIN" -p "$(cat "$PROMPT_FILE")" \
  --permission-mode acceptEdits \
  --allowedTools "Read,Grep,Glob,Bash(git log:*),Bash(git show:*),Bash(git diff:*),Bash(ls:*),Bash(wc:*),Bash(grep:*),Bash(find:*),Bash(sed -n:*)" \
  > "$OUT" 2>"$OUT.err" || true

# 可用性状态: Claude CLI 返回 402 表示额度用完, 这时报告是空的。
# 状态文件供 hook 判断"能不能派子代理", 必须写真实结果, 不能乐观假设 ok。
if grep -q "402 Insufficient Balance" "$OUT" "$OUT.err" 2>/dev/null; then
  echo "402" > "$STATUS_FILE"
  echo "=== 子代理不可用: Claude CLI 返回 402 (额度用完) ==="
  echo "报告: $OUT"
  exit 0
fi
if [ ! -s "$OUT" ]; then
  echo "402" > "$STATUS_FILE"
  echo "=== 子代理不可用: 报告为空 ==="
  echo "stderr:"
  head -5 "$OUT.err"
  exit 0
fi
echo "ok" > "$STATUS_FILE"

echo "=== 子代理完成: $NAME ==="
echo "完整报告: $OUT"
[ -s "$OUT.err" ] && echo "stderr: $OUT.err"
echo "--- 摘要 (前 20 行) ---"
head -20 "$OUT"
