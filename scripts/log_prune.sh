#!/usr/bin/env bash
#
# log_prune.sh —— 清理 logs/ 下超过保留天数的日志文件
#
# 背景: 项目约定本地 logs/ 只保留最近 3 天的日志, 更早的日志要先推送到
#       GitHub 再从本地删除 (内容仍在 git 历史里, 随时可取回)。
#       日常读日志只看最近 1~2 天, 更早的需要用户指定日期再取。
#
# 用法:
#   scripts/log_prune.sh              # 保留最近 3 天 (默认)
#   scripts/log_prune.sh 7            # 保留最近 7 天
#   scripts/log_prune.sh --dry-run    # 只打印要删哪些, 不实际删除
#
# 返回: 0 = 正常 (可能删了文件); 1 = 有未提交/未推送的内容, 已中止并提示
#
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="$ROOT/logs"
KEEP_DAYS=3
DRY_RUN=0

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    *[!0-9]*|"") echo "参数错误: '$arg' (应为天数或 --dry-run)"; exit 1 ;;
    *) KEEP_DAYS="$arg" ;;
  esac
done

[ -d "$LOGDIR" ] || { echo "找不到 $LOGDIR"; exit 1; }

# 今天减去 (KEEP_DAYS - 1) 天 = 保留窗口的起始日期 (保留 3 天 = 今天+昨天+前天)
CUTOFF="$(date -v-$((KEEP_DAYS - 1))d +%Y-%m-%d 2>/dev/null || date -d "$((KEEP_DAYS - 1)) days ago" +%Y-%m-%d)"
echo "保留窗口: ${CUTOFF} 及之后 (最近 ${KEEP_DAYS} 天), 截止日期 ${CUTOFF}"

# --- 前置检查: logs/ 里有没有没提交的改动 ---
DIRTY="$(cd "$ROOT" && git status --porcelain -- logs/ | head -5)"
if [ -n "$DIRTY" ]; then
  echo "❌ logs/ 里有未提交的改动, 先提交再清理:"
  echo "$DIRTY"
  exit 1
fi

# --- 前置检查: 有没有没推送到 GitHub 的提交 (删了就没法从远端取了) ---
UNPUSHED="$(cd "$ROOT" && git log --oneline @{u}..HEAD 2>/dev/null | head -5)"
if [ -n "$UNPUSHED" ]; then
  echo "❌ 有未推送的提交, 先 push 再清理 (否则删掉的日志远端没有):"
  echo "$UNPUSHED"
  exit 1
fi

# --- 找出要删的日志 ---
TO_DELETE=()
for f in "$LOGDIR"/[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9].md; do
  [ -e "$f" ] || continue
  d="$(basename "$f" .md)"
  if [[ "$d" < "$CUTOFF" ]]; then
    TO_DELETE+=("$f")
  fi
done

if [ ${#TO_DELETE[@]} -eq 0 ]; then
  echo "✅ 没有需要清理的日志 (本地已是最近 ${KEEP_DAYS} 天)"
  exit 0
fi

echo "准备删除 ${#TO_DELETE[@]} 个日志文件:"
for f in "${TO_DELETE[@]}"; do
  echo "  - $(basename "$f")"
done

if [ "$DRY_RUN" -eq 1 ]; then
  echo "(dry-run, 未实际删除)"
  exit 0
fi

rm -f "${TO_DELETE[@]}"
echo "✅ 已删除 ${#TO_DELETE[@]} 个日志文件, 记得 git add -A logs/ 后提交并推送"
