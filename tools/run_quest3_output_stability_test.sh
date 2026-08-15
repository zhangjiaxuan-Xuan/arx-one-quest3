#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/quest3"
STAMP="$(date +%Y%m%d_%H%M%S)"
MODE="${1:-synthetic}"
shift || true

case "$MODE" in
  synthetic)
    DURATION="${1:-32}"
    [[ $# -gt 0 ]] && shift
    ;;
  live)
    DURATION="${1:-180}"
    [[ $# -gt 0 ]] && shift
    ;;
  *)
    echo "用法：$0 [synthetic [seconds] | live [seconds] [--quest-host IP]]" >&2
    exit 2
    ;;
esac

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/no_arm_stability_${MODE}_${STAMP}.log"
REPORT_FILE="$LOG_DIR/no_arm_stability_${MODE}_${STAMP}.json"
cd "$ROOT"

if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[v]alidate_.*gravity|[v]alidate_persistent_session_passive.py|[v]alidate_hand_collection_lifecycle.py|[c]apture_demo.py|[r]eplay_pi05.py' >/dev/null; then
  echo "检测到机械臂控制进程。无机械臂测试拒绝与任何SDK会话并行运行。" | tee "$LOG_FILE"
  exit 1
fi

echo "启动无机械臂输出稳定性测试：mode=$MODE duration=${DURATION}s" | tee "$LOG_FILE"
echo "此进程不会构造ARX控制器，也不要求机械臂上电。" | tee -a "$LOG_FILE"
set +e
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_quest3_output_stability.py \
  --mode "$MODE" --duration "$DURATION" --report "$REPORT_FILE" "$@" \
  2>&1 | tee -a "$LOG_FILE"
status="${PIPESTATUS[0]}"
set -e
if [[ "$status" -eq 0 ]]; then
  cp "$REPORT_FILE" "$LOG_DIR/no_arm_stability_latest.json"
  cp "${REPORT_FILE%.json}.npz" "$LOG_DIR/no_arm_stability_latest.npz"
fi
echo "无机械臂测试日志：$LOG_FILE"
echo "无机械臂测试报告：$REPORT_FILE"
exit "$status"
