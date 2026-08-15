#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/quest3"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/right_isolated_$(date +%Y%m%d_%H%M%S).log"

cd "$ROOT"
if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[c]apture_demo.py|[r]eplay_pi05.py|[r]emote_delta_roundtrip.py|[v]alidate_arx_' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动右臂诊断。" | tee "$LOG_FILE"
  exit 1
fi
"$PYTHON" preflight_arx_startup.py | tee "$LOG_FILE"
CAN_LOG="${LOG_FILE%.log}_can1.log"
candump -t a -e -x can1 >"$CAN_LOG" 2>&1 &
CANDUMP_PID=$!
cleanup_trace() {
  kill -TERM "$CANDUMP_PID" 2>/dev/null || true
  wait "$CANDUMP_PID" 2>/dev/null || true
}
trap cleanup_trace EXIT INT TERM
set +e
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_arx_right_arm_isolated.py 2>&1 | tee -a "$LOG_FILE"
TEST_STATUS=${PIPESTATUS[0]}
set -e
cleanup_trace
trap - EXIT INT TERM
echo "右臂CAN原始追踪：$CAN_LOG" | tee -a "$LOG_FILE"
echo "右臂隔离诊断日志：$LOG_FILE"
exit "$TEST_STATUS"
