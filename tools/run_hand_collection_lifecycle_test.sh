#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/quest3"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/hand_lifecycle_$(date +%Y%m%d_%H%M%S).log"

cd "$ROOT"
if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[c]apture_demo.py|[r]eplay_pi05.py|[r]emote_delta_roundtrip.py|[v]alidate_arx_' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动基线测试。" | tee "$LOG_FILE"
  exit 1
fi
"$PYTHON" preflight_arx_startup.py | tee "$LOG_FILE"
CAN0_LOG="${LOG_FILE%.log}_can0.log"
CAN1_LOG="${LOG_FILE%.log}_can1.log"
candump -t a -e -x can0 >"$CAN0_LOG" 2>&1 &
CAN0_PID=$!
candump -t a -e -x can1 >"$CAN1_LOG" 2>&1 &
CAN1_PID=$!
cleanup_trace() {
  kill -TERM "$CAN0_PID" "$CAN1_PID" 2>/dev/null || true
  wait "$CAN0_PID" 2>/dev/null || true
  wait "$CAN1_PID" 2>/dev/null || true
}
trap cleanup_trace EXIT INT TERM
set +e
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_hand_collection_lifecycle.py 2>&1 | tee -a "$LOG_FILE"
TEST_STATUS=${PIPESTATUS[0]}
set -e
cleanup_trace
trap - EXIT INT TERM
for CAN_LOG in "$CAN0_LOG" "$CAN1_LOG"; do
  awk '
    / RX / { if (!rx_first) rx_first=$1; rx_last=$1; rx++ }
    / TX / { if (!tx_first) tx_first=$1; tx_last=$1; tx++ }
    END { printf "%s RX=%d first=%s last=%s TX=%d first=%s last=%s\n", FILENAME, rx, rx_first, rx_last, tx, tx_first, tx_last }
  ' "$CAN_LOG" | tee -a "$LOG_FILE"
done
echo "双臂CAN追踪：$CAN0_LOG | $CAN1_LOG" | tee -a "$LOG_FILE"
echo "手持采集生命周期基线日志：$LOG_FILE"
exit "$TEST_STATUS"
