#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/quest3"
LOG_FILE="$LOG_DIR/clean_gravity_$(date +%Y%m%d_%H%M%S).log"
CAN0_LOG="${LOG_FILE%.log}_can0.log"
CAN1_LOG="${LOG_FILE%.log}_can1.log"
mkdir -p "$LOG_DIR"
cd "$ROOT"

if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[v]alidate_quest3_output_stability.py|[v]alidate_clean_gravity_compensation.py|[v]alidate_persistent_session_passive.py|[v]alidate_hand_collection_lifecycle.py|[v]alidate_arx_|[c]apture_demo.py|[r]eplay_pi05.py' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动第二个SDK实例。" | tee "$LOG_FILE"
  exit 1
fi

echo "[1/2] 检查主机CAN接口；idle允许冷启动，电机在线以官方SDK构造和连续反馈为准" | tee "$LOG_FILE"
"$PYTHON" preflight_arx_startup.py 2>&1 | tee -a "$LOG_FILE"
candump -t a -e -x can0 >"$CAN0_LOG" 2>&1 &
can0_pid=$!
candump -t a -e -x can1 >"$CAN1_LOG" 2>&1 &
can1_pid=$!
cleanup_traces() {
  kill "$can0_pid" "$can1_pid" 2>/dev/null || true
  wait "$can0_pid" "$can1_pid" 2>/dev/null || true
}
trap cleanup_traces EXIT INT TERM HUP
echo "[2/2] 启动${1:-180}秒官方旧基线：仅SDK后台收发、重力补偿和状态读取" | tee -a "$LOG_FILE"
set +e
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_clean_gravity_compensation.py \
  --duration "${1:-180}" --observe-after-loss "${2:-10}" \
  < /dev/tty 2>&1 | tee -a "$LOG_FILE"
status="${PIPESTATUS[0]}"
set -e
cleanup_traces
trap - EXIT INT TERM HUP
echo "干净重力补偿日志：$LOG_FILE"
echo "双臂CAN被动追踪：$CAN0_LOG | $CAN1_LOG"
exit "$status"
