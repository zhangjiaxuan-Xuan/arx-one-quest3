#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/quest3"
LOG_FILE="$LOG_DIR/persistent_motion_$(date +%Y%m%d_%H%M%S).log"
CAN0_LOG="${LOG_FILE%.log}_can0.log"
CAN1_LOG="${LOG_FILE%.log}_can1.log"
mkdir -p "$LOG_DIR"
cd "$ROOT"

can0_pid=""
can1_pid=""
cleanup_can_logs() {
  [[ -z "$can0_pid" ]] || kill "$can0_pid" 2>/dev/null || true
  [[ -z "$can1_pid" ]] || kill "$can1_pid" 2>/dev/null || true
  [[ -z "$can0_pid" ]] || wait "$can0_pid" 2>/dev/null || true
  [[ -z "$can1_pid" ]] || wait "$can1_pid" 2>/dev/null || true
}
trap cleanup_can_logs EXIT

if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[v]alidate_clean_gravity_compensation.py|[v]alidate_persistent_session_(passive|motion).py|[v]alidate_hand_collection_lifecycle.py|[c]apture_demo.py|[r]eplay_pi05.py' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动第二个SDK实例。" | tee "$LOG_FILE"
  exit 1
fi

echo "[1/2] 被动检查双臂CAN接口" | tee "$LOG_FILE"
"$PYTHON" preflight_arx_startup.py 2>&1 | tee -a "$LOG_FILE"
echo "[2/2] 常驻封装运动测试：停机位→初始位→保持${1:-30}秒→停机位；Quest/相机关闭" | tee -a "$LOG_FILE"
candump -t a -e -x can0 >"$CAN0_LOG" 2>&1 &
can0_pid=$!
candump -t a -e -x can1 >"$CAN1_LOG" 2>&1 &
can1_pid=$!
set +e
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_persistent_session_motion.py \
  --hold-seconds "${1:-30}" --refresh-hz "${2:-50}" 2>&1 | tee -a "$LOG_FILE"
status="${PIPESTATUS[0]}"
set -e
cleanup_can_logs
can0_pid=""
can1_pid=""
echo "常驻封装运动测试日志：$LOG_FILE"
echo "原始CAN日志：$CAN0_LOG | $CAN1_LOG"
exit "$status"
