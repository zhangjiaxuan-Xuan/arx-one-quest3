#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/quest3"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/actuation_preflight_$(date +%Y%m%d_%H%M%S).log"

cd "$ROOT"
if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[c]apture_demo.py|[r]eplay_pi05.py|[r]emote_delta_roundtrip.py|[v]alidate_quest3_output_stability.py|[v]alidate_clean_gravity_compensation.py|[v]alidate_persistent_session_passive.py|[v]alidate_hand_collection_lifecycle.py|[v]alidate_arx_' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动第二个SDK实例。" | tee "$LOG_FILE"
  exit 1
fi
"$PYTHON" preflight_arx_startup.py | tee "$LOG_FILE"
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_arx_actuation_preflight.py 2>&1 | tee -a "$LOG_FILE"
echo "微动预检日志：$LOG_FILE"
