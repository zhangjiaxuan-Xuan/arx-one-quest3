#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/quest3"
LOG_FILE="$LOG_DIR/official_lifecycle_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$LOG_DIR"
cd "$ROOT"

if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[v]alidate_.*(gravity|lifecycle|motion|passive)|[c]apture_demo.py|[r]eplay_pi05.py' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动第二个SDK实例。" | tee "$LOG_FILE"
  exit 1
fi

echo "官方SDK无看门狗生命周期测试：初始位→重力补偿${1:-30}s→停机位→重力补偿${1:-30}s→断开" | tee "$LOG_FILE"
set +e
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_official_lifecycle_no_watchdog.py \
  --gravity-seconds "${1:-30}" --move-seconds "${2:-5}" \
  < /dev/tty 2>&1 | tee -a "$LOG_FILE"
status="${PIPESTATUS[0]}"
set -e
echo "官方SDK无看门狗生命周期日志：$LOG_FILE"
exit "$status"
