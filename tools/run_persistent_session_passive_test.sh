#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/quest3"
LOG_FILE="$LOG_DIR/persistent_passive_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$LOG_DIR"
cd "$ROOT"

if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[v]alidate_clean_gravity_compensation.py|[v]alidate_persistent_session_passive.py|[v]alidate_hand_collection_lifecycle.py|[c]apture_demo.py|[r]eplay_pi05.py' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动第二个SDK实例。" | tee "$LOG_FILE"
  exit 1
fi

echo "[1/2] 被动检查CAN接口；不以idle判定电机离线" | tee "$LOG_FILE"
"$PYTHON" preflight_arx_startup.py 2>&1 | tee -a "$LOG_FILE"
echo "[2/2] 启动${1:-180}秒项目常驻会话被动测试；Quest/相机/位置命令均关闭" | tee -a "$LOG_FILE"
set +e
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_persistent_session_passive.py \
  --duration "${1:-180}" 2>&1 | tee -a "$LOG_FILE"
status="${PIPESTATUS[0]}"
set -e
echo "项目常驻会话被动测试日志：$LOG_FILE"
exit "$status"
