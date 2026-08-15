#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
ADB="$ROOT/tools/quest_adb.sh"
PACKAGE="com.openai.arx.openteach.bimanual"
DURATION="${1:-60}"
LOG_DIR="$ROOT/logs/quest3"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/monitor_$(date +%Y%m%d_%H%M%S).log"

device_state="$($ADB get-state 2>/dev/null || true)"
if [[ "$device_state" != "device" ]]; then
  echo "Quest ADB 未就绪（当前：${device_state:-未连接}）" >&2
  exit 1
fi

cd "$ROOT"
python3 quest3_monitor.py --duration "$DURATION" 2>&1 | tee "$LOG_FILE" &
monitor_pid=$!
cleanup() {
  kill "$monitor_pid" 2>/dev/null || true
  "$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# Give the UDP receiver time to bind before the Quest client broadcasts.
sleep 1
$ADB shell am force-stop "$PACKAGE"
$ADB shell monkey -p "$PACKAGE" -c android.intent.category.LAUNCHER 1 >/dev/null
echo "Quest 应用已自动启动；等待边框变绿并接收双手柄数据。"

wait "$monitor_pid"
ln -sfn "$(basename "$LOG_FILE")" "$LOG_DIR/latest_monitor.log"
echo "监视日志：$LOG_FILE"
