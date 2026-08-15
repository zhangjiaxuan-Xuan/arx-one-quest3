#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
ADB="$ROOT/tools/quest_adb.sh"
PACKAGE="com.openai.arx.openteach.bimanual"
DURATION="${1:-90}"
LOG_DIR="$ROOT/logs/quest3"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/calibration_$(date +%Y%m%d_%H%M%S).log"
cd "$ROOT"
python3 quest3_calibrate.py --duration "$DURATION" 2>&1 | tee "$LOG_FILE" &
pid=$!
cleanup() {
  kill "$pid" 2>/dev/null || true
  "$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
sleep 1
$ADB shell am force-stop "$PACKAGE"
$ADB shell monkey -p "$PACKAGE" -c android.intent.category.LAUNCHER 1 >/dev/null
wait "$pid"
ln -sfn "$(basename "$LOG_FILE")" "$LOG_DIR/latest_calibration.log"
echo "标定日志：$LOG_FILE"
