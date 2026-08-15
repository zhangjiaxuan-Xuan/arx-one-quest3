#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
ADB="$ROOT/tools/quest_adb.sh"
PACKAGE="com.openai.arx.openteach.bimanual"
LOG_DIR="$ROOT/logs/quest3"
mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"

receiver_pid=""
stream_pid=""
cleanup() {
  [[ -z "$stream_pid" ]] || kill "$stream_pid" 2>/dev/null || true
  [[ -z "$receiver_pid" ]] || kill "$receiver_pid" 2>/dev/null || true
  "$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

cd "$ROOT"
"$PYTHON" quest3_monitor.py --duration 3600 >"$LOG_DIR/preview_receiver_$STAMP.log" 2>&1 &
receiver_pid=$!
"$PYTHON" quest3_camera_stream.py --fps 20 --quality 80 \
  > >(tee "$LOG_DIR/preview_camera_$STAMP.log") 2>&1 &
stream_pid=$!

sleep 2
if ! kill -0 "$stream_pid" 2>/dev/null; then
  echo "三相机发布器启动失败："
  cat "$LOG_DIR/preview_camera_$STAMP.log"
  exit 1
fi

"$ADB" shell am force-stop "$PACKAGE"
"$ADB" shell monkey -p "$PACKAGE" -c android.intent.category.LAUNCHER 1 >/dev/null
echo "Quest三相机预览已启动：左臂 | 第三视角 | 右臂"
echo "观察完成后按 Ctrl+C；程序会释放相机并退出VR应用。"
wait "$stream_pid"
