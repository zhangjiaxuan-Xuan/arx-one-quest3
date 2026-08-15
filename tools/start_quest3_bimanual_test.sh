#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
ADB="$ROOT/tools/quest_adb.sh"
PACKAGE="com.openai.arx.openteach.bimanual"
LOG_DIR="$ROOT/logs/quest3"
mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
READY_FILE="/tmp/arx_quest3_bimanual_ready_$STAMP"
SHUTDOWN_FILE="/tmp/arx_quest3_bimanual_shutdown_$STAMP"
stream_pid=""
robot_pid=""
robot_started=0
shutdown_requested=0

request_robot_shutdown() {
  if [[ "$shutdown_requested" -eq 1 ]]; then
    return
  fi
  shutdown_requested=1
  if [[ "$robot_started" -eq 0 ]]; then
    return
  fi
  if [[ -n "$robot_pid" ]] && kill -0 "$robot_pid" 2>/dev/null; then
    echo "已请求安全停机：等待机械臂回到停机位置，期间再次 Ctrl-C 不会强杀进程。"
    kill -USR1 "$robot_pid" 2>/dev/null || true
    wait "$robot_pid" 2>/dev/null || true
    robot_pid=""
  fi
  if [[ -f "$SHUTDOWN_FILE" ]]; then
    echo "已收到安全停机确认。"
  else
    echo "警告：未收到安全停机确认标志；启动器没有主动强杀机器人进程。"
  fi
}

cleanup() {
  request_robot_shutdown
  rm -f "$READY_FILE"
  [[ -z "$stream_pid" ]] || kill "$stream_pid" 2>/dev/null || true
  "$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
  rm -f "$SHUTDOWN_FILE"
}
on_interrupt() {
  trap '' INT TERM HUP
  request_robot_shutdown
  exit 130
}
trap cleanup EXIT
trap on_interrupt INT TERM HUP

cd "$ROOT"
if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[c]apture_demo.py|[r]eplay_pi05.py|[r]emote_delta_roundtrip.py|[v]alidate_quest3_output_stability.py|[v]alidate_clean_gravity_compensation.py|[v]alidate_persistent_session_passive.py|[v]alidate_hand_collection_lifecycle.py|[v]alidate_arx_' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动第二个SDK实例。"
  exit 1
fi

echo "[预检1/2] 检查双臂CANable映射、接口状态和总线错误（此阶段不加载SDK）"
if ! "$PYTHON" preflight_arx_startup.py; then
  echo "双臂CAN接口预检失败，拒绝加载ARX SDK。请检查USB-CAN和can0/can1映射。"
  exit 1
fi
echo "[预检2/2] 将在停机位置建立同一常驻SDK会话并验证重力补偿"

duration="${1:-60}"
durability_args=()
if [[ "$duration" -ge 180 ]]; then
  durability_args+=(--durability-plan)
fi
"$PYTHON" quest3_bimanual_test.py --duration "$duration" "${durability_args[@]}" \
  --ready-file "$READY_FILE" --shutdown-status-file "$SHUTDOWN_FILE" \
  > >(tee "$LOG_DIR/bimanual_robot_$STAMP.log") 2>&1 &
robot_pid=$!
robot_started=1

for _ in $(seq 1 200); do
  [[ -f "$READY_FILE" ]] && break
  if ! kill -0 "$robot_pid" 2>/dev/null; then
    echo "双臂SDK初始化或初始位回归失败："
    cat "$LOG_DIR/bimanual_robot_$STAMP.log"
    wait "$robot_pid"
    exit 1
  fi
  sleep 0.1
done
if [[ ! -f "$READY_FILE" ]]; then
  echo "20秒内未完成双臂初始化和采集初始位回归，终止启动。"
  exit 1
fi

"$PYTHON" quest3_camera_stream.py --fps 20 --quality 80 \
  >"$LOG_DIR/bimanual_camera_$STAMP.log" 2>&1 &
stream_pid=$!
sleep 1
if ! kill -0 "$stream_pid" 2>/dev/null; then
  echo "三相机推流启动失败："
  cat "$LOG_DIR/bimanual_camera_$STAMP.log"
  exit 1
fi

"$ADB" shell am force-stop "$PACKAGE"
"$ADB" shell monkey -p "$PACKAGE" -c android.intent.category.LAUNCHER 1 >/dev/null
echo "机械臂已就绪；三相机、Quest姿态与双臂控制已启动。"
wait "$robot_pid"
