#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
ADB="$ROOT/tools/quest_adb.sh"
PACKAGE="com.openai.arx.openteach.bimanual"
LOG_DIR="$ROOT/logs/quest3"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/quest_robot_only_$STAMP.log"
CAN0_LOG="${LOG_FILE%.log}_can0.log"
CAN1_LOG="${LOG_FILE%.log}_can1.log"
READY_FILE="/tmp/arx_quest_robot_only_ready_$STAMP"
robot_pid=""
can0_pid=""
can1_pid=""
mkdir -p "$LOG_DIR"
cd "$ROOT"

request_shutdown() {
  if [[ -n "$robot_pid" ]] && kill -0 "$robot_pid" 2>/dev/null; then
    echo "已撤销Quest授权；等待机械臂安全返回停机位。"
    kill -INT "$robot_pid" 2>/dev/null || true
    wait "$robot_pid" || true
  fi
  robot_pid=""
}
stop_can_traces() {
  [[ -z "$can0_pid" ]] || kill "$can0_pid" 2>/dev/null || true
  [[ -z "$can1_pid" ]] || kill "$can1_pid" 2>/dev/null || true
  [[ -z "$can0_pid" ]] || wait "$can0_pid" 2>/dev/null || true
  [[ -z "$can1_pid" ]] || wait "$can1_pid" 2>/dev/null || true
  can0_pid=""
  can1_pid=""
}
cleanup() {
  request_shutdown
  stop_can_traces
  rm -f "$READY_FILE"
  "$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'request_shutdown; exit 130' INT TERM HUP

if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[v]alidate_quest3_robot_only.py|[v]alidate_persistent_session_(passive|motion).py|[v]alidate_hand_collection_lifecycle.py|[c]apture_demo.py|[r]eplay_pi05.py' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动第二个SDK实例。" | tee "$LOG_FILE"
  exit 1
fi

echo "[1/3] 被动检查双臂CAN接口" | tee "$LOG_FILE"
"$PYTHON" preflight_arx_startup.py 2>&1 | tee -a "$LOG_FILE"
candump -t a -e -x can0 >"$CAN0_LOG" 2>&1 &
can0_pid=$!
candump -t a -e -x can1 >"$CAN1_LOG" 2>&1 &
can1_pid=$!
echo "[2/3] 建立双臂常驻会话并安全回到统一采集初始位" | tee -a "$LOG_FILE"
PYTHONUNBUFFERED=1 "$PYTHON" -u validate_quest3_robot_only.py \
  --duration "${1:-60}" --ready-file "$READY_FILE" \
  < /dev/tty > >(tee -a "$LOG_FILE") 2>&1 &
robot_pid=$!

for _ in $(seq 1 300); do
  [[ -f "$READY_FILE" ]] && break
  if ! kill -0 "$robot_pid" 2>/dev/null; then
    wait "$robot_pid"
    exit $?
  fi
  sleep 0.1
done
if [[ ! -f "$READY_FILE" ]]; then
  echo "30秒内未完成双臂初始化和初始位回归，开始安全退出。" | tee -a "$LOG_FILE"
  request_shutdown
  exit 1
fi

echo "[3/3] 启动Quest APK；相机推流与数据采集保持关闭" | tee -a "$LOG_FILE"
"$ADB" shell am force-stop "$PACKAGE" >/dev/null
"$ADB" shell monkey -p "$PACKAGE" -c android.intent.category.LAUNCHER 1 >/dev/null
echo "测试已启动：Grip按住跟随，Trigger控制夹爪；${1:-60}秒后自动安全回停。" | tee -a "$LOG_FILE"

set +e
wait "$robot_pid"
status=$?
set -e
robot_pid=""
stop_can_traces
echo "Quest双臂无相机测试日志：$LOG_FILE"
echo "双臂CAN被动追踪：$CAN0_LOG | $CAN1_LOG"
exit "$status"
