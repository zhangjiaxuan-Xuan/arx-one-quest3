#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
ADB="$ROOT/tools/quest_adb.sh"
PACKAGE="com.openai.arx.openteach.bimanual"
LOG_DIR="$ROOT/logs/quest3"
mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/collection_test_$STAMP.log"
: > "$LOG_FILE"
SHUTDOWN_FILE="/tmp/arx_quest3_collection_shutdown_$STAMP"
workflow_pid=""
quest_launcher_pid=""
workflow_started=0
workflow_ready=0
startup_emergency=0
emergency_poweroff_confirmed=0
shutdown_requested=0
shutdown_confirmed=0
shutdown_outcome=""
postflight_checked=0

check_post_shutdown_can() {
  if [[ "$postflight_checked" -eq 1 ]]; then
    return
  fi
  postflight_checked=1
  echo "[退出后检查] 验证双臂CAN活动是否对称" | tee -a "$LOG_FILE"
  if ! "$PYTHON" postflight_arx_shutdown.py 2>&1 | tee -a "$LOG_FILE"; then
    if [[ "$shutdown_outcome" == "safe_shutdown" ]]; then
      echo "机械臂已安全停机，但下次启动前需要：sudo ./tools/recover_arx_can.sh" \
        | tee -a "$LOG_FILE"
    elif [[ "$shutdown_outcome" == "operator_power_off" ]]; then
      echo "反馈失联后已由操作者确认关闭机械臂电源；重新上电后请执行：sudo ./tools/recover_arx_can.sh" \
        | tee -a "$LOG_FILE"
    else
      echo "本次从未获得机械臂控制权，且退出后CAN状态异常；请先恢复机械臂控制电源，再执行：sudo ./tools/recover_arx_can.sh" \
        | tee -a "$LOG_FILE"
    fi
  fi
}

wait_for_verified_shutdown() {
  local state=""
  while true; do
    if [[ -f "$SHUTDOWN_FILE" ]] && grep -qx 'SAFE_SHUTDOWN_COMPLETE' "$SHUTDOWN_FILE"; then
      shutdown_confirmed=1
      shutdown_outcome="safe_shutdown"
      break
    fi
    if [[ -f "$SHUTDOWN_FILE" ]] && grep -qx 'NO_CONTROL_ACQUIRED' "$SHUTDOWN_FILE"; then
      shutdown_confirmed=1
      shutdown_outcome="no_control"
      break
    fi
    if [[ -f "$SHUTDOWN_FILE" ]] && grep -qx 'OPERATOR_POWER_OFF_CONFIRMED' "$SHUTDOWN_FILE"; then
      shutdown_confirmed=1
      shutdown_outcome="operator_power_off"
      break
    fi
    if [[ -z "$workflow_pid" ]]; then
      break
    fi
    state="$(ps -o stat= -p "$workflow_pid" 2>/dev/null | tr -d '[:space:]' || true)"
    if [[ -z "$state" || "$state" == Z* ]]; then
      break
    fi
    sleep 0.2
  done

  # Reap only after the robot process has positively confirmed that it
  # reached the shutdown pose and disconnected its SDK, or after it has
  # already died. A signal-interrupted wait must never decide arm safety.
  if [[ -n "$workflow_pid" ]]; then
    wait "$workflow_pid" 2>/dev/null || true
  fi
  if [[ "$shutdown_confirmed" -ne 1 ]]; then
    echo "致命安全错误：机器人主进程退出但没有停机姿态确认。" >&2
    echo "启动器不会把这种退出报告为安全停机；请切断机械臂电源并检查日志。" >&2
    return 1
  fi
  workflow_pid=""
  if [[ "$shutdown_outcome" == "safe_shutdown" ]]; then
    echo "已收到安全停机确认。"
  elif [[ "$shutdown_outcome" == "operator_power_off" ]]; then
    echo "已收到操作者机械臂断电确认；失联SDK已释放。"
  else
    echo "已确认工作流从未获得机械臂控制权；本次没有可执行的回停动作。"
  fi
  check_post_shutdown_can
}

request_robot_shutdown() {
  if [[ "$shutdown_requested" -eq 1 ]]; then
    return
  fi
  shutdown_requested=1
  if [[ "$workflow_started" -eq 0 ]]; then
    return
  fi
  if [[ -n "$workflow_pid" ]] && kill -0 "$workflow_pid" 2>/dev/null; then
    if [[ "$workflow_ready" -eq 0 ]] && grep -q 'Emergency state entered' "$LOG_FILE"; then
      startup_emergency=1
      echo "SDK在构造阶段进入Emergency，未建立完整双臂会话；禁止盲目自动回位。"
      echo "请支撑双臂并关闭机械臂控制电源，然后按Enter或Ctrl-C退出Emergency进程。"
      emergency_poweroff_confirmed=0
      trap 'emergency_poweroff_confirmed=1' INT TERM HUP
      while [[ "$emergency_poweroff_confirmed" -eq 0 ]]; do
        if IFS= read -r </dev/tty; then
          emergency_poweroff_confirmed=1
        fi
      done
      trap on_interrupt INT TERM HUP
      printf '%s\n' 'OPERATOR_POWER_OFF_CONFIRMED' > "$SHUTDOWN_FILE"
      kill -KILL "$workflow_pid" 2>/dev/null || true
    else
      echo "已请求安全停机：等待机械臂回到停机位置。"
      kill -USR1 "$workflow_pid" 2>/dev/null || true
    fi
    wait_for_verified_shutdown
    return
  fi
  wait_for_verified_shutdown
}

cleanup() {
  if [[ -n "$quest_launcher_pid" ]] && kill -0 "$quest_launcher_pid" 2>/dev/null; then
    kill -TERM "$quest_launcher_pid" 2>/dev/null || true
    wait "$quest_launcher_pid" 2>/dev/null || true
  fi
  if [[ "$workflow_started" -eq 1 && "$shutdown_confirmed" -ne 1 ]]; then
    request_robot_shutdown || true
  fi
  if [[ "$shutdown_confirmed" -eq 1 || "$workflow_started" -eq 0 ]]; then
    timeout 2 "$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
    rm -f "$SHUTDOWN_FILE"
  fi
}

start_quest_when_available() {
  # This helper runs in a background subshell. Quest sleep/offline is expected
  # and must never trigger the launcher's EXIT safety trap.
  trap - EXIT INT TERM HUP
  local announced=0
  while [[ -n "$workflow_pid" ]] && kill -0 "$workflow_pid" 2>/dev/null; do
    if timeout 2 "$ADB" get-state >/dev/null 2>&1; then
      timeout 3 "$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
      if timeout 5 "$ADB" shell monkey -p "$PACKAGE" \
          -c android.intent.category.LAUNCHER 1 >/dev/null 2>&1; then
        echo "Quest已唤醒，采集APK已自动启动。" | tee -a "$LOG_FILE"
        return 0
      fi
    fi
    if [[ "$announced" -eq 0 ]]; then
      echo "Quest当前休眠/离线；机械臂保持初始位，后台等待唤醒（最长180秒）。" \
        | tee -a "$LOG_FILE"
      announced=1
    fi
    sleep 2
  done
}
on_interrupt() {
  trap '' INT TERM HUP
  if ! request_robot_shutdown; then
    exit 125
  fi
  exit 130
}
trap cleanup EXIT
trap on_interrupt INT TERM HUP

cd "$ROOT"
if pgrep -f '[c]ollect_workflow.py|[c]alibrate_arx_grippers.py|[q]uest3_bimanual_test.py|[c]apture_demo.py|[r]eplay_pi05.py|[r]emote_delta_roundtrip.py|[v]alidate_quest3_output_stability.py|[v]alidate_clean_gravity_compensation.py|[v]alidate_persistent_session_passive.py|[v]alidate_hand_collection_lifecycle.py|[v]alidate_arx_' >/dev/null; then
  echo "检测到其他机械臂控制进程，拒绝启动第二个SDK实例。"
  exit 1
fi

echo "[预检1/3] 检查双臂CANable映射、接口状态和总线错误（此阶段不加载SDK）"
if ! "$PYTHON" preflight_arx_startup.py 2>&1 | tee -a "$LOG_FILE"; then
  echo "双臂CAN接口预检失败，拒绝加载ARX SDK。请检查USB-CAN和can0/can1映射。"
  exit 1
fi
echo "[预检2/3] 检查三相机与双CANable的USB根总线隔离"
if ! "$PYTHON" preflight_arx_usb_topology.py 2>&1 | tee -a "$LOG_FILE"; then
  echo "USB拓扑不满足持续三视觉采集安全要求，拒绝加载ARX SDK。"
  exit 1
fi
echo "[预检3/3] 将在停机位置建立同一常驻SDK会话并验证重力补偿"

if ! exec {WORKFLOW_STDIN_FD}</dev/tty; then
  echo "无法打开当前终端 /dev/tty；拒绝以无键盘控制的方式启动采集流程。"
  exit 1
fi

PYTHONUNBUFFERED=1 "$PYTHON" -u collect_workflow.py \
  --input quest --sessions-root "$ROOT/sessions_vr" \
  --shutdown-status-file "$SHUTDOWN_FILE" "$@" \
  <&"$WORKFLOW_STDIN_FD" > >(tee -a "$LOG_FILE") 2>&1 &
workflow_pid=$!
workflow_started=1
exec {WORKFLOW_STDIN_FD}<&-

ready=""
for _ in $(seq 1 150); do
  if grep -q 'ARX AC One Quest 3 三视觉双臂采集控制台' "$LOG_FILE"; then
    ready=1
    break
  fi
  if ! kill -0 "$workflow_pid" 2>/dev/null; then
    break
  fi
  if grep -q 'Emergency state entered' "$LOG_FILE"; then
    startup_emergency=1
    break
  fi
  sleep 0.2
done
if [[ -n "$ready" ]]; then
  workflow_ready=1
fi
if [[ -z "$ready" ]]; then
  echo "采集工作流未就绪，拒绝启动Quest："
  tail -120 "$LOG_FILE"
  if [[ "$startup_emergency" -eq 1 ]]; then
    echo "故障定位：厂商SDK构造进入不可返回的Emergency循环；不会执行自动回位。"
  fi
  if grep -q 'None of the motors are initialized' "$LOG_FILE"; then
    cat <<'EOF'
故障定位：官方ARX SDK在构造第一个控制器时未发现任何电机。
这发生在Quest、IK、增益和采集逻辑启动之前；重建CANable/SLCAN不能复位机械臂内部电机控制器。
恢复步骤：支撑双臂并清空工作区，关闭机械臂控制电源，等待10秒后重新上电；随后执行：
  sudo ./tools/recover_arx_can.sh
再重新运行本启动脚本。不要为此反复使用 --usb-reset。
EOF
  fi
  # Never discard the PID merely because readiness was not printed. The child
  # must report either a verified safe return or that it never acquired control.
  request_robot_shutdown
  exit 1
fi

start_quest_when_available &
quest_launcher_pid=$!
echo "采集工作流已启动；Quest休眠不会中止机械臂，唤醒后将自动启动APK。日志：$LOG_FILE"
wait "$workflow_pid"
wait_for_verified_shutdown
