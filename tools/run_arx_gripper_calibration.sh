#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
LOG_DIR="$ROOT/logs/gripper_calibration"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/calibration_$(date +%Y%m%d_%H%M%S).log"
: > "$LOG_FILE"
calibration_pid=""
poweroff_confirmed=0

terminate_after_poweroff() {
  echo "夹爪标定SDK进入Emergency；不会执行任何自动回位。"
  echo "请支撑双臂并关闭机械臂控制电源，然后按Enter或Ctrl-C结束进程。"
  poweroff_confirmed=0
  trap 'poweroff_confirmed=1' INT TERM HUP
  while [[ "$poweroff_confirmed" -eq 0 ]]; do
    if IFS= read -r </dev/tty; then
      poweroff_confirmed=1
    fi
  done
  trap - INT TERM HUP
  if [[ -n "$calibration_pid" ]] && kill -0 "$calibration_pid" 2>/dev/null; then
    kill -KILL "$calibration_pid" 2>/dev/null || true
  fi
  wait "$calibration_pid" 2>/dev/null || true
  calibration_pid=""
  echo "已结束厂商Emergency进程；重新上电前请检查夹爪物理位置。"
  return 1
}

on_interrupt() {
  if grep -q 'Emergency state entered' "$LOG_FILE"; then
    terminate_after_poweroff
    exit 130
  fi
  echo "已中止夹爪标定；未写完的标定不会覆盖现有配置。"
  if [[ -n "$calibration_pid" ]] && kill -0 "$calibration_pid" 2>/dev/null; then
    kill -INT "$calibration_pid" 2>/dev/null || true
  fi
  wait "$calibration_pid" 2>/dev/null || true
  exit 130
}
trap on_interrupt INT TERM HUP

exec {CALIBRATION_STDIN_FD}</dev/tty
PYTHONUNBUFFERED=1 "$PYTHON" -u "$ROOT/calibrate_arx_grippers.py" \
  <&"$CALIBRATION_STDIN_FD" > >(tee -a "$LOG_FILE") 2>&1 &
calibration_pid=$!
exec {CALIBRATION_STDIN_FD}<&-

while kill -0 "$calibration_pid" 2>/dev/null; do
  if grep -q 'Emergency state entered' "$LOG_FILE"; then
    terminate_after_poweroff
    exit 1
  fi
  sleep 0.1
done

wait "$calibration_pid"
calibration_pid=""
echo "夹爪标定与停机姿态捕获完成。日志：$LOG_FILE"
