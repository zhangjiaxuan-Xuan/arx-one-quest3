#!/usr/bin/env bash
set -euo pipefail

LEFT="/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2074339F5743-if00"
RIGHT="/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207A33695743-if00"
USB_RESET=0
if [[ "${1:-}" == "--usb-reset" ]]; then
  USB_RESET=1
elif [[ $# -ne 0 ]]; then
  echo "用法：sudo $0 [--usb-reset]" >&2
  exit 2
fi

if [[ $EUID -ne 0 ]]; then
  echo "请使用 sudo $0" >&2
  exit 2
fi
# A preceding usbreset may return before ttyACM/by-id links are recreated.
for _ in $(seq 1 80); do
  [[ -e "$LEFT" && -e "$RIGHT" ]] && break
  sleep 0.1
done
[[ -e "$LEFT" ]] || { echo "左臂CANable未找到：$LEFT" >&2; exit 1; }
[[ -e "$RIGHT" ]] || { echo "右臂CANable未找到：$RIGHT" >&2; exit 1; }

if pgrep -f '[c]ollect_workflow.py|[q]uest3_bimanual_test.py|[c]apture_demo.py|[r]eplay_pi05.py|[r]emote_delta_roundtrip.py|[v]alidate_quest3_output_stability.py|[v]alidate_clean_gravity_compensation.py|[v]alidate_persistent_session_passive.py|[v]alidate_hand_collection_lifecycle.py|[v]alidate_arx_' >/dev/null; then
  echo "检测到机械臂控制进程，拒绝重置CAN传输层。请先完成安全停机。" >&2
  exit 1
fi

pkill slcand 2>/dev/null || true
sleep 0.5

usb_device_node() {
  local serial_link="$1"
  local tty_name
  local sys_path
  tty_name="$(basename "$(readlink -f "$serial_link")")"
  sys_path="$(readlink -f "/sys/class/tty/$tty_name/device")"
  while [[ "$sys_path" != "/" && (! -f "$sys_path/busnum" || ! -f "$sys_path/devnum") ]]; do
    sys_path="$(dirname "$sys_path")"
  done
  [[ -f "$sys_path/busnum" && -f "$sys_path/devnum" ]] || return 1
  # usbutils usbreset accepts BBB/DDD, not a /dev/bus/usb path.
  printf '%03d/%03d\n' \
    "$((10#$(<"$sys_path/busnum")))" "$((10#$(<"$sys_path/devnum")))"
}

wait_for_serial_link() {
  local serial_link="$1"
  local label="$2"
  for _ in $(seq 1 80); do
    [[ -e "$serial_link" ]] && return 0
    sleep 0.1
  done
  echo "${label}CANable在USB复位后8秒内未重新枚举：$serial_link" >&2
  return 1
}

reset_usb_adapter() {
  local serial_link="$1"
  local label="$2"
  local usb_node output
  wait_for_serial_link "$serial_link" "$label"
  usb_node="$(usb_device_node "$serial_link")" || {
    echo "无法定位${label}CANable USB节点" >&2
    return 1
  }
  echo "复位${label}CANable：$usb_node"
  # usbreset can return EAGAIN while the reset itself has already caused the
  # ACM device to disconnect/re-enumerate.  Capture that transient result,
  # wait for the stable serial link, and continue instead of leaving slcand
  # killed with no can0/can1 interfaces.
  if ! output="$(usbreset "$usb_node" 2>&1)"; then
    echo "$output" >&2
    wait_for_serial_link "$serial_link" "$label"
    echo "${label}CANable已重新枚举；忽略USB reset瞬时返回并继续重建SLCAN" >&2
  else
    echo "$output"
    wait_for_serial_link "$serial_link" "$label"
  fi
}

if [[ "$USB_RESET" -eq 1 ]]; then
  command -v usbreset >/dev/null || { echo "缺少 usbreset" >&2; exit 1; }
  reset_usb_adapter "$LEFT" "左"
  # Resolve the right adapter's bus/device number only after the left reset;
  # USB device numbers are not assumed to remain stable across re-enumeration.
  reset_usb_adapter "$RIGHT" "右"
fi

modprobe slcan
# Exact transport used by the validated 2026-08-13 two-cycle baseline.
slcand -o -f -s8 "$LEFT" can0
slcand -o -f -s8 "$RIGHT" can1

for _ in $(seq 1 30); do
  [[ -e /sys/class/net/can0 && -e /sys/class/net/can1 ]] && break
  sleep 0.1
done
[[ -e /sys/class/net/can0 ]] || { echo "can0创建失败" >&2; exit 1; }
[[ -e /sys/class/net/can1 ]] || { echo "can1创建失败" >&2; exit 1; }

ip link set can0 up
ip link set can1 up
ip -brief link show can0
ip -brief link show can1
echo "ARX CAN接口恢复完成：左臂=can0(2074339F5743)，右臂=can1(207A33695743)"
echo "注意：本脚本只重建主机CANable/SLCAN传输层，不会复位机械臂内部电机控制器。"
echo "若官方SDK仍报告 None of the motors are initialized，请在支撑双臂后重启机械臂控制电源；不要反复使用--usb-reset。"
