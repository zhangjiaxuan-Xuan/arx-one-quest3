#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
GRIPPER_CALIBRATION="$ROOT/shared_poses/gripper_calibration.json"
SHUTDOWN_POSE="$ROOT/shared_poses/shutdown_pose.npy"

if [[ $EUID -eq 0 ]]; then
  echo "请以普通用户运行本脚本；它只会为CAN接口恢复单独调用sudo。" >&2
  exit 2
fi

if pgrep -f '[c]ollect_workflow.py|[c]alibrate_arx_grippers.py|[q]uest3_bimanual_test.py|[c]apture_demo.py|[r]eplay_pi05.py|[r]emote_delta_roundtrip.py|[v]alidate_quest3_output_stability.py|[v]alidate_clean_gravity_compensation.py|[v]alidate_persistent_session_passive.py|[v]alidate_hand_collection_lifecycle.py|[v]alidate_arx_' >/dev/null; then
  echo "检测到机械臂控制进程，拒绝启动第二个SDK实例。" >&2
  exit 1
fi

echo "[一键启动 1/4] 无机械臂合成输出稳定性闸门（不加载ARX SDK）"
if ! "$ROOT/tools/run_quest3_output_stability_test.sh" synthetic 16; then
  echo "无机械臂输出稳定性测试失败；拒绝连接机械臂。" >&2
  exit 1
fi

echo "[一键启动 2/4] 检查并建立双臂CAN接口"
if [[ ! -e /sys/class/net/can0 || ! -e /sys/class/net/can1 ]]; then
  sudo "$ROOT/tools/recover_arx_can.sh"
else
  echo "can0/can1 已存在；交由正式启动器继续校验序列号、活动和总线错误。"
fi

if [[ ! -s "$GRIPPER_CALIBRATION" || "${ARX_RECALIBRATE_GRIPPERS:-0}" == "1" ]]; then
  echo "[一键启动 3/4] 首次手动归零双夹爪"
  echo "标定阶段不执行关节轨迹；请按终端提示完成闭合→张开→闭合复验。"
  "$ROOT/tools/run_arx_gripper_calibration.sh"
  echo "夹爪电机零点已被官方SDK重置。"
  echo "请支撑双臂，将机械臂控制电源关闭10秒后重新上电，然后按Enter继续。"
  IFS= read -r </dev/tty
  sudo "$ROOT/tools/recover_arx_can.sh"
else
  echo "[一键启动 3/4] 已有左右独立夹爪标定；跳过电机零点重置"
  echo "标定文件：$GRIPPER_CALIBRATION"
fi

if [[ "$GRIPPER_CALIBRATION" -nt "$SHUTDOWN_POSE" ]]; then
  echo "检测到夹爪标定晚于旧停机姿态。"
  echo "请在正式SDK启动前把双臂摆到希望的停机位置；保持工作区清空，然后按Enter。"
  IFS= read -r </dev/tty
fi

echo "[一键启动 4/4] 启动双臂、三相机、Quest等待与正式采集工作流"
echo "Quest若在休眠，请戴上头显唤醒；启动器会在后台等待并自动打开APK。"
exec "$ROOT/tools/start_quest3_collection_test.sh" "$@"
