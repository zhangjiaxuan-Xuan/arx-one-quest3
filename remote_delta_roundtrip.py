#!/usr/bin/env python3
"""4090 SSH round-trip and guarded bimanual differential-EEF replay.

The 14-D wire action is:
  right [dx,dy,dz,droll,dpitch,dyaw,gripper_command],
  left  [dx,dy,dz,droll,dpitch,dyaw,gripper_command].
The gripper command is discrete: 0=open, 1=close.
All deltas are per 50 Hz timestep and expressed in each arm's base frame.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import time

import numpy as np
import arx5_interface as arx5
from scipy.spatial.transform import Rotation

from arx_common import FPS, GRIPPER_WIDTH, copy_state, make_arm, pack_bimanual
from legacy_hardware_guard import reject_legacy_direct_hardware
from replay_pi05 import command_pair, set_reduced_gain
from visualize_episode import model_path

WIRE_SCHEMA = "arx_ac_one.bimanual_delta_eef.v1"
MAX_TRANSLATION_DELTA_M = 0.025
MAX_ROTATION_DELTA_RAD = 0.12
# Raw gripper readout can change by about 10 mm in one sample near contact.
# This is an input-validation bound; execution remains limited to 3 mm/tick.
MAX_JOINT_STEP_RAD = 0.025
MAX_GRIPPER_STEP_M = 0.003
GRIPPER_INTENT_DEADBAND_M = 0.0002
GRIPPER_CONTACT_TORQUE = 0.30
GRIPPER_HARD_TORQUE = 0.60
GRIPPER_CONTACT_PRELOAD_M = 0.004
GRIPPER_SLIP_FOLLOW_M = 0.001
IK_TRANSLATION_TOLERANCE_M = 0.001
IK_ROTATION_TOLERANCE_RAD = 0.005
IK_NUMERICAL_EPS = 1e-5


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def solver() -> arx5.Arx5Solver:
    config = arx5.RobotConfigFactory.get_instance().get_config("X5")
    return arx5.Arx5Solver(
        str(model_path()), 6, config.joint_pos_min, config.joint_pos_max
    )


def wrap_angles(values: np.ndarray) -> np.ndarray:
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def pose_delta(previous: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Return a base-frame SE(3) delta; rotations are composed, not subtracted."""
    result = np.empty(6, dtype=np.float64)
    result[:3] = np.asarray(current[:3]) - np.asarray(previous[:3])
    result[3:] = (
        Rotation.from_rotvec(current[3:6])
        * Rotation.from_rotvec(previous[3:6]).inv()
    ).as_rotvec()
    return result


def apply_pose_delta(pose: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """Integrate a base-frame SE(3) delta using proper rotation composition."""
    result = np.asarray(pose, dtype=np.float64).copy()
    result[:3] += np.asarray(delta[:3], dtype=np.float64)
    result[3:] = (
        Rotation.from_rotvec(delta[3:6])
        * Rotation.from_rotvec(result[3:6])
    ).as_rotvec()
    return result


def refine_ik_solution(kin, desired_pose: np.ndarray, joints: np.ndarray) -> np.ndarray:
    """Refine vendor IK and reject its relatively loose orientation tolerance."""
    q = np.asarray(joints, dtype=np.float64).copy()
    for _ in range(8):
        current = np.asarray(kin.forward_kinematics(q), dtype=np.float64)
        error = pose_delta(current, desired_pose)
        if (
            np.linalg.norm(error[:3]) <= IK_TRANSLATION_TOLERANCE_M
            and np.linalg.norm(error[3:]) <= IK_ROTATION_TOLERANCE_RAD
        ):
            return q
        jacobian = np.empty((6, 6), dtype=np.float64)
        for column in range(6):
            perturbed = q.copy()
            perturbed[column] += IK_NUMERICAL_EPS
            pose = np.asarray(kin.forward_kinematics(perturbed), dtype=np.float64)
            jacobian[:, column] = pose_delta(current, pose) / IK_NUMERICAL_EPS
        jj_t = jacobian @ jacobian.T
        step = jacobian.T @ np.linalg.solve(jj_t + np.eye(6) * 1e-4, error)
        q += np.clip(step, -0.08, 0.08)
    current = np.asarray(kin.forward_kinematics(q), dtype=np.float64)
    error = pose_delta(current, desired_pose)
    raise RuntimeError(
        "IK refinement failed: "
        f"translation={np.linalg.norm(error[:3]):.6f}m "
        f"rotation={np.linalg.norm(error[3:]):.6f}rad"
    )


def binary_gripper_commands(width: np.ndarray) -> np.ndarray:
    """Infer teleoperation intent; 0=open and 1=close, with temporal hold."""
    command = np.empty(len(width), dtype=np.float32)
    command[0] = 0.0 if width[0] >= GRIPPER_WIDTH * 0.5 else 1.0
    for i in range(1, len(width)):
        movement = float(width[i] - width[i - 1])
        if movement < -GRIPPER_INTENT_DEADBAND_M:
            command[i] = 1.0
        elif movement > GRIPPER_INTENT_DEADBAND_M:
            command[i] = 0.0
        else:
            command[i] = command[i - 1]
    return command


def delta_eef_actions(states: np.ndarray, kin=None) -> np.ndarray:
    """Encode action[t] as the transition from state[t] to state[t+1]."""
    states = np.asarray(states, dtype=np.float64)
    if states.ndim != 2 or states.shape[1] != 14 or len(states) < 2:
        raise ValueError(f"invalid state array {states.shape}")
    if not np.isfinite(states).all():
        raise ValueError("non-finite source state")
    kin = solver() if kin is None else kin
    right_pose = np.asarray([kin.forward_kinematics(q[:6]) for q in states])
    left_pose = np.asarray([kin.forward_kinematics(q[7:13]) for q in states])
    delta = np.zeros_like(states)
    delta[:-1, :6] = np.asarray(
        [pose_delta(previous, current) for previous, current in zip(right_pose[:-1], right_pose[1:])]
    )
    delta[:-1, 7:13] = np.asarray(
        [pose_delta(previous, current) for previous, current in zip(left_pose[:-1], left_pose[1:])]
    )
    right_gripper = binary_gripper_commands(states[:, 6])
    left_gripper = binary_gripper_commands(states[:, 13])
    # The command paired with state[t] is the intent observed in its outgoing
    # transition.  Hold the final command for the padded terminal transition.
    delta[:, 6] = np.r_[right_gripper[1:], right_gripper[-1]]
    delta[:, 13] = np.r_[left_gripper[1:], left_gripper[-1]]
    return delta


def encode(raw_path: Path, output: Path) -> dict:
    raw = np.load(raw_path, allow_pickle=False)
    states = raw["observation_state"].astype(np.float64)
    timestamps = raw["timestamp"].astype(np.float64)
    if states.ndim != 2 or states.shape[1] != 14 or len(states) < 2:
        raise ValueError(f"invalid state array {states.shape}")
    if not np.isfinite(states).all() or not np.isfinite(timestamps).all():
        raise ValueError("non-finite source data")
    delta = delta_eef_actions(states)
    bounds = {
        "translation_m": float(max(
            np.linalg.norm(delta[:, :3], axis=1).max(),
            np.linalg.norm(delta[:, 7:10], axis=1).max(),
        )),
        "rotation_rad": float(max(
            np.linalg.norm(delta[:, 3:6], axis=1).max(),
            np.linalg.norm(delta[:, 10:13], axis=1).max(),
        )),
        "right_gripper_transitions": int(np.count_nonzero(np.diff(delta[:, 6]))),
        "left_gripper_transitions": int(np.count_nonzero(np.diff(delta[:, 13]))),
    }
    if bounds["translation_m"] > MAX_TRANSLATION_DELTA_M or bounds["rotation_rad"] > MAX_ROTATION_DELTA_RAD:
        raise RuntimeError(f"source delta exceeds safety bounds: {bounds}")
    np.savez_compressed(
        output,
        schema=np.asarray(WIRE_SCHEMA),
        fps=np.int32(FPS),
        timestamp=timestamps,
        initial_joint_state=states[0].astype(np.float32),
        delta_eef_action=delta.astype(np.float32),
    )
    return {"samples": len(states), "duration_s": float(timestamps[-1]), **bounds}


def roundtrip(local_input: Path, local_output: Path, host: str, remote_env: str) -> dict:
    token = f"arx-roundtrip-{int(time.time())}"
    remote_dir = f"/tmp/{token}"
    remote_in = f"{remote_dir}/model_input.npz"
    remote_out = f"{remote_dir}/model_output.npz"
    remote_python = f"/home/guidance/miniforge3/envs/{remote_env}/bin/python"
    subprocess.run(["ssh", host, "mkdir", "-p", remote_dir], check=True)
    try:
        subprocess.run(["scp", str(local_input), f"{host}:{remote_in}"], check=True)
        program = (
            "import numpy as np,shutil,sys; "
            "x=np.load(sys.argv[1],allow_pickle=False); "
            "assert str(x['schema'])=='" + WIRE_SCHEMA + "'; "
            "shutil.copyfile(sys.argv[1],sys.argv[2])"
        )
        remote_command = shlex.join(
            [remote_python, "-c", program, remote_in, remote_out]
        )
        subprocess.run(["ssh", host, remote_command], check=True)
        subprocess.run(["scp", f"{host}:{remote_out}", str(local_output)], check=True)
    finally:
        subprocess.run(["ssh", host, "rm", "-rf", remote_dir], check=False)
    source = np.load(local_input, allow_pickle=False)
    returned = np.load(local_output, allow_pickle=False)
    for key in source.files:
        if key not in returned.files or not np.array_equal(source[key], returned[key]):
            raise RuntimeError(f"round-trip mismatch: {key}")
    input_digest = sha256(local_input)
    output_digest = sha256(local_output)
    if input_digest != output_digest:
        raise RuntimeError("byte-level round-trip SHA-256 mismatch")
    return {
        "input_sha256": input_digest,
        "output_sha256": output_digest,
        "byte_exact_roundtrip": True,
    }


def decode_ik(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    wire = np.load(path, allow_pickle=False)
    if str(wire["schema"]) != WIRE_SCHEMA:
        raise ValueError("wire schema mismatch")
    delta = wire["delta_eef_action"].astype(np.float64)
    initial = wire["initial_joint_state"].astype(np.float64)
    if int(wire["fps"]) != FPS:
        raise ValueError(f"wire fps must be {FPS}")
    if delta.ndim != 2 or delta.shape[1] != 14 or len(delta) == 0:
        raise ValueError(f"invalid delta action shape: {delta.shape}")
    if initial.shape != (14,):
        raise ValueError(f"invalid initial joint state shape: {initial.shape}")
    if not np.isfinite(delta).all() or not np.isfinite(initial).all():
        raise ValueError("wire data contains non-finite values")
    raw_gripper = delta[:, [6, 13]]
    if not np.logical_or(np.isclose(raw_gripper, 0.0), np.isclose(raw_gripper, 1.0)).all():
        raise ValueError("gripper command must be binary 0/1")
    translation = max(
        np.linalg.norm(delta[:, :3], axis=1).max(),
        np.linalg.norm(delta[:, 7:10], axis=1).max(),
    )
    rotation = max(
        np.linalg.norm(delta[:, 3:6], axis=1).max(),
        np.linalg.norm(delta[:, 10:13], axis=1).max(),
    )
    if translation > MAX_TRANSLATION_DELTA_M:
        raise ValueError("model translation delta exceeds safety bound")
    if rotation > MAX_ROTATION_DELTA_RAD:
        raise ValueError("model rotation delta exceeds safety bound")
    kin = solver()
    target = initial.copy()
    right_pose = kin.forward_kinematics(initial[:6])
    left_pose = kin.forward_kinematics(initial[7:13])
    actions = np.empty_like(delta)
    gripper_commands = raw_gripper.astype(np.uint8)
    ik_status = np.zeros((len(delta), 2), dtype=np.int32)
    for i, d in enumerate(delta):
        right_pose = apply_pose_delta(right_pose, d[:6])
        left_pose = apply_pose_delta(left_pose, d[7:13])
        rs, rq = kin.multi_trial_ik(right_pose, target[:6], 8)
        ls, lq = kin.multi_trial_ik(left_pose, target[7:13], 8)
        ik_status[i] = (rs, ls)
        if rs != 0 or ls != 0:
            raise RuntimeError(
                f"IK failed at sample {i}: right={kin.get_ik_status_name(rs)}, "
                f"left={kin.get_ik_status_name(ls)}"
            )
        rq = refine_ik_solution(kin, right_pose, rq)
        lq = refine_ik_solution(kin, left_pose, lq)
        target[:6], target[7:13] = rq, lq
        # Width targets are resolved by the guarded gripper state machine at execution.
        target[6] = GRIPPER_WIDTH if gripper_commands[i, 0] == 0 else 0.0
        target[13] = GRIPPER_WIDTH if gripper_commands[i, 1] == 0 else 0.0
        actions[i] = target
    return actions.astype(np.float32), gripper_commands, ik_status


class GripperGuard:
    def __init__(self) -> None:
        self.preload_target: float | None = None

    def target(self, command: int, measured_width: float, measured_torque: float) -> float:
        if command == 0:
            self.preload_target = None
            return GRIPPER_WIDTH
        torque = abs(float(measured_torque))
        if torque >= GRIPPER_HARD_TORQUE:
            # Stop adding compression at the hard protection threshold. If
            # torque later falls, resume from this measured position instead
            # of permanently latching the old contact width.
            self.preload_target = float(
                np.clip(measured_width, 0.0, GRIPPER_WIDTH)
            )
            return self.preload_target
        if torque >= GRIPPER_CONTACT_TORQUE:
            candidate = max(
                0.0, float(measured_width) - GRIPPER_CONTACT_PRELOAD_M
            )
            self.preload_target = (
                candidate
                if self.preload_target is None
                else min(self.preload_target, candidate)
            )
            return self.preload_target
        if self.preload_target is not None:
            # Lost contact generally means the object shifted. Continue
            # following it closed until contact and preload are restored.
            candidate = max(
                0.0, float(measured_width) - GRIPPER_SLIP_FOLLOW_M
            )
            self.preload_target = min(self.preload_target, candidate)
            return self.preload_target
        return 0.0


def execute(actions: np.ndarray, gripper_commands: np.ndarray, report_path: Path) -> None:
    answer = input(
        "离线校验通过。确认机械臂周围无人、无障碍，并执行低增益重放？输入 EXECUTE："
    ).strip()
    if answer != "EXECUTE":
        raise RuntimeError("execution cancelled")
    left = right = None
    samples = []
    try:
        left, left_robot, left_ctrl = make_arm("can0", gravity_compensation=True)
        right, right_robot, right_ctrl = make_arm("can1", gravity_compensation=True)
        ls, rs = copy_state(left), copy_state(right)
        current, _, _ = pack_bimanual(rs, ls)
        if np.max(np.abs(actions[0] - current)) > 2.0:
            raise RuntimeError("initial pose is farther than 2 rad")
        left_cmd, right_cmd = arx5.JointState(6), arx5.JointState(6)
        left_cmd.pos()[:] = ls[0]; left_cmd.gripper_pos = float(ls[3])
        right_cmd.pos()[:] = rs[0]; right_cmd.gripper_pos = float(rs[3])
        right.set_joint_cmd(right_cmd); left.set_joint_cmd(left_cmd)
        set_reduced_gain(left, left_ctrl, 6); set_reduced_gain(right, right_ctrl, 6)
        right_guard, left_guard = GripperGuard(), GripperGuard()
        for _ in range(FPS * 8):
            ls, rs = copy_state(left), copy_state(right)
            guarded_initial = actions[0].copy()
            guarded_initial[6] = right_guard.target(
                int(gripper_commands[0, 0]), float(rs[3]), float(rs[5])
            )
            guarded_initial[13] = left_guard.target(
                int(gripper_commands[0, 1]), float(ls[3]), float(ls[5])
            )
            command_pair(
                left, right, guarded_initial, left_cmd, right_cmd, 0.004, 0.0003
            )
            time.sleep(1 / FPS)
        kin = solver(); next_tick = time.monotonic()
        for i, action in enumerate(actions):
            ls, rs = copy_state(left), copy_state(right)
            guarded_action = action.copy()
            guarded_action[6] = right_guard.target(
                int(gripper_commands[i, 0]), float(rs[3]), float(rs[5])
            )
            guarded_action[13] = left_guard.target(
                int(gripper_commands[i, 1]), float(ls[3]), float(ls[5])
            )
            command_pair(
                left, right, guarded_action, left_cmd, right_cmd,
                MAX_JOINT_STEP_RAD, MAX_GRIPPER_STEP_M,
            )
            ls, rs = copy_state(left), copy_state(right)
            measured, _, _ = pack_bimanual(rs, ls)
            desired_r = kin.forward_kinematics(action[:6])
            desired_l = kin.forward_kinematics(action[7:13])
            measured_r = kin.forward_kinematics(measured[:6])
            measured_l = kin.forward_kinematics(measured[7:13])
            samples.append((i, action.copy(), measured, desired_r, measured_r, desired_l, measured_l))
            next_tick += 1 / FPS
            delay = next_tick - time.monotonic()
            if delay > 0: time.sleep(delay)
        desired = np.asarray([s[1] for s in samples]); measured = np.asarray([s[2] for s in samples])
        dr = np.asarray([pose_delta(s[4], s[3]) for s in samples])
        dl = np.asarray([pose_delta(s[6], s[5]) for s in samples])
        np.savez_compressed(report_path, desired=desired, measured=measured, right_eef_error=dr, left_eef_error=dl)
        print(json.dumps({
            "joint_rmse_rad": float(np.sqrt(np.mean((desired[:, [*range(6), *range(7,13)]] - measured[:, [*range(6), *range(7,13)]]) ** 2))),
            "right_translation_rmse_m": float(np.sqrt(np.mean(dr[:, :3] ** 2))),
            "left_translation_rmse_m": float(np.sqrt(np.mean(dl[:, :3] ** 2))),
            "right_rotation_rmse_rad": float(np.sqrt(np.mean(dr[:, 3:] ** 2))),
            "left_rotation_rmse_rad": float(np.sqrt(np.mean(dl[:, 3:] ** 2))),
        }, indent=2))
    finally:
        if right is not None: del right
        if left is not None: del left


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--host", default="4090")
    parser.add_argument("--remote-env", default="predimem-upper")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.execute:
        reject_legacy_direct_hardware("remote_delta_roundtrip.py --execute")
    episode = args.episode.resolve()
    local_input = episode / "delta_eef_model_input.npz"
    local_output = episode / "delta_eef_model_output.npz"
    metadata = encode(episode / "raw_demo.npz", local_input)
    metadata.update(roundtrip(local_input, local_output, args.host, args.remote_env))
    actions, gripper_commands, _ = decode_ik(local_output)
    metadata["ik_samples"] = len(actions)
    (episode / "remote_roundtrip_report.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metadata, indent=2))
    print("OFFLINE_ONLY: robot execution requires the unified persistent command session")


if __name__ == "__main__":
    main()
