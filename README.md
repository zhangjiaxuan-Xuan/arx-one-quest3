# ARX AC One → OpenPI π0.5 adapter

This directory contains the proprioception-only validation pipeline for the
ARX AC One dual-arm platform. It validates the robot-side state/action path;
camera observations are still required for real π0.5 training and inference.

## Stable hardware registration

Logical roles are registered in `hardware_registry.json` by immutable USB
serial number, not by volatile `/dev/videoN` or `/dev/ttyACMN` numbers:

- Left arm camera: `AY28162011Y`
- Right arm camera: `AY2816200AA`
- Third-person camera: `SN0001`
- Left arm: `can0`, CANable2 serial `2074339F5743`
- Right arm: `can1`, CANable2 serial `207A33695743`

### Lossless acquisition boundary

The collector is the robot-side acquisition and transport station. It must
preserve native camera samples; model resizing, normalization, cropping, and
color conversion belong on the inference/training server.

- Left/right wrist cameras: native MJPEG, `1280x720 @ 30 FPS`.
- Third-person camera: full-sensor MJPEG, `2048x1536 @ 30 FPS`.
- The acquisition process must not crop, scale, decode/re-encode, or change the
  frame rate. Raw MJPEG packets and V4L2 absolute timestamps are copied into the
  capture container.
- All three V4L2 queues remain one frame. The alignment file, original frame
  indices, timestamps, and SHA-256 hashes travel with every episode.
- Any server-side image transform must be versioned with the model and must not
  alter the robot-side raw episode.

The server action wire format is `arx_ac_one.bimanual_delta_eef.v1`. Each arm
uses six base-frame end-effector deltas followed by one discrete gripper
command. Gripper semantics are fixed as `0 = open`, `1 = close`; model output
must never send a continuous gripper width. Robot-side execution rate-limits
opening/closing, latches the measured width at contact torque, and stops further
closing at the hard torque threshold. These protections remain local even when
actions originate on the 4090 server.
- Excluded laptop camera: `200901010001`

Resolve and validate the currently attached hardware after every reconnect:

```bash
python resolve_hardware.py
eval "$(python resolve_hardware.py --shell)"
```

Application code can call `resolve_registered_hardware()` from `arx_common`.
Resolution fails loudly if a registered device is missing or ambiguous. CAN
interfaces still need to be created after reconnect, but their source TTYs are
resolved by serial through `ARX_LEFT_CAN_TTY` and `ARX_RIGHT_CAN_TTY`.

## π0.5 model boundary

The 14-dimensional proprioceptive state is:

1. right arm joints 1–6
2. right gripper width
3. left arm joints 1–6
4. left gripper width

The 14-dimensional action is a next-timestep transition, ordered as right
`[dx,dy,dz,drx,dry,drz,gripper_01]`, then the same seven values for the left
arm. Rotations are SO(3) rotation vectors composed in the arm base frame; they
are never calculated by subtracting Euler angles or rotation vectors.

The π0.5 model internally pads state/actions to 32 dimensions and predicts
chunks shaped `[50,32]`. Server-side output transforms denormalize and unpad
that tensor. The robot boundary accepts only `[50,14]` carrying
`arx_ac_one.bimanual_delta_eef.v1`, an exact normalization identifier, 50 Hz,
finite values, bounded vector norms, and binary grippers. See
`pi05_arx_adapter.py`; raw normalized model tensors are rejected before the SDK.

## Commands

Run from the `lerobot` Conda environment:

Direct hardware execution through `capture_demo.py`, `replay_pi05.py`, or
`remote_delta_roundtrip.py --execute` is disabled. Those legacy entrypoints
create and destroy their own SDK controllers and therefore bypass the unified
persistent command-session owner. Offline conversion remains available; robot collection must
use the interactive workflow below. Model deployment will be re-enabled only
through the same persistent SDK session and safety latch.

## Interactive collection workflow

Start the terminal collection console from the `lerobot` environment:

```bash
conda activate lerobot
cd /path/to/arx-ac-one-pi05
python collect_workflow.py --quest-host <QUEST_IP> --task "task text"
```

Quest 3 Touch controllers are the lowest-priority input. The left/right Grip
buttons are independent software clutches: an arm follows only while its own
Grip is held, and releasing it holds the last safe position without changing
gain mode or SDK lifetime. Workflow actions use X/Y/A/B gestures only while both
Grips are released; see `QUEST3_PROTOCOL.md`. Ten pending episodes is the
suggested review batch size, not a hard limit.
Entering review first returns both arms to the boot-specific shutdown pose,
keeps the persistent SDK session online in low-damping gravity compensation,
and then audits pending episodes with up to four parallel worker threads. For each audited episode choose
optional visualization (`p`), accept (`a`), or reject (`x`); the next pending
episode loads automatically. Press `b` to pause review and return to collection,
or `q` to preserve the queue and exit safely. Each camera gets a 2-second
UVC initialization allowance followed by a 2-second auto-exposure warmup while
remaining open. Episodes with camera timestamp gaps, invalid resolution, robot
sampling stalls, or non-finite state data cannot be accepted. Audit no longer
blocks collection: ending a demonstration only finalizes its files before
immediately returning to collection-stage waiting.

Collection progress is resumable across process restarts. At startup the console
loads `sessions/workflow_state.json` when available and also reconstructs the
latest task, accepted-round count, and next attempt number from the episode
directories, so older datasets work without migration. Progress is counted per
task text. Entering an existing task text resumes that task; entering a new task
starts its own 0/50 counter. A `pending` episode that already has a valid
`quality_report.json` is restored into the pending review queue.
Incomplete interrupted recordings remain on disk but are never counted as
accepted data.

All three camera queues are fixed to one frame. Raw MJPEG is stored in NUT
containers because NUT preserves the absolute V4L2 timestamps needed to place
all cameras and 50 Hz robot samples on one host clock. `time_alignment.npz`
stores, for every robot timestep, each camera's latest file-frame index,
visual age in seconds, and visual lag measured in 50 Hz timesteps. Collection
and deployment must use the same latest-frame/one-frame-buffer policy.

Both arm controllers use gravity-compensated damping/teach mode during
collection. Review returns the arms to the shutdown pose and destroys both SDK
controller objects, invoking the configured passive shutdown. Returning to
collection reconnects the controllers and restores low-damping teach mode.
Replay is visualization-only and never reconnects or commands the robot.

`p` is visualization-only and never commands the robot. It opens a synchronized
desktop view with all three cameras, left/right end-effector XY trajectories,
current XYZ coordinates, and both gripper states. Press Space to pause and
`q`/Esc to close it. The robot safely restores to the registered initial pose
immediately before every collection. After collection stops, the episode is
sealed and both arms automatically return to that registered pose under guarded
low-gain control. Restore is also available while idle. Keyboard stage commands
remain available only through the maintenance fallback `--input keyboard`.

The console maintains two independent poses. The collection initial pose is
registered by `r` and stored in `shared_poses/collection_initial_pose.npy`.
Registration records the six joints of each arm and always canonicalizes both
grippers to fully open (`0.082 m`); every return to this pose waits for that
open state. The
shutdown pose is captured immediately when controllers first connect after a
real Linux boot and is stored in `sessions/poses/shutdown_pose.npy` together
with `/proc/sys/kernel/random/boot_id`. Restarting the collection or inference
process during the same system boot reloads that shutdown pose and never
overwrites it. Registering only changes the collection pose. A normal exit
first makes the same low-gain, rate-limited return to the boot-specific shutdown
pose and only then disconnects the controllers. The existing 2-radian
automatic-return safety limit remains enforced; if it is exceeded, exit is
refused while gravity compensation remains active so the operator can manually
move closer and retry.

Teach mode uses zero position stiffness and per-joint damping scales of
`[12%, 12%, 12%, 10%, 8%, 5%]`. Gripper damping is 3% of the SDK default.
This keeps gravity-compensated manual guidance light, especially at the wrist
and gripper, while retaining a small amount of velocity damping.
# Quest 3 collection

Quest 3 is the production teleoperation input. Existing episodes remain valid;
the original robot/camera arrays and filenames are unchanged. See
`QUEST3_PROTOCOL.md` for the OpenXR packet schema, dual-Grip deadman behavior,
workflow button map, and calibration gate.

Start a new task:

```bash
python collect_workflow.py --task "pick the fruit and place it in the box"
```

Reuse the prior task:

```bash
python collect_workflow.py
```

The maintenance-only keyboard fallback is:

```bash
python collect_workflow.py --input keyboard --task "task text"
```

The Quest sender implementation is in `quest3_unity/Quest3UdpSender.cs`. Both
arms intentionally ship disabled in `quest3_teleop_config.json`; perform the
documented one-arm-at-a-time coordinate calibration before changing either
`enabled` flag.
