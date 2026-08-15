# Meta Quest 3 control protocol

The production collector listens for UDP JSON packets on port `7447`. The
Quest and robot computer must be on the same trusted LAN. Send at 72--90 Hz;
robot commands are generated at 50 Hz.

Use `--quest-host <QUEST_IP>` when possible. If omitted, the receiver latches
the source IP of the first valid packet and ignores other senders. A sequence
reset is accepted only after the previous stream has been absent for one second.

```json
{
  "schema": "arx.quest3.controllers.v1",
  "sequence": 42,
  "client_time_ns": 1786500000000000000,
  "left": {
    "position_m": [0.0, 1.2, -0.3],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    "thumbstick_xy": [0.0, 0.0],
    "tracking": true,
    "buttons": {
      "grip": false,
      "trigger": 0.0,
      "primary": false,
      "secondary": false,
      "menu": false
    }
  },
  "right": { "...": "same fields" }
}
```

`position_m` and `orientation_xyzw` are OpenXR world-space grip poses. `grip`
is the motion deadman, not the analog grip value. The Unity client must set
`tracking=false` whenever OpenXR pose validity is lost.

## Safety and controls

- Left Grip authorizes only the left arm; Right Grip authorizes only the right.
- Pressing a Grip captures a fresh relative origin. Releasing it freezes that
  arm and restores low-damping gravity compensation.
- Trigger has hysteresis: `>=0.65` closes, `<=0.35` opens, and the middle band
  holds the prior binary command.
- Workflow buttons require a 0.7 second hold and both Grips released.
- Either thumbstick can issue an immediate one-shot workflow command. Push past
  0.75, then return within 0.35 before the next command. Thumbsticks are ignored
  while either Grip is held.
- Holding Left X + Right B together for 2 seconds requests safe shutdown. This
  uses only OpenXR-exposed buttons; Quest reserves the right Oculus button.
- A packet age over 100 ms, invalid tracking, IK failure, or a joint jump freezes
  the affected arm.

| Workflow state | Left primary (X) | Left secondary (Y) | Right primary (A) | Right secondary (B) |
|---|---|---|---|---|
| Collection waiting | Register pose | Restore pose | Start collection | Enter review |
| Recording | — | — | — | Stop and seal episode |
| Review | Reject | Visualize | Accept | Back to collection |

| Workflow state | Stick up | Stick down | Stick left | Stick right |
|---|---|---|---|---|
| Collection waiting | Start | Enter review | Restore pose | Register pose |
| Recording | — | Stop and seal | — | — |
| Review | Accept | Reject | Visualize | Back to collection |

## Calibration gate

`quest3_teleop_config.json` ships with both arms disabled. Do not change
`enabled` to `true` until the Quest-to-robot 3x3 axis matrices have been checked
with the robot raised clear of the table and translation/rotation scales have
been tested at low values. This deliberate gate prevents an uncalibrated OpenXR
coordinate frame from commanding a real arm.

## Data compatibility

The existing `observation_state`, `action`, camera files, timing files and
quality reports are unchanged. New Quest episodes add controller packet data to
`raw_demo.npz`; older keyboard/kinesthetic episodes remain valid because all new
fields are optional.
