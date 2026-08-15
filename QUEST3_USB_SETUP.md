# Isolated Quest 3 USB setup

No system packages, system Python packages, global Android SDK, Unity editor or
Conda environment are modified. Google platform-tools live under
`.quest3-tools/platform-tools`, and ADB keys/state live under
`.quest3-tools/android-home`.

Check the headset:

```bash
cd /path/to/arx-ac-one-pi05
tools/quest_adb.sh devices -l
```

Expected state is `device`. An empty list means developer mode/USB debugging is
not exposed. `unauthorized` means the confirmation dialog must be accepted
inside the headset.

The official Open-Teach APKs and hashes are stored under
`vendor/Open-Teach/`. Do not install `BimanualArm.apk` for AC One production:
it uses bare-hand skeleton keypoints and pinch gestures, whereas AC One keeps
the two physical Touch Grip buttons as independent motion deadman switches.

Robot motion remains disabled in `quest3_teleop_config.json` until controller
packets, USB/Wi-Fi transport, coordinate calibration and one-arm dry runs have
all passed.
