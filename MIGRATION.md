# Computer Migration

## 1. Host requirements

Use Linux x86_64. Install the host tools:

```bash
sudo apt update
sudo apt install -y can-utils ffmpeg v4l-utils usbutils adb git curl
```

Install Miniconda/Miniforge, then run from the extracted package:

```bash
./tools/bootstrap_release_env.sh
conda activate arx-ac-one
```

The bootstrap uses Conda for Python 3.10 and `uv` for Python packages. The exact
ARX binary wheel shipped in `wheelhouse/` is preferred when present.

## 2. Quest

Follow the cable order in `RELEASE_NOTES.md` before connecting any USB data
cable. Install the included APK:

```bash
./tools/install_quest_apk.sh
```

Enable USB debugging and permanently authorize the new computer when prompted.

## 3. First robot startup on a new computer

The release intentionally omits `shutdown_pose_boot_id.txt`. Clear the work
area, physically place the robot in the desired powered-off/shutdown pose, then
start the stack. The software captures a shutdown pose for the current Linux
boot instead of trusting a pose copied from another computer.

Check registered devices without loading the robot SDK:

```bash
python resolve_hardware.py
v4l2-ctl --list-devices
```

Create CAN interfaces only when they do not already exist:

```bash
sudo ./tools/recover_arx_can.sh
```

Do not use `--usb-reset` during routine startup.

## 4. Full system launch

```bash
conda activate arx-ac-one
./tools/start_all_quest3_collection.sh --task "your task text"
```

All launchers locate the project from their own path. `ARX_ROOT` and
`ARX_PYTHON` may be set explicitly, but are normally unnecessary.

## 5. Verification before collecting valuable data

First run the known-good two-cycle robot baseline:

```bash
./tools/run_hand_collection_lifecycle_test.sh
```

Then run one disposable full-system episode. Confirm both arms move, all three
cameras stream, Quest input is fresh, and shutdown returns both arms safely.

## Quest APK rebuild

The prebuilt APK is ready to install. Unity/Android download caches are not
part of the release. For a rebuild, provide a compatible external tool directory
with `ARX_QUEST_TOOLS=/path/to/tools`, then run:

```bash
ARX_QUEST_TOOLS=/path/to/tools ./tools/build_install_run_quest3.sh
```

