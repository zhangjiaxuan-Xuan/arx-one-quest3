# ARX AC One + Quest 3 Stable Release Notes

Release baseline: 2026-08-15, Linux x86_64, Python 3.10, ARX SDK 0.1.3.

This package contains the verified dual-arm collection stack, Quest controller
application and source, three-camera streaming, synchronized data collection,
episode review, pi0.5 action adaptation, hardware serial registration, and the
validated safety lifecycle.

## CRITICAL: Quest cable connection order

> **Always connect the Quest auxiliary power cable first. Wait for power to be
> stable, and only then connect the Quest USB data cable to the computer.**

Never connect the computer data cable first and then hot-plug auxiliary power.
On the validated workstation that order created an electrical/ground disturbance
which corrupted USB-CAN TX/RX, typically making one robot arm stop responding.
The symptom looked like an SDK or watchdog failure but was caused by the Quest
auxiliary Type-C power path, cable quality, and electrical interference.

Recommended sequence:

1. Power the robot and keep both arms physically supported.
2. Connect Quest auxiliary power to its stable power source.
3. Wait several seconds for Quest power to stabilize.
4. Connect the Quest USB data cable to the computer.
5. Connect/verify both CANable adapters and all three cameras.
6. Start the software.

For disconnection, stop the program and let both arms reach the shutdown pose;
then disconnect the computer data cable before changing Quest auxiliary power.
Do not share an electrically noisy hub or power path between Quest auxiliary
power and the two USB-CAN adapters.

## Validated hardware identities

- Left arm CANable: `2074339F5743` -> `can0`
- Right arm CANable: `207A33695743` -> `can1`
- Left camera: `AY28162011Y`
- Right camera: `AY2816200AA`
- Third-person camera: `SN0001`

If the physical devices change, update `hardware_registry.json` before use.

## Scope of the package

Included: runtime source, tests, launchers, hardware registry, shared collection
pose, gripper calibration, Quest Unity source, and the prebuilt Quest APK.

Excluded: collected episodes, logs, camera snapshots, model weights, Python and
Conda caches, Unity `Library`, Android SDK downloads, and machine-specific boot
IDs. The new computer captures its own shutdown pose on first startup.

