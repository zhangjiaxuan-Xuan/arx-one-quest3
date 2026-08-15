# Quest 3 sender build

Create a Unity Android project with the OpenXR plugin and Meta Quest support,
then copy `Quest3UdpSender.cs` into `Assets/Scripts/`. Attach it to one persistent
GameObject in the launch scene and set `Robot Host` to the collection computer's
LAN IPv4 address. Keep `Robot Port` at `7447` and `Send Rate Hz` at `90`.

Enable the OpenXR Meta Quest feature group for Android, select both Touch
controller profiles, build for ARM64, and install the APK with Meta Quest
Developer Hub or `adb install -r <apk>`. The headset and collection computer
must share a trusted LAN. The sender intentionally contains no robot command or
safety logic.

Before enabling robot motion, run the host-side safe packet test:

```bash
python quest3_sender_sim.py --command idle --seconds 5
```

Then launch the APK and verify the console reports Quest online with a packet
age below 100 ms. Keep both arms disabled in `quest3_teleop_config.json` until
the right-handed Quest frame (+X right, +Y up, +Z backward) has been calibrated
to each robot base. Enable and validate one arm at a time, with low translation
and rotation scales and both arms raised clear of the table.
