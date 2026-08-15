#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
APK="$ROOT/vendor/Open-Teach-Bimanual-Quest/Build/ARXOpenTeachBimanual.apk"
PACKAGE="com.openai.arx.openteach.bimanual"
ADB="$ROOT/tools/quest_adb.sh"

[[ -s "$APK" ]] || { echo "Quest APK missing: $APK" >&2; exit 1; }
"$ADB" get-state >/dev/null
"$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
"$ADB" install -r "$APK"
"$ADB" shell dumpsys package "$PACKAGE" | grep -E 'lastUpdateTime|versionName|versionCode' | head -5
echo "Quest APK installed."

