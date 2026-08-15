#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ADB_VENDOR_KEYS="${ADB_VENDOR_KEYS:-$HOME/.android/adbkey}"
quest_tools="${ARX_QUEST_TOOLS:-$project_root/.quest3-tools}"
export ANDROID_USER_HOME="${ANDROID_USER_HOME:-$quest_tools/android-home}"
if [[ -x "$quest_tools/platform-tools/adb" ]]; then
  exec "$quest_tools/platform-tools/adb" "$@"
fi
exec adb "$@"
