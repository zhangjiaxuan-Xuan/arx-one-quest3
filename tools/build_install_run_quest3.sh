#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
PROJECT="$ROOT/vendor/Open-Teach-Bimanual-Quest"
QUEST_TOOLS="${ARX_QUEST_TOOLS:-$ROOT/.quest3-tools}"
UNITY="${ARX_UNITY:-$QUEST_TOOLS/unity/2021.3.5f1/Unity}"
UNITY_HOME="$QUEST_TOOLS/unity-home"
COMPAT="$QUEST_TOOLS/compat-libs/usr/lib/x86_64-linux-gnu"
ADB="$ROOT/tools/quest_adb.sh"
APK="$PROJECT/Build/ARXOpenTeachBimanual.apk"
PACKAGE="com.openai.arx.openteach.bimanual"
LOG="$QUEST_TOOLS/logs/unity-build-apk-latest.log"

cd "$ROOT"
echo "[1/5] 校验主机视觉代码"
PYTHONPATH="$ROOT" "$PYTHON" -m py_compile quest3_camera_stream.py quest3_input.py

echo "[2/5] Unity编译并构建Quest APK"
LD_LIBRARY_PATH="$COMPAT" \
HOME="$UNITY_HOME" \
XDG_CONFIG_HOME="$UNITY_HOME/config" \
XDG_CACHE_HOME="$QUEST_TOOLS/unity-cache" \
ARX_QUEST_TOOLS="$QUEST_TOOLS" \
JAVA_TOOL_OPTIONS='-Dfile.encoding=UTF-8 -DsocksProxyHost=127.0.0.1 -DsocksProxyPort=7897 -Dhttps.protocols=TLSv1.2' \
GRADLE_OPTS='-DsocksProxyHost=127.0.0.1 -DsocksProxyPort=7897 -Dhttps.protocols=TLSv1.2' \
"$UNITY" -batchmode -nographics -quit \
  -projectPath "$PROJECT" \
  -executeMethod QuestBuild.BuildApk \
  -logFile "$LOG"

[[ -s "$APK" ]] || { echo "APK未生成；日志：$LOG" >&2; exit 1; }
if grep -nE 'error CS|BuildFailed|APK build failed' "$LOG"; then
  echo "Unity构建失败；日志：$LOG" >&2
  exit 1
fi
echo "APK: $(stat -c '%y %s bytes' "$APK")"
sha256sum "$APK"

echo "[3/5] 覆盖安装Quest APK"
"$ADB" get-state >/dev/null
"$ADB" shell am force-stop "$PACKAGE" >/dev/null 2>&1 || true
"$ADB" install -r "$APK"

echo "[4/5] 核对设备安装时间"
"$ADB" shell dumpsys package "$PACKAGE" | grep -E 'lastUpdateTime|versionName|versionCode' | head -5

echo "[5/5] 完成"
echo "Quest APK 已构建并覆盖安装；未自动启动VR或占用摄像机。"
echo "需要独立预览时运行：$ROOT/tools/start_quest3_camera_preview.sh"
