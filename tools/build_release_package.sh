#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
VERSION="${1:-v1.0.0}"
PACKAGE_NAME="arx-ac-one-pi05-${VERSION}-linux-x86_64"
OUTPUT_DIR="${ARX_RELEASE_DIR:-$ROOT/release}"
STAGE_PARENT="$(mktemp -d -t arx-release.XXXXXX)"
STAGE="$STAGE_PARENT/$PACKAGE_NAME"

cleanup() {
  rm -rf -- "$STAGE_PARENT"
}
trap cleanup EXIT

mkdir -p "$STAGE" "$OUTPUT_DIR" "$STAGE/sessions" "$STAGE/sessions_vr" "$STAGE/logs"

find "$ROOT" -maxdepth 1 -type f \
  \( -name '*.py' -o -name '*.md' -o -name '*.json' -o \
     -name '*.yml' -o -name 'requirements-*.txt' \) \
  -exec cp -a -t "$STAGE" {} +
cp -a "$ROOT/tools" "$ROOT/quest3_unity" "$ROOT/shared_poses" "$STAGE/"
rm -f "$STAGE/shared_poses/shutdown_pose_boot_id.txt"

mkdir -p "$STAGE/vendor/Open-Teach-Bimanual-Quest/Build"
cp -a \
  "$ROOT/vendor/Open-Teach-Bimanual-Quest/Assets" \
  "$ROOT/vendor/Open-Teach-Bimanual-Quest/Packages" \
  "$ROOT/vendor/Open-Teach-Bimanual-Quest/ProjectSettings" \
  "$STAGE/vendor/Open-Teach-Bimanual-Quest/"
cp -a "$ROOT/vendor/Open-Teach-Bimanual-Quest/README.md" \
  "$STAGE/vendor/Open-Teach-Bimanual-Quest/"
cp -a "$ROOT/vendor/Open-Teach-Bimanual-Quest/Build/ARXOpenTeachBimanual.apk" \
  "$STAGE/vendor/Open-Teach-Bimanual-Quest/Build/"

for reference in Open-Teach Open-Teach-Controllers; do
  if [[ -d "$ROOT/vendor/$reference" ]]; then
    cp -a "$ROOT/vendor/$reference" "$STAGE/vendor/"
  fi
done
find "$STAGE/vendor" -type d -name .git -prune -exec rm -rf -- {} +

if [[ -d "$ROOT/wheelhouse" ]]; then
  cp -a "$ROOT/wheelhouse" "$STAGE/"
fi

if git -C "$ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git -C "$ROOT" rev-parse HEAD > "$STAGE/GIT_COMMIT"
fi

(
  cd "$STAGE"
  find . -type f ! -name RELEASE_MANIFEST.sha256 -print0 \
    | sort -z | xargs -0 sha256sum > RELEASE_MANIFEST.sha256
)

ARCHIVE="$OUTPUT_DIR/$PACKAGE_NAME.tar.zst"
tar --zstd -C "$STAGE_PARENT" -cf "$ARCHIVE" "$PACKAGE_NAME"
sha256sum "$ARCHIVE" > "$ARCHIVE.sha256"

echo "Release archive: $ARCHIVE"
echo "Checksum: $ARCHIVE.sha256"
du -h "$ARCHIVE"
