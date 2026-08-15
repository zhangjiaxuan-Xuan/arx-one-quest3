#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runtime_env.sh"
ENV_NAME="${ARX_CONDA_ENV_NAME:-arx-ac-one}"

if ! command -v conda >/dev/null 2>&1; then
  echo "conda not found; install Miniconda or Miniforge first." >&2
  exit 1
fi

if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  conda env create -n "$ENV_NAME" -f "$ROOT/environment.yml"
fi

ENV_PREFIX="$(conda env list | awk -v name="$ENV_NAME" '$1 == name {print $NF; exit}')"
[[ -x "$ENV_PREFIX/bin/python" ]] || { echo "Cannot resolve environment $ENV_NAME" >&2; exit 1; }

UV="$ENV_PREFIX/bin/uv"
if [[ -d "$ROOT/wheelhouse" ]] && compgen -G "$ROOT/wheelhouse/arx5_interface-0.1.3-*.whl" >/dev/null; then
  "$UV" pip install --python "$ENV_PREFIX/bin/python" \
    --find-links "$ROOT/wheelhouse" -r "$ROOT/requirements-runtime.txt"
else
  "$UV" pip install --python "$ENV_PREFIX/bin/python" \
    -r "$ROOT/requirements-runtime.txt"
fi

"$ENV_PREFIX/bin/python" -c 'import arx5_interface, cv2, numpy, scipy; print("ARX runtime imports OK")'
echo "Environment ready. Run: conda activate $ENV_NAME"

