#!/usr/bin/env bash
# Shared relocatable runtime discovery for every ARX launcher.

ARX_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ARX_ROOT:-$(cd "$ARX_TOOLS_DIR/.." && pwd)}"

if [[ -n "${ARX_PYTHON:-}" ]]; then
  PYTHON="$ARX_PYTHON"
elif [[ -n "${CONDA_PREFIX:-}" && -x "$CONDA_PREFIX/bin/python3.10" ]]; then
  PYTHON="$CONDA_PREFIX/bin/python3.10"
elif [[ -x "$HOME/anaconda3/envs/lerobot/bin/python3.10" ]]; then
  # Compatibility with the validated source workstation.
  PYTHON="$HOME/anaconda3/envs/lerobot/bin/python3.10"
elif command -v python3.10 >/dev/null 2>&1; then
  PYTHON="$(command -v python3.10)"
else
  PYTHON="$(command -v python3)"
fi

export ARX_ROOT="$ROOT"
export ARX_PYTHON="$PYTHON"
