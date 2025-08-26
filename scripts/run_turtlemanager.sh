#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

export QT_QPA_PLATFORM=xcb

if [[ -z "${DISPLAY:-}" ]]; then
  echo "WARNING: \$DISPLAY is empty. If you're on SSH, reconnect with: ssh -Y user@host"
  echo "You can also test headless with: xvfb-run -a python3 -c 'from PyQt6 import QtWidgets; print(\"OK\")'"
fi

python3 TurtleManager.py
