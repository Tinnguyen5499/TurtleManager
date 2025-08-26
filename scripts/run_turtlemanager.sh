#!/usr/bin/env bash
# Launch the TurtleManager GUI. Designed to work in Docker (X11) and native.

set -euo pipefail
cd "$(dirname "$0")/.."

# Force xcb so Qt uses X11 (reliable in Docker/X11)
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# Helpful warning if DISPLAY is missing
if [[ -z "${DISPLAY:-}" ]]; then
  echo "WARNING: \$DISPLAY is empty (no X11)."
  echo " - If you're on SSH: reconnect with: ssh -Y user@host"
  echo " - Docker users: run container with -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw and run 'xhost +local:root' on the host."
  echo " - Headless test: xvfb-run -a python3 -c 'from PyQt6 import QtWidgets; print(\"PyQt6 OK under Xvfb\")'"
fi

# Launch the GUI
exec python3 TurtleManager.py
