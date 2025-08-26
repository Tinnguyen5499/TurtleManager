#!/usr/bin/env bash
# Install all system + Python deps for TurtleManager on Ubuntu 20.04+ and containers.
# Safe on hosts too. Idempotent where possible.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DEBIAN_FRONTEND=noninteractive

echo ">>> [1/7] apt update"
sudo apt-get update -y

echo ">>> [2/7] Install system packages (Qt/X11, terminals, tools)"
# Terminals: we install gnome-terminal to match GUI behavior; tmux always present for logs.
sudo apt-get install -y \
  python3 python3-pip python3-venv python3-dev build-essential git tmux sshpass \
  gnome-terminal \
  libx11-xcb1 libxcb1 libxcb-cursor0 libxcb-icccm4 libxcb-image0 \
  libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-xfixes0 \
  libxcb-shape0 libxcb-xinerama0 libxcb-xkb1 libxkbcommon-x11-0 \
  libgl1 libglu1-mesa libsm6 libxext6 libglib2.0-0

# Wayland users sometimes need this (no-op on X11)
sudo apt-get install -y qtwayland5 || true

echo ">>> [3/7] Ensure recent pip/setuptools/wheel/sip/packaging"
python3 -m pip install -U pip setuptools wheel sip packaging || true

echo ">>> [4/7] Install Python requirements"
if [[ -f "$REPO_ROOT/requirements.txt" ]]; then
  # On older pip (Ubuntu 20.04), --break-system-packages doesn't exist. Detect & adapt.
  if python3 -m pip install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
    PIP_EXTRAS="--break-system-packages --ignore-installed"
  else
    PIP_EXTRAS="--ignore-installed"
  fi

  # Some older environments fail to resolve PyQt6 via requirements alone.
  # Install it explicitly first with a safe version bound, then the rest.
  echo ">>> Installing PyQt6 explicitly (pre-step)"
  python3 -m pip install --no-cache-dir "PyQt6>=6.7,<6.8" ${PIP_EXTRAS} || \
  python3 -m pip install --no-cache-dir "PyQt6>=6.5" ${PIP_EXTRAS}

  echo ">>> Installing remaining requirements"
  python3 -m pip install ${PIP_EXTRAS} -r "$REPO_ROOT/requirements.txt"
else
  echo "ERROR: requirements.txt not found at $REPO_ROOT"
  exit 1
fi

echo ">>> [5/7] ROS build helpers (best-effort)"
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep || true
sudo rosdep init 2>/dev/null || true
rosdep update || true

echo ">>> [6/7] Attempt to build embedded turtle_ws if ROS 2 Foxy exists"
if source /opt/ros/foxy/setup.bash 2>/dev/null; then
  pushd "$REPO_ROOT/turtle_ws" >/dev/null
  # Don’t fail the whole install if a ROS dep is missing; just report.
  rosdep install --from-paths src --ignore-src -r -y || true
  colcon build --symlink-install || true
  popd >/dev/null
  echo ">>> turtle_ws build attempted."
else
  echo "NOTE: /opt/ros/foxy not found. Skipping turtle_ws build."
fi

echo ">>> [7/7] Sanity check for PyQt6 import"
python3 - <<'PY'
try:
    from PyQt6.QtWidgets import QApplication
    import numpy, psutil, paramiko, rowan  # from requirements.txt
    print(">>> PyQt6 and core deps import OK")
except Exception as e:
    print("WARNING: Import check failed:", e)
PY

echo
echo ">>> Done. To run the app: ./scripts/run_turtlemanager.sh"
echo ">>> To view logs (started by GUI in tmux): tmux attach -t optitrack_session"
