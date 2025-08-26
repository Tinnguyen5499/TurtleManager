#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo ">>> [1/6] apt update"
sudo apt-get update -y

echo ">>> [2/6] Install system packages (Qt/X11, tools)"
sudo apt-get install -y \
  python3-pip python3-wheel git tmux sshpass \
  gnome-terminal || true

# Qt/XCB runtime libs for PyQt6
sudo apt-get install -y \
  libx11-xcb1 libxcb1 libxcb-cursor0 libxcb-icccm4 libxcb-image0 \
  libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-xfixes0 \
  libxcb-shape0 libxcb-xinerama0 libxcb-xkb1 libxkbcommon-x11-0 \
  libgl1 libglu1-mesa libsm6 libxext6

# Optional (Wayland users)
sudo apt-get install -y qtwayland5 || true

echo ">>> [3/6] Ensure recent pip"
python3 -m pip install -U pip --break-system-packages || true

echo ">>> [4/6] Install Python requirements"
if [[ -f "$REPO_ROOT/requirements.txt" ]]; then
  python3 -m pip install --break-system-packages --ignore-installed -r "$REPO_ROOT/requirements.txt"
else
  echo "ERROR: requirements.txt not found at $REPO_ROOT"
  exit 1
fi

echo ">>> [5/6] Install ROS build helpers (if available on this system)"
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep || true
sudo rosdep init 2>/dev/null || true
rosdep update || true

echo ">>> [6/6] Try to build the embedded turtle_ws (if ROS 2 Foxy is installed)"
if source /opt/ros/foxy/setup.bash 2>/dev/null; then
  pushd "$REPO_ROOT/turtle_ws" >/dev/null
  rosdep install --from-paths src --ignore-src -r -y || true
  colcon build --symlink-install || true
  popd >/dev/null
  echo ">>> Build attempted. If errors mention missing NatNet SDK or ROS, follow README Build section."
else
  echo "NOTE: /opt/ros/foxy not found. Skipping turtle_ws build."
fi

echo ">>> Done. To run: ./scripts/run_turtlemanager.sh"
