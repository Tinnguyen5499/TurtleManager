#!/usr/bin/env bash
set -euo pipefail

# minimal first-run builder (no virtualenv)
# prerequisites: ROS 2 Foxy installed, colcon + rosdep available

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$_here/.." && pwd)"
WS_DIR="$REPO_DIR/turtle_ws"

echo "==> Using repo: $REPO_DIR"
echo "==> Workspace : $WS_DIR"

if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: 'colcon' not found. Install: sudo apt-get update && sudo apt-get install -y python3-colcon-common-extensions"
  exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
  echo "ERROR: 'rosdep' not found. Install: sudo apt-get install -y python3-rosdep && sudo rosdep init && rosdep update"
  exit 1
fi

# source ROS 2 Foxy for this *subprocess*; users don’t need to do this manually
source /opt/ros/foxy/setup.bash

# python deps (system install; no venv)
if [ -f "$REPO_DIR/requirements.txt" ]; then
  python3 -m pip install --upgrade pip >/dev/null
  python3 -m pip install -r "$REPO_DIR/requirements.txt"
fi

# rosdep (ignore-src to avoid apt’ing your source pkgs)
pushd "$WS_DIR" >/dev/null
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y

  # build everything in turtle_ws
  colcon build --symlink-install
popd >/dev/null

echo "==> Done. Workspace built at: $WS_DIR/install"
