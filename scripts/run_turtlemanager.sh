#!/usr/bin/env bash
set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$_here/.." && pwd)"
WS_DIR="$REPO_DIR/turtle_ws"

# Source ROS Foxy and the embedded workspace for THIS process only.
source /opt/ros/foxy/setup.bash
source "$WS_DIR/install/setup.bash"

cd "$REPO_DIR"
exec python3 TurtleManager.py
