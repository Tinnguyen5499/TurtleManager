#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/foxy/setup.bash
cd "$(dirname "$0")/.."
cd turtle_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  mocap_optitrack_interfaces \
  mocap_optitrack_client \
  mocap_optitrack_inv_kin \
  mocap_optitrack_w2b \
  process_mocap
