#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if test -f install_runtime/setup.bash; then
  source install_runtime/setup.bash
else
  source install/setup.bash
fi
set -u
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p repeat_rate:=10.0
