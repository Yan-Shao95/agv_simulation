#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if test -f install_runtime/setup.bash; then
  source install_runtime/setup.bash
else
  source install/setup.bash
fi
set -u
ros2 run agv_drive_controller keyboard_teleop
