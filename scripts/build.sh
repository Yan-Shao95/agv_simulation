#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
CC=gcc CXX=g++ colcon build --symlink-install
