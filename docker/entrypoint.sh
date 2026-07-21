#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
test -f /workspace/install/setup.bash && source /workspace/install/setup.bash
exec "$@"
