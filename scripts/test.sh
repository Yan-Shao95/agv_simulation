#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
test -f install/setup.bash && source install/setup.bash
set -u
colcon test --event-handlers console_direct+
colcon test-result --verbose
