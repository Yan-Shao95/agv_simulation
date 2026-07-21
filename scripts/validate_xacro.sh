#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
test -f install/setup.bash && source install/setup.bash
set -u
command -v xacro >/dev/null || { echo '缺少 xacro，请运行 ./scripts/install_native.sh' >&2; exit 1; }
command -v check_urdf >/dev/null || { echo '缺少 check_urdf，请运行 ./scripts/install_native.sh' >&2; exit 1; }
xacro src/agv_description/urdf/agv.urdf.xacro -o /tmp/agv.urdf
check_urdf /tmp/agv.urdf
echo "JOINTS=$(grep -c '<joint' /tmp/agv.urdf)"
echo "LINKS=$(grep -c '<link' /tmp/agv.urdf)"
grep -q '<samples>720</samples>' /tmp/agv.urdf
grep -q '<max>20.0</max>' /tmp/agv.urdf
