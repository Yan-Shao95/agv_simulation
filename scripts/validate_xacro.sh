#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
test -f install/setup.bash && source install/setup.bash
set -u
command -v xacro >/dev/null || { echo '缺少 xacro，请运行 ./scripts/install_native.sh' >&2; exit 1; }
command -v check_urdf >/dev/null || { echo '缺少 check_urdf，请运行 ./scripts/install_native.sh' >&2; exit 1; }
for model in agv mecanum ackermann; do
  output="/tmp/${model}.urdf"
  xacro "src/agv_description/urdf/${model}.urdf.xacro" -o "$output"
  check_urdf "$output"
  echo "MODEL=${model} JOINTS=$(grep -c '<joint' "$output") LINKS=$(grep -c '<link' "$output")"
  grep -q '<samples>720</samples>' "$output"
  grep -q '<max>20.0</max>' "$output"
done
