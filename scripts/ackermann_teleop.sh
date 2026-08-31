#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname "$script_dir")"
workspace_dir="$(cd "$project_dir/../.." && pwd)"

if test -f "$workspace_dir/install_three_vehicle/setup.bash"; then
  source "$workspace_dir/install_three_vehicle/setup.bash"
elif test -f "$workspace_dir/install/setup.bash"; then
  source "$workspace_dir/install/setup.bash"
else
  echo "未找到工作空间安装环境，请先构建项目。" >&2
  exit 1
fi

set -u
exec ros2 run agv_drive_controller ackermann_keyboard
