#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT_DIR"

colcon build --symlink-install --base-paths "$ROOT_DIR/src"

set +u
source "$ROOT_DIR/install/setup.bash"
set -u

launch_args=("$@")
has_launch_arg() {
  local name="$1"
  local arg
  for arg in "${launch_args[@]}"; do
    if [[ "$arg" == "$name":=* ]]; then
      return 0
    fi
  done
  return 1
}

if ! has_launch_arg gui; then
  launch_args=("gui:=true" "${launch_args[@]}")
fi

if ! has_launch_arg rviz; then
  launch_args=("rviz:=true" "${launch_args[@]}")
fi

if ! has_launch_arg rtabmap; then
  launch_args=("rtabmap:=false" "${launch_args[@]}")
fi

if ! has_launch_arg controller_start_delay; then
  launch_args=("controller_start_delay:=6.0" "${launch_args[@]}")
fi

if ! has_launch_arg nav2_start_delay; then
  launch_args=("nav2_start_delay:=14.0" "${launch_args[@]}")
fi

if ! has_launch_arg rtabmap_start_delay; then
  launch_args=("rtabmap_start_delay:=18.0" "${launch_args[@]}")
fi

exec ros2 launch zebrat nav2_area_full.launch.py "${launch_args[@]}"
