#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
source /opt/ros/jazzy/setup.bash

if [ ! -f "$ROOT_DIR/install/setup.bash" ]; then
  colcon build --symlink-install --base-paths "$ROOT_DIR/src"
fi

source "$ROOT_DIR/install/setup.bash"
exec ros2 launch zebrat sim_control.launch.py "$@"
