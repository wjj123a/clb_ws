#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${R1_CLEAN_ENV:-}" ]; then
  exec env -i \
    HOME="${HOME:-$WS_DIR}" \
    USER="${USER:-$(id -un)}" \
    LOGNAME="${LOGNAME:-${USER:-$(id -un)}}" \
    SHELL=/bin/bash \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    DISPLAY="${DISPLAY:-:0}" \
    WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}" \
    PULSE_SERVER="${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}" \
    R1_CLEAN_ENV=1 \
    /bin/bash --noprofile --norc "$0" "$@"
fi

QUIET=0
if [ "${1:-}" = "--quiet" ]; then
  QUIET=1
fi

say() {
  if [ "$QUIET" -eq 0 ]; then
    echo "$@"
  fi
}

unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu

set +u
source /opt/ros/noetic/setup.bash
if [ -f "$WS_DIR/devel/setup.bash" ]; then
  source "$WS_DIR/devel/setup.bash"
fi
set -u

export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export ROS_HOME="${ROS_HOME:-$HOME/.ros}"

if rosservice type /gazebo/delete_model >/dev/null 2>&1; then
  say "Deleting old R1 model from Gazebo..."
  rosservice call /gazebo/delete_model "{model_name: 'R1'}" >/dev/null 2>&1 || true
fi

if rosnode list >/dev/null 2>&1; then
  mapfile -t nodes < <(rosnode list 2>/dev/null | grep -E '^/(gazebo|gazebo_gui|robot_state_publisher|spawn_model|map_server|move_base|amcl|odom_sanitizer|joint_state_publisher|static_joint_state_publisher|rviz|slam_gmapping|ekf_mapping|rgbd_sync|rtabmap|cmd_vel_limiter)')
  if [ "${#nodes[@]}" -gt 0 ]; then
    say "Stopping stale ROS nodes..."
    rosnode kill "${nodes[@]}" >/dev/null 2>&1 || true
  fi
fi

say "Stopping stale Gazebo/RViz processes..."
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/gazebo_ros/gzserver' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/gazebo_ros/gzclient' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/rviz/rviz' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/home/w/clb_ws/devel/lib/r1/cmd_vel_limiter.py' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/gazebo_ros/spawn_model' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/home/w/clb_ws/devel/lib/r1/odom_sanitizer.py' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/home/w/clb_ws/devel/lib/r1/static_joint_state_publisher.py' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/move_base/move_base' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/map_server/map_server' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/amcl/amcl' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/slam_gmapping/slam_gmapping' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/robot_localization/ekf_localization_node' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/rtabmap_sync/rgbd_sync' >/dev/null 2>&1 || true
pkill -u "$(id -u)" -f '/opt/ros/noetic/lib/rtabmap_slam/rtabmap' >/dev/null 2>&1 || true

sleep 2
say "Cleanup complete."
