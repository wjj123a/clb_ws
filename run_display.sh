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

unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu
R1_PYYAML_SITE_PACKAGES="$WS_DIR/python_vendor"
if [ -d "$R1_PYYAML_SITE_PACKAGES/yaml" ]; then
  export PYTHONPATH="$R1_PYYAML_SITE_PACKAGES"
fi
export DISABLE_ROS1_EOL_WARNINGS=1
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
export PULSE_SERVER="${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export LIBGL_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri

set +u
source /opt/ros/noetic/setup.bash

if [ -f "$WS_DIR/devel/setup.bash" ]; then
  source "$WS_DIR/devel/setup.bash"
else
  echo "devel/setup.bash not found. Run ./build_r1.sh first."
  exit 1
fi
set -u

export PATH="$WS_DIR/devel/bin:/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export ROS_HOME="${ROS_HOME:-$HOME/.ros}"

cd "$WS_DIR"
exec roslaunch r1 display.launch "$@"
