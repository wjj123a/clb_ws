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
    R1_CLEAN_ENV=1 \
    /bin/bash --noprofile --norc "$0" "$@"
fi

unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu
export DISABLE_ROS1_EOL_WARNINGS=1

set +u
source /opt/ros/noetic/setup.bash
set -u

cd "$WS_DIR"
exec catkin_make --pkg r1 "$@"
