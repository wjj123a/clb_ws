#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "$WS_DIR/run_mapping_laser.sh" \
  world_name:="$WS_DIR/src/R1/worlds/obstacle_arena.world" \
  "$@"
