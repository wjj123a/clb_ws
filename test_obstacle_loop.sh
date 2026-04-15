#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$WS_DIR"
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
export PATH="$WS_DIR/devel/bin:/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
R1_PYYAML_SITE_PACKAGES="$WS_DIR/python_vendor"
if [ -d "$R1_PYYAML_SITE_PACKAGES/yaml" ]; then
  export PYTHONPATH="$R1_PYYAML_SITE_PACKAGES${PYTHONPATH:+:$PYTHONPATH}"
fi

GOAL_TIMEOUT="${GOAL_TIMEOUT:-90}"

publish_goal() {
  local x="$1"
  local y="$2"
  local qz="$3"
  local qw="$4"
  rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position:
    x: ${x}
    y: ${y}
    z: 0.0
  orientation:
    z: ${qz}
    w: ${qw}" >/dev/null
}

read_ground_truth() {
  local line
  line="$(rostopic echo -n 1 /ground_truth_odom 2>/dev/null | awk '
    /position:/ {pos=1; next}
    pos && /x:/ && px=="" {px=$2; next}
    pos && /y:/ && py=="" {py=$2; next}
    pos && /z:/ && pz=="" {pz=$2; pos=0; next}
    /orientation:/ {ori=1; next}
    ori && /x:/ && ox=="" {ox=$2; next}
    ori && /y:/ && oy=="" {oy=$2; exit}
    END {printf "%s %s %s %s %s\n", px, py, pz, ox, oy}
  ')"
  echo "${line}"
}

wait_for_goal() {
  local label="$1"
  local goal_x="$2"
  local goal_y="$3"
  local deadline=$((SECONDS + GOAL_TIMEOUT))
  while (( SECONDS < deadline )); do
    local status_dump status
    status_dump="$(rostopic echo -n 1 /move_base/status 2>/dev/null || true)"
    status="$(printf '%s\n' "${status_dump}" | awk '/status: / {print $2; exit}')"

    local gt px py pz ox oy
    gt="$(read_ground_truth)"
    read -r px py pz ox oy <<<"${gt}"
    printf '[%s] pos=(%s,%s,%s) tilt_q=(%s,%s) status=%s\n' "${label}" "${px}" "${py}" "${pz}" "${ox}" "${oy}" "${status:-unknown}"

    local near_goal=1
    if [[ -n "${px:-}" && -n "${py:-}" ]]; then
      if awk -v px="${px}" -v py="${py}" -v gx="${goal_x}" -v gy="${goal_y}" \
        'BEGIN {dx = px - gx; dy = py - gy; exit !((dx * dx + dy * dy) <= 0.20 * 0.20)}'; then
        near_goal=0
      fi
    fi

    if [[ -n "${ox:-}" && -n "${oy:-}" ]]; then
      if awk -v x="${ox}" -v y="${oy}" 'BEGIN {ax = (x < 0 ? -x : x); ay = (y < 0 ? -y : y); exit !(ax > 0.20 || ay > 0.20)}'; then
        echo "[${label}] detected large roll/pitch; robot likely tipped."
        return 2
      fi
    fi

    case "${status:-}" in
      3)
        if [[ "${near_goal}" -eq 0 ]]; then
          echo "[${label}] goal reached."
          return 0
        fi
        ;;
      4|5|6|8|9)
        echo "[${label}] move_base failed with status=${status}."
        return 1
        ;;
    esac

    sleep 1
  done

  echo "[${label}] timed out after ${GOAL_TIMEOUT}s."
  return 1
}

run_goal() {
  local label="$1"
  local x="$2"
  local y="$3"
  local qz="$4"
  local qw="$5"
  echo
  echo "=== ${label} ==="
  echo "Publishing goal: (${x}, ${y})"
  publish_goal "${x}" "${y}" "${qz}" "${qw}"
  wait_for_goal "${label}" "${x}" "${y}"
}

echo "Starting obstacle loop test."
run_goal "north_west" "-0.40" "1.60" "0.0" "1.0"
run_goal "north_east" "3.20" "1.60" "0.0" "1.0"
run_goal "south_east" "3.20" "-1.60" "-0.70710678" "0.70710678"
run_goal "south_west" "-0.40" "-1.60" "1.0" "0.0"
run_goal "return_home" "0.00" "0.00" "0.0" "1.0"
echo
echo "Obstacle loop test finished successfully."
