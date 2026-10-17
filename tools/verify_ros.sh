#!/usr/bin/env bash
set -eo pipefail
if [[ $# != 1 ]]; then
  echo "Usage: bash tools/verify_ros.sh OUTPUT" >&2
  exit 2
fi
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output=$(realpath -m "$1")
mkdir -p "$output"
run=$(mktemp -d "$output/ros-${ROS_DISTRO:-unknown}-XXXXXX")
trap 'printf "ROS artifacts at %s\n" "$run"' EXIT
source "/opt/ros/${ROS_DISTRO:?Source a supported ROS distro}/setup.bash"
set -u
export PYTHONPATH="$repo${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((20 + $$ % 180))}
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$run/ros-logs"
export ROS2CLI_DISABLE_DAEMON=1
python3 -m tests.fixtures "$run/ws/src" >"$run/generate.log" 2>&1
(
  cd "$run/ws"
  timeout --signal=INT --kill-after=10s 180s colcon build --event-handlers console_direct+ \
    --cmake-args -DBUILD_TESTING=OFF
) 2>&1 | tee "$run/build.log"
set +u
source "$run/ws/install/setup.bash"
set -u
cd "$run"
python3 "$repo/tools/verify_ros.py" --output "$run" --install "$run/ws/install" \
  2>&1 | tee "$run/runtime.log"
