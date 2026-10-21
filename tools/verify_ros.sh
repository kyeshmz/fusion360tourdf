#!/usr/bin/env bash
set -eo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: bash tools/verify_ros.sh OUTPUT [NATIVE_BUNDLE | --fixture=frames]" >&2
  exit 2
fi
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$1"
output=$(cd "$1" && pwd -P)
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
input=(--timeout 45)
if [[ $# == 2 && "$2" == --fixture=* ]]; then
  python3 -m tests.fixtures "$run/ws/src" "$2" >"$run/generate.log" 2>&1
  input+=("$2")
elif [[ $# == 2 ]]; then
  bundle=$(cd "$2" && pwd -P)
  python3 - "$bundle" "$run/ws/src" >"$run/generate.log" <<'PY'
from pathlib import Path
import shutil
import sys
from tools.export_bundle import read_bundle
package, expected = read_bundle(sys.argv[1])
destination = Path(sys.argv[2]) / package.name
shutil.copytree(package, destination)
print('Copied native Fusion export:', package, 'to', destination)
PY
  input+=(--bundle "$bundle")
else
  python3 -m tests.fixtures "$run/ws/src" >"$run/generate.log" 2>&1
fi
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
  "${input[@]}" 2>&1 | tee "$run/runtime.log"
