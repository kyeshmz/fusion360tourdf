#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 2 ]]; then
  echo 'Usage: bash tools/capture_ros.sh NATIVE_BUNDLE OUTPUT' >&2
  exit 2
fi
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
bundle=$(cd "$1" && pwd -P)
mkdir -p "$2"
output=$(cd "$2" && pwd -P)
run=$(mktemp -d "$output/rviz-XXXXXX")
image="fusion360tourdf-verify:$(basename "$run")"
container="fusion360tourdf-rviz-$$"
cleanup() {
  docker rm -f "$container" >/dev/null 2>&1 || true
  docker image rm --no-prune "$image" >/dev/null 2>&1 || true
  echo "RViz evidence: $run"
}
trap cleanup EXIT
docker build --build-arg ROS_DISTRO=jazzy --build-arg WITH_GUI=true \
  -f "$repo/tools/Dockerfile.ros" -t "$image" "$repo" >"$run/build.log" 2>&1
docker run --name "$container" --init --network none --user "$(id -u):$(id -g)" \
  --env HOME=/tmp --env LIBGL_ALWAYS_SOFTWARE=1 --env QT_X11_NO_MITSHM=1 \
  --mount "type=bind,src=$bundle,dst=/input,readonly" \
  --mount "type=bind,src=$run,dst=/output" \
  "$image" bash -c '
    set -euo pipefail
    bash tools/verify_ros.sh /output /input
    install=$(find /output -type d -path "*/ws/install" -print -quit)
    set +u
    source "$install/setup.bash"
    set -u
    export PYTHONPATH="/repo${PYTHONPATH:+:$PYTHONPATH}" ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
    export ROS_LOG_DIR=/output/ros-logs ROS2CLI_DISABLE_DAEMON=1
    xvfb-run -a -s "-screen 0 1280x720x24" python3 tools/capture_rviz.py \
      --bundle /input --install "$install" --output /output/capture
  ' >"$run/capture.log" 2>&1
echo "PASS real ROS 2 Jazzy RViz screenshots and motion: $run/capture"
