#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 2 || ! "$1" =~ ^(humble|jazzy)$ ]]; then
  echo "Usage: bash tools/verify_docker.sh humble|jazzy OUTPUT" >&2
  exit 2
fi
distro=$1
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output=$(realpath -m "$2")
mkdir -p "$output"
run=$(mktemp -d "$output/docker-$distro-XXXXXX")
image="fusion360tourdf-verify:$(basename "$run")"
container="fusion360tourdf-$distro-$$"
cleanup() {
  docker rm -f "$container" >/dev/null 2>&1 || true
  docker image rm --no-prune "$image" >/dev/null 2>&1 || true
  printf 'Docker artifacts at %s\n' "$run"
}
trap cleanup EXIT
docker version >"$run/docker-version.log" 2>&1
docker build --build-arg "ROS_DISTRO=$distro" -f "$repo/tools/Dockerfile.ros" \
  -t "$image" "$repo" 2>&1 | tee "$run/docker-build.log"
docker run --name "$container" --init --network none \
  --user "$(id -u):$(id -g)" --env HOME=/tmp \
  --mount "type=bind,src=$run,dst=/output" \
  "$image" 2>&1 | tee "$run/docker-run.log"
printf 'PASS Docker ROS %s. Artifacts at %s\n' "$distro" "$run"
