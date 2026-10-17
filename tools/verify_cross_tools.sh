#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 ]]; then
  echo "Usage: bash tools/verify_cross_tools.sh OUTPUT" >&2
  exit 2
fi
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output=$(realpath -m "$1")
mkdir -p "$output"
run=$(mktemp -d "$output/cross-tools-XXXXXX")
trap 'printf "Cross-tool artifacts at %s\n" "$run"' EXIT
python=${PYTHON:-python3}
blender=${BLENDER:-blender}
command -v "$python" >/dev/null
command -v "$blender" >/dev/null
export PYTHONPATH="$repo${PYTHONPATH:+:$PYTHONPATH}"
export VERIFY_OUTPUT="$run/tests"
"$python" -m unittest discover -s "$repo/tests/integration" -v 2>&1 | tee "$run/tests.log"
"$python" "$repo/tools/verify_loaders.py" --output "$run/models" >"$run/loaders.log" 2>&1
for variant in articulated fixed boxes; do
  "$blender" --background --factory-startup --python-exit-code 1 \
    --python "$repo/tools/verify_blender.py" -- \
    --urdf "$run/models/fixture_$variant.urdf" \
    --package "$run/models/fixture_${variant}_description" \
    --expected "$run/models/fixture_$variant.expected.json" \
    --output "$run/blender-$variant" >"$run/blender-$variant.log" 2>&1
done
printf 'PASS cross-tool verification. Artifacts at %s\n' "$run"
