#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: bash tools/verify_cross_tools.sh OUTPUT [--fixture=frames]" >&2
  exit 2
fi
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$1"
output=$(cd "$1" && pwd -P)
run=$(mktemp -d "$output/cross-tools-XXXXXX")
trap 'printf "Cross-tool artifacts at %s\n" "$run"' EXIT
python=${PYTHON:-python3}
blender=${BLENDER:-blender}
command -v "$python" >/dev/null
command -v "$blender" >/dev/null
export PYTHONPATH="$repo${PYTHONPATH:+:$PYTHONPATH}"
export VERIFY_OUTPUT="$run/tests"
"$python" -m unittest discover -s "$repo/tests/integration" -v 2>&1 | tee "$run/tests.log"
selection=()
if [[ $# == 2 ]]; then selection=("$2"); fi
"$python" "$repo/tools/verify_loaders.py" --output "$run/models" "${selection[@]}" >"$run/loaders.log" 2>&1
variants=$("$python" - "${selection[@]}" <<'PY'
import argparse
from tests.fixtures import fixture_variants, OPTIONAL_VARIANTS
parser = argparse.ArgumentParser()
parser.add_argument('--fixture', choices=OPTIONAL_VARIANTS)
print(' '.join(fixture_variants(parser.parse_args().fixture)))
PY
)
for variant in $variants; do
  "$blender" --background --factory-startup --python-exit-code 1 \
    --python "$repo/tools/verify_blender.py" -- \
    --urdf "$run/models/fixture_$variant.urdf" \
    --package "$run/models/fixture_${variant}_description" \
    --expected "$run/models/fixture_$variant.expected.json" \
    --output "$run/blender-$variant" >"$run/blender-$variant.log" 2>&1
done
printf 'PASS cross-tool verification. Artifacts at %s\n' "$run"
