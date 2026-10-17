---
name: verify-fusion360tourdf
description: Verify fusion360tourdf exports after changes to Fusion extraction, ROS 2 package generation, geometry, or verification tools. Run the repository's offline, Docker ROS 2, and Blender checks and distinguish fixture evidence from live Fusion evidence.
---

# Verify fusion360tourdf

## Launch

Run commands from the repository root. Use the existing scripts, not a second
robot-description generator. Create an isolated evidence directory:

```bash
set -euo pipefail
run=$(mktemp -d "${TMPDIR:-/tmp/opencode}/fusion360tourdf-verify-XXXXXX")
python3 -m unittest discover -s tests -v 2>&1 | tee "$run/offline.log"
```

The offline suite needs Python 3.11 or newer. The exporter itself runs inside
Fusion. No local server or port is required for the verification commands.

## Doctor

Check only the tools needed by the selected recipe:

```bash
python3 --version
docker version
command -v blender
command -v check_urdf
```

Docker needs an accessible engine. Do not change socket permissions or use
sudo to bypass a failed check. Report the missing access. Optional Python
dependencies are in [requirements-test.txt](../../../requirements-test.txt).
Use `PYTHON`, `BLENDER`, and `CHECK_URDF` to select installed tool paths.

## Drive

Read the [feature map](features/README.md). Choose the relevant recipe:

- [Fusion export](features/fusion-export.md) covers the actual CAD boundary.
- [Cross-tool loading](features/cross-tool-loading.md) covers geometry and FK.
- [ROS 2 runtime](features/ros2-runtime.md) covers installed packages and motion.

For writer changes, run both Docker distributions and cross-tool checks.
For extraction changes, also run the offline tests and require a live Fusion
export before claiming that vendor behavior is verified. A fixture callback
does not replace that last check.

Run `python3 tools/verify_fusion_coverage.py --output "$run/fusion-coverage"`
to check the declared input cases. Read its support status and evidence kind.
Passing rejection tests does not implement a missing input. Keep
`--require-complete` nonzero when only offline evidence is available.

## Evidence

Require successful command exits and inspect the reports. Docker `ros.json`
records the distribution, RMW, active controllers, trajectory result, joint
feedback, and TF. Cross-tool runs retain `loaders.json`, `blender.json`, and
`inspection.blend` for articulated, fixed, and box-collision variants.

The loader suite must reject changed axes, shifted meshes, and missing meshes.
Do not weaken those checks to accommodate a failure. Blender proves geometry
placement, not full URDF semantics or physics. Mock control proves command and
state propagation, not physical hardware or Gazebo dynamics.

Record the tested revision, whether the tree is dirty, commands, report paths,
and unexecuted gates. Read [the testing guide](../../../docs/testing.md) for
runtime requirements and the explicit CycloneDDS test-image choice.

## Cleanup

Each Docker script owns a unique container and image tag and attempts their
removal on exit. Confirm cleanup after the recipe:

```bash
docker ps -a --filter name=fusion360tourdf --format '{{.Names}} {{.Status}}'
docker image ls fusion360tourdf-verify --format '{{.Repository}}:{{.Tag}}'
```

Identify another run's resources before acting on nonempty output. The scripts
retain parent build layers. Do not prune shared images or kill processes by
name. Keep the evidence directory. Cross-tool commands are short-lived and
preserve their `.blend` files without a server.

## Helpers

The recipes use `bash tools/verify_docker.sh` and
`bash tools/verify_cross_tools.sh`. No separate skill helpers are needed.
Update the feature map when commands, entry points, or proof requirements change.
