# Build and command a ROS 2 robot

The user installs an exported package, launches it headlessly, and observes
controller-driven joint feedback and TF.

## Sub-features

- Build and discover packages from install space on Humble and Jazzy.
- Launch articulated, fixed-only, and primitive-collision descriptions.
- Activate the joint-state broadcaster and trajectory controller.
- Complete a trajectory and compare feedback and TF against independent values.

## How to get to it (user POV)

Use `display.launch.py` for visualization and `control.launch.py` for mock
control. The Docker scripts exercise those generated launch files.

## Driving it with Docker

Preconditions: `docker version` reaches the engine without sudo, and image
construction can download its packages.

```bash
bash tools/verify_docker.sh humble "$run" 2>&1 | tee "$run/humble.log"
bash tools/verify_docker.sh jazzy "$run" 2>&1 | tee "$run/jazzy.log"
```

Require `ros.json` from both distributions. Confirm controllers are active,
the action error code is zero, feedback matches the commanded values, and TF
matches the expected poses. Verify output ownership and container cleanup.

To verify an actual CAD export rather than generated fixtures, pass the native bundle as
the third argument to `tools/verify_docker.sh`. Use `tools/verify_export.py BUNDLE --output OUTPUT`
for the complete loader, Blender, and both-distribution recipe. Require
`evidence_kind: installed-native-fusion-export` and installed native-asset hashes in each ROS report.

For real GUI media, run `bash tools/capture_ros.sh NATIVE_BUNDLE OUTPUT`. Require the same
controller/feedback/TF checks before zero/moved RViz screenshots, frame-locked component labels,
and the trajectory recording. See [the showcase recipe](../../../../docs/showcase.md).
Do not substitute Blender previews for ROS/RViz captures or label mock control as physics.

## Gotchas

The image explicitly selects CycloneDDS. A pass does not certify other RMW
implementations. Mock control does not simulate physics. GUI nodes are disabled
in this recipe. Preserve reports from failed runs instead of overwriting them.
