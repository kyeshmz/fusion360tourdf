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
bash tools/verify_docker.sh humble "$run" --fixture=frames 2>&1 | tee "$run/frames-humble.log"
bash tools/verify_docker.sh jazzy "$run" --fixture=frames 2>&1 | tee "$run/frames-jazzy.log"
for distro in humble jazzy; do
  bash tools/verify_docker.sh "$distro" "$run" --fixture=generalized
  bash tools/verify_docker.sh "$distro" "$run" --fixture=mimic
  bash tools/verify_docker.sh "$distro" "$run" --fixture=materials
done
```

Require `ros.json` from both distributions. Confirm controllers are active,
the action error code is zero, feedback matches the commanded values, and TF
matches the expected poses. Verify output ownership and container cleanup.

The opt-in frame fixture retains three commanded joints and all literal
zero/moved TF poses. Its empty root and internal arm have explicit kinds and
no physical assets. Physical links still require installed geometry and every
visual/collision mesh resource. The report remains `generated-fixtures`.
The optional selector is passed through the existing wrappers and does not
call native-bundle validation or claim original CAD evidence.

The mimic fixture commands only the independent rotation. Require the follower
slide's actual joint-state feedback and TF as well. GenericSystem compatibility
uses zero-offset native delta coordinates on both distributions. Explicit
nonzero-offset URDF packages reject control launch on Humble because that
implementation does not propagate offsets. Jazzy offset behavior remains a
separate runtime gate. Installed material cases verify every OBJ/MTL/image
resource and hash rather than treating them as binary STL.

The same registry adds `policy_rotation`, `policy_boxes_dh`, `policy_floor`,
`mesh`, `surface`, `hull`, `materials_hull`, and `materials_cells`. Both Docker
wrappers accept these generated selectors without native-bundle validation.
TF is measured from `root_frame.output_reference_link` when present, otherwise
`base_link`. Compare the original base link's nonidentity pose, not an assumed
identity. The mesh fixture commands only `drive` and requires derived `follow`
feedback, both physical meshes, and frame/reference TF. Surfaces retain supplied
SI provenance and zero thickness. No controller or geometry result proves CAD.

`per_body` and `per_body_hull` require every installed OBJ/MTL/image resource
and nonzero `turn` control on both distributions. Control applicability derives
from the central oracle's command samples, never a fixture-name allowlist.
Require active controllers, successful action, fresh joint feedback, and both
zero/moved TF from the output reference. A zero-display pass is not control
proof. Fixed/material/surface fixtures remain display-only.

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
