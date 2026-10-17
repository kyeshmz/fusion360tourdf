# fusion360tourdf

Export an Autodesk Fusion assembly as a ROS 2 URDF description package with
STL meshes, RViz launch files, and optional mock-control launch files.
The exporter runs inside Fusion and uses its Python API plus the standard
library. ROS and Blender are verification tools, not Fusion dependencies.

This is a fork of [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf).
The maintained output target is ROS 2 Humble and Jazzy. PyBullet-specific
behavior, other CAD systems, ROS 1, SDF, MJCF, and simulator backends are out of
scope. Fusion input cases are tracked separately from those output exclusions.

## Read the support and evidence boundaries

Use Fusion May 2025 or newer for explicit STL export units. TOML configuration
requires Python 3.11 or newer, as supplied by current Fusion releases.

The [Fusion use-case matrix](docs/fusion-coverage.md) distinguishes implemented
inputs, executable offline tests, native-Fusion verification, and remaining
input gaps. [Nine pinned repository audits](docs/related-work.md) inform that
matrix. Source inspection is not a runtime pass or a universal superiority claim.

The local test suites exercise the real package writer, mesh loaders, Blender,
and Docker ROS 2 motion. Fusion API doubles do not establish that the actual
application exports proxy geometry correctly. See
[the verification results](docs/verification-results.md) for exact evidence.

## Install in Fusion

Copy the complete `URDF_Exporter` directory, including its `core`, `utils`,
manifest, and license files. Do not copy only `URDF_Exporter.py`.

| Platform | Fusion API scripts directory |
| --- | --- |
| Windows | `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts` |
| macOS | `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts` |

In Fusion, open **Utilities**, **Add-ins**, and **Scripts and Add-ins**.
Use the Scripts tab to add or refresh the `URDF_Exporter` script. You can also
add the checkout's script directory directly instead of copying it.

Before updating an existing installation, preserve your `xacro_export.toml`.
Remove obsolete options only after reviewing the configuration table below.
Save your work and restart Fusion after replacing exporter modules so cached
Python imports do not continue running an older version.
Running Fusion through Wine is not a supported Autodesk platform and has not
been certified by these tests.

Fusion and ROS do not need to run on the same computer. Copy the generated
description package into your ROS 2 workspace after export. The shell-based
verification instructions target Linux or WSL2 with Bash and Docker Engine.

## Prepare the assembly

Export reads the active design's root assembly, not only the currently activated
component. Select a complete connected mechanism through visibility and
configuration. Hidden geometry does not automatically suppress its constraints.

### Components, bodies, and names

- Make each independently moving physical link a component occurrence.
  Several bodies can belong to one link.
- Repeated instances must keep their distinct occurrence paths. Reusing one
  component definition is different from giving multiple instances one identity.
- Nested components can contain their own bodies and child occurrences. Each
  link's mesh and mass must use only its own selected bodies, not its descendants
  a second time.
- Name the intended chassis `base_link` for the least ambiguous default.
  Use `root_occurrence` when selecting another root explicitly.
- Avoid names that become identical after unsafe characters are replaced with
  underscores. The exporter rejects collisions instead of overwriting meshes.
- Root-owned BRep bodies support imported CAD without requiring a new component
  solely to hold the chassis. Mesh-only imports remain a separate input gap.

### Mechanics and physical properties

- Ordinary and as-built rigid, revolute, and slider joints are the maintained
  joint types. A revolute without bounds becomes continuous. A slider needs
  finite lower and upper travel limits.
- Set both bounds or neither where that joint type permits it. One-sided bounds
  must not silently become zero travel.
- The current design pose becomes URDF joint zero. Bounds subtract the current
  driven position so the physical travel range is retained.
- Suppressing a joint changes the mechanism. Hiding its marker does not.
  Locking a joint means fixed-at-current-pose behavior, not an active actuator.
- Rigid groups are Fusion assembly constraints. They must become fixed
  relationships or produce an explicit conflict, not disappear silently.
- Ground To Parent is also a fixed relationship. Physical parents and proven
  fixed ancestry are translated without changing source flags. An unresolved
  bodyless parent/world frame fails explicitly instead of being welded to a
  guessed root. This is partial grounding support, not general bodyless-frame
  export.
- Every selected link must connect to the root. A disconnected part is not
  automatically welded merely because it appears near another body.
- Assign physical materials and check mass properties in Fusion. Visual
  appearance colors and physical material density are different properties.

The current STL-based output has one appearance color per link. It does not
preserve per-face textures or a distinct material for every body in a combined
mesh. Those are fidelity limits, not simulator exclusions.

Color extraction supports modern primary colors and known legacy
`appearanceProperties` color values. Native RGB and opacity values are bytes,
not normalized fractions. A usable primary color can be retained even when an
appearance also has texture metadata; the texture pattern is not exported.
When no supported flat color is available, the writer uses silver rather than
another material's color. This does not change physical density or mass.

Use Z-up and X-forward where possible. Fusion uses centimeters internally.
The exporter converts joint positions and physical properties to SI units,
requests millimeter STL output explicitly, and writes mesh scale `0.001`.

### Root selection and endpoint direction

The configured `root_occurrence` takes precedence. Use an exact occurrence path,
not a component name that can refer to several instances. Use `design_root` to
select direct root bodies explicitly. Without configuration,
selection prefers an unambiguous `base_link`, direct root bodies, a unique
grounded occurrence, and then a unique graph root. Ambiguous candidates require
an explicit choice rather than a first-match guess.

Joint edges are oriented outward from the selected root. Reversal must update
frames and axes as well as endpoint names. Grounding the base is not a reason
to modify the source design before export.

Selecting the root changes topology, not the design coordinate system. Meshes
remain in the design pose, with URDF visual origins accounting for link frames.
It does not restrict the export to that occurrence's subtree. Do not recenter
meshes by hand after export.

## Keep design history intact

**Do not disable Capture Design History to make an export work.** Autodesk
documents that changing a parametric design to direct mode removes its timeline
and history. The exporter must not perform that conversion, rewind the timeline,
reset driven joints, rename source components, or move bodies.

The extraction path reads current joint transforms instead of the original
face, edge, or origin references. This matters after edits invalidate geometry
that originally defined a joint. It does not fall back to old geometry when a
required current transform is unavailable. Rigid as-built joints legitimately
have no joint geometry. Their fixed relationship comes from the occurrence pose.

If a moving joint has no usable current frame, repair it in Fusion. Valid old
geometry alone does not rescue this case. Do not replace its anchor with an
arbitrary origin or delete history to hide the failure. Inspect suppressed,
invalid, or unresolved features and recompute the design before retrying.

Parametric and direct inputs have separate regression cases. Native confirmation
requires [the real Fusion verifier](tools/verify_fusion/README.md), not a Docker
test or a temporary direct-design staging document.

## Export the package

1. Open the intended design in Fusion's Design workspace.
2. Set geometry visibility and inspect the joints and physical properties.
3. Run **URDF_Exporter** from Scripts and Add-ins.
4. Choose an output parent folder. Cancellation does not export a package.
5. Inspect the resulting `<robot>_description` directory and any reported errors.

Leave the assembly unchanged until export completes. Concurrent editing or
driving the same design from another script is not a verified workflow.

The UI uses the first whitespace-delimited token of the root/design name,
sanitized to a lowercase ROS name. For example, `My Robot` produces
`my_description`, not `my_robot_description`. Use a fresh package destination.
Existing output is not overwritten. Files are staged
privately and published only after required mesh exports pass validation.
Do not run concurrent exports into the same destination.

The package layout is:

```text
<robot>_description/
  package.xml
  CMakeLists.txt
  urdf/<robot>.xacro
  meshes/<link>.stl
  launch/display.launch.py
  launch/control.launch.py
  config/display.rviz
  config/controllers.yaml
  config/robot_data.yaml
```

The robot xacro is self-contained. Movable robots receive control launch and
controller configuration. Fixed-only robots do not need empty hardware systems.
`robot_data.yaml` contains topology, export metadata, warnings, and removed
loop-closing constraints. JSON formatting in these `.yaml` files is valid YAML.

URDF describes a tree. A closure sidecar records information but does not enforce
the original mechanism. A four-bar linkage reduced to a tree is not equivalent
closed-chain motion. Generalized joints and motion coupling must not be
advertised as preserved unless their conversion is implemented and tested.

## Configure the export

Copy `URDF_Exporter/xacro_export.template.toml` to `xacro_export.toml` beside the
entry script. Use TOML values with their actual types:

```toml
mesh_refinement = "low"
collision_method = "visual"
effort = 100.0
velocity = 100.0
root_occurrence = ""
```

| Option | Meaning |
| --- | --- |
| `mesh_refinement` | `low`, `medium`, or `high` Fusion tessellation. Higher detail increases mesh size. |
| `collision_method` | `visual` reuses detailed geometry. `primitive_box` uses a body bounding box when available and otherwise keeps mesh collision. |
| `effort` | Positive finite default joint effort, in newton-meters for rotation or newtons for translation. |
| `velocity` | Positive finite default velocity, in radians per second or meters per second for the joint type. |
| `root_occurrence` | Empty for automatic root selection, an exact source occurrence path, or `design_root` for direct root bodies. |

Unknown keys, malformed values, and nonfinite limits abort export. Old
`ros_version`, `use_legacy_copy`, and transmission/control switches are no
longer valid. There is no destructive copy mode to enable as a workaround.

## Load in ROS 2

Place the generated package under a workspace's `src` directory. In a shell
with the desired ROS distribution sourced, run from the workspace root.

The commands below use `my_robot_description` as the example package name.
Replace it with the actual directory/package name from your export. For Jazzy,
source `/opt/ros/jazzy/setup.bash` first. Use `/opt/ros/humble/setup.bash` for
Humble instead.

```bash
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install
	source install/setup.bash
	ros2 launch my_robot_description display.launch.py
```

The default display launch starts the joint-state GUI and RViz. For headless
loading, use:

```bash
	ros2 launch my_robot_description display.launch.py gui:=false rviz:=false
```

For a movable robot's mock hardware, use:

```bash
	ros2 launch my_robot_description control.launch.py
```

This starts the joint-state broadcaster and joint-trajectory controller.
Do not run a separate joint-state publisher against the same joints at the
same time. Mock hardware checks command/state propagation. It does not connect
to real hardware or simulate forces, collisions, or dynamics.

The verified Docker environment uses CycloneDDS. On a native ROS installation
with `rmw_cyclonedds_cpp` installed, select the same backend with
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The recorded Humble FastDDS
service-response failure remains separate from exporter behavior.

## Run the verification suites

The exporter has no third-party Python dependency. The offline suite needs
Python 3.11 or newer for TOML-file tests:

```bash
	python3 -m unittest discover -s tests -v
	python3 tools/verify_fusion_coverage.py --output artifacts/fusion-coverage
	bash tools/verify_docker.sh humble artifacts
	bash tools/verify_docker.sh jazzy artifacts
```

Docker commands require engine access but no sudo once access is configured.
Docker socket access grants root-equivalent capability. Do not make the socket
world-writable. Each test run keeps logs and JSON reports in a unique directory.

Install optional loader requirements into a separate environment. With Blender
and urdfdom available, run:

```bash
	python3 -m venv .venv
	.venv/bin/python -m pip install -r requirements-test.txt
	PYTHON="$PWD/.venv/bin/python" bash tools/verify_cross_tools.sh artifacts
```

The cross-tool suite checks real STL geometry, numeric FK, and negative cases.
Blender saves `inspection.blend` and `blender.json`. It is a geometry/placement
check, not a native complete URDF validator. Docker tests build/install generated
packages, activate controllers, send a trajectory, and compare feedback and TF.

Run [the native Fusion verifier](tools/verify_fusion/README.md) to test real
history modes, proxy mesh coordinates, and source-state preservation. A prepared
script is not an executed native pass. `--require-complete` on the coverage
runner must not turn offline success into full Fusion proof.

## Known input gaps

Full Fusion use-case coverage is not yet proven. These remain in-scope gaps,
not simulator or other-CAD exclusions:

- Motion-link conversion into affine URDF mimic relations is unimplemented.
- Mesh-only or mixed-mesh links lack a validated physical-property/export policy.
- Ordinary joints requiring bodyless assembly or reference-frame links are unimplemented.
- Active cylindrical, planar, pin-slot, and ball joints have no verified DOF decomposition.
- Combined STL output does not preserve separate body/face materials or textures.

Ground To Parent support is partial. Physical parents and explicitly fixed
ancestry work in the offline corpus. Unresolved bodyless/world frames and
unproven world anchors fail explicitly. A closed-loop sidecar records closure
information but does not enforce the original closed mechanism.

Native Fusion execution remains unverified. The prepared verifier's original
joint-reference invalidation case is explicitly `not_exercised`, even if its
other cases pass. See [the generated matrix](docs/fusion-coverage.md) for the
exact implementation and evidence status of each declared case.

## Troubleshoot an export

| Symptom | Action |
| --- | --- |
| Export stopped after a history edit. | Inspect the named joint's current frame and unresolved references. Keep history enabled. Do not roll/delete the timeline as an exporter workaround. |
| Rigid as-built joint has no geometry. | Null geometry is legitimate. Use the current exporter; it should derive the fixed pose from occurrences. Report a failure with the verifier's evidence. |
| Root is missing or ambiguous. | Name one chassis `base_link` or configure an exact `root_occurrence`. Distinguish repeated instances and grounded candidates. |
| Link is disconnected. | Add the intended mechanical joint/rigid constraint or exclude the geometry deliberately. Spatial proximity is not a fixed connection. |
| A hidden joint marker still exports. | Hiding a marker does not suppress a joint. Use suppression only when you intend to remove the constraint. |
| A selected mechanism depends on hidden geometry. | Make the required endpoint visible or change the assembly selection. Do not drop one side of an active constraint silently. |
| Grounding has no representable parent or world anchor. | Inspect the named fixed ancestry and root selection. Choose a proven grounded physical root where appropriate. Do not disable grounding merely to suppress the error. Missing bodyless frames remain an implementation gap. |
| Duplicate identity or missing mesh reference. | Inspect occurrence paths and names after sanitization. Component names alone do not identify repeated instances. |
| Invalid mass or inertia. | Check solid geometry, material density, units, and Fusion physical-property calculation. Mesh-only inputs need a separate physical-property policy. |
| Unsupported joint, motion link, or rigid conflict. | Read the named input and the use-case matrix. Do not reinterpret an unimplemented Fusion input as a successful free-moving mechanism. |
| Output directory exists. | Choose a fresh output parent/name. Preserve the previous valid package. |
| Scale or placement is wrong. | Check explicit STL units and compare a known dimension in Blender and Fusion. Do not fix scale by offsetting meshes manually. |
| ROS controller service fails. | Retain runtime logs and selected RMW. Repeat the documented Docker test; see the recorded CycloneDDS/FastDDS distinction. |
| Docker reports permission denied. | Fix engine access outside the agent or restart the login/session after group changes. Do not use a world-writable socket. |

See [the full testing guide](docs/testing.md) and
[model requirements](DESIGN_RULES.md) for additional checks.

## Maintain the implementation

`Joint.py` owns joint extraction. `Link.py` owns physical properties.
`utils.py` owns canonical source selection and STL export. `Write.py` owns the
pure self-contained package writer. UI and native verification share
`export_design`, while offline fixtures can call the writer without importing
Fusion. Verification dependencies stay in `tests` and `tools`.

The [project verification skill](.opencode/skills/verify-fusion360tourdf/SKILL.md)
records repeatable checks for later changes. Update the evidence and use-case
matrix when inputs or export semantics change, not just the test count.

`Example/Basic_Robot.f3d` is the upstream sample design. Its adjacent generated
package is historical ROS 1 output, not the maintained ROS 2 reference.
Re-export the design with this script for ROS 2.

## License and attribution

MIT. Original exporter by Toshinori Kitamura and contributors. See
[LICENSE](LICENSE), [CHANGELOG.md](CHANGELOG.md), and the
[source-cited related-work audit](docs/related-work.md). The audit identifies
use cases and tradeoffs. It does not establish that this converter is better
than every other exporter on every Fusion model.
