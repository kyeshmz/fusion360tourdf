# Verify a robot description

Use separate checks for XML structure, geometry, kinematics, and ROS behavior.
The Fusion exporter has no third-party Python dependencies. Install validation
tools in a separate environment or use the ROS container tests.

## Run the Docker suites

With a working Docker Engine and socket access, run these commands from the
repository root. Neither command requires sudo:

```bash
	bash tools/verify_docker.sh jazzy artifacts
	bash tools/verify_docker.sh humble artifacts
```

Each command builds `tools/Dockerfile.ros` for the selected ROS distribution.
It generates three fixture packages, builds and installs them with colcon, and
checks installed resources and headless display launches. The articulated
fixture also requires active controllers, a successful trajectory action,
matching joint feedback, and matching TF matrices.

Image construction needs network access for packages. The verification
container runs without networking beyond its loopback interface. It runs as
your user, so the mounted output is not root-owned. Each run uses its own
image tag and container name and removes them on exit. Build logs, expanded
URDFs, installed packages, and `ros.json` remain under the output directory.

The test image explicitly selects CycloneDDS. Humble's FastDDS backend aborted
while replying to a controller service in the tested environment. UDP-only
FastDDS passed once but failed in a fresh run, so that configuration was not
accepted. CycloneDDS passed fresh Humble and Jazzy runs without changes to the
exporter or assertions. The lower-level FastDDS failure remains unresolved.

Generated packages do not select a middleware implementation. `ros.json`
records the selected RMW and Python version so runtime results have an explicit
environment. These checks cover mock articulation, not GUI rendering or
physics.

On a native ROS installation with `rmw_cyclonedds_cpp` installed, select the
same tested backend before launching:

```bash
	export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The [verification workflow](../.github/workflows/verify.yml) runs both Docker
distributions, offline tests, and the cross-tool suite on pushes and pull
requests. A workflow definition is not evidence that its jobs have passed.

## Run offline regressions

From the repository root, run:

```bash
	python3 -m unittest discover -s tests -v
```

The complete offline suite requires Python 3.11 or newer for TOML-file tests.
Programmatic fixture generation also supports Humble's Python 3.10 without
adding a TOML dependency to the exporter.

The installer and viewer archive checks run without Fusion or ROS:

```bash
	python3 -m unittest tests.test_install_fusion tests.test_install_launchers tests.test_viewer_archive -v
```

Installer tests use temporary Scripts folders. They do not replace a real
Autodesk installation. The launcher tests invoke real Bash on POSIX and real
`cmd.exe` on native Windows. They copy a checkout into a temporary path with
spaces, Unicode, and shell punctuation. They exercise caller-relative targets,
dry runs, installed bytes, configuration and backup preservation, and backend
failure exit codes. Interpreter checks cover Python 3.11+ selection and disabled
automatic runtime downloads. Bash PATH stand-ins also check argument boundaries
and one backend dispatch after probing. On Windows, the missing-interpreter
test can skip if a runtime remains discoverable outside its isolated PATH.

On POSIX, also check Bash syntax:

```bash
	bash -n install.sh
```

Native CMD tests skip on Linux and macOS. Bash execution tests skip on Windows.
Linux source inspection does not verify Windows batch behavior. Backend Windows
and macOS default-path unit tests use supplied platform/environment values, not
those operating systems. The workflow defines installer jobs for both native
platforms and runs the backend and launcher suites in each job. A job definition
is not a completed hosted run. Native Fusion GUI behavior remains a separate
unverified gate until exercised in Fusion.

These tests use deterministic Fusion doubles and generated packages. They do
not open Autodesk Fusion or establish that its STL exporter uses the expected
coordinate frame.

Run the declared Fusion input cases and retain a machine-readable report:

```bash
	python3 tools/verify_fusion_coverage.py --output artifacts/fusion-coverage
```

The [coverage catalog](fusion-use-cases.json) names the exact regression methods.
[The generated matrix](fusion-coverage.md) distinguishes input support from
explicit rejection and unimplemented cases. Skipped or missing tests are not
successful evidence. The report always labels its evidence as offline API
doubles, not native Fusion.

`--require-complete` returns a nonzero result rather than converting successful
offline checks into complete Fusion-use-case proof. Run the native verifier
separately. Regenerate the human-readable catalog after changing its cases:

```bash
	python3 tools/verify_fusion_coverage.py --output artifacts/fusion-coverage \
	  --write-markdown docs/fusion-coverage.md
```

For a structural check of an exported file, run:

```bash
	python3 tools/validate_urdf.py path/to/robot.xacro --meshes path/to/package/meshes --strict
```

The linter rejects invalid link topology, misplaced collisions, and nonfinite
physical values. It is not a replacement for xacro expansion or urdfdom.
Unresolved xacro includes produce warnings. Strict mode fails on warnings.

## Check an installed package with ROS 2

Build the generated description in a sourced ROS 2 workspace. Source its
install space before resolving the package:

```bash
	colcon build
	source install/setup.bash
	share="$(ros2 pkg prefix --share <robot>_description)"
	xacro "$share/urdf/<robot>.xacro" > robot.urdf
	check_urdf robot.urdf
	ros2 launch <robot>_description display.launch.py gui:=false rviz:=false
```

`check_urdf` establishes that urdfdom can parse the model. It does not establish
that meshes load or that joints move correctly. Verify mesh resources from the
installed package rather than accidentally using source-tree files.

Use a normal copy install for this strict portable-package check. With
`--symlink-install`, resolved meshes can point into the source workspace and
fail the package-contained resource guard. A failure at that guard is not
evidence that ROS cannot launch the package, but it leaves that install mode's
full runtime behavior unverified here.

The installed display assets are `viewers/display.launch.py` and
`viewers/display.rviz`. ROS package lookup still uses the launch filename,
without a `viewers/` prefix. The Docker suites require that lookup to resolve
the installed viewer path and reject legacy duplicate viewer files.
`launch/control.launch.py` and `config/controllers.yaml` keep their paths.

For a movable robot, `control.launch.py` starts mock ros2_control. Its
joint-state broadcaster and trajectory controller must become active. A
behavioral check must send a trajectory and observe the resulting joint states
and TF transforms. A process that merely remains alive is insufficient.

## Check geometry with Blender and another loader

With Blender and `check_urdf` available on your path, install the optional
Python requirements and run the complete cross-tool suite:

```bash
	python3 -m venv .venv
	.venv/bin/python -m pip install -r requirements-test.txt
	PYTHON="$PWD/.venv/bin/python" bash tools/verify_cross_tools.sh artifacts
```

Use `BLENDER` and `CHECK_URDF` to select alternate executable paths. The suite
compares articulated, fixed-only, and primitive-collision packages with
literal expected poses and bounds. Negative tests alter an axis, shift a
valid mesh, and remove a mesh. They must fail validation.

Blender does not provide a built-in complete URDF validator. The headless check
imports STL meshes and applies the visual origins, scales, and joint poses from
the expanded URDF. It checks geometry and placement and can save a `.blend`
file for inspection. It does not test inertia, contacts, controller activation,
or simulation dynamics.

An independent URDF loader provides a separate kinematics check. Use real mesh
loading and compare zero and nonzero joint positions against fixture
expectations. Merely parsing XML with a second library is not independent proof
of geometry or motion.

Run Blender with `--background --factory-startup --python-exit-code 1`.
Without `--python-exit-code`, a Python assertion can fail while Blender returns
a successful process exit code.

To inspect an expanded export rather than the fixtures, run:

```bash
	.venv/bin/python tools/verify_loaders.py \
	  --urdf robot.urdf --package path/to/robot_description --output artifacts/export
	blender --background --factory-startup --python-exit-code 1 \
	  --python tools/verify_blender.py -- \
	  --urdf robot.urdf --package path/to/robot_description \
	  --positions '{"joint_name": 0.5}' --output artifacts/blender
```

The Blender result includes `inspection.blend` and `blender.json`. Its pose
collections preserve the geometry and transforms for inspection. For a real
export without expected-value tables, these tools check loading and internal
consistency. You must still compare the result against the Fusion design.

## Verify a real Fusion export

Use [the native Fusion verifier](../tools/verify_fusion/README.md) for isolated
owned-document fixtures in parametric and direct modes. It is a real application
entry point, not a mocked test. Until an actual run produces its report, its
native cases remain unverified.

For an additional manual assembly check:

1. Make an asymmetric assembly with a rotated link, a slider, and two
   occurrences of one component. Keep a known dimension for the scale check.
2. Export through Fusion into a fresh destination.
3. Confirm that every URDF mesh URI resolves to its intended occurrence file.
4. Load the expanded description in Blender and the independent URDF loader.
5. Compare dimensions and placements with the Fusion design at zero position.
6. Build and launch the package in ROS 2. Command nonzero joint positions and
   compare the observed joint states and TF transforms.

For synthetic `base_link`, repeat with root-owned geometry. Confirm that its
mesh does not contain child geometry a second time. Mock-export tests cannot
prove how the real Fusion API handles this case.

## Interpret the results

| Check | What passing establishes | What it does not establish |
| --- | --- | --- |
| Offline regressions | The exercised conversion rules and failure cases match their assertions. | Real Fusion API or ROS compatibility. |
| xacro and urdfdom | Expansion and URDF tree parsing succeed. | Mesh loading, joint motion, or physics. |
| Independent URDF loader | The exercised mesh resources and kinematic poses load correctly. | ROS launch or controllers. |
| Blender | The exercised geometry, meter scale, and placements agree with the expected scene. | Full URDF semantics or physics. |
| ROS install-space tests | Package discovery, launch, controller activation, and exercised joint-state and TF behavior work. | Real hardware or simulation dynamics. |
| Docker execution | The selected image can build and run the tests through Docker. | Behavior on the Fusion host. |

A missing runtime or an unexecuted CI job is not a passing check. The provided
ROS integration targets Humble and Jazzy. Gazebo and ROS 1 are outside this
verification scope.
