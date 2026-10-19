# fusion360tourdf

Export Autodesk Fusion assemblies as ROS 2 robot description packages, including STL meshes, joints, mass, and inertia. The exporter runs inside Fusion without third-party Python dependencies and preserves the source design’s editable history.

Generated packages target **ROS 2 Humble and Jazzy**. Launch them in RViz or use mock ros2_control hardware to command joints and inspect their feedback.

[Install and export](#install-and-export) · [Use in ROS 2](#use-the-package-in-ros-2) · [Reproduce the demo](#reproduce-the-verified-demo) · [Configuration](#configure-the-export) · [Support and limits](#supported-inputs-and-limits)

## Example: Fusion to URDF

The six-link rotary-slide demo includes a grounded mast, a revolute joint, a nested slider, and two instances of one sensor component.

| Native Fusion assembly | Exported URDF: zero pose | Exported URDF: moved pose |
| --- | --- | --- |
| ![Rotary-slide robot in the native Fusion viewport](docs/images/rotary-slide-fusion.png) | ![Exported rotary-slide meshes rendered in Blender at joint zero](docs/images/rotary-slide-zero.png) | ![Exported rotary-slide meshes after 45 degrees of rotation and 25 mm of extension](docs/images/rotary-slide-moved.png) |

The Blender pictures use the actual exported meshes with neutral preview shading. The moved pose rotates 45 degrees and extends 25 mm; it does not simulate physics. [Reproduce the model and pictures](docs/demo-workflow.md).

## What you get

Each export creates a self-contained ROS 2 description package:

```text
your_robot_description/
├── package.xml
├── CMakeLists.txt
├── urdf/your_robot.xacro
├── meshes/*.stl
├── launch/display.launch.py
├── launch/control.launch.py
└── config/
    ├── display.rviz
    ├── controllers.yaml
    └── robot_data.yaml
```

Movable robots include the control launch file and controller configuration. Fixed-only robots omit those assets. `robot_data.yaml` records the topology, warnings, and any removed loop-closing constraints; its JSON formatting is valid YAML.

## Requirements

Install the tools needed for your workflow:

| Task | Requirements |
| --- | --- |
| Export from CAD | Autodesk Fusion May 2025 or newer on macOS or Windows; Python 3.11+ for TOML configuration |
| Load the package | ROS 2 Humble or Jazzy on your ROS machine |
| Run offline tests | Python 3.11+ |
| Reproduce end-to-end checks | Fusion, Python test requirements, Blender, and a running Docker Engine with free disk space |

ROS and Blender are not exporter dependencies. Fusion and ROS can run on different computers. Native verification used Fusion 2705.1.30; the README renders used Blender 5.2.2 LTS.

## Install and export

Add the complete exporter directory to Fusion, prepare a connected assembly, then choose a fresh output destination.

1. Clone or download this repository.
2. In Fusion, open **Utilities > Add-ins > Scripts and Add-ins > Scripts**.
3. Click **Add** and select the checkout’s `URDF_Exporter` directory. Keep its `core`, `utils`, manifest, and license files together.
4. Prepare your model using the checklist below, then run **URDF_Exporter**.
5. Choose an output parent folder and inspect the resulting `your_robot_description` directory.

You can also copy the complete `URDF_Exporter` directory into Fusion’s scripts folder:

| Platform | Scripts folder |
| --- | --- |
| macOS | `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts` |
| Windows | `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts` |

Before replacing an installation, preserve your `xacro_export.toml`. Save your work and restart Fusion after updating Python modules to avoid running cached code.

### Prepare the model

Check these conditions before exporting:

- Make each independently moving link a component occurrence; a link can contain multiple solid bodies.
- Name the chassis `base_link`, or configure an exact `root_occurrence` path.
- Connect every selected link with rigid, revolute, or slider joints, fixed groups, or supported grounding relationships.
- Set both bounds for a bounded joint. Sliders require finite lower and upper travel limits.
- Assign physical materials and check mass properties in Fusion.
- Use visibility to select geometry, but suppress a joint only when you intend to remove its constraint.

**Keep Capture Design History enabled.** Export reads the active design’s root assembly, not only the activated component. It must not reset joints, rename components, move bodies, or remove history. Leave the assembly unchanged while export runs.

Existing output is never overwritten. The exporter stages files and publishes the package only after mesh validation succeeds; canceling the folder dialog creates no package. The first word of the design/root name determines the package name: `My Robot` produces `my_description`.

See the [model requirements](DESIGN_RULES.md) and [assembly/export guide](docs/exporting.md) for nested components, root selection, frames, visibility, and history-related errors.

## Use the package in ROS 2

Copy the generated description package into your ROS workspace’s `src` directory. In a sourced ROS environment, install dependencies, build the workspace, and launch the display:

```bash
source /opt/ros/jazzy/setup.bash
cd your_ros2_workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch your_robot_description display.launch.py
```

For Humble, source `/opt/ros/humble/setup.bash` instead. Replace `your_robot_description` with the generated package name.

The display launch starts RViz and the joint-state GUI. To run without either GUI:

```bash
ros2 launch your_robot_description display.launch.py gui:=false rviz:=false
```

For a movable robot, start the mock joint-state broadcaster and trajectory controller:

```bash
ros2 launch your_robot_description control.launch.py
```

Do not run a separate joint-state publisher against the same joints during mock control. Mock hardware tests commands and state propagation, not forces, collisions, dynamics, or physical hardware. The Docker tests use CycloneDDS; generated packages do not select a middleware implementation.

## Configure the export

Copy `URDF_Exporter/xacro_export.template.toml` to `xacro_export.toml` beside the entry script, then set only the options you need:

```toml
mesh_refinement = "low"
collision_method = "visual"
effort = 100.0
velocity = 100.0
root_occurrence = ""
```

| Option | Values or meaning |
| --- | --- |
| `mesh_refinement` | `low`, `medium`, or `high` STL tessellation |
| `collision_method` | `visual` mesh collision, or `primitive_box` where bounds are available |
| `effort` | Positive finite limit in N·m for rotation or N for translation |
| `velocity` | Positive finite limit in rad/s or m/s, depending on joint type |
| `root_occurrence` | Automatic selection when empty; otherwise an exact occurrence path or `design_root` for root-owned bodies |

Unknown options and invalid values stop export. Obsolete ROS 1, legacy-copy, and transmission/control switches are not supported. See the [configuration and frame details](docs/exporting.md#configure-the-exporter).

## Reproduce the verified demo

Run the Fusion script in `tools/create_demo` to create the assembly pictured above. It exports a native `.f3d` archive, a ROS package, independent pose expectations, a source-preservation receipt, and a Fusion viewport PNG.

From the checkout root, install the separate verification requirements and test that actual native export:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-test.txt
.venv/bin/python tools/verify_export.py /path/to/rotary-slide-bundle \
  --output artifacts
```

The runner checks mesh loading and forward kinematics, renders zero/moved previews in Blender, then builds and commands the supplied package in Docker on Humble and Jazzy. It does not generate a substitute robot fixture.

Follow the [demo workflow](docs/demo-workflow.md) for Fusion setup, commands, evidence files, and [README picture refresh instructions](docs/demo-workflow.md#refresh-the-readme-pictures).

## Verification

Local checks on October 18, 2026 covered the following cases:

| Check | Observed result |
| --- | --- |
| Offline regressions | 194 tests passed |
| Native Fusion fixtures | Eleven parametric/direct cases passed |
| Native six-link demo | 449 CAD/export assertions passed |
| Independent loader and Blender | Real exported meshes, zero/moved placement, and rejected wrong-axis/shifted/missing-mesh controls |
| ROS 2 Humble and Jazzy | Installed native assets matched hashes; controllers activated; trajectory, joint feedback, and six-link transforms passed |

See the [verification results](docs/verification-results.md) for versions, source hashes, local receipts, and coverage limits. These results cover the exercised inputs, not every Fusion assembly or a comparison run against other exporters.

To run the offline and generated-package suites from the checkout root:

```bash
python3 -m unittest discover -s tests -v
python3 tools/verify_fusion_coverage.py --output artifacts/fusion-coverage
bash tools/verify_docker.sh humble artifacts
bash tools/verify_docker.sh jazzy artifacts
```

The [testing guide](docs/testing.md) covers cross-tool loading, native fixtures, runtime checks, and cleanup. A passing offline suite is not native Fusion proof; `--require-complete` remains nonzero when full input coverage is unproven.

## Supported inputs and limits

Use BRep solid assemblies with physical links and a connected mechanism. The maintained inputs include ordinary/as-built rigid, revolute, continuous, and slider joints; nested/repeated occurrences; root-owned bodies; and locked joints fixed at their current pose.

Rigid groups and representable grounding relationships become fixed connections. Ground To Parent support is partial: unresolved bodyless/world frames fail rather than receiving guessed attachments.

These input and fidelity gaps remain:

- Motion-link conversion into affine mimic relations
- Mesh-only input with a validated mass/inertia and export policy
- General bodyless assembly or reference-frame endpoints
- Active cylindrical, planar, pin-slot, and ball joint decomposition
- Distinct materials or textures per body or face in combined STL output

URDF describes a tree. A loop-closure sidecar records removed constraints but does not enforce closed-chain motion. ROS 1, SDF, MJCF, simulator backends, and other CAD systems are outside the maintained output scope.

Consult the [input-support matrix](docs/fusion-coverage.md) for case-level status and the [pinned repository audit](docs/related-work.md) for the source comparisons behind the test cases.

## Documentation

Choose the guide for your next task:

- [Model requirements](DESIGN_RULES.md): assembly, frames, and output invariants
- [Assembly and export guide](docs/exporting.md): root selection, physical properties, configuration, and troubleshooting
- [Native demo workflow](docs/demo-workflow.md): create, verify, and photograph a real Fusion model
- [Testing guide](docs/testing.md): offline, loader, Blender, Docker, and native recipes
- [Verification results](docs/verification-results.md): executed evidence and remaining limits

`Example/Basic_Robot.f3d` is the inherited sample. Its adjacent ROS 1 package is historical output; re-export the design for ROS 2.

## License and attribution

MIT. This project is a fork of [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf), originally authored by Toshinori Kitamura and contributors. See [LICENSE](LICENSE) and [CHANGELOG.md](CHANGELOG.md).
