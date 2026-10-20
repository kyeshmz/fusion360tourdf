# Verification overview

This page collects the native demo gallery, dated verification summary, and reproduction entry points.
The [README](../README.md) covers installation and use. The [execution ledger](verification-results.md) retains source hashes, receipts, and the scope of each run.
Historical counts below describe their recorded snapshots, not a new run of the current checkout.

## Native demo gallery

The six-link rotary-slide demo includes a grounded mast, a revolute joint, a nested slider, and two instances of one sensor component.

| Native Fusion assembly | Exported URDF at zero pose | Exported URDF at moved pose |
| --- | --- | --- |
| ![Rotary-slide robot in the native Fusion viewport](images/rotary-slide-fusion.png) | ![Exported rotary-slide meshes rendered in Blender at joint zero](images/rotary-slide-zero.png) | ![Exported rotary-slide meshes after 45 degrees of rotation and 25 mm of extension](images/rotary-slide-moved.png) |

The Blender pictures use the actual exported meshes with neutral preview shading in Blender **5.2.2 LTS**.
The moved pose rotates 45 degrees and extends 25 mm. It does not simulate physics.
The [image receipt](images/images.json) retains dimensions, image hashes, and native-source provenance.
Follow the [demo workflow](demo-workflow.md) to recreate the model and pictures.

## Real ROS views

| Real ROS 2 Jazzy / RViz 2 at zero pose | Real ROS 2 Jazzy / RViz 2 at commanded pose |
| --- | --- |
| ![Real RViz capture with numbered URDF component labels at joint zero](media/ros-jazzy-zero.png) | ![Real RViz capture after verified 45-degree rotation and 25 mm slider extension](media/ros-jazzy-moved.png) |

These views use the installed native package and verified mock-controller feedback, not Blender previews.
Numbered text/leader lines are presentation markers attached to the corresponding URDF frames.
The filename legends match the [component catalog](../Example/rotary_slide_README.md#available-urdf-files).
This is mock hardware, not physical-hardware or dynamics evidence.

The [media index](media/README.md) retains recordings, transcripts, source captures, and hash receipts.
The [showcase guide](showcase.md) explains how to capture and rebuild the labeled media from a fresh native export.

## Requirements and tested environments

Host Python, Fusion's embedded Python, and Python supplied by ROS are different runtimes.
Installing a newer host Python does not upgrade Fusion.

| Workflow | Required / supported | Evidence and boundaries |
| --- | --- | --- |
| Install, build the single-file CLI, run offline tests | Host CPython **3.11+** on `PATH` | CI matrix: **3.11, 3.12, 3.13, 3.14**; configured coverage is not a claim that each hosted job has passed |
| Export CAD | Autodesk Fusion **May 2025+**, macOS or Windows; embedded Python **3.11+** for TOML | Native checks: Fusion **2705.1.30**, embedded Python **3.14.0**, on macOS; not every Fusion build is verified |
| Display and command the exported package | ROS 2 **Humble / Ubuntu 22.04** or **Jazzy / Ubuntu 24.04** | Installed-package and mock-controller checks on both; ROS uses its own Python (Humble's Python 3.10 is not the host-tool requirement) |
| Reproduce full verification | Fusion, optional [test requirements](../requirements-test.txt), Blender, and Docker Engine | External tools, not exporter dependencies; see [CONTRIBUTING.md](../CONTRIBUTING.md) |
| Rebuild movie / labeled ROS pictures | Optional [media requirement](../requirements-media.txt), FFmpeg/ffprobe, GUI-enabled Docker image | See [showcase instructions](showcase.md) |

Fusion is not natively supported on Linux. Linux can consume the ROS packages and run verification.
Custom installer paths do not certify Fusion under Wine.
Windows launcher tests are configured in CI. Local macOS results are not native Windows evidence.
ROS and Blender are not required to export. Fusion and ROS can run on different computers.

## Reproduce the verified demo

Run the Fusion script in `tools/create_demo` to create the assembly pictured above.
It exports a native `.f3d` archive, a ROS package, independent pose expectations, a source-preservation receipt, and a Fusion viewport PNG.

From the checkout root, install the separate verification requirements and test that actual native export:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-test.txt
.venv/bin/python tools/verify_export.py /path/to/rotary-slide-bundle \
  --output artifacts
```

The runner checks mesh loading and forward kinematics, renders zero/moved previews in Blender, then builds and commands the supplied package in Docker on Humble and Jazzy.
It does not generate a substitute robot fixture.
Follow the [demo workflow](demo-workflow.md) for Fusion setup, commands, evidence files, and [picture refresh instructions](demo-workflow.md#refresh-the-readme-pictures).
Use the [testing guide](testing.md) for separate offline, loader, Blender, Docker, and native procedures.
See [CONTRIBUTING.md](../CONTRIBUTING.md) for development setup and review requirements.

## Verification

On October 19, 2026, the macOS integration run pulled `7bf5a13` and verified the combined component-URDF and `viewers/` changes.
The labeled RViz captures and movie use that fresh native export:

| Check | Observed result |
| --- | --- |
| Offline regressions | 275 tests on CPython 3.11.11 and 3.13.2: 262 passed, 13 platform skips per run |
| Offline in Docker Jazzy / Python 3.12 | Same 275 tests: 263 passed, 12 platform skips |
| Single-file installer | Install, dry-run, safe update/backup, private-config exclusion, license inclusion, and POSIX execution passed |
| Native six-link demo | 449 CAD/export assertions passed |
| Component URDFs | All six loaded independently with their shared visual/collision meshes |
| Independent loader and Blender | Seven integration tests and three Blender variants passed; wrong-axis/shifted/missing-mesh controls rejected |
| ROS 2 Humble and Jazzy | Installed native assets matched hashes; controllers activated; trajectory, joint feedback, and six-link transforms passed |

An earlier eleven-case native fixture run covers its own source snapshot, not the entire merged tree.
Native Windows execution and hosted Python 3.12/3.14 jobs require their own CI evidence.
See [verification results](verification-results.md) for source hashes, receipts, and limits.
These checks do not establish universal CAD coverage, real hardware, or a head-to-head comparison with other exporters.

## Evidence limits and methods

The native helper checks disjoint analytic boxes, not arbitrary CAD geometry.
Original-reference invalidation remains `not_exercised`. Native joints are not actuated at nonzero values.
The nonzero motion checks articulate the exported URDF and ROS mock hardware, not Fusion's solver.
The eleven-case fixture receipts and demo do not certify the complete [input-support matrix](fusion-coverage.md).
Broader edited-topology, grounding, and appearance cases still need separate native inputs.

Use the normal copy install, `colcon build`, for a self-contained ROS install space.
`--symlink-install` leaves meshes dependent on the source workspace.
Strict portable-mesh checks, including the native-package old-root guard, reject external source-workspace targets.
The symlink-install probes did not reach controller verification and are not passing full-runtime evidence.

Mock hardware checks commands, joint feedback, and transforms, not forces, collisions, dynamics, or physical hardware.
The Docker tests use CycloneDDS. Generated packages do not select a middleware implementation.
The [testing guide's interpretation table](testing.md#interpret-the-results) separates parsing, geometry, kinematics, and ROS behavior.
The [pinned repository audit](related-work.md) is a source comparison, not a native-runtime ranking.

## Archive exported files

The display launcher and RViz configuration live together in `viewers/`.
Control files stay under `launch/` and `config/`. Fixed-only robots omit the control launch file and controller configuration.
The exporter does not copy Blender or validation helpers into the robot package.
`config/robot_data.yaml` records topology, warnings, and any removed loop-closing constraints. Its JSON formatting is valid YAML.

This layout applies to new exports. Re-export an older model into a fresh directory.
Moving old files alone does not update their launch and CMake paths.

From the export's parent directory, create either archive with Python's standard ZIP command:

```bash
python3 -m zipfile -c my_robot_description.zip my_robot_description
python3 -m zipfile -c my_robot_viewers.zip my_robot_description/viewers
```

The first archive contains the complete buildable ROS package.
The second contains only `viewers/` and requires the matching package's URDF, meshes, and metadata.
Extract the viewer folder into that package. Keep the full directory layout when building the complete package.
The launch command remains `ros2 launch my_robot_description display.launch.py`.
