# Walkthrough transcript

Illustrated instructions with an actual ROS/RViz motion segment. No audio narration.

## Fusion to ROS 2

A real six-link native CAD export.

Nested repeated parts, rigid mounts, a rotary joint, and a slider.

## 1. Install the exporter

From the repository root, copy the complete exporter to Fusion's Scripts folder.

macOS: ~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts

Windows: %APPDATA%/Autodesk/Autodesk Fusion 360/API/Scripts

The launchers require host Python 3.11+. Use --dry-run to inspect placement; --update retains backups and TOML settings.

```text
bash install.sh
.\install.cmd
```

## Optional: a single-file installer

Build a standard-library .pyz that bundles the exporter and its license notices.

Recipients need Python 3.11+, but do not need the source checkout.

The executable uses the same installer policy. It is not a native binary or a headless CAD converter.

```text
python3 tools/build_installer.py
python3 artifacts/fusion360tourdf-install.pyz --dry-run
```

## 2. Prepare the assembly

Use connected solid component occurrences; give moving links their own components.

Choose base_link or an exact root_occurrence. Check materials and finite joint limits.

Keep Capture Design History enabled. Hide geometry deliberately; hiding a joint marker does not suppress it.

## 3. Export inside Fusion

Utilities > Add-ins > Scripts and Add-ins > Scripts

Select URDF_Exporter and Run. If absent, Add the installed folder.

Choose a fresh output parent folder and leave the assembly unchanged.

The installer only places files; it does not run a headless CAD export.

## 4. Inspect the URDF catalog

The full robot retains joints and mock-control configuration.

Each component URDF is a one-link model with its original visual, collision, mass, and inertia.

Component files share the package's meshes; copy/install the full package.

Display assets live in viewers/; control files remain in launch/ and config/.

```text
urdf/rotary_slide.urdf
urdf/rotary_slide.xacro
urdf/components/base_link.urdf
urdf/components/mast_1.urdf
urdf/components/boom_1.urdf
urdf/components/boom_1_carriage_1.urdf
urdf/components/boom_1_carriage_1_sensor_1.urdf
urdf/components/boom_1_carriage_1_sensor_2.urdf
```

## Archive the package or viewers

A complete package ZIP includes URDF, component files, meshes, viewers, control configuration, and package metadata.

A viewer-only ZIP is not a standalone robot. Extract viewers/ into its matching description package.

Use copy installation with colcon build; symlink installation is not portable-mesh evidence.

```text
python3 -m zipfile -c rotary_slide_description.zip rotary_slide_description
python3 -m zipfile -c rotary_slide_viewers.zip rotary_slide_description/viewers
```

## 5. Build and open RViz

Copy the generated package to your ROS workspace's src folder.

Use Humble or Jazzy. Source your workspace after building.

Choose display OR mock control; do not run both joint-state publishers together.

The next segment is actual RViz footage with numbered presentation labels.

```text
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch rotary_slide_description display.launch.py
ros2 launch rotary_slide_description display.launch.py gui:=false rviz:=false
ros2 launch rotary_slide_description control.launch.py rviz:=true
```

## 6. Command mock joints

Actual ROS 2 Jazzy / RViz 2 capture.

A successful trajectory rotates 45 deg and extends 25 mm.

Joint-state feedback and all link transforms match the native expectations.

Mock hardware is not a physics simulation or physical hardware.

## Send the same goal yourself

Keep control.launch.py running, then use a second sourced ROS terminal.

From the repository root, send the supplied JSON-formatted YAML goal.

The two joint names and positions match the native demo and recorded controller checks.

```text
ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$(cat Example/rotary_slide_goal.yaml)"
```

## 7. Configure supported output

mesh_refinement: low, medium, or high

collision_method: visual meshes or primitive_box

effort / velocity: finite positive limits in the joint's SI units

root_occurrence: automatic, an exact instance path, or design_root

Rigid/revolute/continuous/slider joints, locked joints, fixed groups, and representable grounding are supported.

Edit the TOML beside the installed entry script; a copied installation does not read an unrelated checkout config.

```text
Scripts/URDF_Exporter/xacro_export.toml
```

## 8. Verify and contribute

Run the real native demo and verify its actual package, not a substitute fixture.

The verifier checks geometry, negative controls, controllers, feedback, and transforms.

Contributor setup and test recipes live in CONTRIBUTING.md.

```text
python3 -m unittest discover -s tests -v
python3 tools/verify_export.py /path/to/native-bundle --output artifacts
```

## Know the limits

URDF is a tree. A closure sidecar records removed constraints but does not enforce closed-chain motion.

Motion-link/mimic conversion, mesh-only physical policy, generalized joints, and general bodyless frames remain gaps.

Combined STL output does not preserve per-face textures/materials.

Keep the source history and retain reports; a picture is not universal CAD or physics proof.

## Community work and this integration

This is an integration fork of syuntoku14/fusion2urdf, not a from-scratch exporter.

Frame, validation, visibility, and precision approaches credit haoyan-dev, Adriaeik, newtonjeri, and ACDC4Robot.

Other pinned repositories informed comparison and regression cases, not necessarily copied code.

changelog.md lists inherited work, adapted ideas, comparison-only references, and the fixes/tooling added here.

LICENSES.md links the full pinned license texts; installed copies retain THIRD_PARTY_NOTICES.md and licenses/.
