# Native rotary-slide example

`rotary_slide_description/` is a ready-to-build package exported from the native Fusion demo, not a separate fixture generator. It includes the complete assembly, six one-link URDFs, shared meshes, and the merged `viewers/` layout.

Copy the whole `rotary_slide_description` directory to your ROS workspace’s `src` folder. Source ROS 2 Humble or Jazzy, build with `colcon build`, source the install space, and run:

```bash
ros2 launch rotary_slide_description display.launch.py
```

For mock control instead of the joint-state GUI, stop the display launch and run:

```bash
ros2 launch rotary_slide_description control.launch.py rviz:=true
```

With mock control running, use a second sourced ROS terminal. From this repository’s root, send the supplied JSON-formatted YAML goal:

```bash
ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$(cat Example/rotary_slide_goal.yaml)"
```

The goal rotates `turn` by pi/4 radians and extends `boom_1_extend` by 0.025 m over three seconds. It matches the trajectory used by the verified RViz capture. Do not run the display launch’s separate joint-state publisher at the same time.

## Available URDF files

The assembly file retains joints and control data. Each component file contains exactly one link, with the same visual, collision, mass, inertia, and local frame as the assembly. Component files share `package://rotary_slide_description/meshes/` resources; a single XML download without the package meshes is incomplete.

| Label | Link | URDF |
| --- | --- | --- |
| Complete robot | Six links and two moving joints | [rotary_slide.urdf](rotary_slide_description/urdf/rotary_slide.urdf) |
| 1 | `base_link` | [Base](rotary_slide_description/urdf/components/base_link.urdf) |
| 2 | `mast_1` | [Mast](rotary_slide_description/urdf/components/mast_1.urdf) |
| 3 | `boom_1` | [Rail](rotary_slide_description/urdf/components/boom_1.urdf) |
| 4 | `boom_1_carriage_1` | [Carriage](rotary_slide_description/urdf/components/boom_1_carriage_1.urdf) |
| 5 | `boom_1_carriage_1_sensor_1` | [Sensor 1](rotary_slide_description/urdf/components/boom_1_carriage_1_sensor_1.urdf) |
| 6 | `boom_1_carriage_1_sensor_2` | [Sensor 2](rotary_slide_description/urdf/components/boom_1_carriage_1_sensor_2.urdf) |

The numbered RViz labels are a presentation overlay from `tools/capture_rviz.py`, not extra robot geometry. Fixed sensor link frames use the carriage’s rigid anchor; the two visual/collision offsets still place their bodies separately. These files are not a hardware driver, physics simulator, or per-sensor calibration model.

The complete package is installable on its own. A ZIP containing only `viewers/` still needs this package’s URDF, meshes, and metadata. See [the native demo workflow](../docs/demo-workflow.md) and [showcase instructions](../docs/showcase.md) to regenerate and verify it.
