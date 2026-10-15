# Changelog

## Unreleased

### Overcome upstream limits (surveyed: syuntoku14, dheena2k2, Adriaeik,
### haoyan-dev, cadop, ACDC4Robot, runtimerobotics, david-dorf, newtonjeri)

- Joint/link orientations: full `xyz + rpy` origins (`R_parent^-1 * R_child`,
  axes in the parent frame) instead of hardcoded `rpy="0 0 0"`.
- Nested assemblies flatten to leaf links via `allOccurrences` global walk;
  hidden occurrences (light bulb off) select the export set.
- Non-destructive WYSIWYG STL export by default; legacy `copy_occs` behind
  `use_legacy_copy`.
- ROS1 + ROS2 output from one export (`ros_version = 1|2|both`), including
  `ros2_control` xacro, controllers config, and `launch/*.launch.py`.
- Collision choice: `visual` mesh or `primitive_box` from Fusion bounding
  boxes, per `collision_method`.
- Per-link Fusion appearance colors instead of global `silver` only.
- Fail-closed preflight (actionable errors, warnings before mesh export)
  instead of silent abnormal URDF.
- Closed-loop detection: tree stays valid, extra joints go to
  `config/robot_data.yaml` sidecar.
- Synthetic `base_link` from design-root bodies for imported CAD.
- Physics precision: no `round(x, 6)` truncation of inertia; scientific
  notation for tiny values.
- Configurable effort/velocity/mesh refinement via `xacro_export.toml`
  (stdlib `tomllib`, no new dependencies).
- Offline test suite: 22 tests covering transforms, loops, config,
  materials, writers, and preflight.
