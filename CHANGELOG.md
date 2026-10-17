# Changelog

## Unreleased

### ROS 2-only package generation and verification

- Replace separate ROS 1 and ROS 2 writer sequences with one complete-package
  operation. Remove ROS 1 and mismatched Gazebo generation.
- Nest collision geometry in its owning link. Keep materials and mock
  ros2_control in the self-contained robot description.
- Verify exported models through independent loaders, headless Blender,
  and ROS 2 install-space and controller checks. Keep validation dependencies
  outside the Fusion exporter.
- Preserve existing output and reject incomplete mesh exports instead of
  announcing success after a partial write.
- Read nested joint proxies, preserve repeated-instance identities, and rebase
  travel limits relative to the exported assembly pose.
- Add repeatable Docker Humble and Jazzy suites, cross-tool checks, and a
  project-local verification skill. Record executed results and remaining
  live-Fusion limits in `docs/verification-results.md`.
- Add exact root selection, outward joint reversal, mixed direct-body
  aggregation, and an unfiltered context registry that preserves hidden
  repeated-instance identity.
- Preserve supported fixed Ground To Parent/world relationships without
  changing source flags. Reject unresolved bodyless/world ancestry explicitly.
- Read modern and legacy native appearance properties with byte RGB/opacity
  semantics. Respect explicit overrides and selected-body appearance before
  material fallback.
- Add a prepared native Fusion verifier for owned parametric/direct documents,
  editable-history preservation and code provenance. Native execution remains
  unverified; source inspection and offline tests do not prove universal parity.
- Publish the pinned nine-repository Fusion-only census and executable
  use-case catalog. Keep unsupported inputs distinct from passing rejection
  checks.

### Earlier changes

The entries below describe the inherited implementation. The ROS 2-only
changes above supersede its dual-mode and destructive-copy options.

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
