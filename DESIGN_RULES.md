# Model requirements

## Assembly and topology

- One visible component occurrence represents one link. Repeated instances
  need distinct occurrence identities and distinct posed mesh files.
- Name the chassis `base_link` or select an exact `root_occurrence`. Root-owned
  bodies can supply a synthetic base without exporting descendants twice.
- The exporter orients edges outward from the chosen root. Component1 and
  Component2 ordering need not be rebuilt just to reverse an edge.
- Active rigid, revolute, and slider joints are supported. A locked joint,
  including a generalized joint, is fixed at its current pose with provenance.
  Other active joint types fail by name until their motion is implemented.
- Every exported link must be reachable from `base_link`. Disconnected
  geometry must not disappear silently.
- Ground To Parent is a fixed constraint, not just a root-selection hint.
  Physical parents and proven fixed ancestry are handled. Missing bodyless
  parent/world frames remain explicit input gaps. Export does not clear flags
  or weld geometry to an unproven world anchor.
- URDF represents a tree. Additional loop-closing joints belong in
  `config/robot_data.yaml`, not in the URDF tree. The sidecar does not enforce
  physical constraints.
- The same selected direct bodies supply meshes and physical properties.
  Hiding a required constraint endpoint produces a named error rather than
  silently changing the mechanism. Hiding only a joint marker does not suppress
  that joint. Full context identity is retained even for excluded occurrences.

## Frames and physics

- Use Z-up and X-forward. Meshes use their design pose in the root frame.
- URDF visual origins undo the link pose. Do not pre-offset STL meshes.
- URDF joint axes are expressed in the joint frame, not the parent frame.
- Local centers of mass and inertia tensors must describe the same link
  frame as the joint tree. Rotation matters as well as translation.
- STL coordinates use millimeters. URDF mesh scale converts them to meters.
- Set both limits for a bounded revolute or slider joint. An unbounded
  revolute joint is continuous. A slider needs finite travel bounds.
- The current assembly pose becomes URDF joint zero. Bounded limits subtract
  the current driven position so the physical travel range stays unchanged.

## Output and verification

- Choose a fresh package destination. Existing output is not overwritten.
- Mesh export errors must abort publication. A successful XML write is not
  a successful robot export.
- ROS 2 is the only generated package format. Obsolete ROS 1 and destructive
  copy settings are not part of the supported configuration.
- Mock ros2_control provides articulated command and state tests, not
  simulation physics or physical hardware control.
- Real Fusion export remains a separate check from deterministic fixture
  tests. Confirm an asymmetric assembly with repeated occurrences before
  trusting its root-frame mesh convention.

See [the verification guide](docs/testing.md) for repeatable loader and ROS
checks, and [the related-work comparison](docs/related-work.md) for sources.
