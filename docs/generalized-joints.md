# Generalized motion backend contract

`URDF_Exporter.core.generalized.expand_motion` returns a pure intermediate
representation used by the public `export_design` path. Existing default
behavior and locked-joint handling are unchanged. Offline integration does not
establish original-issue reproduction or live Fusion correctness.

## Public opt-in

Set `generalized_joints = "decompose"` in `xacro_export.toml` or the public
`config` mapping. The default is `"reject"`. Unbounded Ball additionally needs
`ball_chart = "export-local-rx-ry-rz"`. This declares a root-aligned local chart
at the current pivot, not native pitch/yaw/roll semantics.

Macro orientation and loop-edge selection happen before expansion. Compound
edges remain active motion during rigid-group validation. Removed macro edges
retain their omitted DOF names and limits in the sidecar, but create no helper
links or actuators. Generated frames carry plain current poses and provenance.
They have no SDK occurrence, bodies, mass, inertia, or mesh callback target.
The adapter uses parent-relative rotations and current design-root positions.
The existing inverse-link visual origin preserves root-frame geometry once.

Use exact generated names in `joint_limits` for effort and velocity. Macro
selectors and fixed attachments reject. Missing finite travel rejects before
mesh publication with `URDF_FINITE_TRAVEL_REQUIRED`; no travel is fabricated.

## Supported coordinates

| Input `kind` | Ordered moving factors | Native `JointMotionTypes` IDs | Backend status |
|---|---|---|---|
| `cylindrical` | Rotate, slide on the same axis | Rotate 3, slide 4 | Native scalar deltas and bounds |
| `pin_slot` | Slide, rotate | Slide 6, rotate 5 | Native scalar deltas and bounds |
| `planar` | Primary slide, secondary slide, rotate | Primary 8, secondary 9, rotate 7 | Native scalar deltas and bounds |
| `ball`, all bounds disabled | Export-local X, Y, Z rotations | No native mapping | Full SO(3) orientation set, with a singular chart |
| `ball`, any bound enabled | None | Pitch 0, roll 1, yaw 2 | `BALL_NATIVE_BOUNDS_UNPROVEN` |
| Native Ball MotionLink coordinate | None | 0, 1, 2 | `BALL_NATIVE_COORDINATES_UNPROVEN` |

The first three rows describe the backend's mathematical interpretation of
documented axes and scalar positions. A native adapter still needs live pose
evidence for vector context, endpoint convention, and current pivot resolution.

## Caller inputs

The keyword-only contract accepts these inputs:

- `source` is `SourceIdentity(native_object, assembly_context_object)`.
  Both references must come from the caller's resolved identity owner.
  Root-context `context` may be `None`. Tokens, names, and numeric IDs are
  rejected as source identities. Lookup uses Python object identity, not token
  strings or overloaded equality. Canonicalize repeated API wrappers before
  calling this module. The module does not resolve or mutate a source registry.
- `source_name`, `parent`, and `child` are allocated ASCII export aliases.
  Endpoints must differ. User aliases cannot start with `fusion_helper_` or
  `rigid_attachment_`. `source_path` is a nonempty diagnostic path, not identity.
- `parent_pose_cm` and `child_pose_cm` are the actual current exported link-frame
  poses in a common design root. They are flat row-major rigid 4x4 matrices.
  Translation is in Fusion database centimeters. Link frames may be offset and
  rotated from the moving pivot. Supply the exported frame, not an assumed
  occurrence frame if your link convention differs.
- `joint_pose_cm` is the current translated moving pivot in that same root.
  Its rotation is any proper frame basis. For Ball, that basis defines the
  export-local X, Y, and Z axes. For other types, native vectors define axes.
- `vector_rotation` is a proper row-major 3x3 rotation from the coordinate
  system of the supplied SDK vectors to the common root. It defaults to the
  identity only for vectors that are already root-resolved. No API coordinate
  convention is inferred from an object's name or context.
- `motion` exposes only the documented read properties listed below.
  This function does not import `adsk` and never writes those properties.
- `reverse` selects the opposite orientation of the complete macro edge.
  Determine the outward direction on the macro graph before expansion.
  Pass endpoints and poses in their original source order. Set `reverse=True`
  when the outward direction opposes that order. Do not pre-swap the inputs
  and then set `reverse=True`. Do not reverse resulting moving joints
  independently afterward.
- `occupied_names` contains all allocated link and joint names, including
  earlier expansions. Collisions reject. The caller remains the sole global
  allocation owner and must pass the complete set.
- `effort` and `velocity` are positive finite defaults, both `1.0` if omitted.
  Rotations use N m and rad/s. Slides use N and m/s. Callers should pass
  deliberate values rather than assume one physical unit across the chain.
- `dof_settings` maps exact generated moving-joint names to positive `effort`
  and `velocity` overrides. Macro names and unknown selectors reject. Bounds
  cannot be changed through this mapping. The fixed attachment is not a DOF.

All rotations and homogeneous rows are validated. Axes must be finite and
nonzero. Normalization preserves their sign. Pin-slot axes must be orthogonal.
All three planar axes must be pairwise orthogonal. A left-handed supplied
planar slide pair remains valid. The module reads the documented secondary
direction instead of manufacturing it with a cross product.

## Documented motion properties

The backend reads these fields, with no alternative guessed getters:

| Kind | Axes | Current values | Limits |
|---|---|---|---|
| Cylindrical | `rotationAxisVector` for both factors | `rotationValue`, `slideValue` | `rotationLimits`, `slideLimits` |
| Pin-slot | `slideDirectionVector`, `rotationAxisVector` | `slideValue`, `rotationValue` | `slideLimits`, `rotationLimits` |
| Planar | `primarySlideDirectionVector`, `secondarySlideDirectionVector`, `normalDirectionVector` | `primarySlideValue`, `secondarySlideValue`, `rotationValue` | `primarySlideLimits`, `secondarySlideLimits`, `rotationLimits` |
| Ball | No native reference vectors or angles used | Actual pose comes from the caller | `pitchLimits`, `yawLimits`, `rollLimits` |

Each limits object supplies `isMinimumValueEnabled` and
`isMaximumValueEnabled`. The module reads `minimumValue` or `maximumValue`
only when its corresponding flag is enabled.

Lengths convert from cm to m once at this boundary. Angles stay in radians.
The exported coordinate is `q_export = q_native_SI - q_current_SI`.
Each enabled bound subtracts that same current position. Disabled bounds remain
`None`, including one-sided bounds. An angular coordinate with no bounds is
`continuous`. A partially bounded angular coordinate is `revolute` in this IR.

The IR preserves unbounded slides without fabricated travel. Standard URDF
requires finite lower and upper limits for a prismatic joint. A later serializer
must reject absent bounds with a named representation error, or obtain explicit
user travel settings through a separately reviewed policy. It must not silently
replace `None` with zero, a huge constant, or a guessed design extent. The same
representation gate applies to one-sided revolute limits. This backend does not
claim that those cases are already publishable as standard URDF.

## Returned records

`Expansion` contains immutable tuples of frozen dataclasses:

- `primitives` contains one moving `Primitive` per scalar DOF, followed by one
  fixed attachment. Each record has its exact name, endpoints, type, coordinate,
  child-frame axis, parent-relative `origin`, bounds, effort, and velocity.
  `Pose.xyz` uses meters. `Pose.rotation` is a row-major 3x3 matrix.
- `frames` contains one generated `Frame` per DOF. Each has `kind='frame'`,
  its current design-root pose, and the source diagnostic path. No frame has
  bodies, mass, inertia, visuals, collisions, or a mesh target.
- `bindings` contains one `MotionBinding` per supported native scalar coordinate.
  It retains the exact `SourceIdentity`, integer native motion ID, generated
  primitive name, SI current position, unit, and `coordinate_scale=1.0`.
  `binding_for(native, context, motion_type)` rejects a different identity or
  an unavailable native coordinate. Do not parse joint-name suffixes to bind
  later MotionLinks.
- `parameterization` is `native_scalar_deltas` or `export_local_rx_ry_rz`.
  Ball has no native bindings and explicitly blocks IDs `(0, 1, 2)`.

Generated moving names have the form `fusion_helper_<source_name>__<coordinate>`.
Generated frames use `fusion_helper_<source_name>__frame_<index>`.
The final fixed joint is `fusion_helper_<source_name>__attachment`.

For N DOFs, the result contains N frame links and N+1 joints. The fixed
attachment preserves the caller's physical child frame, including arbitrary
pivot offsets. The physical child appears exactly once as the final child.
Its geometry and inertia stay with the original source record. No physical
source, CAD helper body, fake occurrence, or fabricated mass is returned.

The alternative with N-1 frame links requires moving the physical child's
frame to the pivot or treating an offset child origin as the rotation center.
This backend uses the explicit fixed attachment instead. It leaves physical
frame ownership with the caller and preserves arbitrary caller poses.

These records are not drop-in `Core.Joint` dictionaries. The current exporter
uses design-frame origins, child-frame axes, and parent-relative RPY. An adapter
must use the returned poses consistently and provide generated frame rotations
directly. It must not fabricate an `adsk` owner for `_source_rotation`.
When it emits RPY, it must preserve the returned rotation matrix.

## Current-pose factorization and reversal

Let H be the current pivot pose in the common root. Let P and C be the current
parent and child poses. A moving factor is a spatial translation along its
root axis or a spatial rotation about the current pivot. Each factor is
parameterized by its source delta, not by its absolute native position.

The chain represents `inverse(P) E1(d1) ... EN(dN) C`. At zero, every Ei is
the identity and the chain preserves `inverse(P) C`. Pin-slot uses slide then
rotation. Planar uses both translations then rotation. The current pivot
already includes the current source slide offsets. Subtracting the source
positions is therefore valid even at a nonzero current pose.

Reversal represents `inverse(C) inverse(EN) ... inverse(E1) P`.
The backend reverses the factor order, swaps the physical endpoints, and
negates every spatial axis. It then recomputes all local axes and origins.
Bounds and the source-coordinate sign stay unchanged. Reversal does not negate
MotionLink ratios merely because the endpoint direction changed. Later affine
binding must also account for source SI scaling and both SI current positions.

## Ball chart and remaining native uncertainty

The unbounded Ball chain chooses `Rx(a) Ry(b) Rz(c)` in the basis of H.
All three exported coordinates are zero at the current actual pose.
This parameterization reaches every orientation in SO(3). It is not a claim
about Fusion pitch, yaw, roll, their order, their reference vectors, or limits.

At `cos(b)=0`, the angular-rate Jacobian loses rank and coordinates are not
unique. Forward kinematics remains defined. The backend performs no inverse
Euler extraction, rate inversion, automatic chart switch, or native-coordinate
conversion. A controller must avoid rate inversion at that singularity or
use an explicitly designed chart policy. The backend does not claim globally
nonsingular three-coordinate control.

Every enabled native Ball bound blocks expansion, including a single bound.
Every request to bind native Ball IDs blocks. This prevents silently dropping
bounds or using a valid SO(3) chart as an unproved native Euler mapping.

## Verification and integration gates

Run `python3 -m unittest tests.test_generalized_motion -v` from the repository
root. The tests use independent homogeneous matrix multiplication, Rodrigues
rotations, and explicit native factor equations. They do not import production
transform functions to compute expected poses.

The principal native-coordinate matrix evaluates 324 poses. It covers three
motion kinds, two current poses, two edge orientations, and 27 samples per
combination. Each combination includes current-pose zero, absolute native
zero, a mixed pose, and 24 seeded samples. The Ball matrix evaluates 200 poses
across both edge orientations, including both chart singularities and 96
seeded samples per orientation. Seeds are `20261020` and `731`.

Another 60 evaluations vary the root axes and helper basis independently.
Another 50 use independent normalized-Gaussian quaternion targets in SO(3).
The corresponding seeds are `914` and `187`.

Those 634 FK evaluations are mathematical backend evidence. Frame ownership,
current poses, SI rebasing, native IDs, exact identity, naming, invalid input,
bound preservation, and selector semantics have separate assertions. No loader,
physics, native API execution, original issue CAD, or public feature completion
follows from these tests.

Public tests cover macro orientation, frames, exact settings, sidecars,
MotionLinks, command membership, closure reduction, and unchanged defaults.
`python3 -m tests.fixtures OUTPUT --fixture=generalized` creates an opt-in
reversed cylindrical fixture with current nonzero native positions and analytic
zero/moved pose, geometry, mass, and independent-DOF oracles. It is generated
evidence, not a native bundle. Parent-owned loader, Blender, ROS 2, and live
Fusion gates remain required. Bounded/native-coupled Ball additionally needs
authoritative native composition and limit evidence.

## Official SDK references

The following current Autodesk pages were read on 2026-10-20. Context7 lookup
failed with its monthly quota message, so the official help pages supplied the
contract evidence. Their field descriptions are not native pose experiments.

- [CylindricalJointMotion](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_CylindricalJointMotion.htm).
- [CylindricalJointMotion.rotationAxisVector](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_CylindricalJointMotion_rotationAxisVector.htm).
- [PinSlotJointMotion](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_PinSlotJointMotion.htm).
- [PlanarJointMotion](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_PlanarJointMotion.htm).
- [BallJointMotion](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BallJointMotion.htm).
- [Joint.setAsBallJointMotion](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Joint_setAsBallJointMotion.htm).
- [BallJointMotion.pitchValue](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BallJointMotion_pitchValue.htm).
- [JointMotionTypes](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_JointMotionTypes.htm).
- [MotionLink](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_MotionLink.htm).

The Ball pages describe directions that angles are measured from. They do not
establish an Euler composition order. No such order is inferred here.
