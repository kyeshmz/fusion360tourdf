# MotionLink affine backend

This read-only native adapter and pure planner are used by the public exporter
and package writer. Passing offline tests does not close cadop #16 or establish
live Fusion behavior.

## Public modes and mock control

The default `motion_links = "reject"` retains safe rejection of active native
MotionLinks. Opt in with `motion_links = "mimic"`. Generalized coordinates also
require their separate decomposition option. Ordinary and as-built joints bind
IDs 10/11; generalized scalar bindings cover IDs 3 through 9. Ball stays blocked.

The source adapter inventories root and component-local links in each actual
enclosing occurrence context. Actual SDK equality and the held joint/context
inventory canonicalize wrappers. Token strings and joint-name suffixes do not
resolve identity. A null native link context uses its enclosing occurrence.
Suppressed and fully excluded links have distinct receipts. A selected/excluded
crossing, unknown source, suppressed source joint, locked coordinate, or invalid
getter fails closed. SDK references never enter the plain joint records.

`write_ros2_package(..., motion_relations=[...])` also accepts plain affine
relations. Explicit joint `mimic` dictionaries are collected into the same plan.
Planning happens once after topology reduction and exact effort/velocity
selection. Native source receipts are retained. All removed macro DOF names
participate in dependency validation.

Only independent joints receive command interfaces and trajectory-controller
membership. Followers have state interfaces. Initialization chooses one common
feasible independent pose and derives every follower from it. Humble's current
GenericSystem reads composed root/multiplier parameters but does not implement
nonzero offsets. Its generated control launch explicitly rejects such packages
on Humble. Native current-pose delta links have zero offsets. Jazzy's current
GenericSystem reads standard URDF mimic offsets. Both runtime paths still need
real execution. No torque redistribution, hardware, dynamics, or physics claim
follows from mock control.

## Consumer contract

```python
from URDF_Exporter.core.coupling import read_motion_link, apply_couplings

relations = [read_motion_link(link, resolve_binding) for link in selected_links]
plan = apply_couplings(plain_joint_records, relations,
                       removed_joint_names=removed_closure_dofs)
if plan['representation_blockers']:
    raise ValueError(plan['representation_blockers'])
```

The caller selects MotionLinks in the intended assembly context. There is no
root-collection fallback. `resolve_binding(joint, link_context, motion_id)` is
owned by the existing identity owner. It must resolve actual SDK wrappers into
canonical `generalized.MotionBinding` records. Both ordinary Joint and
AsBuiltJoint references use this same callback. A proxy wrapper need not be the
same Python object as its canonical native joint or assembly context. This
module therefore does not second-guess the resolver with wrapper equality.

The resolver must reject excluded, ambiguous, missing, locked, removed, or
otherwise unavailable source DOFs. It must not compare entity-token strings or
derive identity from names or generated suffixes. The binding contains
`SourceIdentity(native_object, context_object)`, a documented integer motion ID,
an allocated primitive name, a finite SI current position, SI unit `m` or `rad`,
and a finite nonzero `coordinate_scale`. The default scale is `1`.

These SDK references are transient. `read_motion_link` returns only plain
serializable data. `apply_couplings` accepts only plain nested dictionaries,
sequences, strings, booleans, finite floats, integers, and nulls. It copies them
without invoking `deepcopy` on arbitrary objects. SDK references accidentally
placed in a joint or provenance record fail with `PLAIN_DATA` before planning.

## Native read surface

Only these documented MotionLink fields are read:

- `isValid`, `name`, and `isSuppressed` are required for every link.
- Active links require `assemblyContext`, `jointOne`, `jointTwo`, `motionOne`,
  `motionTwo`, `valueOne.value`, `valueTwo.value`, and `isReversed`.
- Null `jointTwo` means two motions of `jointOne`. It is not a missing second
  source and does not trigger a collection search.
- A suppressed link returns `status='suppressed'` and its diagnostic name.
  Active fields and the resolver are not accessed.

`read_motion_link(..., enclosing_context=actual_occurrence)` supplies the
explicit context for a component-local native link whose `assemblyContext` is
null. `ExcludedMotion` is reserved for a resolver-proven excluded source.
Two excluded sources yield `status='excluded'`, never `suppressed`.

Unreadable getters preserve their original exceptions as `CouplingError`
causes. Invalid validity/suppression/reversal flags, empty names, non-object
source references, unknown motion IDs, and invalid resolver outputs fail closed.
ModelParameter expressions are never read, interpreted, evaluated, or changed.
No timeline, document, position, joint, parameter, or native field is written.

| Native coordinate | IDs | Backend behavior |
|---|---|---|
| Cylindrical rotation, slide | 3, 4 | SI scalar binding, including same-joint rotate/slide |
| Pin-slot rotation, slide | 5, 6 | SI scalar binding |
| Planar rotation, primary slide, secondary slide | 7, 8, 9 | SI scalar binding |
| Ordinary or as-built revolute, slider | 10, 11 | Same binding shape and resolver contract |
| Ball pitch, roll, yaw | 0, 1, 2 | `BALL_NATIVE_COORDINATES_UNPROVEN` |
| Suppressed link | Any unread active data | Recorded without an active dependency |
| Nonlinear, cyclic, duplicate-follower, or self dependency | Not affine/tree-resolvable | Explicit rejection |

The Ball backend's export-local SO(3) chart is not a native Euler mapping.
It cannot be used to manufacture Ball MotionLink bindings.

## Units, phase, and reversal

`affine_relation(driver, follower, value_one, value_two, is_reversed=False,
link_name=None)` is the pure math entry point. Step values are database cm for
linear coordinates and radians for angles. Convert each step to SI first.
Let `vd`, `vf` be the SI steps, `sd`, `sf` the binding scales, and `d0`, `f0`
the source SI current positions. The relation is:

```text
r_native = (-1 if isReversed else 1) * vf / vd
native_phase = f0 - r_native * d0
q_driver = sd * (native_driver_SI - d0)
q_follower = sf * (native_follower_SI - f0)
q_follower = (sf / sd) * r_native * q_driver + 0
```

The phase is explicitly inferred from the current positions. It is not a
retrieved native phase parameter or an independently verified native rest pose.
Both source positions, step values before/after conversion, units, IDs, scales,
direction flag, native ratio, and phase basis are retained in provenance.
No native objects or context tokens appear in that provenance.

For example, 2 rad driving 5 cm gives `0.025 m/rad`. At source current positions
`0.7 rad` and `0.13 m`, the inferred native phase is `0.1125 m`, but the emitted
delta-coordinate mimic offset is exactly zero. Export zero remains the actual
current assembly pose. Setting `isReversed` negates the ratio. Reversing the
macro edge does not negate it again. Generalized endpoint reversal already
negates spatial axes while preserving source-coordinate signs.

Internal rational arithmetic avoids avoidable intermediate overflow and
cancellation. Finite floating outputs are required. A nonzero SI step, ratio,
composite coefficient, phase, or projected constraint that overflows or rounds
to zero fails with `NUMERIC_RANGE`; it never becomes a silent follower lock.
An explicitly zero follower step is a genuine constant relation. A zero driver
step always fails with `ZERO_DENOMINATOR`.

## Pure application and effective constraints

`apply_couplings(joints, relations, removed_joint_names=())` accepts a mapping
from allocated joint names to existing plain joint records. Moving types are
`revolute`, `continuous`, and `prismatic`. `fixed` records remain in the output
but cannot be dependency endpoints. Optional travel bounds use `None` for
disabled sides. Optional velocity `None` means no supplied cap, not infinity.
Negative velocity caps and inverted/nonfinite travel bounds reject. A source
continuous joint with explicit travel bounds is ambiguous and rejects.

An active relation contains exactly `driver`, `follower`, `multiplier`,
`offset`, optional `status='active'`, and optional plain `provenance` mapping.
Unknown schema fields reject rather than silently dropping a nonlinear rule.
Unlike native current-pose links, explicit pure affine inputs may have nonzero
offsets. Suppressed records contain their status and provenance.

Application validates both moving endpoints, self-dependencies, duplicate
followers, existing mimic metadata, cycles, coefficients, and removed closure
DOFs. Removed DOF references report `REMOVED_CLOSURE_DOF` even when the joint
is absent from the final tree. No dangling mimic dependency is returned.
The caller must provide the complete removed-name set, including expanded DOF
names when removing a generalized closure edge.

Chains and one-to-many relations are resolved to `q_j = A_j*q_root + B_j`.
Each source travel interval projects back to the root. A negative coefficient
swaps the projected sides. Each source velocity cap projects to
`velocity_root <= velocity_j / abs(A_j)`. All projected intervals/caps intersect
with the root's own source limits. A constant follower is checked against its
bounds and imposes no root travel or velocity constraint. Empty domains and
out-of-bounds constants reject. Bounds are rounded inward and velocity caps
downward when converting the exact intersection to floats.

The return value has these plain fields:

- `joints` is the sole owned result mapping. Followers receive only a standard
  `mimic` dictionary with `joint`, `multiplier`, and `offset`. Their original
  limits and effort remain unchanged. Independent joints receive effective
  travel bounds and velocity caps. No torque redistribution is inferred.
- `independent_joints` is the command membership derived from this dependency
  graph, in source insertion order. There is no second controller list.
- `relations` preserves the direct relation provenance, including suppression.
- `dependencies` records each moving joint's independent root and composed
  coefficient/offset. Standard mimic records still reference the direct driver.
- `source_limits` preserves type, original travel bounds, and original velocity.
  `effective_limits` records independent travel and velocity intersections.
- `limit_receipts` records each source constraint, its root projection, its
  composed coefficient/offset, and whether the source is constant.
- `representation_changes` explains a continuous-to-revolute conversion when
  follower travel actually constrains the independent angle.
- `representation_blockers` identifies missing finite travel sides for
  revolute/prismatic URDF records or missing finite velocity caps. Integration
  must reject these before publication. No arbitrary finite limits are inserted.

Planning is not hardware enforcement. It does not implement dynamic torque
sharing, transmissions, Gazebo physics, a closed-loop solver, collision-safe
motion, or physical limit enforcement. It does not certify that current-pose
zero is feasible when the caller supplies inconsistent source positions and
bounds. Downstream initialization must use a common feasible independent pose
and derive followers from it, not clamp each joint independently.

## Verification and remaining gates

Run from the checkout root:

```bash
python3 -m unittest tests.test_motion_coupling tests.test_generalized_motion -v
python3 -m unittest discover -s tests -v
```

The coupling tests include 264 independently computed scalar equation samples,
132 articulated 3D FK comparisons, 1,001 independent feasible-domain samples,
and six velocity samples. The spatial oracle uses independent homogeneous
multiplication, Rodrigues rotation, and explicit absolute native-coordinate
factor equations from the accepted test utilities, not production transform
helpers or the coupling implementation. It covers cylindrical, pin-slot, and
planar current-pose coupling, nontrivial frames/pivots, native reversal, and both
endpoint orientations. SDK doubles exercise every required getter failure,
parameter failures, same-joint null handling, wrapper canonicalization through
the identity owner, numeric range failures, Ball rejection, graph validation,
source ownership, limits, constants, and representation gates.

Public tests cover extraction, repeated component contexts, primitive bindings,
closure removal, exact settings, XML, sidecars, and command/state membership.
`--fixture=mimic` uses the one existing fixture registry. It commands only a
reversed cylindrical rotation and independently expects the slide's state and
TF at zero and nonzero motion. Live Fusion must establish phase semantics and
actual wrapper/vector contexts. Loader, Blender, Humble/Jazzy runtime, and the
original cadop #16 CAD reproduction remain separate proof gates.

## Current official SDK sources

Read on 2026-10-20 after Context7 returned its monthly-quota error:

- [MotionLink](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_MotionLink.htm) defines the read fields, null same-joint case, token warning, and November 2025 introduction.
- [MotionLink.setMotionData](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_MotionLink_setMotionData.htm) defines database cm/radian numeric steps.
- [ModelParameter.value](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_ModelParameter_value.htm) defines database-unit numeric values and failure for text parameters.
- [JointMotionTypes](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_JointMotionTypes.htm) defines IDs 0 through 11.

Documentation reads are contract evidence, not native execution.
