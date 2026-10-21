# Explicit export policies

The ROS 2 package writer accepts default-off coordinate, frame and inertia
policies. `URDF_Exporter.utils.export_policy` owns the pure mathematical boundary.
The existing writer remains the only serializer. These synthetic tests do not
establish live Fusion, RViz, Gazebo, controller, or hardware support.

The original requests are syuntoku14/fusion2urdf issues #30 and #55, and
cadop/fusion360descriptor issue #12. Their saved inventory texts describe a
90-degree rolled model, a proposed `1e-4` inertia cutoff, and a convenient
DH-friendly joint-position option. This module supplies explicit policies,
not reproductions of those authors' CAD or a claim that their issues are closed.

## Public configuration

Pass these keys in `write_ros2_package(..., config={...})`, or put them in
`xacro_export.toml` beside `URDF_Exporter.py` for normal export. Omit a key to
disable it. Programmatic `None` also disables it. TOML has no null literal.

```toml
output_rotation_preset = "y_up_to_z_up"
frame_convention = "dh_common_normal"
minimum_principal_inertia = 0.0001
```

These are three independent, explicit choices, not recommended defaults.
`output_rotation_preset` accepts only `"y_up_to_z_up"`. Alternatively,
`output_rotation` accepts a flat row-major nine-number proper rotation matrix.
It must be orthonormal with determinant +1 within `1e-12`. Matrix and preset
are mutually exclusive after TOML and programmatic overrides are merged.
Overrides replace individual keys, so clear the other selector with `None`
when changing representation programmatically. Normalization is repeatable.

`frame_convention` accepts only `"dh_common_normal"` when selected.
`minimum_principal_inertia` is finite and nonnegative, in kg*m², not a diagonal
component cutoff. Booleans, numeric strings, unknown selectors, reflections,
scaled matrices and malformed vectors are errors. Invalid file options cannot
be hidden by a later override. These checks, source physics
validation, and mode applicability checks finish before the mesh callback.
Failures and cancellation never publish a partial destination.

All four keys default to `None`. With all policies disabled, the public writer
uses its original validation and arithmetic. It does not route legacy inputs
through the stricter pure helper. Unused independently omitted box fields,
tiny valid tensors and finite large valid tensors retain their old behavior.
The public zero-floor-only path is also an identity operation. It uses original
geometry and tensor serialization, without imposing the helper's paired-box
schema or a finite-principal-value requirement. Its receipts include
`principal_moment_scale_kg_m2`. Multiply the recorded before/after moments by
this scale to recover kg*m². Usually the scale is 1. For overflowing principal
values it is the source tensor magnitude and moments are normalized finite
numbers. No infinite number is emitted. Zero does not reconstruct the tensor.
Meaningful rotation/DH policies still require the helper's source schema below.

When a selected rotation makes the root pose nonidentity, the writer emits an
empty `export_output_reference` link and a fixed transform to the original
`base_link`. Collision-safe numeric suffixes avoid all source link/joint names.
The reference has no CAD owner, mesh, mass, or inertial tag. The original CAD
`base_link` remains the same physical body with its source axes. The output
reference is the coordinate system in which the requested conversion is
visible. RViz's fixed frame is that reference. Thus source +Y really becomes
output-reference +Z for the preset, rather than merely changing a receipt label.
An identity root does not need an extra link. A floor alone does not rotate axes.

`robot_data.yaml` remains JSON-compatible YAML. Selected exports add
`export_policies` with normalized choices, full receipts, zero-pose link frames,
the unchanged-source-design-mm mesh contract, and complete original closing
joint records. `root_frame` names the source root, output reference, root pose,
and optional fixed joint. `links`, `joints`, `link_sources` and component URDFs
include the generated reference coherently. Source identifiers and source-link
kinds survive copying. Every physical component URDF shares the package assets;
the generated reference component is empty.

The writer first reduces loops using the existing rooted-tree selection. Global
rotation and inertia flooring operate on that tree. Removed closing joints keep
every existing metadata field; their known design-frame `xyz` is rotated once,
while parent-relative `rpy` and joint-local `axis` stay unchanged. Opaque custom
metadata is not reinterpreted. Complete pre-policy records, including original
source fields, are retained in `export_policies.source_closing_joints`.
The sidecar does not enforce closure dynamics. DH with any removed closure is
rejected before export instead of applying an incomplete gauge conversion.

All emitted STL and OBJ visuals and every hull or cell collision piece share
the source-design-mm emission origin in `_robot_xml`. Portable OBJ/MTL/image
bundles remain byte-identical under output rotation. Native compound provenance
rejects DH even after lowering to primitive helper chains. Rotation and floors
can coexist with generalized or mimic motion without replanning coupling.

## Caller and ownership

```python
from URDF_Exporter.utils.export_policy import (
    apply_export_policies, regularize_inertia, Y_UP_TO_Z_UP,
)

faithful = apply_export_policies(joints, inertial)
selected = apply_export_policies(
    joints, inertial,
    output_rotation=Y_UP_TO_Z_UP,
    frame_convention="dh_common_normal",
    minimum_principal_inertia=1e-4,
)
```

Every option defaults to `None`, meaning disabled. Valid input dictionaries are
deep-copied. With all policies disabled, returned `joints` and `inertial` equal
their inputs exactly, including tiny tensors, omitted optional keys, and metadata.
Validation still runs. Invalid physics or topology does not become acceptable
because a policy is disabled. No inputs, CAD components, STL bytes, or files are
modified. There are no Fusion imports, mutable global state, or dependencies
outside the standard library and the existing pure `transforms` module.

One module owns mathematical validation, frame selection, spectral flooring, and
the derived emitter contract. A parallel serializer was rejected. The existing
writer consumes the contract directly. Private helpers are implementation details;
`__all__` names the three
public imports. Python does not prohibit private imports.

## Source schema and validation

The input schema matches the current writer's source quantities.

* `joints` maps names to `parent`, `child`, `type`, and absolute design-frame
  `xyz` in meters. Optional `rpy` defaults to zero and is **parent-relative**,
  in radians, with `R = Rz(yaw) Ry(pitch) Rx(roll)`. A primitive joint's `axis`
  is expressed in its own joint frame. Positive coordinates retain the original
  motion sign. Supported primitives are revolute, continuous, and prismatic.
  Fixed joints are supported unless DH-friendly placement is selected.
* `inertial` maps physical link names to positive mass in kg, signed
  `center_of_mass` in design-frame meters, and the **central** inertia tensor in
  design-frame kg*m². The six-value order is `xx, yy, zz, xy, yz, xz`, with actual
  tensor off-diagonal signs, not negated CAD product-of-inertia values.
* Optional `bbox_center_m` and `bbox_size_m` must occur together. They describe
  the existing **source-design-axis-aligned box**, not a newly computed AABB.
  Both may be `None`, matching the existing extractor's unavailable-box result.
  That case produces no box emitter record and does not fabricate dimensions.
* A link explicitly tagged `kind="frame"` carries no mass, COM, tensor, or box.
  It receives a frame and joints but no mesh, box, or inertial emitter record.
  The public writer preserves these explicit frame-only source links as well.
* `source_root_frame={"t": [...], "R": [...]}` optionally supplies the base
  link's absolute zero-pose design frame. The default is identity. `R` is a flat,
  row-major, active 3-by-3 rotation. All other returned rotations use that shape.

Relevant numeric fields must have the exact vector sizes and finite non-boolean
numbers. Axes must be nonzero. Rotations must be proper and orthonormal within
`1e-12`. Reflections and scale matrices are rejected. Known optional joint-limit
fields must be finite and supplied lower/upper limits must not be inverted.
This is not the complete writer preflight. Writer-specific name safety, limits,
material, and packaging rules still belong to their existing owners.

The topology must be one connected tree rooted at `base_link`. Cycles, multiple
parents, root incoming joints, missing links, and disconnected links are rejected.
This module does not choose a spanning tree or discard closing joints. The
public writer makes that decision first and preserves closure records separately.
Derived numeric overflow is an error, not a usable partial result.

## Output rotation and exact emitter contract

Let `G` be the selected source-design-to-output-design rotation, or identity.
There is no global translation policy. `Y_UP_TO_Z_UP` is a positive 90-degree
rotation about X. It maps `(x,y,z)` to `(x,-z,y)`, so positive source Y becomes
positive output Z. It is not automatically selected from CAD metadata.

The returned design quantities obey:

```
p_output = G p_source
c_output = G c_source
I_output = G I_source G^T
F_output_zero = [G,0] F_source_zero
```

All six central tensor components are transformed. Translating a link frame
does not shift a central tensor away from the COM and does not require a
parallel-axis addition. Without reframing, parent-relative `rpy` and joint-local
`axis` are left unchanged. Rotating the joint-local axis by G again is wrong.

`link_frames[name] = {"t", "R"}` is the chosen absolute output-design zero
frame of each link. The public emitter honors the following returned records.
They are ready-to-emit poses, not additional corrections to the old origins.

| Record | Required interpretation |
| --- | --- |
| `emission.root_pose` | Absolute output-design pose of the root. Preserve it with an explicit external reference/fixed parent when nonidentity. Do not silently reset it to identity. |
| `emission.joint_origins[joint]` | Parent-link-to-child-link zero pose `F_parent^-1 F_child`. Use its `t` and `R` once. Returned joint dictionaries also contain equivalent absolute `xyz`, parent-relative `rpy`, and joint-local `axis`. |
| `mesh_source_to_output` | Rigid pose `[G,0]` of the unchanged source-design geometry in output-design coordinates. |
| `emission.mesh_origins[link]` | `F_link^-1 [G,0]`. Apply it to **unchanged source-design STL vertices**, after `emission.mesh_scale = [0.001]*3` converts millimeters to meters. The same origin applies to visual and mesh collision. |
| `emission.inertials[link]` | Original mass, local COM `R_link^T(c_output-t_link)`, and local central tensor `R_link^T I_output R_link`. The inertial pose uses this COM and identity rotation. Do not rotate the tensor a second time. |
| `emission.boxes[link]` | Box pose `F_link^-1 [G,G c_box_source]` and the **unchanged source box side lengths** in meters. Its rotation carries the source box basis. Do not use the mesh translation as the box center, rotate an already recomputed AABB, or apply STL scaling. |

STL bytes remain source-design-frame millimeters. Rotated dictionaries without
the mesh contract are insufficient. Passing only `result["joints"]` and
`result["inertial"]` into legacy inverse-link mesh math does **not** implement
this feature. That math omits G and box orientation needs its rotated basis.
The public writer selects the emitter records instead of applying legacy
corrections to them a second time.

## DH-friendly common-normal convention

`frame_convention="dh_common_normal"` is the only accepted selected convention.
It applies to a single rooted serial chain of primitive joints. Branching, fixed
spacers, closed chains, compound joints, and near-parallel ill-conditioned axis
pairs fail explicitly. None are automatically flattened or approximated.

At the source zero pose, let joint i's oriented world axis line be
`p_i + s z_i`. Positive `z_i` follows the original signed motion axis. For every
nonterminal joint, consider the next joint's line `p_next + t z_next`.

1. For nonparallel, nonintersecting lines, choose the two unique closest feet.
   The current frame origin is the foot on line i. Its +X points from that foot
   toward the foot on the next line. Its +Z is `z_i` and +Y is `Z cross X`.
   Thus X is the actual common-normal direction, not an arbitrary transverse axis.
2. For intersecting nonparallel lines, use the intersection as origin. Choose
   +X as normalized `z_i cross z_next`, +Z as `z_i`, and +Y as `Z cross X`.
3. For parallel distinct lines, the family of common normals is nonunique.
   Choose the origin by projecting the **source-root reference origin** onto
   line i. Project that foot onto the next line. Choose X toward the next foot.
   This also handles antiparallel oriented axes without reversing motion signs.
4. For coincident lines, choose the same root-origin projection. Choose X by
   projecting source-root +X into the plane transverse to `z_i`. If degenerate,
   try source-root +Y, then +Z. The reference rotates with output conversion.
5. The terminal joint has no next axis. Keep its source pivot and use the same
   transverse source-root-axis rule. The root frame itself is never DH-reframed.

The parallel angular band is `|z_i cross z_next| <= 64*ulp(1)`. The interval
between that band and `1e-8` is rejected as ill-conditioned. Intersection and
coincidence use a distance tolerance of
`1e-12 * max(1 meter, |p_i|, |p_next|)`. The source-root reference projection
uses a transverse norm cutoff of `1e-8`. These explicit floating-point choices
resolve otherwise nonunique axes. They are not a promise of exact classification
for arbitrarily tiny offsets or enormous coordinate origins.

Each origin is displaced only **along its own joint axis**. Each orientation
preserves that oriented axis. A constant frame gauge change of this form
commutes with that joint's revolute or prismatic motion. Recomputing all relative
joint poses and local geometry/mass records therefore preserves zero and
nonzero FK. No source component moves. Receipts name each case, the next joint,
the selected origin, and the next-axis foot.

This is a documented convenience frame convention. It is not a standard-DH or
modified-DH parameter table, a guarantee that all local translations meet a
particular textbook table indexing scheme, or a proof for arbitrary mechanisms.

## Spectral inertia floor

`regularize_inertia(tensor, minimum_principal_inertia=None)` exposes the same
operation separately. A selected threshold must be finite, nonnegative, and
expressed in kg*m². `None` is disabled. Zero is explicitly enabled but does not
raise any already nonnegative moment. No mass is fabricated or changed.

Validation precedes flooring. Negative diagonals, materially negative principal
moments, nonfinite values, and violated principal triangle inequalities fail even
when a large threshold was requested. A scale-normalized symmetric Jacobi solve
uses a `64*ulp(1)` normalized roundoff allowance for eigenvalue and triangle
tests. This permits roundoff at physically semidefinite and triangle boundaries,
not a modeling tolerance with a fixed absolute inertia cutoff.

For admissible `I = V diag(lambda) V^T`, the operation is
`V diag(max(lambda, threshold)) V^T`. It preserves eigenspaces, is invariant to
eigenvector signs and repeated-root choices, and is basis-equivariant up to
floating-point roundoff. Flooring nonnegative physical principal moments keeps
their triangle inequalities. It is not a six-component clamp. Reconstruction
adds only changed spectral contributions to the original tensor. Disabled or
unchanged tensors retain their original values without reconstruction.

Each receipt returns owned `original` and `final` tensors, before/after principal
moments, `threshold_kg_m2`, `enabled`, `changed`, and a warning when changed.
In the combined API these tensors are in the **source design basis**, before
coordinate conversion. Returned inertial dictionaries use the output design
basis. Coordinate conversion is a basis change, not a physics change. A spectral
floor is an intentional physical-properties change and is never silently selected
to make a simulation appear stable. No controller stability, dynamics accuracy,
or hardware safety claim follows from the floor.

## Offline evidence and integration gates

Run from the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_export_policy -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_public_export_policies -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
git diff --check
```

The analytic tests use their own Euler application, Rodrigues screw motion,
matrix products, prescribed axis lines, and anisotropic tensors. They do not
reuse production transform or eigenvalue helpers as their oracle.

* A translated/rotated root with a revolute joint and an oblique signed slider
  exercises three global rotations, two frame selections, and three motion
  samples, including zero. Four source-design mesh points, eight original box
  corners, signed COM, and all nine tensor entries are checked per link/sample.
* A five-revolute chain prescribes skew, parallel-offset, coincident-antiparallel,
  intersecting, and terminal axes. Analytic closest feet distinguish true
  common-normal placement from Z alignment. Two rotations and three motion
  samples check world invariance after reframing.
* Rotated anisotropic, tiny, zero, semidefinite, triangle-boundary, and invalid
  tensors check disabled fidelity, selected floor, equivariance, and provenance.
  Validation and successful operations check input immutability.

For those two FK scenes there are eight policy configurations, 24 configuration
motion samples, and 90 link/sample cases. The checks compare 360 mesh points,
720 box corners, 90 COM positions, and 810 tensor components. These are synthetic
pure-math checks, not STL writer, independent loader, or native Fusion results.

Public tests separately parse emitted XML and binary STL. A rotated/translated
asymmetric rotary-plus-slider scene uses four policy configurations and three
motion samples. It checks 36 link/sample cases, 144 mesh points, 288 original box
corners, 36 signed COM positions, 324 tensor components, and independently
derived link frames. The oracle uses source-space Rodrigues screw motion and
an independently solved common normal, not production policy helpers. Exact
seed XML/sidecar SHA-256 values protect disabled output. Other tests cover strict
configuration, zero-floor fidelity, named applicability errors, root/reference
resources, closure provenance, floor warnings, and atomic failure/cancellation.

Generate fresh packages and expected-value tables for separate runtime checks:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_public_export_policies --fixtures OUTPUT
```

Default repository fixture variants are unchanged. The generated tables contain
independent source-space geometry and common-normal frame expectations for zero
and two nonzero samples. Actual cross-tool loading, Blender and both ROS 2
distributions remain separate gates. A live Fusion export is still required
before claiming native CAD support. No runtime gate is waived by offline tests.
