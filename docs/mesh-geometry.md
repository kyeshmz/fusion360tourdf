# Pure mesh geometry helpers

These standard-library helpers are algorithm backends. The public exporter and
writer opt in with `collision_method='convex_decomposition'`. They preserve CAD
physics and original visual STL bytes and publish all pieces transactionally.
Neither pure helper reads or writes files. Offline evidence does not prove
native Fusion behavior.

## Public export configuration

```toml
collision_method = "convex_decomposition"
[convex_decomposition]
resolution = 16
max_pieces = 32768
max_cells = 32768
max_work = 2000000
max_triangles = 10000
max_pair_checks = 2000000
```

Unknown options, booleans, fractional values and out-of-range integers reject.
`max_pieces` and `max_work` bound aggregate package output/work. Other limits
apply per physical link. `max_pair_checks` also independently bounds validation
work. Staged and per-body STL file sizes are checked before reading collision
input bytes, and aggregate link triangles are bounded before concatenation.
Fusion's own mesh-export call is a vendor operation; these controls cannot
interrupt or budget its internal tessellation work. Pieces retain the staged
STL's design-frame millimeters and use the same
inverse-link origin as the original visual. Each piece gets its own collision
element, package URI and contained `meshes/collision/<link>/piece_*.stl` asset.
Component URDFs contain the same compound collisions. Frame-only links receive
no decomposition or mesh target.

The sidecar's `collision_decomposition` records controls, asset association,
per-link policy, measured counts and approximation bounds. The public
`export_design` and `write_ros2_package` APIs accept an optional `cancelled`
predicate. Cancellation, predicate exceptions, invalid geometry, resource
excess, or a failure on any later link abort the whole staged publication.
This mode accepts only a combined physical-link STL that passes the single-shell
validator. Ordinary multi-body links may therefore reject here, even though
the default visual and primitive-box modes still support them. It does not
guess solid unions, fill cavities, repair meshes, or compute density/physics.

## Uniform-density physical properties

```python
from URDF_Exporter.utils.mesh_geometry import mass_properties

physics = mass_properties(binary_stl_bytes, length_unit='mm', density_kg_m3=2700)
```

The density in this example is a caller-supplied example, not a material lookup or
default. Both `length_unit` and `density_kg_m3` are required. Supported length
units are `m`, `cm`, and `mm`. Density must be a finite, positive number in
kg/m³. Geometry can be binary STL bytes or an iterable of triangles, each
containing three finite three-coordinate points. Iterable coordinates are
converted to Python floats. Binary STL coordinates are the actual binary32
values, not an inference about the original CAD surface. STL normal fields must
be finite but are not trusted for winding.

The returned dictionary contains these values.

* `volume_m3` and `mass_kg` are positive SI floats.
* `center_of_mass_m` is a three-tuple in the input coordinate frame, in meters.
* `inertia_kg_m2` contains `ixx`, `iyy`, `izz`, `ixy`, `ixz`, and `iyz`.
  These are the **central tensor entries**, not positive products of inertia.
  In particular, `ixy = -integral((x-cx)*(y-cy) dm)`.
* `provenance` records method, explicit density, length unit, shell policy,
  orientation, triangle and vertex counts, and validation pair count.

Each finite floating-point coordinate is converted to an exact integer divided
by one common power-of-two denominator. Tetrahedra use an input vertex as the
reference. Signed volume, first moments, and second moments are exact integer
sums with rational scale factors. The central second moment subtracts the
parallel-axis term **exactly**, before any conversion to floating point. Input
unit scaling and the supplied density are also applied as rational factors.
Only final SI values are rounded to Python floats. This avoids cancellation
from a distant world origin and avoids intermediate underflow for tiny but
representable physical results. No mass adjustment, guessed density, welding
tolerance, inertia floor, or geometry repair is applied. Nonzero results that
overflow or underflow the final float format are rejected, not silently clamped.

## Accepted solid interpretation

`validate_mesh(source, *, max_triangles=10000, max_pair_checks=2000000,
cancelled=None)` returns an immutable `ClosedMesh`. Its integer coordinates,
denominator, indexed triangles, orientation, and pair count are available for
the collision backend. Validation establishes all of the following.

1. Every triangle has nonzero exact area and finite coordinates.
2. Every undirected edge has exactly two opposite directed uses.
3. Every vertex link is one cycle, excluding pinched vertices.
4. All triangles form one edge-connected shell.
5. Triangle intersections contain only their shared topological vertex or edge.
   Exact plane/segment predicates also reject coplanar overlap.
6. The shell has nonzero exact signed volume.

A consistent global reversal is accepted and recorded as `inward`. Integrals
are sign-normalized. Mixed winding is rejected. Multiple disjoint shells,
overlapping solids, nested shells, and cavities are **rejected**. Supporting
them requires an explicit nesting/material interpretation that this API does
not invent. A hollow part must therefore not be silently treated as solid by
discarding its cavity shell. Single-shell concave solids are supported.
Coordinates must share exact vertex values. Near-coincident seams remain open.

The complete triangle-pair count is checked before pair intersection work.
Triangle AABBs prune expensive intersection predicates, but every broad-phase
pair consumes the declared pair allowance. This deliberately rejects large
CAD meshes instead of performing unbounded quadratic validation. The default
two-million-pair allowance permits at most 2000 triangles. The larger
`max_triangles` limit also bounds ingestion before topology is known.

## Explicit conservative collision approximation

```python
from URDF_Exporter.utils.convex_collision import decompose_stl

collision = decompose_stl(
    binary_stl_mm,
    resolution=16,
    max_pieces=32768,
    max_cells=32768,
    max_work=2000000,
    max_triangles=10000,
    max_pair_checks=2000000,
    cancelled=lambda: False,
)
```

The input is binary STL bytes **in millimeters**. `pieces` contains binary STL
byte strings in millimeters, each representing a closed convex axis-aligned
box with twelve outward-wound triangles. There is one piece per occupied cell.
There is no global hull, box around the whole link, merging, external engine,
or native application dependency. Neighboring pieces share boundaries and may
overlap by their outward binary32 rounding envelopes. This is an opt-in
conservative voxel approximation, not an exact or minimal decomposition.

The longest input bounding-box extent is divided into `resolution` equal cell
sides. The other grid dimensions round upward. Occupied cells are selected by
two exact tests in an integer grid.

1. For each triangle, visit only cells in its closed AABB range. Mark a cell
   when the triangle intersects the closed cell, using the thirteen-axis
   triangle/box separating-axis test. Touching counts as intersection.
2. For remaining cells, classify their centers by exact ray parity. No such
   cell meets the surface, so its entire connected interior is inside or
   outside the solid. Rays that touch projected edges or lie in triangle planes
   are retried using deterministic integer directions `(1, k, k*k)` for
   `k=1..16`. Unresolved ambiguity fails the call rather than guessing parity.

Surface cells are never selected solely by a center sample. A feature thinner
than a cell, including an oblique feature, remains covered by surface cells.
Coarse cells can nevertheless close narrow gaps or erase narrow concavities.
Choose a resolution whose declared error is acceptable for the intended
collision clearance. Large L and U notches remain empty in the independent
tests; no universal topology-preservation promise is made.

### Error contract

Let `S` be the validated closed solid represented by the **input bytes**, `h`
the exact cell side in meters, and `V` the union of selected exact cells.
Every point of `S` is in a surface cell or an interior cell, hence `S ⊆ V`.
Every selected surface cell contains a point of `S`. Its diameter is
`sqrt(3)*h`. Every selected nonsurface cell is inside `S`. Therefore
`d_H(S, V) <= sqrt(3)*h`, where `d_H` is solid-set Hausdorff distance, not a
surface-to-surface or volume-error bound.

Serialized box bounds are rounded **outward** to binary32. Their extra
displacement is bounded by the sum of the largest exact outward error on each
axis, `r`. This L1 bound also bounds Euclidean displacement. If `P` is the
serialized piece union, `S ⊆ V ⊆ P` and
`d_H(S, P) <= sqrt(3)*h + r`. The published `hausdorff_bound_m` uses the exact
rational upper approximation `17320508075688774 / 10^16 > sqrt(3)`, adds exact
rational `r`, and rounds upward once more with `nextafter`. The input STL's own
approximation of CAD geometry is **not** included in this bound. No relative
volume, contact-normal, thin-gap, or dynamics accuracy is promised.

Metadata reports `policy=conservative-voxel-cells-v1`, `approximation=True`,
units, coverage, SI error bound, cell side and its exact rational ratio,
outward rounding bound, grid shape/count, piece count, surface-cell count,
surface candidate visits, consumed work, validation pair checks, triangle
count, all limits, ordering, and unchanged visual/physics policies. Piece
ordering is lexicographic grid index and winding does not affect occupancy.

### Resource and failure policy

All controls require positive integers, excluding booleans. `resolution` is
bounded by 64; `max_cells` and `max_pieces` by 262144; `max_work` and
`max_pair_checks` by ten million; `max_triangles` by 100000. Defaults are lower.
One collision work unit is a triangle-AABB candidate visit, one grid-cell
visit, or one ray/triangle test. Each candidate uses at most thirteen fixed
separating axes. Exact coordinate bit lengths are bounded by finite binary32
input. Grid storage, output count, ingestion, validation pairs, and geometric
work are all bounded independently. Candidate ranges avoid a full
triangle-by-grid scan, but worst-case interior classification can still be
quadratic in triangles and cells. It aborts at `max_work`; this is not a claim
of a fast general CAD mesher. Pieces are not coalesced for runtime performance.

`MeshGeometryError` is a `ValueError` for invalid geometry, controls, unsupported
interpretation, unresolved exact-ray ambiguity, or unrepresentable output.
`ResourceLimitError` derives from it and identifies a declared budget failure.
`CancelledError` is a `RuntimeError` raised when `cancelled()` returns true.
Caller exceptions from that predicate propagate unchanged. Checks occur during
ingestion, periodically during validation/work, and for each serialization
piece. No call returns partial pieces or writes files on any failure. A caller
must also keep its own output publication transactional.

## Verification and integration gates

Run the scoped tests and the existing offline harness.

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_mesh_geometry tests.test_collision_decomposition -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```

Independent physical oracles use analytic cuboid tensors, rotated tensors,
and cuboid union/parallel-axis sums. Collision tests decode returned bytes,
check face half-spaces for convexity, sample known L/U solids, test excluded
notches, measure returned corners against an independently transformed box,
and sample thin-feature input triangles. Sampling complements, but does not
replace, the exact-predicate coverage argument above.

Public writer tests cover complete multi-file publication, all compound URIs,
component origins, supplied CAD physics and original visual bytes. Pure-helper
tests do not replace loader, Blender, installed ROS, actual Fusion, original
issue CAD, or practical simulation-cost evidence. Mesh-input density/unit glue
is a separate policy gate and remains unimplemented here. These changes address
algorithm and writer requirements behind ACDC4Robot #2/#6 and cadop #11, not
whole-issue closure.
