# Optional single convex hull collision

Set `collision_method = "convex_hull"` to replace each physical link's collision
geometry with **one true convex hull of that link's supplied STL vertices**.
The default remains `visual`. This option addresses the separate single-hull
simplification demand in cadop/fusion360descriptor issue 11. It is not the
conservative voxel-cell decomposition used by `convex_decomposition`.

```toml
collision_method = "convex_hull"

[convex_hull]
source_policy = "closed_shell"
max_vertices = 4096
max_faces = 8192
max_work = 2000000
max_triangles = 10000
max_pair_checks = 2000000
max_output_bytes = 4194304
max_pieces = 1024
```

The same configuration works with `export_design` or `write_ros2_package`.
Pure mesh callers can use the standard-library backend directly.

```python
from URDF_Exporter.utils.convex_hull import hull_stl

result = hull_stl(binary_stl_bytes, max_vertices=4096, max_work=2000000)
single_hull_bytes, = result['pieces']
```

## Meaning and supported domain

The input is finite binary STL in root/design-frame **millimeters**. By default it must
describe one embedded, closed, consistently wound triangle shell with nonzero
enclosed volume. Either globally inward or outward source winding is accepted.
The existing exact `validate_mesh` backend rejects open/nonmanifold edges,
pinched vertices, intersecting/overlapping triangles, duplicate or zero-area
faces, inconsistent winding, multiple shells and separately modeled cavity
shells. A link containing disconnected physical bodies fails the default
`closed_shell` policy. There is no silent fallback.

For multiple bodies in one physical link, explicitly select the point-set policy.

```toml
collision_method = "convex_hull"

[convex_hull]
source_policy = "triangle_vertices"
```

The pure API accepts `hull_stl(data, source_policy='triangle_vertices')`.
`HULL_SOURCE_POLICIES` owns the two choices. Explicit `closed_shell` and omission
retain the previous configuration, metadata, and package bytes.

`triangle_vertices` validates binary framing/counts, finite coordinates and
stored normals, and exact nonzero area for every triangle. It accepts multiple
disconnected bodies, nested shells, and open three-dimensional collections.
Duplicate nondegenerate triangles and winding changes do not change the hull.
Source closure, manifoldness, intersections, winding, and solid topology are
**not verified**. Coplanar point sets and zero-area triangles still fail. No
coordinates are repaired, welded, inflated, or quantized. ASCII STL and
unrestricted point clouds remain unsupported.

The output is the hull of the **decoded float32 input vertices**, not of the
unavailable exact CAD surfaces. It intentionally fills concave notches, holes,
gaps between bodies, cavities, and other nonconvex empty regions. It does not preserve source topology or
provide a decomposition error bound. The `fills_concavities` metadata flag
declares this policy, not a measured change for an already convex source.
Single-hull collision may be inappropriate for grippers, sockets, or mechanisms
that need those empty regions. No practical simulation-cost, dynamics,
collision-solver, or native Fusion accuracy claim follows from this option.

Detailed visual STL bytes remain unchanged. Supplied CAD mass, center of mass,
and inertia are not recomputed from the hull or its volume. Meshes keep their
design-frame coordinates. URDF collision origins use the same inverse link
transform as visuals, with scale `0.001 0.001 0.001`. Frame-only links have no
hull, mesh, mass, or collision. Each physical link and its component URDF refer
to one staged asset at `meshes/collision/<link>/piece_000000.stl`.

## Exact geometry and serialization argument

`hull_stl` first bounds and decodes the binary input. The default passes triangles
to the existing exact closed-shell validator. The explicit point-set policy
checks exact triangle areas before building unique integer points. Both then
use the same hull builder and exact manifold/containment audit. All finite binary32 coordinates
have a common power-of-two denominator. Hull construction uses the resulting
integer coordinates. No distance epsilon, angular tolerance, coordinate weld,
or floating-point orientation predicate decides geometry.

Vertices are sorted lexicographically. The first noncollinear and noncoplanar
vertices establish a tetrahedron. Each remaining vertex removes exactly those
faces whose outward signed determinant is positive and joins the horizon.
After each insertion, triangles on the same exact oriented support plane are
merged. A projected exact monotone-chain polygon removes redundant collinear
edge vertices. Each polygon is oriented outward and triangulated as a fan from
its lexicographically smallest vertex. Zero-area horizon triangles are omitted
before this planar reconstruction. This reconstruction is necessary for
edge-collinear insertions to avoid T-junctions. Final facets and triangles have
canonical ordering independent of input triangle order, cyclic vertex order,
global source winding, and the sign of zero.

The final exact audit requires every source vertex to lie on or behind every
outward face, every directed edge to have exactly one opposite use, Euler
characteristic two, nonzero triangle areas, and positive enclosed volume.
Every emitted hull vertex is one of the original float32 vertices. Converting
its exact integer/denominator coordinate back to binary STL reproduces that
same representable float32 value. Consequently the source-vertex serialization
bound is **exactly zero meters**. This is not a numeric tolerance or an inward
rounding allowance. Since every face is an exact supporting plane and all
output vertices are input vertices, the closed convex output is precisely the
point-set convex hull. Convexity contains every source triangle, not just sampled
points. This does not claim that open collections have a solid interior or
that their topology was verified.

Stored STL normals are finite normalized display normals. Exact vertices and
winding define geometry. Very small normal components may round to zero in
float32 without moving a vertex. Input geometry that already collapsed during
upstream float32 serialization fails the closed-shell/degeneracy checks rather
than being repaired or inflated.

## Bounds and failures

All numeric controls are positive integers. Booleans, nonintegers, zero, nonfinite
values, unknown tables/keys, and values above hard maxima reject, including
when the hull mode is not selected.

| Control | Default | Hard maximum | Scope |
| --- | ---: | ---: | --- |
| `max_vertices` | 4096 | 8192 | Unique source vertices per link, checked during ingestion |
| `max_faces` | 8192 | 16384 | Triangles on every working hull surface per link |
| `max_work` | 2000000 | 10000000 | Hull work units, aggregate per package |
| `max_triangles` | 10000 | 100000 | Source triangles per link |
| `max_pair_checks` | 2000000 | 10000000 | Closed-shell source triangle pairs per link; unused under `triangle_vertices` |
| `max_output_bytes` | 4194304 | 67108864 | Collision STL bytes, aggregate per package |
| `max_pieces` | 1024 | 65536 | Hull assets, aggregate per package, always one per physical link |

`max_triangles` bounds input bytes to `84 + 50 * max_triangles` before an STL
file is read into memory or a supplied byte buffer is copied. A count/length
mismatch rejects. Ingestion stops as soon as `max_vertices` would be exceeded.
Closed-shell validation separately prechecks and bounds all `T*(T-1)/2` triangle-pair
visits. Thus the default pair budget can reject before the triangle limit.
It is not an accelerated general-purpose CAD topology validator.
`triangle_vertices` performs no topology-pair checks and reports exactly zero
`validation_pair_checks`. The configured pair limit remains strictly validated.

Incremental construction and planar rebuilding take at most
`O(V * F * log(F))` operations for `V` input vertices and bounded working face
count `F`, followed by `O(V * F)` support auditing. They do not enumerate every
source triple against every source vertex. Exact integer bit widths are also
bounded by finite float32 input, including subnormals and large translations.
Sorting comparisons and individual big-integer arithmetic steps are not
separate work units. Their input sizes remain bounded by the vertex/face and
float32 limits. These bounds are not a wall-clock responsiveness guarantee.

A work unit counts an ingestion row/vertex visit, triangle-area or integer-point
conversion under `triangle_vertices`, seed-search/seed-face visit,
visibility face test, visible/horizon edge visit, planar grouping triangle,
projected polygon vertex/turn test, triangulation face, final source/face
support test, audited edge, or serialized face. Source topology pair checks
are recorded separately. Cancellation is checked before work, during input
validation, at most every 128 work units, at every insertion/facet/audited or
serialized face, and immediately before returning or publishing.

`MeshGeometryError` is a `ValueError` for invalid input or controls.
`ResourceLimitError` derives from it for an exhausted declared budget.
`CancelledError` is a `RuntimeError` when the optional predicate returns true.
An exception raised by that predicate propagates unchanged. The helper writes
no files and never returns partial pieces. The public writer computes each
hull once in its existing temporary package transaction. Invalid geometry,
unknown controls, resource failure, callback exceptions, or late cancellation
leave no published package or partial destination. An existing destination is
never replaced.

The `convex_hull` section of `config/robot_data.yaml` records controls, per-link
assets, exact policy, input/hull counts, positive finite hull volume in SI and
an exact rational form, zero serialization bound, work/pair counts, byte counts,
and unchanged visual/physics policies. Hull volume is informational, not a
replacement for CAD mass or a density assumption.

Point-set receipts add `source_policy: triangle_vertices`,
`source_validation: finite-nonzero-area-triangles`,
`source_topology_verified: false`, `shell_policy: topology-not-verified`, and
`fills_gaps_and_cavities: true`. Actual counts, the same exact hull audit, zero
vertex-serialization bound, budgets, and gap-filling warning remain explicit.
Density integration and decomposition retain their strict closed-shell policy.
This option neither infers mass nor validates CAD. A separately selected inertia
floor changes the output tensor and is recorded as such, not unchanged physics.

## Reproducible verification

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_convex_hull tests.test_collision_decomposition tests.test_mesh_geometry -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_convex_hull --artifacts /absolute/fresh/hull-evidence
PYTHONDONTWRITEBYTECODE=1 python3 -m tests.test_convex_hull --triangle-vertex-artifacts /absolute/fresh/triangle-hull-evidence
```

The new tests independently decode emitted STL, use rational determinants for
every face/source-vertex half-space, require opposite directed edge uses,
check nonzero faces and positive volume, and compare analytic tetrahedron and
prism volumes. Cases include an oblique box, concave L, oblique U, coplanar
grids, smallest float32 subnormals, tiny and large translated geometry, thin
oblique shells, and 24 seeded nonconvex roof shells. L/U hulls fill selected
notches but have strictly smaller volume than their global axis-aligned boxes.
Permutation tests require identical hull bytes. Public tests cover the Fusion
fixture entry, frame-only links, source aliases, supplied physics, original
visual bytes, component URIs/origins/scale, strict controls, exact resource
thresholds, aggregate budgets, and transactional failure/cancellation.

The artifact command retains eight analytic and 24 seeded roof source/hull
pairs, independent numeric audit receipts, and fresh articulated/fixed ROS 2 packages with literal
zero/moved pose and bounds expectations. Use the existing loader and Blender
tools against those packages. See the [verification guide](testing.md).
Offline and fixture results are not native CAD evidence. Real loader, Blender,
installed Humble/Jazzy, and actual Fusion/original-issue CAD checks remain
separate gates for the integrating parent.

The triangle-vertex artifact command retains four analytic source/hull pairs
and fresh `combined_bodies_description` and `oblique_bodies_description`
packages. Literal volumes, bounds, mass 4 kg, COM `(0.05, 0.10, 0.065)` m, and
full tensor sums are independent of the emitted output. Strict-policy negatives,
finite/framing/area errors, resource caps, aggregate limits, and early/late
cancellation remain tested. The `per_body_hull` registry fixture adds separate
body visuals, global rotation, a positive spectral floor, and nonzero motion.
