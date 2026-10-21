# MeshBody physical-input policy

Visible Fusion MeshBody inputs require explicit physics. Export does not infer
density from a material name, call `MeshBody.getPhysicalProperties`, convert a
mesh to BRep, copy a body, or change the timeline. Existing solid BRep inputs continue
to use Fusion's physical properties. Frame-only links remain massless and have
no geometry.

## Configuration reference

Place `xacro_export.toml` beside the exporter, or pass the same nested mapping
as `export_design(..., config=...)`. `config.py` owns parsing and numeric
normalization. It copies nested values. An override `mesh_physics` table replaces
the file's entire `mesh_physics` table.

To apply one explicit uniform density to every selected mesh body:

```toml
[mesh_physics]
density_kg_m3 = 7800.0
```

Density must be a finite positive number in kg/m³. Each mesh body must contain
one closed, consistently wound, embedded triangle shell. The exact backend
rejects open edges, nonmanifold vertices, self-intersections, duplicate faces,
multiple shells, and cavities. Reversed winding is accepted. Reported
`isClosed` and `isOriented` flags are recorded, not trusted as validation.
Different selected bodies are integrated separately and combined with the
parallel-axis theorem. Intersections between distinct bodies are not tested.
Their masses are additive, as they are for distinct BRep bodies.

For a per-body density, use an exact raw occurrence path and body name:

```toml
[[mesh_physics.bodies]]
occurrence = "module:2+scan:1"
body = "Imported scan"
density_kg_m3 = 2700.0
```

Use `design_root` for a root-owned body. Selectors do not use exported aliases,
sanitized names, entity tokens, collection indices, or suffix matching. Repeated
instances use different full occurrence paths without source renaming. Two
selected bodies with the same name in one occurrence make that selector
ambiguous. Use a uniform density without that selector instead. Unknown,
duplicate, ambiguous, hidden, and unused selectors reject. A configured uniform
density that matches no selected body also rejects. Per-body entries override
the uniform density when both are present.

For a consciously modeled surface or a measured assembly, supply complete
central physical properties instead of a density:

```toml
[[mesh_physics.bodies]]
occurrence = "design_root"
body = "Surface scan"
mass_kg = 2.0
center_of_mass_m = [0.01, 0.02, 0.0]
inertia_kg_m2 = [0.003, 0.004, 0.005, -0.0002, 0.0001, -0.0003]
frame = "native"
provenance = "Measured assembly model. kg, m, kg*m^2. Central tensor."
```

`inertia_kg_m2` is the central symmetric tensor in the order
`[ixx, iyy, izz, ixy, iyz, ixz]`. Cross terms are signed tensor entries, not
unsigned products of inertia. Values must be finite. Mass and tensor diagonals
must be positive. The tensor must be positive semidefinite and obey the
principal-moment triangle inequalities, within the existing scale-relative
floating-point validation tolerance. A density and supplied tensor cannot be
mixed in one body entry.

`frame` must explicitly be `native` or `design_root`. Both use SI values.
`native` means the body's parent-component axes. The captured occurrence pose
places its COM and rotates its central tensor into design-root axes once.
`design_root` means the supplied COM and central tensor already use those axes.
The provenance string is mandatory. The sidecar records the supplied provenance,
original `Fusion MeshBody` type, frame, and a deliberate-override warning. It
never labels supplied values as native Fusion mass.

Supplied physics permits open, finite, nondegenerate triangle visuals. It does
not assert that a surface is a solid, infer a thickness, or validate that the
supplied physics matches the visual. This mode performs no solid-topology or
inter-body intersection analysis. Such interpretation remains the caller's
responsibility. Degenerate geometry, invalid indices, nonfinite coordinates,
and ambiguous placement still reject.

## Geometry and publication

Selection uses visible direct bodies only. Descendants are not included in a
parent's mesh or mass. A mixed link includes selected BRep and MeshBody inputs
once each. Hidden bodies and occurrences, including hidden ancestor contexts,
remain excluded. Frame-linked and repeated mesh occurrences retain their raw
context identity until the existing alias allocator runs.

One export reads each selected native `displayMesh` once, then captures each
selected occurrence's placement. Proxy display geometry is not read and then
transformed a second time. Native cm coordinates become root-frame coordinates
and binary STL millimeters. Normals are regenerated from the placed triangles.
The immutable triangle payload supplies the staged STL directly. MeshBody is
never passed to `exportManager.createSTLExportOptions`.

Density mode validates the original native shell before serialization. A small
source seam cannot become a solid merely because STL quantization closes it.
Density integration then independently validates and uses the exact serialized
float32 millimeter triangles.
This keeps physical integrals consistent with the emitted geometry, including
STL quantization. Geometry that overflows float32 or collapses after float32
conversion rejects. Mesh extraction applies no implicit mass or inertia floor.
A separately selected export-policy floor is an explicit recorded physics change.
Caller-supplied SI
properties remain the caller's model rather than an integral of STL geometry.
The existing writer transforms root-frame properties into each URDF link frame.
The public joint and inertial dictionary contract is unchanged.

The sidecar's optional `mesh_physics` map contains per-link body provenance.
Existing `link_sources` entries retain their physical or frame classification.
Single-RGBA STL output does not preserve textures or multiple face materials.
Native MeshBody snapshots explicitly reject nonlegacy face-material modes
because they do not expose BRep faces or validated UVs. The public writer can
still combine mesh physics with a caller-supplied validated visual bundle.

Surface bounds are derived from actual triangles. Axis-aligned planar geometry
has no positive-volume box, so no thickness is fabricated. Default `visual`
collision mode retains the triangle mesh. `primitive_box` rejects when positive
box bounds are unavailable. `convex_decomposition` retains its existing
closed-single-shell requirement and rejects an open surface during staging.
No successful publication occurs on extraction, cancellation, staging, or
decomposition failure. Export does not mutate source geometry or physical data.

## Resource controls

The `[mesh_physics]` table accepts these integer controls:

| Option | Default | Maximum | Scope |
| --- | ---: | ---: | --- |
| `max_triangles` | 10000 | 100000 | Aggregate selected MeshBody triangles per export |
| `max_nodes` | 30000 | 300000 | Nodes in each native display mesh |
| `max_pair_checks` | 2000000 | 10000000 | Aggregate native-shell and emitted-shell density validation pairs per export |

Repeated instances count toward the triangle and emitted-shell pair budgets.
Native-shell validation is counted once per distinct native mesh in density
mode. Native geometry reads are cached only within one export. Density integration
and extraction check the caller's cancellation predicate. The limits are not a
performance or simulator-complexity claim. Separate convex-collision controls
still apply to the combined staged link STL.

## SDK and verification limits

The implementation follows current Autodesk documentation retrieved on
2026-10-20:

- [MeshBody](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_MeshBody.htm)
  documents `displayMesh`, `nativeObject`, `assemblyContext`, `isValid`, and the
  topology flags.
- [TriangleMesh](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_TriangleMesh.htm)
  documents `nodeCoordinatesAsDouble`, `nodeIndices`, `nodeCount`,
  `triangleCount`, and `isValid`.
- [Occurrence.meshBodies](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Occurrence_meshBodies.htm)
  returns root-context proxies for root-context occurrences. Autodesk lists
  this property as introduced in April 2026. Older clients are not certified.
- [Occurrence.transform2](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Occurrence_transform2.htm)
  supplies the occurrence pose in its assembly context.

`tests.test_mesh_inputs` drives `export_design` with documented API doubles.
It checks analytic mass, signed COM, all six central tensor terms, posed shared
native geometry, mixed ownership, surfaces, selectors, and failure publication.
These tests do not establish native Fusion placement behavior. They do not
reproduce the original ACDC#2 design or prove installed ROS, Blender, or
independent-loader acceptance.

The loader and Blender verifiers now distinguish explicitly declared planar
geometry from the default volumetric contract. Independent expectations must
name `geometry_dimensions[link] = 2` and exact positive `geometry_triangles`
counts for both visual and collision geometry. Every triangle still needs
finite coordinates, nonzero area, valid resources, correct poses and bounds.
Missing faces, missing resources, nonfinite coordinates, zero-area triangles,
and undeclared collapsed solids remain failures. No sidecar flag alone grants
the exception. Use the existing `--fixture=surface` runtime route. The offline
contract tests are not an actual-loader or Blender execution claim.

### Surface BRep policy

The Fusion frontend checks each selected BRep body's documented Boolean
[`isValid`](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepBody_isValid.htm)
and [`isSolid`](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepBody_isSolid.htm)
properties before requesting physical properties.
An invalid or unreadable body fails closed with its occurrence/body identity.
A non-solid BRep fails with `BREP_SURFACE_PHYSICS_UNSUPPORTED`, including in a
mixed solid/surface link: positive aggregate mass must not hide a surface with
no chosen volumetric physical interpretation. Hidden, unselected surfaces are
neither exported nor queried for physics.

This is an explicit rejection policy, **not surface-BRep conversion support**.
No surface thickness, density, or mass is inferred, and `mesh_physics` does not
silently apply to BRep bodies. The supplied-physics surface route above applies
to MeshBody inputs. It does not convert or edit a BRep or its history. Public
writer callers may supply their own complete physical model and validated mesh
resources; the native Fusion frontend has no BRep-surface physics override.

`tests/test_brep_surfaces.py` exercises surface-only, mixed, nested, hidden,
invalid-metadata and unused-density cases through `export_design`, checking
named diagnostics, rollback, selection, and source/history preservation with
API doubles. Actual native surface-BRep reproduction remains unverified.

Arbitrary mesh repair, cavities, material-density inference, and universal mesh
support are not implemented.
