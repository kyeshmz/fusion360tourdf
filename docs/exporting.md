# Prepare and configure a Fusion export

Prepare the selected assembly, choose its root, and export a fresh ROS 2 description package. The default uses connected BRep solids; the explicit input policies below also cover selected mesh geometry. This guide explains the [README workflow](../README.md#export-in-fusion).

## Choose the exported assembly

Export reads the active design’s root assembly, not only the currently activated component. Select geometry through visibility and configuration; hidden geometry does not automatically suppress its constraints.

### Select the root

Set `root_occurrence` to an exact occurrence path when you need an explicit root. Use `design_root` for the design-root context, including a legitimately referenced bodyless root. Selected frame-only contexts receive no fabricated mass or geometry; see [frame links](frame-links.md).

Without configuration, root selection prefers these candidates in order:

1. An unambiguous `base_link` occurrence
2. Direct root-owned bodies
3. A unique grounded occurrence
4. A unique graph root

Ambiguous candidates require an explicit choice. Component names alone cannot identify repeated instances; use a full path such as `boom:1+carriage:1`.

Joint edges point outward from the selected root. Reversing an edge must update its frame and axis, not only its endpoint names. Grounding the base does not require modifying the design before export.

Selecting a root changes topology, not the design coordinate system or export selection. It does not restrict export to that occurrence’s subtree. Meshes remain in the design pose, and URDF visual origins account for link frames; do not recenter meshes by hand.

### Build links and constraints

Use a component occurrence for each independently moving physical link. Multiple bodies can belong to one link, including a parent component with its own bodies and nested children.

Check the following conditions:

- Keep each repeated instance’s full occurrence identity and posed mesh target.
- Export only each link’s selected direct bodies, not its descendants a second time.
- Keep source names and full contextual identities. Unicode and sanitized-name collisions receive deterministic portable aliases; genuinely duplicate or ambiguous identities still stop export. See the [identifier policy](identifier-policy.md).
- Use root-owned BRep solids for imported chassis geometry when a separate holding component is unnecessary.
- Connect every selected physical link to the root. Nearby but disconnected parts are not automatically welded.

Ordinary and as-built rigid, revolute, and slider joints work by default. An unbounded revolute becomes continuous; a slider requires finite lower and upper limits. Opt-in [generalized decomposition](generalized-joints.md) adds supported cylindrical, pin-slot, planar, and explicitly export-local unbounded Ball coordinates. Native bounded Ball coordinates remain unsupported. Never replace missing required travel bounds with arbitrary large numbers.

The current design pose becomes URDF joint zero. The exporter subtracts the current driven position from bounds to retain the physical travel range.

Hiding a joint marker does not suppress its constraint. Suppressing a joint changes the mechanism; locking a joint exports it as fixed at the current pose, with its original motion recorded in the sidecar.

Rigid groups use their actual occurrence membership or produce an explicit conflict. Ground To Parent also defines a fixed relationship. Referenced frame-only links and proven fixed ancestry are supported; containment alone never welds descendants. Unresolved, hidden, or invalid required references fail rather than receive guessed connections.

## Preserve history and source state

**Do not disable Capture Design History to make an export work.** Converting a parametric design to direct mode removes its timeline and history. Export must not perform that conversion, roll the timeline, reset driven joints, rename components, or move bodies.

Extraction reads current joint transforms rather than the faces, edges, or origins that originally defined them. This allows current frames to remain usable after edits invalidate old references. Rigid as-built joints legitimately have no joint geometry.

If a moving joint lacks a usable current frame, repair the joint and recompute the design in Fusion. Valid old geometry is not a replacement for the missing current frame. Do not substitute an arbitrary anchor or delete history to hide the error.

Parametric and direct inputs have separate native regression fixtures. Fusion only creates as-built joints in parametric designs; the direct fixed fixtures use grounding instead. See the [native verifier](../tools/verify_fusion/README.md) for the exact cases and proof requirements.

When updating the installed exporter, preserve `xacro_export.toml` and restart Fusion after replacing modules. Leave the assembly unchanged until export finishes; concurrent edits or another script driving the design are not a verified workflow.

## Check units, frames, and physical properties

Use Z-up and X-forward where possible. Fusion uses centimeters internally; the exporter requests millimeter STL output and writes mesh scale `0.001` to convert it to meters.

The output uses SI units:

| Quantity | Output unit |
| --- | --- |
| Length, center of mass, and slider travel | m |
| Mass | kg |
| Centroid inertia | kg·m² |
| Joint rotation | rad |
| Effort | N·m for rotation; N for translation |
| Velocity | rad/s for rotation; m/s for translation |

Meshes, visual/collision origins, joint frames, center of mass, and inertia must describe the same link placement. The exporter explicitly transforms native-body STL triangles into root context; a proxy STL export alone does not establish world-coordinate placement.

Assign physical materials and inspect mass properties in Fusion. Visual appearance and physical material density are separate properties. The same selected bodies must supply both geometry and physical properties; descendant-inclusive component mass would double-count nested links.

### Appearance fidelity

Default `visual_materials = "legacy"` output uses combined STL and one flat appearance color per link. That default does not preserve distinct body/face materials or textures.

Extraction recognizes modern primary colors and known legacy `appearanceProperties` colors. Native RGB and opacity values are bytes, not normalized fractions. A supported flat color can remain usable when an appearance also has texture metadata, but the texture pattern is not exported.

If no supported flat color is available, the writer uses silver rather than another material’s color. Color fallback does not change density or mass. The Blender README renders use neutral preview shading rather than testing this appearance policy.

Explicit `strict` and lossy `diffuse-subset` modes retain supported per-face diffuse RGBA, PNG/JPEG images, and supplied UVs in portable OBJ/MTL assets. `visual_meshes = "per_body"` adds separate body files and visuals without splitting the physical link. Read the [visual asset policies and limits](visual-assets.md); arbitrary shaders, procedural baking, ancestor overrides, and native render equivalence are not implied.

## Configure the exporter

Copy `URDF_Exporter/xacro_export.template.toml` to `xacro_export.toml` beside `URDF_Exporter.py`. TOML-file configuration requires Python 3.11 or newer; the exporter otherwise uses only the standard library.

Use TOML values with their actual types:

```toml
mesh_refinement = "low"
collision_method = "visual"
effort = 100.0
velocity = 100.0
root_occurrence = ""
```

| Option | Behavior |
| --- | --- |
| `mesh_refinement` | `low`, `medium`, or `high` Fusion tessellation; higher detail increases mesh size |
| `collision_method` | `visual` reuses detailed meshes; `primitive_box` requests available body bounds; `convex_decomposition` and `convex_hull` create separate collision assets with explicit source/resource limits |
| `effort` | Positive finite default joint effort, in N·m or N according to joint type |
| `velocity` | Positive finite default joint velocity, in rad/s or m/s according to joint type |
| `root_occurrence` | Empty for automatic selection, an exact source occurrence path, or `design_root` |

Use the dedicated references for advanced settings and opt-in modes:

| Options | Reference |
| --- | --- |
| `joint_limits` | [Per-joint effort/velocity and exact exported selectors](joint-settings.md) |
| `generalized_joints`, `ball_chart` | [Primitive decomposition and coordinate/bound limitations](generalized-joints.md) |
| `motion_links` | [Affine mimic relations, feasible limits, and independent commands](motion-coupling.md) |
| `mesh_physics` | [Explicit density or supplied central SI physics](mesh-inputs.md); no inferred density or surface thickness |
| `visual_materials`, `visual_meshes` | [Strict/subset appearance and combined/per-body files](visual-assets.md) |
| `convex_decomposition`, `convex_hull` | [Conservative convex cells](mesh-geometry.md) versus [one true hull and its source policies](convex-hull.md) |
| `output_rotation`, `output_rotation_preset`, `frame_convention`, `minimum_principal_inertia` | [Output reference, DH-friendly placement, and explicit spectral floors](export-policies.md) |

Keep the detailed defaults unless the selected policy fits the model. A supplied-physics surface without positive-volume bounds cannot use `primitive_box`; no thickness is invented. A hull can intentionally fill gaps, whereas density integration and convex-cell decomposition retain their stricter source-topology requirements. New input modes have generated-fixture checks, not original-CAD or universal native Fusion proof.

Unknown keys, malformed values, and nonfinite limits abort export. Old `ros_version`, `use_legacy_copy`, transmission, and control switches are invalid. There is no destructive-copy mode to enable as a workaround.

Finite inputs can still yield an unrepresentable derived origin. Numeric XML and JSON fields fail closed rather than publish `inf` or `NaN`. Axis normalization avoids squared-norm overflow, and checked affine initialization preserves representable cancelling mimic results. A range failure may occur during staging, but never publishes a partial package.

## Publish a fresh package

The UI uses the first whitespace-delimited word of the root/design name, sanitized to a lowercase ROS name. For example, `My Robot` produces `my_description`, not `my_robot_description`.

Choose a fresh destination. Existing output is not overwritten, cancellation creates no package, and required mesh validation must pass before staged files become the published package. Do not run concurrent exports into the same destination.

The plain URDF and self-contained xacro reference package-contained meshes and any selected image/material assets. Copy the complete package, not only its XML or STL files. Movable robots receive controller configuration and a mock-control launch file; fixed-only robots do not need an empty hardware system.

`config/robot_data.yaml` records topology, metadata, warnings, and removed loop-closing constraints. JSON formatting in these `.yaml` files is valid YAML. A closure sidecar records information but does not enforce the closed mechanism’s motion.

## Troubleshoot an export

Use the named error to find the source condition instead of changing the design to hide it:

| Symptom | Action |
| --- | --- |
| Export stops after a history edit | Inspect the joint’s current frame and unresolved references. Keep history enabled and recompute the design. |
| A rigid as-built joint has no geometry | Null geometry is legitimate. Retain the verifier’s report if the fixed relationship fails to export. |
| Root selection is missing or ambiguous | Name one chassis `base_link` or configure an exact occurrence path. Distinguish repeated instances and grounded candidates. |
| A link is disconnected | Add the intended constraint or exclude the geometry deliberately. Spatial proximity is not a fixed connection. |
| A hidden joint marker still exports | Suppress the joint only if you intend to remove its mechanics. Hiding its marker is not suppression. |
| A mechanism depends on hidden geometry | Make the required endpoint visible or change the selection; do not silently drop one side of a constraint. |
| Grounding has no representable parent or world anchor | Inspect actual fixed ancestry, root choice, and unavailable references. Keep grounding flags; containment alone does not prove a fixed connection. |
| A duplicate identity or mesh target appears | Inspect full raw occurrence identities and `source_identifiers`; output aliases do not resolve source endpoints. |
| Mass or inertia is invalid | Check solids, material density, units, and physical properties. Mesh-only inputs require a validated physical-property policy. |
| `BREP_SURFACE_PHYSICS_UNSUPPORTED` names a body | The selected BRep is non-solid. The frontend rejects it rather than infer mass/thickness or hide it beside a solid; see the [surface-BRep policy](mesh-inputs.md#surface-brep-policy). |
| A joint or motion link is unsupported | Consult the [input matrix](fusion-coverage.md); do not reinterpret rejection as preserved motion. |
| The output directory already exists | Choose a fresh name and preserve the previous package. |
| Scale or placement is wrong | Compare a known dimension against Fusion and Blender. Do not compensate by manually offsetting meshes. |
| A ROS controller service fails | Retain the runtime log and selected middleware. Repeat the Docker checks; see the [CycloneDDS/FastDDS notes](testing.md#run-the-docker-suites). |
| Docker reports permission denied | Fix engine access outside the exporter. Never make the Docker socket world-writable. |

Continue with the [ROS 2 loading instructions](../README.md#use-the-package-in-ros-2), [testing guide](testing.md), or [native demo workflow](demo-workflow.md).
