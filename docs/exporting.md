# Prepare and configure a Fusion export

Prepare a connected solid assembly, select its root, and export a fresh ROS 2 description package. This guide describes the assembly, frame, and configuration rules behind the [README workflow](../README.md#export-in-fusion).

## Choose the exported assembly

Export reads the active design’s root assembly, not only the currently activated component. Select geometry through visibility and configuration; hidden geometry does not automatically suppress its constraints.

### Select the root

Set `root_occurrence` to an exact occurrence path when you need an explicit root. Use `design_root` to select direct root bodies rather than a component occurrence.

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
- Avoid names that collide after unsafe characters become underscores; collisions stop export rather than overwrite files.
- Use root-owned BRep solids for imported chassis geometry when a separate holding component is unnecessary.
- Connect every selected physical link to the root. Nearby but disconnected parts are not automatically welded.

Ordinary and as-built rigid, revolute, and slider joints are the maintained joint types. An unbounded revolute becomes continuous; a slider requires finite lower and upper limits. Set both bounds or neither where the joint type permits it.

The current design pose becomes URDF joint zero. The exporter subtracts the current driven position from bounds to retain the physical travel range.

Hiding a joint marker does not suppress its constraint. Suppressing a joint changes the mechanism; locking a joint exports it as fixed at the current pose, with its original motion recorded in the sidecar.

Rigid groups become fixed relationships or produce an explicit conflict. Ground To Parent also defines a fixed relationship. Physical parents and proven fixed ancestry are supported; unresolved bodyless/world frames fail rather than receive guessed connections.

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

Combined STL output has one flat appearance color per link. It does not preserve distinct body/face materials or textures.

Extraction recognizes modern primary colors and known legacy `appearanceProperties` colors. Native RGB and opacity values are bytes, not normalized fractions. A supported flat color can remain usable when an appearance also has texture metadata, but the texture pattern is not exported.

If no supported flat color is available, the writer uses silver rather than another material’s color. Color fallback does not change density or mass. The Blender README renders use neutral preview shading rather than testing this appearance policy.

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
| `collision_method` | `visual` reuses detailed meshes; `primitive_box` uses body bounds where available and otherwise retains mesh collision |
| `effort` | Positive finite default joint effort, in N·m or N according to joint type |
| `velocity` | Positive finite default joint velocity, in rad/s or m/s according to joint type |
| `root_occurrence` | Empty for automatic selection, an exact source occurrence path, or `design_root` |

Unknown keys, malformed values, and nonfinite limits abort export. Old `ros_version`, `use_legacy_copy`, transmission, and control switches are invalid. There is no destructive-copy mode to enable as a workaround.

## Publish a fresh package

The UI uses the first whitespace-delimited word of the root/design name, sanitized to a lowercase ROS name. For example, `My Robot` produces `my_description`, not `my_robot_description`.

Choose a fresh destination. Existing output is not overwritten, cancellation creates no package, and required mesh validation must pass before staged files become the published package. Do not run concurrent exports into the same destination.

The self-contained xacro and STL meshes describe the selected robot. Movable robots receive controller configuration and a mock-control launch file; fixed-only robots do not need an empty hardware system.

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
| Grounding has no representable parent or world anchor | Inspect the fixed ancestry and root choice. Bodyless/world frame gaps require supported input or an implementation change, not cleared grounding flags. |
| A duplicate identity or mesh target appears | Inspect sanitized occurrence paths rather than component names alone. |
| Mass or inertia is invalid | Check solids, material density, units, and physical properties. Mesh-only inputs require a validated physical-property policy. |
| A joint or motion link is unsupported | Consult the [input matrix](fusion-coverage.md); do not reinterpret rejection as preserved motion. |
| The output directory already exists | Choose a fresh name and preserve the previous package. |
| Scale or placement is wrong | Compare a known dimension against Fusion and Blender. Do not compensate by manually offsetting meshes. |
| A ROS controller service fails | Retain the runtime log and selected middleware. Repeat the Docker checks; see the [CycloneDDS/FastDDS notes](testing.md#run-the-docker-suites). |
| Docker reports permission denied | Fix engine access outside the exporter. Never make the Docker socket world-writable. |

Continue with the [ROS 2 loading instructions](../README.md#use-the-package-in-ros-2), [testing guide](testing.md), or [native demo workflow](demo-workflow.md).
