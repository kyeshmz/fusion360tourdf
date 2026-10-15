# Design rules for fusion360tourdf exports

Compact rules this exporter actually enforces. For the full philosophy
(adaptive design styles, rigid groups, frame helpers, `!`-tags), see
Adriaeik/fusion2URDF `DESIGN_RULES.md`, cadop/fusion360descriptor docs,
and ACDC4Robot `InstructionsForAssembly.md`.

## Must

1. **One component per link.** Leaf components become links; subassembly
   containers (no direct bodies) are structure, not links. Nested
   assemblies are flattened automatically (unlike upstream, which rejected
   them).
2. **Name the root chassis `base_link`** when modelling cleanly. Imported
   CAD with loose bodies under the design root gets a synthetic `base_link`
   instead of an error.
3. **Parent = Component2, child = Component1.** Flipped joints invert the
   tree; preflight reports the exact parent/child pair to swap.
4. **URDF output is a tree.** A link with two parents (parallel jaws,
   four-bars) or a cycle exports the first joint normally and stashes the
   rest in `config/robot_data.yaml` as `closing_joints` for downstream
   tools (Isaac Sim / MuJoCo). It does not silently break the URDF.
5. **Only Rigid, Revolute, Slider.** Cylindrical / PinSlot / Planar / Ball /
   Inferred abort with a named error instead of invalid URDF.

## Should

6. **Keep exportable geometry visible.** Hidden occurrences (light bulb
   off) are skipped, so toggles select the export set.
7. **Set joint limits on both sides** for Revolute/Slider, or leave both
   unset for a `continuous` joint. Half-limited joints abort with the
   joint name.
8. **Design Z-up, X-forward.** Joint orientations are now exported
   (R_parent^-1 * R_child) with axes resolved in the parent frame, but
   starting in ROS convention avoids surprises.
9. **Use descriptive names.** They flow into links, joints, meshes, and
   `robot_data.yaml`; unsafe characters collapse to `_`.
10. **Configure via `xacro_export.toml`.** ROS version, mesh refinement,
    collision method, effort/velocity live there; see the template.

## Gotchas

- Meshes export WYSIWYG in the design pose; link visual origins undo the
  link frame pose, so do not pre-offset meshes.
- STL scale assumes millimeter export units (Fusion default); the exporter
  requests millimeter STL units best-effort.
- `use_legacy_copy = true` restores the destructive duplicate-components
  flow; back up the design first in that mode.
- `primitive_box` collision needs Fusion bounding boxes; links without one
  fall back to mesh collision.
