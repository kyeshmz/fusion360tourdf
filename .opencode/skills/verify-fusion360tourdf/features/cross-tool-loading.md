# Load and articulate exported geometry

The user checks the generated model in xacro, urdfdom, yourdfpy, and Blender.

## Sub-features

- Load visual and collision geometry in meters.
- Check fixed, revolute, continuous, and prismatic poses against literal expectations.
- Reject wrong axes, shifted meshes, and missing resources.
- Preserve an inspectable Blender scene and numeric reports.
- Render checked zero/moved poses for the README without changing the input scene on disk.

## How to get to it (user POV)

Use the [cross-tool instructions](../../../../docs/testing.md#check-geometry-with-blender-and-another-loader).
Install validation dependencies separately from the exporter.

## Driving it with shell scripts

Preconditions: the chosen Python has `requirements-test.txt` installed, and
Blender and `check_urdf` are available. Use executable-path overrides if needed.

```bash
bash tools/verify_cross_tools.sh "$run" 2>&1 | tee "$run/cross-tools.log"
bash tools/verify_cross_tools.sh "$run" --fixture=frames 2>&1 | tee "$run/frames-cross-tools.log"
bash tools/verify_cross_tools.sh "$run" --fixture=generalized
bash tools/verify_cross_tools.sh "$run" --fixture=mimic
bash tools/verify_cross_tools.sh "$run" --fixture=materials
```

Inspect `loaders.json` and each variant's `blender.json`. Require zero and moved
pose results for articulated and box variants and a zero pose for the fixed
variant. Confirm all three `inspection.blend` files survive the run.

The optional frame fixture uses the same writer and literal five-link poses.
`base_link` and `arm` are explicitly frame-only. Three actuated joints remain.
Only the other three links have physical bounds and resources. Require both
zero/moved frame poses and physical geometry. Negative controls reject missing
physical geometry, physical tags on a frame, and a missing second collision.
Compound-collision integration checks require every emitted piece to load,
not just one resource per owner. Frame fixtures are generated evidence, never
native-bundle evidence. Default `VARIANTS` and the default three models remain
unchanged.

The three additional opt-in selectors share that registry and stay generated
evidence. Generalized/mimic cases require analytic helper-frame and physical
zero/moved poses, source mass, independent command sets, and follower state.
Materials require every OBJ material part, MTL opacity and diffuse channels,
per-corner UV association, decoded PNG and both JPEG pixel oracles, and all
manifest hashes. Blender keeps every imported object's intrinsic transform.
Per-part planar triangles do not replace aggregate physical-volume checks.
An independently declared surface fixture alone permits positive-area planar
geometry, with exact triangle counts for both kinds. Missing resources/faces,
nonfinite/zero-area triangles and undeclared collapsed solids still fail.

The same optional registry includes `policy_rotation`, `policy_boxes_dh`,
`policy_floor`, `mesh`, `surface`, `hull`, `materials_hull`, and `materials_cells`.
Pass any name through `--fixture=NAME` to this existing wrapper. These remain
generated evidence. Require separate visual/collision bounds, every collision
piece, unchanged material manifests, and the emitted output-reference root pose.

`per_body` and `per_body_hull` use that registry too. Require both separate OBJ
files on every physical link, exact body/material triangle membership, all
RGBA/UV/image receipts, and literal per-body zero/moved bounds. The latter also
requires a triangle-vertex hull, cyclic global rotation and positive inertia
floor. Every visual must consume the same policy mesh origin. Source assets
stay in design millimeters with `.001` scale. Reject a missing, dropped or
shifted second visual. These checks do not certify native CAD or rendering.

`tools/verify_export.py` also runs `tools/render_readme.py` against the checked inspection scene.
Inspect the PNGs visually and retain `images/images.json`; renders are illustrations, not substitutes
for the loader/Blender assertions. See the [README image recipe](../../../../docs/demo-workflow.md#refresh-the-readme-pictures).

## Gotchas

Always use Blender's `--python-exit-code 1`, as the script does. An unexpanded
xacro is not URDF. Real exports without expected-value tables establish loading
and internal consistency, not agreement with the CAD design.
