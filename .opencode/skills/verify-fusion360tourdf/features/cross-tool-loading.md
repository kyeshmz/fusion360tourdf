# Load and articulate exported geometry

The user checks the generated model in xacro, urdfdom, yourdfpy, and Blender.

## Sub-features

- Load visual and collision geometry in meters.
- Check fixed, revolute, continuous, and prismatic poses against literal expectations.
- Reject wrong axes, shifted meshes, and missing resources.
- Preserve an inspectable Blender scene and numeric reports.

## How to get to it (user POV)

Use the [cross-tool instructions](../../../../docs/testing.md#check-geometry-with-blender-and-another-loader).
Install validation dependencies separately from the exporter.

## Driving it with shell scripts

Preconditions: the chosen Python has `requirements-test.txt` installed, and
Blender and `check_urdf` are available. Use executable-path overrides if needed.

```bash
bash tools/verify_cross_tools.sh "$run" 2>&1 | tee "$run/cross-tools.log"
```

Inspect `loaders.json` and each variant's `blender.json`. Require zero and moved
pose results for articulated and box variants and a zero pose for the fixed
variant. Confirm all three `inspection.blend` files survive the run.

## Gotchas

Always use Blender's `--python-exit-code 1`, as the script does. An unexpanded
xacro is not URDF. Real exports without expected-value tables establish loading
and internal consistency, not agreement with the CAD design.
