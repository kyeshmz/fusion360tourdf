# Export an assembly from Fusion

The user runs the Fusion script and gets a complete ROS 2 description package
without assembly mutation or partial output publication.

## Sub-features

- Extract nested and repeated occurrences with distinct identities.
- Preserve current-pose joint ranges, link frames, and physical properties.
- Export root-owned bodies separately from child occurrences.
- Reject invalid configuration, ambiguous links, and failed mesh exports.

## How to get to it (user POV)

In Fusion, open Utilities, Add-ins, Scripts, and run URDF_Exporter. Use the
[model requirements](../../../../DESIGN_RULES.md) and a fresh destination.

## Driving it with Fusion and loader scripts

Preconditions: a real Fusion installation and a disposable asymmetric design
with rotated joints, a slider, and repeated component instances.

For automated owned-document fixtures, follow
[the native verifier](../../../../tools/verify_fusion/README.md). Restart Fusion
before the run so cached modules cannot stand in for the current checkout.
Add `tools/verify_fusion` from the repository as a Fusion script and run it.
The expected eleven cases include separate parametric/direct documents and an
editable sketch/extrusion case. Keep `report.json` and generated packages.
Parametric fixed cases use as-built joints; direct fixed cases use grounding because Fusion
does not create as-built joints in direct mode. These are distinct evidence cases.

Inspect the report's provenance, source-state preservation and cleanup, not
only its overall status. Original-reference invalidation remains explicitly
`not_exercised` until a genuine native failure is induced. A prepared script or
its offline math tests are not a native pass.

Run the actual script. Inspect the published package and compare the assembly
with its expanded URDF in the loader and Blender recipes. Verify the known
dimensions and nonzero joint poses against the CAD design. Repeat with direct
root bodies. A failed export must not publish a new partial destination.

## Gotchas

For a retained live model and actual-export Docker motion checks, use
[`tools/create_demo` and `tools/verify_export.py`](../../../../docs/demo-workflow.md).
The downstream verifier must receive the native bundle, not regenerate a substitute robot.

Offline Fusion doubles do not prove vendor mesh coordinates. No command in
this repository launches Fusion headlessly. If the application is unavailable,
report this gate unverified. Do not claim fixture-driven ROS tests exported the
real CAD design.
