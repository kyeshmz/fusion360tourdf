# Run owned-document Fusion verification

Run this script inside Autodesk Fusion. Do not run `verify_fusion.py` with system Python.
Offline tests do not establish a native Fusion pass. Inspect the report from your actual run.

1. Keep the checkout intact so `tools/verify_fusion` and `URDF_Exporter` share the same repository root.
2. Save your existing work and restart Fusion after updating exporter or verifier modules, and before each verification run. Do not run the exporter first.
3. In Fusion, open **Utilities > Add-ins > Scripts and Add-ins > Scripts**.
4. Click **Add** and select this checkout's `tools/verify_fusion` directory.
5. Select **verify_fusion** and click **Run**.
6. Choose a writable parent folder for evidence. The script creates a unique `fusion-proof-*` directory.
7. Read the result message and open the reported `report.json` path.
8. If a case fails, retain its report and any exported packages. Read the recorded traceback and failed assertion before rerunning.

Expect eleven case entries in separate new documents. Both modes cover root-owned solids with a hidden body,
ordinary revolute and slider joints with finite limits and hidden joint markers, and translated/rotated nested repeated components.
Parametric fixed fixtures use geometry-free rigid as-built joints. Direct fixed fixtures use native Ground To Parent instead:
Fusion refuses as-built-joint creation in direct designs. These are different input cases, not direct-mode as-built proof.
An additional parametric fixture creates an editable rectangle sketch and extrusion driven by the `proof_height` user parameter.
The ordinary joints start at zero motion. Rotated asymmetric boxes expose axis swaps independently of joint motion.

Use the report's `source_before`, `source_after`, and `source_preservation` fields to inspect design mode, design intent, history marker and count,
occurrence paths and transforms, direct body ownership and visibility, and current joint motion values.
Inspect `timeline.features` for feature names, types, suppression, and health state.
Inspect `parameters` for user and model parameter expressions, units, and values.
The runner retains API references and checks entity equality before and after export.
The `source_entities_before` and `source_entities_after` tokens are evidence only. Different token strings can identify the same entity.
Unsupported proxy-token getters are recorded as `token_unavailable`; retained API-reference equality remains mandatory.
The script changes design mode only in an empty document that it created. It never rolls a timeline.
The runner checks Hybrid design intent when the SDK exposes that property. It does not change design intent.
It attempts to close only its owned documents without saving and restores the previously active document after each case.
If Fusion refuses cleanup, inspect the failed cleanup entry and close the remaining fixture yourself.

Check `assertions` for the actual binary STL bounds in root-frame meters and direct-body CAD bounds.
The mesh checks reject degenerate facets, non-box surfaces, open or inconsistently wound edges, and incorrect enclosed box volumes.
These checks apply to the disjoint analytic box fixtures, not arbitrary CAD meshes.
The XML checks cover zero-pose visual and collision placement, mass, world COM, analytic box inertia, joint types, signed world axes, and limits.
Revolute joints must lie on the literal expected world axis line. A pivot shift along that line is physically equivalent and accepted.
The runner also evaluates exported URDF FK at nonzero joint positions and compares moved mesh bounds and COM against literal fixture motion.
Expected motion uses fixture world axes and pivots, never exported joint origins or production transform helpers.
STL bytes use millimeters. Native body triangles are explicitly transformed into root context; proxy STL export alone is not sufficient.
Fusion dimensions use centimeters. XML mass and inertia use kilograms and kilogram square meters.
Native body mass supplies the analytic box density. The expected dimensions, centers, and 90-degree rotations do not use exporter transform helpers.
The runner calls `URDF_Exporter.URDF_Exporter.export_design` with a fresh description-directory path.
The resolved default configuration is recorded and passed explicitly to that operation.

Inspect `provenance` before comparing runs. It records the Git revision and dirty state when Git is available.
If Git is absent inside Fusion, the report records that limitation explicitly and still retains SHA256 hashes of production and verifier input files.
Hashes are captured before production imports and cases, then checked again at the end, including after partial failures.
Loaded-code evidence records source paths and bytecode hashes and compares source-defined functions and ordinary class methods with the files on disk.
The runner rejects already loaded exporter modules and code from another checkout.
The loaded-code comparison does not verify mutable module globals or SDK behavior. Restart Fusion as instructed above.
File hashes alone do not prove that Fusion executed the exporter, and offline tests do not establish a native pass.

Do not interpret an overall `pass` as universal Fusion coverage. Original joint-geometry invalidation remains `not_exercised`.
This script does not induce topology edits that invalidate original geometry, move the history marker, or actuate native joints at nonzero positions.
It does not automate or launch the native application outside Fusion's authorized script interface.
An interrupted run can leave `running` entries and an open owned fixture. Such a run is not a pass.

For a retained, inspectable robot whose actual exported package is built and commanded in Docker, use
the [rotary-slide demo workflow](../../docs/demo-workflow.md).

To check the independent math, source-state comparisons, report persistence, and document ownership guards offline, run these commands from the repository root:

```bash
python3 -m unittest tests.test_fusion_verifier -v
python3 -m py_compile tools/verify_fusion/verification.py tools/verify_fusion/verify_fusion.py
```

For the fixture-creation API contracts, consult Autodesk's
[temporary box method](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_TemporaryBRepManager_createBox.htm),
[body persistence requirements](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepBodies_add.htm),
[point joint geometry](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_JointGeometry_createByPoint.htm),
and [null geometry for rigid as-built joints](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_AsBuiltJoints_createInput.htm).
Autodesk's [document creation sample](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/DocumentCreation_Sample.htm)
states that `Documents.add` creates Hybrid documents.
