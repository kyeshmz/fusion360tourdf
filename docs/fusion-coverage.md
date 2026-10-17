# Fusion input coverage

This is the declared input-support matrix, not a native Fusion certification.
The [case catalog](fusion-use-cases.json) is its source of truth.
The [nine-repository census](fusion-repositories.json) records the audited scope.

Run `python3 tools/verify_fusion_coverage.py --output artifacts/fusion-coverage`
for actual offline results. `--require-complete` fails rather than treating
offline fixtures as native or full-scope proof. A rejection is not input support.

| Use case | Implementation | Offline evidence | Native Fusion |
| --- | --- | --- | --- |
| Current joint frame wins over obsolete history geometry | supported | [tests.test_fusion_history](../tests/test_fusion_history.py) | unverified |
| Direct designs without original joint geometry or timeline access | supported | [tests.test_fusion_history](../tests/test_fusion_history.py) | unverified |
| Rigid as-built joints with null geometry | supported | [tests.test_fusion_history](../tests/test_fusion_history.py) | unverified |
| As-built current transform precedes obsolete geometry | supported | [tests.test_fusion_history](../tests/test_fusion_history.py) | unverified |
| Ordinary and as-built revolute, continuous and slider motions agree | supported | [tests.test_fusion_as_built](../tests/test_fusion_as_built.py) | unverified |
| Suppressed mechanics differ from hidden joint markers | supported | [tests.test_fusion_history](../tests/test_fusion_history.py) | unverified |
| Current-pose revolute and slider limits preserve physical travel | supported | [tests.test_limits_overcome](../tests/test_limits_overcome.py) | unverified |
| Nested ordinary and as-built root-context joints | supported | [tests.test_exporter_logic](../tests/test_exporter_logic.py) | unverified |
| Repeated component instances retain distinct posed mesh targets | supported | [tests.test_exporter_logic](../tests/test_exporter_logic.py) | unverified |
| Direct root-body geometry and mass exclude assembly descendants | supported | [tests.test_exporter_logic](../tests/test_exporter_logic.py) | unverified |
| Mixed parent and child bodies share exact visible geometry and mass selection | supported | [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Explicit, grounded and graph roots preserve separate source identities | supported | [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Reversed revolute and slider endpoints retain correct motion and bounds | supported | [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Overlapping rigid groups form fixed trees through movable attachments | supported | [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Locked joints become fixed at the current pose with provenance | supported | [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Ground To Parent and world grounding retain proven fixed relationships | partial | [tests.test_fusion_grounding](../tests/test_fusion_grounding.py) | unverified |
| Hidden contextual endpoints never bind a visible repeated instance | supported | [tests.test_fusion_contexts](../tests/test_fusion_contexts.py), [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Known Fusion color-property paths preserve per-link RGBA and fallback | supported | [tests.test_fusion_materials](../tests/test_fusion_materials.py), [tests.test_limits_overcome](../tests/test_limits_overcome.py) | unverified |
| Link, joint, visual, collision, CoM and inertia frames | supported | [tests.test_package](../tests/test_package.py) | unverified |
| Mesh failure and existing output never publish partial success | supported | [tests.test_exporter_logic](../tests/test_exporter_logic.py), [tests.test_package](../tests/test_package.py) | unverified |
| Closed mechanisms retain closure metadata outside the URDF tree | urdf-limited | [tests.test_package](../tests/test_package.py) | unverified |
| Fusion motion links converted into affine mimic relations | unimplemented | Rejection only. [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Mesh-only input with a validated mass and inertia policy | unimplemented | Rejection only. [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Bodyless assembly endpoints and reference-frame links | unimplemented | Rejection only. [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Cylindrical, planar, pin-slot and ball joint decomposition | unimplemented | Rejection only. [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Distinct materials and textures per body or face | unimplemented | No input-support test | unverified |

## Limits and gaps

- Ground To Parent and world grounding retain proven fixed relationships. Physical parents and explicitly fixed ancestry are supported; required unresolved bodyless/world frames are named implementation gaps.
- Closed mechanisms retain closure metadata outside the URDF tree. The emitted tree does not enforce the original closed-chain constraint.
- Fusion motion links converted into affine mimic relations. An in-scope Fusion input gap, not a simulator exclusion.
- Mesh-only input with a validated mass and inertia policy. An in-scope input gap; BRep conversion or explicit physical properties are needed.
- Bodyless assembly endpoints and reference-frame links. The current physical-link writer requires geometry and mass.
- Cylindrical, planar, pin-slot and ball joint decomposition. No verified decomposition into URDF joints; these inputs must be rejected by name.
- Distinct materials and textures per body or face. Combined STL output currently has one visual appearance per physical link.

Native fixtures must be executed inside Autodesk Fusion. Docker, Blender,
source inspection, and API doubles cannot replace that gate. See the
[native verifier](../tools/verify_fusion/README.md) and
[testing guide](testing.md).
