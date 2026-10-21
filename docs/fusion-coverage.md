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
| Ground To Parent and world grounding retain proven fixed relationships | supported | [tests.test_fusion_grounding](../tests/test_fusion_grounding.py), [tests.test_issue_bodyless](../tests/test_issue_bodyless.py) | unverified |
| Hidden contextual endpoints never bind a visible repeated instance | supported | [tests.test_fusion_contexts](../tests/test_fusion_contexts.py), [tests.test_fusion_inputs](../tests/test_fusion_inputs.py) | unverified |
| Known Fusion color-property paths preserve per-link RGBA and fallback | supported | [tests.test_fusion_materials](../tests/test_fusion_materials.py), [tests.test_limits_overcome](../tests/test_limits_overcome.py) | unverified |
| Link, joint, visual, collision, CoM and inertia frames | supported | [tests.test_package](../tests/test_package.py) | unverified |
| Mesh failure and existing output never publish partial success | supported | [tests.test_exporter_logic](../tests/test_exporter_logic.py), [tests.test_package](../tests/test_package.py) | unverified |
| Closed mechanisms retain closure metadata outside the URDF tree | urdf-limited | [tests.test_package](../tests/test_package.py) | unverified |
| Fusion motion links converted into affine mimic relations | partial | [tests.test_integrated_policies](../tests/test_integrated_policies.py), [tests.test_motion_coupling](../tests/test_motion_coupling.py), [tests.test_public_motion](../tests/test_public_motion.py) | unverified |
| Mesh-only input with explicit density or supplied central SI physics | partial | [tests.test_integrated_policies](../tests/test_integrated_policies.py), [tests.test_mesh_inputs](../tests/test_mesh_inputs.py) | unverified |
| Bodyless assembly endpoints and reference-frame links | supported | [tests.test_frame_verification](../tests/test_frame_verification.py), [tests.test_issue_bodyless](../tests/test_issue_bodyless.py) | unverified |
| Collision-safe Unicode source aliases retain mesh and physics association | supported | [tests.test_issue_aliases](../tests/test_issue_aliases.py) | unverified |
| Opt-in conservative convex-cell collision preserves visual bytes and CAD physics | partial | [tests.test_collision_decomposition](../tests/test_collision_decomposition.py) | unverified |
| Cylindrical, planar, pin-slot and ball joint decomposition | partial | [tests.test_generalized_motion](../tests/test_generalized_motion.py), [tests.test_public_motion](../tests/test_public_motion.py) | unverified |
| Distinct materials and textures per body or face | partial | [tests.test_integrated_policies](../tests/test_integrated_policies.py), [tests.test_per_body_hull](../tests/test_per_body_hull.py), [tests.test_per_body_visuals](../tests/test_per_body_visuals.py), [tests.test_public_visuals](../tests/test_public_visuals.py), [tests.test_visual_asset_review](../tests/test_visual_asset_review.py), [tests.test_visual_assets](../tests/test_visual_assets.py) | unverified |
| Explicit global rotation, serial common-normal frames and spectral inertia floor | partial | [tests.test_export_policy](../tests/test_export_policy.py), [tests.test_integrated_policies](../tests/test_integrated_policies.py), [tests.test_public_export_policies](../tests/test_public_export_policies.py) | unverified |
| True single convex hull collision distinct from voxel cells | partial | [tests.test_convex_hull](../tests/test_convex_hull.py), [tests.test_integrated_policies](../tests/test_integrated_policies.py), [tests.test_per_body_hull](../tests/test_per_body_hull.py) | unverified |

## Limits and gaps

- Ground To Parent and world grounding retain proven fixed relationships. Required visible frame contexts are represented; unresolved, hidden or invalid references fail closed. Native and original CAD remain unverified.
- Closed mechanisms retain closure metadata outside the URDF tree. The emitted tree does not enforce the original closed-chain constraint.
- Fusion motion links converted into affine mimic relations. Explicit motion_links=mimic preserves affine delta coordinates, effective independent limits and derived follower states; native phase/context and original CAD remain unverified; bounded Ball and Humble nonzero-offset control reject.
- Mesh-only input with explicit density or supplied central SI physics. Density requires one validated embedded closed shell per body before STL quantization; supplied central SI physics permits positive-area planar MeshBody visuals without invented thickness. Selected non-solid BRep bodies are individually named rejections, not surface-BRep conversion support; mixed links cannot hide them behind aggregate mass. Native MeshBody face-material adaptation remains explicitly unsupported; live Fusion and original-CAD proof remain separate gates; see docs/mesh-inputs.md.
- Bodyless assembly endpoints and reference-frame links. Explicit frame kind has no mass, geometry or STL. Containment does not weld descendants. Historical null/tombstone vendor semantics and original CAD remain unverified.
- Collision-safe Unicode source aliases retain mesh and physics association. Aliases depend on the selected identity set. Native linked revision and original CAD behavior remain unverified.
- Opt-in conservative convex-cell collision preserves visual bytes and CAD physics. One exact-validated embedded shell only. Multiple bodies, multiple shells and cavities reject. Coarse cells may close narrow gaps; no dynamics or practical performance guarantee.
- Cylindrical, planar, pin-slot and ball joint decomposition. Explicit generalized_joints=decompose lowers supported compounds into primitive chains and massless frames; Ball additionally requires an export-local chart and rejects native bounds or coupling; native Fusion and original CAD remain unverified.
- Distinct materials and textures per body or face. Opt-in validated OBJ/MTL PNG/JPEG assets preserve diffuse channels, opacity, UV association and resource bytes; visual_meshes=per_body emits one separate visual/file per selected BRep body with strict or diffuse-subset materials; combined remains the default; subset losses remain explicit; native MeshBody face materials, appearance precedence, arbitrary shaders, procedural baking and render equivalence are not certified.
- Explicit global rotation, serial common-normal frames and spectral inertia floor. Default-off policies run after tree reduction and coupling; output reference carries nonidentity root pose; DH rejects compounds, branches, fixed spacers and closures; flooring is a recorded physics change, not controller-stability proof.
- True single convex hull collision distinct from voxel cells. Default closed_shell requires one validated embedded closed shell per link; explicit triangle_vertices accepts finite nonzero-area triangles from multiple bodies without verifying source topology and fills gaps/cavities; both use the same exact source-float32 hull with zero vertex rounding and bounded work/pieces/bytes; density and decomposition remain strict; no native accuracy, dynamics or practical performance guarantee.

Native fixtures must be executed inside Autodesk Fusion. Docker, Blender,
source inspection, and API doubles cannot replace that gate. See the
[native verifier](../tools/verify_fusion/README.md) and
[testing guide](testing.md).
