# Fusion-to-URDF source comparison

The comparison scope is Fusion assembly input exported to ROS 2 URDF. PyBullet,
other CAD adapters, MJCF, SDF, and simulator-specific output are excluded.
Rigid groups, history/direct modeling, nested assemblies, root selection,
visibility, and motion coupling remain Fusion input concerns.

All nine repositories below were inspected at pinned revisions on October 17,
2026. The [machine-readable census](fusion-repositories.json) records exact
revisions and inspected source paths. This is source inspection, not native
Fusion execution or a universal ranking.

## Pinned repository findings

| Repository and revision | Inspected Fusion behavior | Important qualification |
| --- | --- | --- |
| [syuntoku14/fusion2urdf, 21fd305](https://github.com/syuntoku14/fusion2urdf/tree/21fd3050752c2baa62dfdc1bfa05ec51fc472122) | Flat occurrence/joint extraction, rigid/revolute/slider branches, BRep copying and STL export. | [Input extraction](https://github.com/syuntoku14/fusion2urdf/blob/21fd3050752c2baa62dfdc1bfa05ec51fc472122/URDF_Exporter/core/Joint.py) is root-local. [Copying](https://github.com/syuntoku14/fusion2urdf/blob/21fd3050752c2baa62dfdc1bfa05ec51fc472122/URDF_Exporter/utils/utils.py) renames source components. |
| [dheena2k2/fusion2urdf-ros2, 8e883d2](https://github.com/dheena2k2/fusion2urdf-ros2/tree/8e883d2d66529a77c67edc28d29f2c3ba96169f6) | ROS 2 packaging around the upstream Fusion input flow. | Its [joint module](https://github.com/dheena2k2/fusion2urdf-ros2/blob/8e883d2d66529a77c67edc28d29f2c3ba96169f6/URDF_Exporter_Ros2/core/Joint.py) shares the upstream blob. Generated-package lint tests do not establish broader CAD input support. |
| [haoyan-dev/fusion2urdf, dc7a2e6](https://github.com/haoyan-dev/fusion2urdf/tree/dc7a2e657ccf4f44b68e52e157067958f3de99c7) | Recursive collections, explicit joint-origin conventions, and transform composition. | [Joint extraction](https://github.com/haoyan-dev/fusion2urdf/blob/dc7a2e657ccf4f44b68e52e157067958f3de99c7/URDF_Exporter/core/Joint.py) requires JointOrigin endpoints. [Name formatting](https://github.com/haoyan-dev/fusion2urdf/blob/dc7a2e657ccf4f44b68e52e157067958f3de99c7/URDF_Exporter/utils/utils.py) strips instance suffixes, requiring repeated-instance collision tests. |
| [Adriaeik/fusion2URDF, 4411ffb](https://github.com/Adriaeik/fusion2URDF/tree/4411ffbdffc0364dc6f5922c40b3e14fd7b642f8) | Nested input, explicit rigid-group merging, appearance extraction, and world-frame tests. | [Rigid merging](https://github.com/Adriaeik/fusion2URDF/blob/4411ffbdffc0364dc6f5922c40b3e14fd7b642f8/core/robot_model.py#L544-L765) has different semantics from keeping fixed-linked members. [Visual-failure tests](https://github.com/Adriaeik/fusion2URDF/blob/4411ffbdffc0364dc6f5922c40b3e14fd7b642f8/tests/test_export_pipeline.py#L2947-L3037) accept inertial-only output in some failures. |
| [cadop/fusion360descriptor, 4d28621](https://github.com/cadop/fusion360descriptor/tree/4d28621a97e4e2c4be9a9dbecdfefb231813e9da) | Grounded/configured roots, outward endpoint orientation, rigid-group fixed edges, explicit merged links, and body visuals. | [Joint extraction](https://github.com/cadop/fusion360descriptor/blob/4d28621a97e4e2c4be9a9dbecdfefb231813e9da/Descriptor/core/parser.py#L385-L553) resets driven positions and reads ordinary allJoints. As-built support proposed in an unmerged change is not part of this revision. |
| [ACDC4Robot/Fusion360, f1152d7](https://github.com/ACDC4Robot/Fusion360/tree/f1152d7b6fb067d25f82885f86ae7f79bbb45b5e) | Ordinary/as-built discovery and non-overlapping nested rigid subtrees. | The [ordinary URDF writer](https://github.com/ACDC4Robot/Fusion360/blob/f1152d7b6fb067d25f82885f86ae7f79bbb45b5e/Add-IN/ACDC4Robot/core/write.py#L70-L91) calls module functions. Correct shared helpers or tests for another output do not establish that this active path uses them. |
| [runtimerobotics/fusion360-urdf-ros2, e317c5e](https://github.com/runtimerobotics/fusion360-urdf-ros2/tree/e317c5e00e1044996d19d5e4ff97431ed8816245) | Flat assemblies and ordinary joints with ROS 2 package output. | [Extraction](https://github.com/runtimerobotics/fusion360-urdf-ros2/blob/e317c5e00e1044996d19d5e4ff97431ed8816245/Fusion_URDF_Exporter_ROS2/core/Joint.py#L121-L231) remains root-local. [Utilities](https://github.com/runtimerobotics/fusion360-urdf-ros2/blob/e317c5e00e1044996d19d5e4ff97431ed8816245/Fusion_URDF_Exporter_ROS2/utils/utils.py#L16-L128) mutate copied/source component names and round tiny inertia. |
| [david-dorf/ExportURDF, bbf2af2](https://github.com/david-dorf/ExportURDF/tree/bbf2af2a19e73052aee16b37270ef7318e1e579e) | Fusion ordinary/as-built input. Only its Fusion adapter is included here. | [Origin extraction](https://github.com/david-dorf/ExportURDF/blob/bbf2af2a19e73052aee16b37270ef7318e1e579e/Fusion360/FusionURDF.py#L209-L224) reads null rigid-as-built geometry. [Joint/inertia emission](https://github.com/david-dorf/ExportURDF/blob/bbf2af2a19e73052aee16b37270ef7318e1e579e/Fusion360/FusionURDF.py#L227-L351) motivates branching and SI-unit regression cases. |
| [newtonjeri/fusion2urdf-master, 5238bd6](https://github.com/newtonjeri/fusion2urdf-master/tree/5238bd60e90d3996f06d18f78463c624c4f7d617) | Nested geometry grouping, temporary direct-design staging, appearance, and small-inertia precision. | [Input joints](https://github.com/newtonjeri/fusion2urdf-master/blob/5238bd60e90d3996f06d18f78463c624c4f7d617/URDF_Exporter/core/Joint.py#L123-L255) still use root.joints. A [direct staging document](https://github.com/newtonjeri/fusion2urdf-master/blob/5238bd60e90d3996f06d18f78463c624c4f7d617/URDF_Exporter/utils/utils.py#L24-L117) does not prove direct-mode input handling. |

The upstream, dheena2k2, and haoyan example directories share the same Git tree.
They are inherited assets, not three independent model-validation results.
Missing tests in a repository establish neither runtime failure nor success.

## History is not an export workaround

The [upstream README](https://github.com/syuntoku14/fusion2urdf/blob/21fd3050752c2baa62dfdc1bfa05ec51fc472122/README.md#L84-L175)
and the
[dheena2k2 README](https://github.com/dheena2k2/fusion2urdf-ros2/blob/8e883d2d66529a77c67edc28d29f2c3ba96169f6/README.md#L52-L143)
recommend turning Capture Design History off in some cases. Those statements
are documentation advice, not tests establishing equivalent exports.

Autodesk's [designType contract](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Design_designType.htm)
states that converting to direct design removes the timeline and history.
The export path here must not perform that conversion or require it as a fix.
The optional
[ACDC flatten helper](https://github.com/ACDC4Robot/Fusion360/blob/f1152d7b6fb067d25f82885f86ae7f79bbb45b5e/Add-IN/ACDC4Robot/tools/FlattenOccs.py#L11-L111)
does change design type and delete emptied descendants. It is separate from
that project's ordinary URDF command and is not a harmless exporter setting.

## What the comparison requires locally

The source audit yields these acceptance categories:

- Current/history-on and direct/history-off input, with unchanged source state.
- Ordinary and as-built joints, including null-geometry rigid attachments.
- Nested and repeated contexts with distinct names, frames, and mesh targets.
- Grounded/configured roots and correctly reversed endpoint directions.
- Direct-body ownership, root imports, rigid groups, and visibility-consistent mass.
- Unit and frame invariants for meshes, CoM, inertia, limits, and custom axes.
- Coupling, bodyless frames, mesh-only imports, and generalized-joint policies.
- Transactional output, explicit mesh failure, cancellation, and existing-output preservation.

[The local use-case matrix](fusion-coverage.md) binds those categories to
executable evidence and names missing input support. A successful rejection
test is not an implemented-input pass. Neither a source audit nor our own
fixture tests establishes that we outperform every exporter on real Fusion
assemblies. Native comparison needs the same input designs exported by each
tool and checked against the same independent expectations.

## Autodesk contracts checked

- [Component.allJoints](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Component_allJoints.htm)
  and [allAsBuiltJoints](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Component_allAsBuiltJoints.htm)
  return nested joints in root assembly context.
- [Current joint transforms](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Joint_geometryTwoTransform.htm)
  and [as-built transforms](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_AsBuiltJoint_transform.htm)
  address unavailable original geometry.
- [Rigid as-built geometry](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_AsBuiltJoint_geometry.htm)
  legitimately returns null.
- [Occurrence.bRepBodies](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Occurrence_bRepBodies.htm)
  supplies component bodies as proxies in the occurrence's assembly context.
- [Rigid groups](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Component_allRigidGroups.htm)
  are a separate native constraint collection.
- [STL export units](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_STLExportOptions_unitType.htm)
  can be selected explicitly starting with Fusion May 2025.
- [World-coordinate moments](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_PhysicalProperties_getXYZMomentsOfInertia.htm)
  are returned in kilograms times centimeters squared.
- [Ground To Parent](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Occurrence_isGroundToParent.htm)
  is a native fixed relationship, distinct from merely choosing a root.
- [Appearance properties](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_Appearance_appearanceProperties.htm)
  predate the July 2026
  [primary color property](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_Appearance_color.htm).
  Native [Color.opacity](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_Color_opacity.htm)
  is a byte channel.

These contracts are not a completed live Fusion test. Full revisions and
source paths are retained in [the census](fusion-repositories.json).
