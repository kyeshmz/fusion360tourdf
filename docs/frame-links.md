# Physical links and reference frames

The exporter represents a selected empty reference context as a frame-only URDF
link. It does not give the frame a fabricated mass, inertia, visual, collision,
or STL file. Physical links still require finite positive mass and admissible
inertia. They use the same selected direct bodies for physical properties and
meshes, once per occurrence context.

## Selection

Visible physical sources remain selected as before. Empty contexts become links
only when an active joint or an explicit rigid-group member references them,
`root_occurrence` selects them, or an actual Ground To Parent relationship needs
them. Unused empty assemblies and suppressed constraints do not add links.

An exact `root_occurrence` may identify an empty occurrence or `design_root`.
The selected source is named `base_link`. A legitimate `None` joint endpoint
identifies the actual `design_root` context, even when another physical source
has the `base_link` name. It never identifies an arbitrary nearby physical link
or a nested assembly parent.

The joint and its non-`None` endpoints must expose readable, valid `isValid`
metadata. Both-`None` endpoints, unresolved identities, missing required validity
metadata, hidden required sources, and unreadable required frames fail before
output publication. Intentionally excluded constraints with no selected
endpoints can still be reported as excluded. An unresolved reference is not
treated as an intentionally excluded source.

## Constraints, not containment

An empty parent does not weld its children. A child needs an actual joint,
explicit rigid membership, or grounding relationship to become connected.
`RigidGroup.occurrences` is the membership authority. A parent occurrence in that
list is not proof that all descendants belong to the group. Fusion's group
creation API has an explicit `includeChildren` argument, so the exporter must
not reconstruct that choice from containment.

Ground To Parent can connect a physical child to an empty moving parent frame.
Proven fixed ancestry can still be reduced through unreferenced intermediate
contexts. A referenced or configured frame remains represented explicitly.
Grounding a container never grounds an otherwise unconstrained descendant.
A free selected physical root is not silently rebound to the world anchor.

Link-coordinate conventions are unchanged. The root link uses the design
coordinate system. Other link origins use the incoming joint's current pivot,
with the selected occurrence's orientation. A frame-only source uses this same
kinematic convention; its occurrence identity does not imply an extra weld or
an additional transform at the occurrence translation.

## Maintainer contract

`LinkSource.kind` is either `physical` or `frame`. Physical sources own a nonempty
body tuple. Frame sources own an empty tuple. Construction enforces this
distinction. The registry owns source paths and context resolution.

Link-data entries carry `kind` and `source_path`. A frame entry has no physical
or geometry fields. The writer accepts omitted `kind` as `physical` for existing
programmatic callers, but rejects unknown kinds and physical data attached to a
frame. Every link participates in topology validation and gets a component URDF.
Only physical links enter the mesh callback's target mapping. A frame-only
package calls that callback with an empty mapping.

`export_design` discovers a complete joint-record snapshot once and passes it to
source selection and joint extraction. `Joint.make_joints_dict` does the same
when called without a snapshot. Direct body/mesh-only users of `link_sources`
can pass `joint_records` when joint-driven frame selection is required; the
default does not make standalone mesh extraction depend on constraint APIs.

`config/robot_data.yaml` includes `link_sources`, mapping exported names to
`kind` and original context `path`. Frame entries have no mesh URI. Physical
mesh URIs remain package-relative and survive package relocation.

## Evidence and boundaries

Run `python3 -m unittest tests.test_issue_bodyless -v` for analytic API-double
exports through the public entry point. The main scene combines a bodyless
design root, an internal empty frame, rotated 3D joints, a slider, and two
physical bodies. Independent Rodrigues rotations check zero and nonzero poses.
The tests also cover as-built constraints, configured empty roots, explicit
rigid membership, grounding, resource relocation, and unchanged source state
after success and mesh-export failure.

These are offline API-shape tests, not original issue CAD or a live Fusion
receipt. Current Autodesk endpoint documentation does not establish a universal
rule distinguishing every historical missing reference from a legitimate null
root endpoint. No such native guarantee is claimed. This change does not edit
the timeline, disable design history, or clear grounding flags.

Verification uses explicit independent `link_kinds` expectations or the
exporter's `link_sources` receipt. Missing physical geometry is never guessed
to be an intentional frame. Every visual/collision mesh resource is checked.
Strict offline lint accepts `--link-kinds config/robot_data.yaml`; bare links
without a declaration still warn. The opt-in generated `--fixture=frames`
retains the existing three moving joints and literal zero/moved poses with an
empty root and internal arm frame. Default three-case fixtures are unchanged.
Real Fusion, installed ROS 2 loading/motion, and Blender checks remain separate
gates. Copying the package
into an installation-shaped directory verifies relocation, not an ament build.

Official API references consulted on October 20, 2026:

- [Joint.occurrenceOne](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Joint_occurrenceOne.htm).
- [Joint.occurrenceTwo](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Joint_occurrenceTwo.htm).
- [Occurrence.isValid](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Occurrence_isValid.htm).
- [Occurrence.isGroundToParent](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Occurrence_isGroundToParent.htm).
- [RigidGroup.occurrences](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_RigidGroup_occurrences.htm).
- [RigidGroups.add](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_RigidGroups_add.htm).
