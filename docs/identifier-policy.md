# Exported identifier policy

Fusion source names, occurrence paths, joint names, history, and external links
are inputs. Export never renames them to make URDF identifiers work.

## Identity and aliases

The complete root-context occurrence path identifies a link source. Joint source
identity is its root-context assembly path followed by `+` and its original joint
name, or just its name for a root-owned joint. Native repeated endpoints still
require an unambiguous assembly context. Export aliases do not resolve endpoints.
Duplicate source identities and ambiguous endpoint contexts remain errors.

Root selection uses source identities before alias allocation. Set
`root_occurrence` to the exact complete path, including revision and instance
suffixes, for example `base_link v7:1`. The selected source always becomes
`base_link`. The exact `design_root` selector retains its existing meaning.

`URDF_Exporter/utils/names.py` owns the shared link/joint allocation policy.

1. Sanitize each source identity to ASCII letters, digits, and underscores.
   Invalid character runs become one underscore. Strip edge underscores, use
   `link` for an empty result, and prefix leading digits with `link_`.
2. Keep an ordinary unambiguous, unreserved sanitized name unchanged.
3. Compare sanitized aliases with ASCII case folding so distinct source names
   such as `Arm:1` and `arm:1` cannot overwrite STL or component files on
   case-insensitive filesystems. Disambiguate every member of a collision. Append `__` and the
   first 12 hexadecimal digits of SHA-256 over the exact UTF-8 source identity.
   Canonically distinct Unicode spellings remain distinct inputs.
4. Reserve `base_link` for the selected root. Reserve `rigid_attachment_` and
   `fusion_helper_` prefixes for generated relationships and future helper frames.
   These output reservations and candidate checks are also case-insensitive;
   source identities and endpoint resolution remain case-sensitive.
   User sources with those prefixes receive `source_` before their hashed alias.
5. Protect all unchanged ordinary aliases first. Allocate remaining aliases in
   source-identity lexical order. If a hashed candidate is already occupied,
   append underscores until free. This covers a user deliberately naming a source
   like another source's hash alias, as well as truncated-hash collisions.

Allocation depends on the complete selected identity set, not collection order.
Reordering Fusion collections therefore does not swap aliases or their geometry.
Adding or renaming a colliding source can change aliases. Do not treat aliases as
permanent document identifiers across arbitrary source edits.

For example, `arm-pin:1` and `arm pin:1` become
`arm_pin_1__8c4963c33bb2` and `arm_pin_1__7574e42ac5bd`. The same aliases key direct
body selection, physics, joint endpoints, component URDFs, and STL filenames.
Generated rigid attachments use `rigid_attachment_` plus the final child alias.

## Package provenance and configuration

Packages exported through `export_design` retain `source_identifiers` in
`config/robot_data.yaml`. `links` maps output link aliases to exact source paths,
including the root alias. `joints` maps extracted Fusion joint aliases to their
original contextual identities before topology reduction. Excluded constraints
do not appear in this mapping. Generated rigid attachments instead retain
their existing `rigid_constraints` provenance. Original fixed constraints can be
represented by generated attachments, and closing joints remain in sidecar
metadata rather than the URDF tree. The allocation receipt is provenance, not a
second endpoint lookup table.

`joint_limits` selectors remain **exact exported joint names**. They are not raw
Fusion names, sanitized-name shortcuts, wildcards, or identity selectors. For
colliding names, export without overrides first and inspect `source_identifiers`
and `joints`, then use the complete alias, including its hash suffix. An absent
selector fails with the available aliases. Fixed/generated fixed joints still
reject effort and velocity overrides. Collision disambiguation never lets a
selector choose which source keeps an unsuffixed ambiguous name.

## Evidence and limits

`tests/test_issue_aliases.py` drives public export with Unicode, punctuation,
repeated native instances in colliding Unicode contexts, revision-suffixed exact
roots, reserved namespaces, and a deliberately competing hash-like source name.
Its expected aliases and per-instance geometry, mass, central inertia, and zero
and moved FK values are independent literals. Ordinary/as-built collections and
their reversed order are covered. Duplicate identities, hidden/ambiguous native
endpoints, and forced mesh failure remain no-publication checks.

`tests/test_alias_frame_combinations.py` also checks combined empty-frame and
physical aliases, nonzero geometry association, case-folded package filenames,
and case-variant reserved names. This is a portable-name policy check, not native
Windows or macOS execution.

These are offline Fusion API doubles, not exports of the original reported CAD
documents. They do not verify native Fusion mesh coordinates, external-document
revision behavior, ROS 2 runtime, or physical dynamics. Existing ASCII aliases
and their existing platform path-length and filesystem constraints are not a
portable filesystem encoding guarantee.
