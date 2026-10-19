# Verification results

## October 18, 2026: native Fusion and actual-export E2E

A dirty working tree based on `3dc5306b9df4afc398ee13929292350bd96273d1` was tested
through the real Fusion MCP. Final native execution used Fusion `2705.1.30` and Python `3.14.0`.
Reports retain exact source hashes and loaded-code comparisons; this is not a claim about the unchanged Git commit.

The local bundle is `artifacts/rotary-slide-demo-20261018/` (ignored by Git, not distributed).
It includes `assembly.f3d`, the package, `native.json`, independent `expected.json`,
and a verbatim copy of the end-to-end evidence under `e2e/`.
The full run, including failed candidates, remains at
`/private/var/folders/q6/tz5lqjtn66q2cw4jg9qtfcj00000gn/T/opencode/fusion360tourdf-e2e-LVpJ8i/`.

| Check | Observed result | Local receipt within the bundle |
| --- | --- | --- |
| Offline regressions | 190 tests passed | `evidence/offline.log` |
| Native fixtures | Eleven cases, 1,742 assertions; source preservation and cleanup passed | `evidence/native-regressions.json` |
| Native demo | Six links and two moving joints; 449 assertions and 24 retained entity identities passed | `native.json` |
| Independent loader | Actual CAD meshes and zero/moved FK and bounds passed | `e2e/loaders.json` |
| Negative controls | Wrong axis, 100 mm mesh shift, and missing mesh rejected | `e2e/summary.json` |
| Blender 5.2.2 LTS | Twelve visual/collision objects per pose; both poses passed and scene saved | `e2e/blender/` |
| Actual export, Humble | Native package built/installed; 13 installed assets matched hashes; display, controllers, trajectory, feedback, and six-link TF passed | `e2e/humble/docker-humble-NrmxDG/ros-humble-pm28fY/ros.json` |
| Actual export, Jazzy | The same actual-export checks passed | `e2e/jazzy/docker-jazzy-Z2MtHq/ros-jazzy-5BdLBi/ros.json` |
| Existing cross-tool suite | Six integration tests and three Blender variants passed; urdfdom ran in Docker | `evidence/cross-tools/` |
| Cleanup | No owned test container or tagged image remained | `evidence/cleanup.json` |

Both actual-export ROS reports identify `rmw_cyclonedds_cpp`, active joint-state/trajectory
controllers, trajectory error code zero, and feedback matching `turn = pi/4` and
`boom_1_extend = 0.025` meters. They are labeled `installed-native-fusion-export`, not generated fixtures.
The generated-fixture Docker suites also passed on both distributions.

Native execution exposed component-local proxy STL output. Production export now explicitly
exports native bodies and transforms their triangles and normals into root context without
moving source geometry. Verifier fixes cover immutable root names, parametric-only parameter
collections, unavailable proxy tokens, and contextual joint names. Direct fixed fixtures use
grounding because the SDK rejects as-built-joint creation in direct mode.

Demo authoring captures an explicitly placed nested copy before constraining it. Geometry-free
rigid sensor joints use the owning carriage anchor; distinct sensor placement is preserved in
visual/collision origins and independently checked bounds. An earlier candidate failed an
overly strict component-frame assumption in the downstream oracle. Its receipt is retained;
geometry assertions and negative controls were not relaxed.

The eleven-case run used fresh exporter imports. The final demo-only oracle change was
explicitly reloaded without restarting Fusion; loaded-code checks matched disk. Production
exporter and native-regression helper hashes were unchanged between those runs.

This proves the exercised pipeline, not every Fusion input, competing exporters, native nonzero
actuation, physics, hardware, or GUI RViz rendering. See the [reproduction guide](demo-workflow.md).

## October 17, 2026: historical offline and runtime evidence

The Fusion-only expansion was checked on October 17, 2026 in an uncommitted
working tree based on `b637eed`. The latest local run passed the offline,
cross-tool, and Docker ROS 2 checks below. **Full Fusion coverage and universal
superiority are not verified.** No native Fusion application case or hosted CI
job was executed.

### Historical local evidence

Detailed evidence is retained under
`/tmp/opencode/fusion360tourdf-fusion-proof-hSWDaJ/`. These are local paths,
not files distributed with the repository.
`delivery/summary.json` indexes the observed results, and
`delivery/inputs-sha256.json` identifies the exact tested code and case catalog.
The canonical append-only decision trail is
`/tmp/opencode/fusion360tourdf-AAMjrw/decisions.tsv`.

| Check | Observed result | Local receipt |
| --- | --- | --- |
| Dependency-free regressions | 175 tests passed. | `delivery-offline.log` |
| Dependency-free regressions in Docker Jazzy | The same 175 tests passed on Python 3.12 with a read-only checkout mount. | `delivery-offline-docker.log` |
| Declared Fusion case checks | All runnable checks passed, including named rejection checks. | `delivery/catalog/fusion-coverage.json` |
| Independent loader suite | Six tests passed. | `delivery-cross-tools.log` |
| Shared Fusion export boundary | API-double inputs produced real packages and closed STL solids. yourdfpy checked rigid-group and Ground To Parent attachments at zero and nonzero positions. | `delivery/cross-tools/` |
| xacro, urdfdom and yourdfpy | Articulated, fixed-only and primitive-box variants expanded, parsed, loaded meshes, and matched literal FK and bounds. | `delivery/cross-tools/cross-tools-8R3tb1/` |
| Blender 5.0.1 | All three variants passed placement checks and saved inspection scenes. | `delivery/cross-tools/cross-tools-8R3tb1/blender-*/` |
| Docker Humble, Python 3.10 | Three packages built/installed; display, controllers, trajectory, feedback and TF passed. | `delivery/docker/docker-humble-ErrQPO/ros-humble-i9Ge7E/ros.json` |
| Docker Jazzy, Python 3.12 | The same installed-package and motion checks passed. | `delivery/docker/docker-jazzy-I4PVnf/ros-jazzy-PGV7Ee/ros.json` |
| Cleanup | No owned verification container or tagged image remained. | `delivery/cleanup.json` |

Both Docker reports identify `rmw_cyclonedds_cpp`. The joint-state broadcaster
and trajectory controller were active. The action returned error code zero.
Observed feedback matched shoulder pi/2, slider 0.04 meters, and spin minus pi/2.
The numeric TF matrices matched their independent fixture expectations.

Humble's default FastDDS previously aborted during a controller service reply.
UDP-only FastDDS was unreliable across fresh runs. CycloneDDS passed without
weaker assertions or a production middleware dependency. The lower-level
FastDDS failure remains unresolved. Generated packages do not select an RMW.

## Coverage is not a test-count ranking

The [case catalog](fusion-use-cases.json) contains 26 declared families. Its
implementation states are 19 supported, one partial grounding case, one
URDF-tree-limited case, and five unimplemented inputs/fidelity cases. These
are declarations tied to executable evidence, not native certification.

Twenty-five entries have passing offline checks. Some of those checks prove
safe rejection, not input support. The remaining body/face appearance fidelity
gap has no input-support test. The machine report explicitly retains
`native_fusion_verified: false` and `full_scope_verified: false`.
`--require-complete` returned exit code 2 despite passing selected offline
checks. See `require-complete.log`.

All nine in-scope GitHub exporters were source-audited at pinned revisions.
One pinned source artifact per repository was also fetched and fingerprinted
in `pinned-source-receipts.json`. No competing exporter ran inside Fusion.
The [comparison](related-work.md) therefore records code paths and limitations,
not a head-to-head native-runtime ranking.

## Negative evidence and independent review

- History regressions were red before fixes. Current transforms now take
  precedence over obsolete geometry; rigid as-built joints accept legitimate
  null geometry without timeline access.
- Endpoint review found wrong-instance publication after hiding a repeated
  occurrence. SourceRegistry now retains unfiltered root-context identity
  before selection. Independent review replayed all 45 before/after actor
  cases and checked owner-order invariance. Receipts are under
  `/tmp/opencode/context-registry-repair/run-4j4Z70/`.
- Grounding review replayed six successful exports and four named rejections,
  then checked 30 additional exports with independent 3D FK and occurrence
  permutations. Receipts are under `/tmp/opencode/grounding-independent-95W4k5/`.
- Appearance review checked actual Link-to-XML behavior. Selected green bodies
  no longer inherit an unrelated red component material. Unsupported explicit
  overrides use silver instead of falling through to a different material.
  Byte RGB/opacity and legacy appearance properties have literal regressions.
  Receipts are under `/tmp/opencode/fusion-appearance-selection-askygG/`.
- Loader mutations reject changed axes, shifted meshes and missing meshes.
  A generic expanded-export test also caught and fixed integer zero values
  passed to yourdfpy's float configuration API.
- Native-verifier review found bounds-only checks accepted degenerate facets
  and compensated hinge offsets. The helper now rejects both, checks closed
  analytic-box volume, axis lines, nonzero FK and COM, editable history fields,
  retained entity identity, and loaded-code provenance. Thirty offline helper
  tests passed. Current retained literal artifacts are under
  `native-proof-current-u8a6e5rs/`.

Reviews used separate sessions on the inherited model. They are not
cross-family corroboration. Code hashes and the decision trail identify
which source snapshot each receipt covers.

## Remaining native Fusion coverage limits

The [native verifier](../tools/verify_fusion/README.md) has executed evidence for eleven
owned-document cases, including separate parametric/direct fixtures and editable history.
These receipts do not promote the complete declared input matrix to native certification.
The coverage runner intentionally retains its separate offline evidence level.

The native helper's mesh checks target disjoint analytic boxes, not arbitrary
CAD geometry. Its original-reference invalidation case remains
`not_exercised`; it does not actuate native joints at nonzero values. The helper
evaluates exported URDF motion numerically instead. Proxy mesh placement, SDK entity equality,
grounding, and source-history preservation were exercised for these fixtures/demo only.
General edited-topology behavior, original-reference invalidation, broader grounding cases,
and appearance/fidelity policies still need separate native inputs.

Motion-link/mimic conversion, mesh-only physical/export policy, general bodyless
frames, active generalized-joint decomposition, and per-body/face materials or
textures remain in-scope gaps. Grounding requires a representable physical
parent or proven fixed ancestry/world anchor. Closure metadata does not enforce
a closed-chain mechanism. Passing the current tests does not close these gaps.

No RViz GUI, physical hardware, Gazebo, dynamics, or hosted-CI run is covered.
Repeat the commands in [the testing guide](testing.md), retain the resulting
reports, and inspect their evidence level before marking a new revision verified.
