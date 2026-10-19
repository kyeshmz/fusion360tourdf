# Verification results

## October 19, 2026: pulled installer/viewer changes and component showcase

The macOS integration run fast-forwarded to `7bf5a13`, retained the shared `install.sh`/`install.cmd`/`tools/install_fusion.py` implementation, and combined its separate `viewers/` layout with plain assembly and six per-component URDF files. A recovery stash and external copy preserved the pending work during conflict resolution; the other contributor’s installer and portability checks were not replaced by the earlier duplicate prototype.

Fresh native demo execution passed 449 assertions after explicitly reloading the changed pure writer. The source snapshots and retained entities remained unchanged. The earlier eleven-case cold-import run passed before the pull and is a different source snapshot, not certification of this merged tree.

The actual merged-layout export passed independent loading, negative controls, Blender, Humble/Jazzy install-space checks, component urdfdom parsing, controller activation, trajectory feedback, and six-link transforms. Each of the six component files also loaded its shared visual/collision meshes independently. The expanded cross-tool suite passed seven loader integration tests and three Blender variants.

Real RViz 2 GUI capture ran under Xvfb with software OpenGL on ROS 2 Jazzy/CycloneDDS. It recorded zero and commanded poses after feedback/transform checks and a successful mock-controller trajectory. Numbered labels and leader lines stayed locked to the matching URDF frames. The chaptered walkthrough combines those actual GUI frames with illustrated instructions, a tested demo goal command, and the optional single-file installer; it has no audio narration and is not a continuous recording of Fusion menu clicks. Its published receipt records duration, chapter timing, and hashes.

The expanded offline suite ran 275 tests on both host CPython 3.11.11 and 3.13.2: 262 passed and 13 platform-specific cases were skipped in each run. Single-file installer regressions exercised execution away from the checkout, caller-relative paths, dry-run, update refusal, retained config/backups, license inclusion, and POSIX execution. Native Windows CMD/archive execution is configured in CI, not claimed as locally run. Python 3.12/3.14 hosted matrix jobs likewise require their own results.

The same 275-test suite also passed with a read-only checkout mount in Docker Jazzy/Python 3.12: 263 passed and 12 platform-specific cases were skipped. This is local Linux evidence, not a hosted-job result. The archive returned exit code 1 under a separate Python 3.10.16 probe before payload extraction, correctly reporting its 3.11 minimum. The final 122.65-second, fourteen-chapter MP4 decoded fully and its instruction cards/labeled screenshots were visually reviewed.

The license audit checked the actual license files at all nine pinned source revisions. Seven distinct MIT texts are retained; repeated upstream notices share one copy. EOF newlines were normalized in two files without altering their copyright/permission/disclaimer text. Notices distinguish inherited/adapted guidance from comparison-only sources and travel with installed exporter files and the `.pyz`.

The ready example is [the native package and component catalog](../Example/rotary_slide_README.md). Published media, raw source-window captures, source recording, and hash receipts live under [`docs/media`](media/README.md). The combined-version source-credit ledger is [changelog.md](../changelog.md).

Detailed run receipts remain at `/private/var/folders/q6/tz5lqjtn66q2cw4jg9qtfcj00000gn/T/opencode/fusion360tourdf-showcase-qawpBv/`: `rotary-slide-5m5j8l7c/native.json`, `merged-e2e/export-e2e-_zlqdr1r/summary.json`, `merged-gui/rviz-v6gbkW/capture/rviz.json`, `component-loaders.json`, `cross-tools.log`, `offline-publish-311.log`, `offline-publish-313.log`, `offline-publish-312-docker.log`, and `single-file-proof.json`. These receipts identify dirty source snapshots; they do not establish universal Fusion coverage, real-hardware operation, or a native Windows installer pass.

## October 19, 2026: installer/native-workflow integration

The publication merge combines installer/viewer work from `d54d4c9` with the
native export and documentation work in `ecbdd6d`. Both histories are retained.
Local receipts are under `/var/tmp/opencode/fusion360tourdf-publish-OAeUuE/`.

- The merged offline suite ran 261 tests on the Linux host and in Docker
  Jazzy. Each run passed 249 tests and skipped 12 Windows-only checks. See
  `merged-offline.log` and `merged-offline-docker.log`.
- Six independent loader checks and three Blender variants passed. See
  `cross-tools.log`. Both generated-package Docker suites also passed installed
  display discovery, controller activation, trajectory result zero, feedback,
  and TF. See `runtime-receipts.json`.
- Layout regressions keep newly generated packages strict about `viewers/`.
  An explicit validated native source bundle may retain the earlier
  `launch/display.launch.py` and `config/display.rviz` paths. Its assets and
  recorded hashes are not rewritten. These compatibility tests use offline
  receipt doubles, not new native Fusion evidence.
- The native gallery and the dated October 18 evidence below were preserved.
  They describe their original source snapshot, not a native rerun of this
  merge. Native Windows/macOS installer execution and this merge's hosted CI
  remain unverified by the local checks.

The following baseline sections preserve receipts from separate source
snapshots. The October 18 native record below is retained
verbatim from a run on another Mac. It is not a native run by this Linux
publication session. The shell and installer/viewer checks cover independent
local baselines. Their test counts are not cumulative.

## Shell installer entry points

The later usability change adds root `install.sh` and `install.cmd` entry
points. Evidence is under
`/var/tmp/opencode/fusion360tourdf-shell-install-55S1rE/`. New receipts use the
disk-backed directory because the earlier `/tmp` scratch location hit its quota.

| Check | Observed result | Local receipt |
| --- | --- | --- |
| Real Bash command | Help, executable invocation, caller-relative paths, dry-run, exact installed bytes, repeats, preserved config/backups, and exit codes 1 and 2 passed. | `bash-cli-final.log`, `bash-real-cli-yskjse5j/receipt.json` |
| Interpreter selection | Too-old/missing interpreter cases, exact argv, one backend dispatch, no-download environment, and caller-environment preservation passed. | `function-shadow-green.log` |
| Exported-function regression | A shell function could previously bypass the external executable's version check. The launcher now probes and executes the same resolved external pathname. The regression changed from red to green. | `function-shadow-red.log`, `function-shadow-green.log` |
| Full offline suite | 240 tests ran on the host. 228 passed and 12 Windows-only tests skipped. | `offline-final.log` |
| Offline suite in Docker Jazzy | The same counts passed with a read-only checkout mount. | `offline-docker-final.log` |
| Linked temporary paths | Backend and launcher suites passed through a symlinked temporary root. | `alias-suite-aj42qp01/tests.log` |

The twelve local skips are eleven native-CMD checks and the existing Windows
junction check. They are not passing native-platform evidence. The Windows
and macOS installer jobs now include both installer test modules, but those
hosted jobs have not run for this change. Actual execution used Linux Bash
5.3.9. Bash 3.2 compatibility is source inference until tested on macOS.

Independent review accepted the wrapper contracts and checked additional
punctuation and CDPATH cases. A separate targeted review reran the final
external-executable regression and focused suites. Both reviews used inherited
same-model sessions, not cross-family review. No comments were deleted.

`frozen-inputs.json` identifies the unchanged backend, exporter, and viewer
runtime inputs. The prior ROS and Blender receipts below remain their evidence;
those suites were not rerun for this wrapper-only change. The launchers do not
change copying, configuration, backup, or rollback policy. These local wrapper
checks did not run the native Fusion GUI or change a real Autodesk
installation. No commit or push was performed during those wrapper checks.
This limit does not negate the separate October 18 native evidence below.

## Installer and viewer layout baseline

The earlier installer/viewer change was tested in a dirty working tree based
on `3dc5306`, before the shell entry points above were added.
Evidence is under `/tmp/opencode/fusion360tourdf-install-viewers-rl286j/`.

| Check | Observed result | Local receipt |
| --- | --- | --- |
| Full offline suite on the host | 210 tests ran. 209 passed and one native-Windows junction test was skipped. | `offline-final.log` |
| Full offline suite in Docker Jazzy | The same results passed on Python 3.12 with a read-only checkout mount. | `offline-docker-final.log` |
| Real installer CLI | Fresh copy, repeated install, refused replacement, config-preserving update, complete backup, and isolated installed-writer import passed in a temporary Unicode path. | `install-cli-final.log`, `installer-cli-_tptvpcl/receipt.json` |
| Installer portability | Actual cp1252 output preserved success/update results and refusal errors. UTF-8 paths stayed unchanged. The full installer suite passed through a linked temporary root. | `portability-cli-final.log`, `portability-cli-526p4gqx/receipt.json` |
| Existing Wine location | Dry-run selected the real Scripts parent without creating an exporter there. | `wine-dry-run.log`, `native-location-before.json` |
| Viewer ZIP commands | Viewer-only and full-package archives matched the generated bytes for articulated, fixed, and box variants. | `viewer-zip-cli.log`, `viewer-zip-cli-cjtx30tq/receipt.json` |
| Package and viewer tests | All 34 scoped checks passed, including archive relocation and failure handling. | `parent-viewer-tests.log` |
| Loaders and Blender | Six loader tests and all three Blender variants passed. | `cross-tools.log`, `cross-tools/cross-tools-eaEw4c/` |
| Docker Humble | Installed viewer discovery, headless display, active controllers, trajectory result zero, feedback, and TF passed. | `docker/docker-humble-3a3vzZ/ros-humble-lNLghH/ros.json` |
| Docker Jazzy | The same installed-package and motion checks passed. | `docker/docker-jazzy-obgyWo/ros-jazzy-UFxuJU/ros.json` |

Both display and control launch files reference `viewers/display.rviz`.
The installed-share checks require exactly one display launcher at
`viewers/display.launch.py` and no legacy viewer copies. The normal package
launch command is unchanged. `installed-rviz-check.json` records parsed RViz
configuration checks; it does not certify GUI rendering.

Additional `--symlink-install` probes built all three packages on both ROS
distributions and passed relocated launch discovery. They stopped at the
strict mesh guard because resolved meshes pointed back into the source
workspace. Those probes did not reach controller verification and are not
passing full-runtime receipts. See `symlink-humble-1gsffur0/run.log` and
`symlink-jazzy-q_clo0vs/run.log`. The README now recommends the normal copy
install used by the successful Docker suites.

Installer tests use temporary destinations. Default-path tests supply Windows
and macOS environment values but run on Linux. The workflow defines installer
jobs for both native hosts, but those hosted jobs have not run for this change.
Windows reparse-point behavior and native Fusion registration remain unverified.
No actual Autodesk installation was replaced.

The installer preserves an entire previous tree on updates and restores it on
handled publication failure when possible. Its tests exercise retained recovery
backups when restoration cannot succeed. This does not establish power-loss
recovery or concurrent-installer safety. Close Fusion and run one installer at
a time.

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
working tree based on `b637eed`, then published as `3dc5306`. These historical
receipts cover that baseline, not the later installer/viewer layout or merged
revision. That run passed the offline, cross-tool, and Docker ROS 2 checks below.
**Full Fusion coverage and universal superiority are not verified.** No native
Fusion application case or hosted CI job was executed for these October 17
receipts.

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

### Coverage is not a test-count ranking

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

### Negative evidence and independent review

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
