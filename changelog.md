# Changelog and source credits

This project is an amalgamation of inherited Fusion-to-URDF work, approaches learned from other exporters, and integration work developed in this repository. It is not a from-scratch implementation. The lists below distinguish those relationships rather than treating every repository in the comparison as copied code.

## 0.1.0: combined baseline (unreleased)

This is the first documented combined version, not a claim that a release tag or package has been published. It consolidates the earlier working changes and the current demo, installation, and verification tooling.

### Inherited foundation

- **[syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf)**: the original Fusion exporter by Toshinori Kitamura and contributors, its Python entry point/module structure, and the starting link, joint, mass/inertia, STL, and URDF flow. Original module headers and the MIT copyright/license are retained. The inherited `Example/Basic_Robot.f3d` also comes from that lineage.

### Approaches adapted or used as implementation guidance

- **[haoyan-dev/fusion2urdf](https://github.com/haoyan-dev/fusion2urdf)**: full translation/rotation transforms, relative-frame composition, and Euler-angle handling informed `URDF_Exporter/utils/transforms.py`. This is credit for the documented approach, not a claim that its parser was merged wholesale.
- **[Adriaeik/fusion2URDF](https://github.com/Adriaeik/fusion2URDF)**: relative link-frame math, visibility-aware selection, early topology validation, and loop-closure sidecar ideas informed the local transform/validation code. The local rigid-group implementation keeps fixed-linked members; it is not the same as that repository’s rigid-link merging.
- **[newtonjeri/fusion2urdf-master](https://github.com/newtonjeri/fusion2urdf-master)**: visible/light-bulb export selection and retaining tiny inertia values informed the local visibility and precision policy. Its temporary direct-design staging is not used as proof of direct-mode input support here.
- **[ACDC4Robot/Fusion360](https://github.com/ACDC4Robot/Fusion360)**: fail-closed preflight and ordinary/as-built discovery informed local validation requirements and test cases. Its optional destructive flattening helper is not an export workaround in this project.

The local source comments identify these influences. They do not establish line-for-line provenance for every later rewrite. Exact inspected revisions and source paths are retained in the [pinned audit](docs/related-work.md) and [repository census](docs/fusion-repositories.json).

### Repositories used as comparison and regression references

- **[dheena2k2/fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2)**: reviewed its ROS 2 packaging around the upstream extraction path. Its shared upstream code and inherited example are not counted as independent implementations or native test passes.
- **[cadop/fusion360descriptor](https://github.com/cadop/fusion360descriptor)**: reviewed configured/grounded roots, outward edge orientation, fixed relationships, and driven-position handling. These informed acceptance cases; this changelog does not claim a direct parser import.
- **[runtimerobotics/fusion360-urdf-ros2](https://github.com/runtimerobotics/fusion360-urdf-ros2)**: reviewed its ROS 2 output, flat extraction, source naming, and inertia precision as regression targets. No wholesale implementation import is claimed.
- **[david-dorf/ExportURDF](https://github.com/david-dorf/ExportURDF)**: reviewed its Fusion ordinary/as-built adapter, rigid null-geometry behavior, and SI-unit handling. These informed failure cases; other CAD adapters and simulator formats are not incorporated here.

Source inspection is not a head-to-head execution of those exporters. Missing tests elsewhere do not establish failure, and these credits do not imply universal superiority or feature parity.

### Integration and changes developed here

- Maintain ROS 2 Humble/Jazzy description packages rather than the earlier dual ROS 1/ROS 2 and mismatched simulator-output paths.
- Use one transactional package writer with self-contained xacro, plain assembly URDF, per-component URDF files, meshes, launch files, controller configuration, and metadata.
- Preserve every component’s visual, collision, and inertial data in its one-link URDF, with shared package meshes and no invented per-part controllers.
- Preserve source history, names, transforms, joint positions, and grounding flags during export; reject existing output and partial mesh publication.
- Retain nested and repeated occurrence identities, including hidden contextual endpoints, rather than binding a constraint to the wrong visible instance.
- Convert native-body millimeter STL geometry and normals into root context explicitly after native testing exposed component-local proxy export.
- Rebase joint limits to the exported current pose and orient edges outward from the selected root without discarding axis/frame changes.
- Support root-owned solids, visibility-consistent direct-body mass aggregation, explicit root selection, fixed groups, and representable grounding ancestry.
- Reject unresolved bodyless/world frames, unsupported active generalized joints, motion coupling, and mesh-only physical/export policies by name instead of silently changing the mechanism.
- Extract supported modern/legacy appearance colors with byte RGB/opacity semantics, while keeping physical material density separate from visual color.
- Generate mock ros2_control configuration and verify actual controller activation, successful trajectories, joint-state feedback, and numeric transforms on Humble and Jazzy.
- Add independent loader checks, Blender geometry checks, wrong-axis/shifted/missing-mesh negative controls, and install-space resource checks.
- Add owned-document native Fusion fixtures for parametric/direct inputs, editable-history preservation, retained entity identity, mesh/physics checks, and loaded-code provenance.
- Create the native rotary-slide demo with a root-owned base, grounded mast, nested slider, repeated sensors, and automatic viewport capture.
- Add correctly identified Fusion, Blender, and real ROS/RViz media, numbered component labels, and a reproducible walkthrough build from actual RViz motion.
- Integrate the shared installer contribution from [d54d4c9](https://github.com/kyeshmz/fusion360tourdf/commit/d54d4c9): `install.sh` and `install.cmd` delegate to one Python 3.11+ helper, support dry runs and caller-relative custom paths, recognize identical installs, preserve TOML configuration, retain backups outside Scripts, and avoid automatic runtime downloads or administrator prompts. The Windows launcher runs from PowerShell or Command Prompt.
- Retain the same contribution’s separate `viewers/` directory, viewer-only ZIP workflow, launch discovery, and portable installed-mesh checks. Keep control files under `launch/` and `config/`; a viewer-only archive still requires its matching description package.
- Separate user instructions from `CONTRIBUTING.md`, record Python runtime/tool requirements, and preserve exact source/evidence hashes instead of treating test counts as universal CAD proof.
- Add a standard-library executable `.pyz` installer that bundles the same shared installer and exporter payload, excludes private TOML/cache files, and needs host Python 3.11+ rather than claiming an interpreter-free CAD converter.
- Add linked CI/Python/ROS/license badges, an explicit runtime/platform matrix, a short quick start, and separate installation/recovery/CLI guides. Configure offline CI for Python 3.11–3.14 without labeling unexecuted jobs as passes.
- Retain full pinned MIT copyright/license texts for the inherited/adapted/comparison sources under the installed exporter, with a relationship ledger and license index rather than a blanket claim that every referenced project was copied.

### Earlier working states superseded by this baseline

- Early snapshots offered `ros_version = 1|2|both` and an optional `use_legacy_copy` path. Those options are no longer supported.
- The initial offline suite had 22 tests. Later suites add extraction, package, context, grounding, appearance, source-preservation, installer, and media checks; executed counts belong in the dated verification reports.
- Early native-verifier notes described prepared but unexecuted scripts. Subsequent real Fusion runs are recorded separately in [verification results](docs/verification-results.md).
- Early nested-assembly flattening was expanded into explicit physical-link selection with distinct repeated-instance identities, not general bodyless-frame export.

## Remaining gaps

- Motion-link/mimic conversion, generalized-joint decomposition, general bodyless endpoints, mesh-only physical/export policy, and per-body/face texture fidelity remain incomplete.
- A closure sidecar does not enforce closed-chain physics. Mock control is not a simulator or physical-hardware connection.
- Native verification covers specific inputs and source snapshots, not every Fusion model or every implementation in the source comparison.
