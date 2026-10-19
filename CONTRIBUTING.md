# Contribute to fusion360tourdf

Use a separate Python environment for verification tools. Keep the Fusion exporter dependency-free and preserve the source design when changing extraction or package generation.

## Set up development tools

Run these commands from the checkout root with Python 3.11 or newer:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-test.txt
.venv/bin/python -m unittest discover -s tests -v
```

The exporter uses Fusion’s embedded Python, not this virtual environment. Native testing recorded Python 3.14.0; ROS Humble fixture generation also runs on Python 3.10. The README’s tool badge describes the host test requirement, not a replacement Fusion runtime.

## Keep implementation ownership clear

The main boundaries are:

- `URDF_Exporter/core/Joint.py`: constraint extraction, frames, and joint direction
- `URDF_Exporter/core/Link.py`: physical properties
- `URDF_Exporter/utils/utils.py`: source selection and STL export
- `URDF_Exporter/core/Write.py`: pure ROS 2 package and component-URDF generation
- `tools/` and `tests/`: native fixtures, loaders, rendering, ROS checks, and media capture

Use the existing `export_design` and package writer. Do not create a second robot generator for a demonstration or test. Keep installation policy in `tools/install_fusion.py`; `install.sh` and `install.cmd` are launchers, not separate implementations.

## Run the relevant checks

Run offline regressions and the declared input catalog for each change:

```bash
python3 -m unittest discover -s tests -v
python3 tools/verify_fusion_coverage.py --output artifacts/fusion-coverage
```

For package-writer changes, run both Docker distributions and the cross-tool suite:

```bash
bash tools/verify_docker.sh humble artifacts
bash tools/verify_docker.sh jazzy artifacts
PYTHON="$PWD/.venv/bin/python" bash tools/verify_cross_tools.sh artifacts
```

The cross-tool suite requires Blender and `check_urdf`. Docker needs a running engine and package-download access. Do not use sudo, change socket permissions, or prune shared Docker data to bypass an access or disk-space problem.

For Fusion extraction changes, also run the [native verifier](tools/verify_fusion/README.md) inside Fusion. A mocked callback, Docker run, or prepared script does not replace native evidence. Keep parametric and direct inputs distinct; never disable history to make export pass.

Test installer behavior without touching your real Fusion installation:

```bash
python3 -m unittest tests.test_install_fusion tests.test_install_launchers tests.test_installer_archive tests.test_license_notices -v
```

Run the same tests with Windows Python from PowerShell to exercise the CMD launcher. Tests substitute temporary locations rather than your real Fusion installation:

```powershell
py -3 -m unittest tests.test_install_fusion tests.test_install_launchers tests.test_installer_archive tests.test_license_notices -v
```

## Preserve evidence and failure checks

Retain the tested revision, dirty state, commands, report paths, source hashes, and missing runtimes. Require successful exits and inspect the reports instead of treating partial files as passes.

Keep these checks strict:

- Source history, entity identity, transforms, joint positions, and visibility must remain unchanged during export.
- Wrong axes, shifted or missing meshes, and partial package publication must fail.
- Standalone component URDFs must preserve link data and resolve their shared meshes.
- ROS checks must observe controller activation, trajectory success, joint feedback, and numeric transforms.
- Pictures and video must identify Fusion, Blender, or real RViz correctly. Preview renders do not prove physics or hardware behavior.

`--require-complete` must remain nonzero while full input coverage is unproven. Do not turn rejection tests into input-support claims. See the [testing guide](docs/testing.md) and [input matrix](docs/fusion-coverage.md).

## Update documentation and media

Keep usage in `README.md`, detailed export rules in `docs/exporting.md`, and contributor instructions in this file. Update the [project verification recipes](.opencode/skills/verify-fusion360tourdf/SKILL.md) when entry points or proof requirements change.

Use the [demo workflow](docs/demo-workflow.md) for native captures and exported-pose images. Preserve image/source hashes and inspect every screenshot for correct labels, framing, and loaded geometry. Do not overwrite historical receipts to make new code appear tested.
The [showcase guide](docs/showcase.md) covers real RViz recording and the chaptered movie. Keep inherited code, adapted ideas, comparison-only sources, and local work distinct in [changelog.md](changelog.md), and preserve upstream license notices.

For copied/adapted code or example assets, record the upstream revision, affected files, and relationship in [the license notices](URDF_Exporter/THIRD_PARTY_NOTICES.md). Retain the full license/copyright text under `URDF_Exporter/licenses/` so installations and `.pyz` distributions carry it. Comparison-only references must not be relabeled as code imports. The [license index](LICENSES.md) explains the distribution boundary; optional external tools keep their own licenses.

## Submit a change

Describe the behavior changed, the tests actually run, and any unverified platforms or input cases. Include reproduction steps and retained reports for bugs. Keep generated build/install directories and local evidence under ignored `artifacts/`; publish only intentional example packages and documentation media.

Do not include credentials, local account data, or unrelated desktop contents in logs or captures. Do not force-push or discard another contributor’s changes as part of a repair.
