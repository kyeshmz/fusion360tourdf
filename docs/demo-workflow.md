# Create and verify a native Fusion robot

This workflow creates a rotary-slide inspection arm in Fusion, exports it with the production exporter,
then loads and commands that exact package outside Fusion. It does not substitute generated ROS fixtures
for the CAD export.

## Create the robot inside Fusion

Keep this checkout intact. Save existing work and restart Fusion after changing Python modules.
In **Utilities > Add-ins > Scripts and Add-ins > Scripts**, add the checkout's
`tools/create_demo` directory, select **create_demo**, and run it. Choose an output parent folder.
Cancellation creates nothing.

The script creates a new parametric **Rotary Slide Demo** document and a unique `rotary-slide-*`
bundle. It leaves the new document open and unsaved; it neither saves nor modifies existing documents.
An `assembly.f3d` archive in the bundle lets you reopen the native assembly later.
The same run captures a 1280 × 960 `fusion.png` directly from the Fusion viewport.
On failure, it retains `native.json`, closes only its own new document, restores the previous document,
and raises the error.

The robot has a 240 × 180 × 20 mm editable base, a grounded mast, a rotated rail,
an as-built revolute joint, a nested slider, and two instances of one nested sensor component.
A large hidden service jig must not enter exported geometry or mass.

From a Fusion MCP script, call the same entry point instead of automating a folder dialog:

```python
import adsk.core
import sys

def run(_context: str):
    sys.path.insert(0, '/absolute/path/to/fusion360tourdf')
    from tools.create_demo.create_demo import create_demo
    print(create_demo(adsk.core.Application.get(), '/absolute/path/to/output-parent'))
```

This is a Fusion script, not a system-Python or headless Fusion launcher.
Fusion's MCP URL can change after a restart. Reconnect to the current URL shown by Fusion;
do not assume a previously configured localhost port is still the active server.

## Verify the actual export

Requirements: Python 3.11+, Blender on `PATH` (or `BLENDER` set), and Docker Engine with enough free VM disk space.
The Docker builds download ROS packages and preserve package-signature verification.
Do not make the Docker socket world-writable or prune other projects' data without permission.

From the checkout root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-test.txt
.venv/bin/python tools/verify_export.py /absolute/path/to/rotary-slide-BUNDLE --output artifacts
```

One command performs xacro expansion, strict URDF lint, independent visual/collision mesh and FK checks,
negative controls, Blender geometry checks, and ROS 2 Humble and Jazzy tests.
`--distros jazzy` selects a narrower run; its report covers only that distribution.
The shell drivers work with macOS's system Bash and do not require GNU `realpath`.
Host `check_urdf` is not required for this workflow: both ROS containers run real urdfdom.

For a single Docker distribution without the host loader/Blender stages:

```bash
bash tools/verify_docker.sh jazzy artifacts /absolute/path/to/rotary-slide-BUNDLE
```

Without the third argument, that driver still runs the existing generated-fixture suite.

## Refresh the README pictures

Create a fresh bundle with the updated Fusion script and run `tools/verify_export.py` as above.
The verifier saves three PNGs under its reported `export-e2e-*/images/` directory:
native Fusion, exported zero pose, and the exported 45-degree/25-mm pose.
It also writes `images.json` with source and image hashes. Older bundles without `fusion.png`
still verify, but only produce the two exported-pose pictures.

After a successful run, publish that image set from the repository root:

```bash
run=/absolute/path/to/export-e2e-RUN
cp "$run/images/rotary-slide-fusion.png" docs/images/
cp "$run/images/rotary-slide-zero.png" docs/images/
cp "$run/images/rotary-slide-moved.png" docs/images/
cp "$run/images/images.json" docs/images/
```

The Blender renderer rechecks the saved inspection geometry against the independent bounds,
uses the same framing for both poses, hides collision overlays, and adds neutral preview
shading and lighting without changing robot geometry. Blank renders fail the command.
These are illustrations of real CAD/export data, not additional proof of materials, physics,
or controller behavior. The numerical and ROS checks remain separate.

## Evidence and acceptance

The native script compares binary STL geometry, box mass/COM/inertia, joint axes/limits,
and nonzero exported motion against independent fixture dimensions and transforms.
It compares source snapshots and retained entity identities before and after export.
The bundle binds the package, independent `expected.json`, and native archive with SHA256 hashes.
Downstream stages reject missing or changed files. A receipt is local provenance, not a signed attestation.

The verifier writes a fresh `export-e2e-*` directory and `summary.json`, even on a stage failure.
Require exit zero and `status: pass`; partial files or a running report are not success.
Retained evidence includes:

- `native.json`, `expected.json`, `assembly.f3d`, and the exported package in the input bundle.
- `robot.urdf`, `lint.log`, `loaders.json`, and rejection evidence for wrong axes, shifted meshes, and missing meshes.
- `blender/blender.json` and `blender/inspection.blend`, with zero and combined nonzero poses.
- `images/` with native/zero/moved PNGs and image/source hashes for README publication.
- Each distribution's build/install logs and `ros.json`. Installed native assets must match the CAD export byte-for-byte.
- Active joint-state broadcaster and trajectory controller, successful trajectory result, measured joint feedback,
  and numeric TF matching independent zero/moved expectations. The trajectory rotates 45 degrees and extends 25 mm.

Each Docker driver removes only its own uniquely named container and image tag on exit.
Build cache is retained. Shared-image pruning is not part of the workflow.

## Audited failure classes exercised

| Failure class from the pinned source audits | Demo check |
| --- | --- |
| Root-local extraction misses nested parts or joints | The slider and repeated sensors are nested beneath a body-owning rail. |
| Instance suffix stripping or wrong proxy context | Shared sensor components need distinct links and transformed mesh files. |
| Proxy STL exporter drops occurrence transforms | Native meshes receive root-context transforms; independent bounds reject local-frame output. |
| Null rigid-as-built geometry | Sensor mounts use geometry-free rigid as-built joints. |
| History disabling or source renaming | Editable history, parameter expressions, names, transforms, and entities must remain unchanged. |
| Grounding or root ambiguity | Root-owned base geometry and explicit `design_root` anchor a Ground To Parent mast. |
| Hidden geometry contaminates mass | The hidden root-owned jig is excluded from both STL and inertia. |
| Unit, frame, limit, or small-inertia errors | Known asymmetric geometry, SI physics, rotated frames, and bounded motion are checked numerically. |
| Fixture-only ROS success | Containers build the supplied native package; installed resource hashes and commanded TF are checked. |

This is not a head-to-head execution of other exporters or proof of every Fusion input.
The demo uses a connected, open, physical-link tree with supported joints.
Motion links/mimic conversion, generalized joints, mesh-only inputs, bodyless endpoints, closed-chain dynamics,
and per-face textures remain outside this demo's proof. Native joints begin at zero; nonzero tests articulate
the exported URDF and ROS mock hardware, not Fusion's solver or physical hardware.
