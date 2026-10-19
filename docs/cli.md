# Command-line integration

## Choose the right boundary

| Task | Entry point | Fusion needed? |
| --- | --- | --- |
| Install/update exporter files | `bash install.sh`, `.\install.cmd`, or `python3 tools/install_fusion.py` | No |
| Distribute a one-file installer | Build/run `fusion360tourdf-install.pyz` below | No; recipients need Python 3.11+ |
| Export a CAD assembly | Run `URDF_Exporter` inside Fusion | **Yes**; no standalone shell converter |
| Archive a complete package or its viewers | `python3 -m zipfile -c ...` | No |
| Verify a previously exported native bundle | `tools/verify_export.py` | No live Fusion for this step; Blender, Docker, and optional Python tools are required |
| Load/display/control the package | `colcon` and `ros2 launch` | No; run on a ROS machine |

The pure package writer is not a public CAD-import API. It cannot read `.f3d` files without Fusion extraction. Do not treat internal Python functions as a versioned external SDK.

## Build a single-file executable installer

From a trusted complete checkout, with Python 3.11+:

```bash
python3 tools/build_installer.py --output artifacts/fusion360tourdf-install.pyz
```

The build prints a SHA-256 digest and refuses to overwrite an existing output. It needs only Python's standard library; there is no pip install, remote code download, PyInstaller dependency, or embedded runtime. Use a fresh output filename for each build.

The archive contains the current exporter source, the unmodified shared installer helper, the TOML template, and license notices. Local `xacro_export.toml`, bytecode caches, and `.DS_Store` are excluded; symlinked exporter payloads are rejected. Review the source tree before distributing it. This is executable code, not a sandbox.

Copy **just the `.pyz`** to another machine. The original checkout is not required:

```bash
python3 fusion360tourdf-install.pyz --help
python3 fusion360tourdf-install.pyz --dry-run
python3 fusion360tourdf-install.pyz --scripts-dir "/custom/Fusion/API/Scripts"
python3 fusion360tourdf-install.pyz --update
```

On macOS/Linux, the built file has an executable shebang: `./fusion360tourdf-install.pyz --help`. If transferring strips the executable bit, use `python3` explicitly. On Windows:

```powershell
py -3 fusion360tourdf-install.pyz --dry-run
py -3 fusion360tourdf-install.pyz --scripts-dir "C:\Fusion Scripts"
```

Ensure the selected runtime is Python 3.11 or newer. The `.pyz` checks that version before extracting into a temporary directory and calling the **same** `tools/install_fusion.py` implementation. It preserves the caller's current directory, arguments, and exit status. Temporary payload extraction also happens for `--help` and `--dry-run`; it does not install into Fusion's destination until requested. See [installation policy and recovery](installation.md).

This is **not** a native Windows `.exe` or an interpreter-free binary. It does not bundle Autodesk, Python, ROS, or validation/rendering dependencies. A frozen native executable would need per-OS builds, bundled-license auditing, signing, and platform tests, while CAD export would still require Fusion. No such binaries or release signatures are published here.

## Options and automation contract

| Option | Behavior |
| --- | --- |
| `--scripts-dir PATH` | Scripts parent; relative paths are caller-relative; required on Linux/Wine |
| `--dry-run` | Validate/report without changing the install destination |
| `--update` | Explicitly replace a differing install, preserve TOML, retain a backup outside Scripts |
| `--help` | Print current options |

Exit codes: **0** for success (including unchanged/dry-run/help), **1** for a handled install/build failure or an unsupported Python runtime, **2** for invalid command arguments. Check process status; human-readable stdout is not a stable JSON API. For direct use of `tools/install_fusion.py`, the caller must supply a supported Python runtime.

The `.pyz` and checkout launchers are alternative transports for the same installer, not conflicting installation paths. They do not close Fusion, edit the user's `PATH`, or modify the source checkout. Execute only one installer at a time.

## Verify before redistributing

Compare the builder's printed SHA-256 with a digest calculated on the received file (`shasum -a 256 FILE` on macOS or `Get-FileHash FILE -Algorithm SHA256` in PowerShell). A checksum detects changed bytes; it does **not** prove publisher identity. Review the source/build you trust and preserve the bundled licenses.

CI is configured to test the archive and upload it as an installer-job artifact. A workflow artifact is not a signed release, and a configured job is not evidence that it passed. Developer test commands and platform limits are in [CONTRIBUTING.md](../CONTRIBUTING.md).
