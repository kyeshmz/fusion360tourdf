# Install, update, and recover

## Run from a complete checkout

Install host Python **3.11+** separately and reopen the terminal. Fusion's embedded Python does not run the installer. On macOS/Linux, the launcher looks for `python3` or `python`; on Windows it checks `py.exe`, `python3.exe`, and `python.exe` without using batch interpreter shims. No runtime download, administrator prompt, or interactive pause is performed.

From the checkout root:

```bash
bash install.sh
```

From Windows PowerShell or Command Prompt:

```powershell
.\install.cmd
```

The launchers delegate to `tools/install_fusion.py`. You can run that helper directly with a known Python 3.11+ runtime: `python3 tools/install_fusion.py` or `py -3 tools/install_fusion.py` (ensure the selected runtime is 3.11+). All entry points accept the same options. To distribute a single file instead, see [the CLI guide](cli.md).

## Destination and contents

| Platform | Default Scripts parent |
| --- | --- |
| Windows | `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts` |
| macOS | `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts` |

These are Autodesk's [always-searched folders](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/WritingDebugging_UM.htm#DefaultFolders). If your Fusion API folder preference differs, pass that Scripts parent explicitly:

```bash
bash install.sh --scripts-dir "/path/to/Autodesk Fusion 360/API/Scripts" --dry-run
bash install.sh --scripts-dir "/path/to/Autodesk Fusion 360/API/Scripts"
```

```powershell
.\install.cmd --scripts-dir "C:\Fusion Scripts" --dry-run
.\install.cmd --scripts-dir "C:\Fusion Scripts"
```

Do **not** append `URDF_Exporter` to the custom path. Relative paths stay relative to your terminal's current directory, even when invoking the launcher by another path. Linux/Wine requires an explicit destination; that is a copying option, not certification of Fusion on an unsupported platform.

The installer copies the complete `URDF_Exporter` directory, including the entry script, manifest, `core`, `utils`, template, MIT license, third-party notices, and retained upstream licenses. It excludes bytecode caches and leaves the checkout intact. A checkout's local `xacro_export.toml`, if present, is copied on first installation; the distributable `.pyz` deliberately excludes that private file.

`--dry-run` validates and reports the operation without changing the destination. The `.pyz` temporarily extracts its payload even in dry-run/help mode and removes that temporary directory on exit.

## Register and run in Fusion

Open **Utilities > Add-ins > Scripts and Add-ins > Scripts**. If the installed exporter is absent after a restart, use **+** to link the installed directory printed by the installer. Alternatively, link the checkout's complete `URDF_Exporter` directory without copying it.

The installer does not open Fusion, export CAD, alter folder preferences, or register the script through Fusion's UI. Prepare the model, select `URDF_Exporter`, and choose **Run**. Use a fresh output parent; existing export packages are not overwritten.

## Safe updates and backups

Save your work and **close Fusion** first. For a changed installation:

```bash
bash install.sh --update --dry-run
bash install.sh --update
```

On Windows, substitute `.\install.cmd` with the same arguments. Include the same `--scripts-dir` again if you initially used one. An identical installation reports `Unchanged` without needing `--update`.

Updates preserve the installed `xacro_export.toml` byte-for-byte and retain the previous directory outside Scripts:

```text
<directory containing Scripts>/.fusion360tourdf-backups/URDF_Exporter-<unique>/URDF_Exporter
```

The backup is outside the chosen Scripts directory. Review any preserved older configuration; the installer does not migrate options. Restart Fusion to clear cached modules. Run only one installer at a time.

On a handled publication failure, the installer tries to restore the previous directory. If restoration fails, the error identifies the retained recovery backup. Power loss is not automatically recovered: inspect both destination and backup before retrying. Never merge stale modules into a new installation.

## Troubleshooting and manual fallback

- **Python missing/too old:** install 3.11+ and check the terminal's `PATH`, not Fusion's embedded runtime.
- **Wrong location:** compare Fusion's API folder preference with the printed destination; pass `--scripts-dir` if needed.
- **Existing install differs:** close Fusion and use `--update`, or retain the existing installation for review.
- **Exporter still looks old:** restart Fusion and confirm which linked/copied directory it runs.
- **Unreadable path character in reports:** output may use a backslash escape; actual on-disk names remain unchanged.
- **Manual installation:** copy or link the complete `URDF_Exporter` folder. Keep its modules, configuration, manifest, and license notices together, not only `URDF_Exporter.py`.

Native macOS execution does not establish a native Windows pass. Inspect [the verification workflow](../.github/workflows/verify.yml) and its actual job results before claiming a hosted-platform result. Running Fusion under Wine remains unsupported by Autodesk and unverified here.
