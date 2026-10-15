# fusion360tourdf

Fusion 360 script that exports a robot model as a ROS `_description` package
(URDF/xacro + STL meshes + launch files).

Fork of [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf),
updated for the current Fusion 360 API and Python 3.12+.

## What changed from `fusion2urdf`

- Uses `Occurrence.transform2` (the `transform` property was retired in July
  2025 and returns wrong results for nested/jointed occurrences).
- Falls back to `geometryOneTransform` / `geometryTwoTransform` when joint
  geometry is no longer available after model edits.
- Explicit `JointTypes` mapping (Rigid=0, Revolute=1, Slider=2) with a clear
  error for Cylindrical / PinSlot / Planar / Ball / Inferred joints instead
  of emitting invalid URDF.
- Reads `asBuiltJoints` in addition to regular `joints`.
- Single `sanitize_name()` owner in `URDF_Exporter/utils/utils.py`, so link,
  joint, and mesh names agree (`[^A-Za-z0-9_]` → `_`).
- `shutil.copytree(..., dirs_exist_ok=True)` instead of removed
  `distutils` (Python 3.12 compatibility).
- STL export de-duplicates by component, creates `meshes/` with
  `os.makedirs(..., exist_ok=True)`, and best-effort forces millimeter STL
  units so the URDF `scale 0.001` (mm → m) stays correct.
- No more bare `quit()` inside the writers; joint-connection errors raise
  `RuntimeError` with the parent/child hint.
- Offline unit tests with a mocked `adsk` API: `python3 -m unittest`.

## Layout

- `URDF_Exporter/URDF_Exporter.py` — Fusion script entry point (`run()`).
- `URDF_Exporter/core/Joint.py` — joint dict + URDF joint/transmission XML.
- `URDF_Exporter/core/Link.py` — mass, center of mass, inertia + link XML.
- `URDF_Exporter/core/Write.py` — xacro / launch / yaml writers.
- `URDF_Exporter/utils/utils.py` — Fusion helpers (transforms, STL, package).
- `URDF_Exporter/package/` — ROS package template copied into the output.
- `Example/` — sample model carried over from the upstream repo.
- `tests/` — offline logic tests (no Fusion needed).

## Requirements

- Autodesk Fusion 360 with API scripts enabled.
- A model where each URDF link is a component, with a component named
  `base_link`.
- Joints defined with parent as Component2 and child as Component1.
- Only Rigid, Revolute, and Slider joints are supported.
- STL meshes assume millimeter document/export units (Fusion default).

## Install

Copy `URDF_Exporter` into Fusion's scripts folder, then restart Fusion or
refresh scripts.

Windows (PowerShell):

```powershell
cd <path to fusion360tourdf>
Copy-Item ".\URDF_Exporter\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
```

macOS (bash/zsh):

```bash
cd <path to fusion360tourdf>
cp -r ./URDF_Exporter "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/"
```

Linux (Fusion under Wine, e.g. `~/.autodesk_fusion/wineprefixes/default`):

```bash
cd <path to fusion360tourdf>
WIN_APPDATA="$(winepath -u "$APPDATA" 2>/dev/null || echo "$HOME/.autodesk_fusion/wineprefixes/default/drive_c/users/$USER/Application Data")"
cp -r ./URDF_Exporter "$WIN_APPDATA/Autodesk/Autodesk Fusion 360/API/Scripts/"
```

## Use

1. Back up your Fusion design — the exporter duplicates components during STL export.
2. Name the base component `base_link` and unground it if needed.
3. In Fusion: Utilities → ADD-INS → Scripts → `URDF_Exporter` → Run.
4. Pick an output folder. The script creates `<robot>_description/` with `urdf/`, `meshes/`, `launch/`, `CMakeLists.txt`, `package.xml`.
5. Move `<robot>_description` into a ROS workspace and build.

```bash
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
roslaunch <robot>_description display.launch
```

## Tests

```bash
python3 -m unittest tests.test_exporter_logic -v
```

## Credits

Upstream work by Toshinori Kitamura and contributors under MIT
(see `LICENSE`). This fork keeps that license and history in view; see
`git log` for upstream commits.

## Citation

```
@misc{toshinori2020fusion2urdf,
    author = {Toshinori Kitamura},
    title = {Fusion2URDF},
    year = {2020},
    publisher = {GitHub},
    journal = {GitHub repository},
    howpublished = {\url{https://github.com/syuntoku14/fusion2urdf}}
}
```
