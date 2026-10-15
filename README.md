# fusion360tourdf

Fusion 360 script that exports a robot model as a ROS `_description` package
(URDF/xacro + STL meshes + launch files), for ROS 1, ROS 2, or both.

Fork of [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf),
updated for the current Fusion 360 API and Python 3.12+.

## Related work surveyed

| Repository | What it fixed beyond upstream |
|---|---|
| [dheena2k2/fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2) | ROS 2 ament package + `.launch.py` output |
| [Adriaeik/fusion2URDF](https://github.com/Adriaeik/fusion2URDF) | Nested assemblies, rigid groups, full orientations, collision strategies, loop sidecar, `ros2_control`, frame layer, pre-export warnings |
| [haoyan-dev/fusion2urdf](https://github.com/haoyan-dev/fusion2urdf) | `Transform` math (xyz+rpy composition/inversion), templates, lint |
| [cadop/fusion360descriptor](https://github.com/cadop/fusion360descriptor) | GUI, grounded-base root, WYSIWYG meshes, yaml config, name mapping |
| [ACDC4Robot/Fusion360](https://github.com/ACDC4Robot/Fusion360) | URDF+SDF+MJCF, GUI panel, fail-closed preflight, provenance |
| [runtimerobotics/fusion360-urdf-ros2](https://github.com/runtimerobotics/fusion360-urdf-ros2) | ROS 2 docs/UX, Jazzy+Humble paths |
| [david-dorf/ExportURDF](https://github.com/david-dorf/ExportURDF) | Unified multi-CAD (Fusion/OnShape) converter library |
| [yanshil/Fusion2PyBullet](https://github.com/yanshil/Fusion2PyBullet) | PyBullet-targeted URDF, early API-change fixes |
| [newtonjeri/fusion2urdf-master](https://github.com/newtonjeri/fusion2urdf-master) | Nested components, light-bulb selection, colors, temp files, precision, ROS 1/2 dialog |

This repo ports the portable subset of those ideas into the original
flat-pipeline architecture (no new dependencies, offline-testable).

## Limits overcome here

- **Orientations**: full `xyz + rpy` joint/link origins
  (`R_parent^-1 * R_child`, axes in the parent frame) instead of hardcoded
  `rpy="0 0 0"`. Pure math lives in `URDF_Exporter/utils/transforms.py`.
- **Nested assemblies**: `allOccurrences` global walk flattens to leaf links;
  subassembly containers are structure, not links. Hidden occurrences
  (light bulb off) select the export set.
- **Non-destructive default**: WYSIWYG STL export without duplicating
  components; legacy `copy_occs` only with `use_legacy_copy = true`.
- **ROS 1 + ROS 2**: `ros_version = "1" | "2" | "both"` (default `both`),
  with `ros2_control` xacro, controllers config, and `launch/*.launch.py`.
- **Collision choice**: `visual` mesh or `primitive_box` from Fusion bounding
  boxes via `collision_method`.
- **Materials**: per-link Fusion appearance colors; `silver` fallback kept.
- **Preflight**: fail-closed validation with actionable errors and warnings
  shown *before* mesh export; loop-closing joints go to
  `config/robot_data.yaml` while the URDF stays a valid tree.
- **Robust base**: synthetic `base_link` from design-root bodies for
  imported CAD; `transform2` with legacy fallback; `asBuiltJoints` support;
  explicit `JointTypes` errors; no `quit()`; physics precision without
  `round(x, 6)` truncation.
- **Config**: `xacro_export.toml` (stdlib `tomllib`); see
  `URDF_Exporter/xacro_export.template.toml`.
- **Tests**: 22 offline tests (`python3 -m unittest discover -s tests`).

See `DESIGN_RULES.md` (compact rules) and `CHANGELOG.md`.

## Layout

- `URDF_Exporter/URDF_Exporter.py` — Fusion script entry point (`run()`).
- `URDF_Exporter/core/Joint.py` — joint dict + URDF joint/transmission XML.
- `URDF_Exporter/core/Link.py` — mass, CoM, inertia, colors, bboxes + link XML.
- `URDF_Exporter/core/Write.py` — xacro / launch / yaml / ROS2 / sidecar writers.
- `URDF_Exporter/utils/transforms.py` — pure xyz+rpy math (single owner).
- `URDF_Exporter/utils/validate.py` — preflight + loop detection (pure).
- `URDF_Exporter/utils/config.py` — TOML config (pure).
- `URDF_Exporter/utils/materials.py` — appearance colors (pure).
- `URDF_Exporter/utils/utils.py` — Fusion helpers (occurrence walk, STL, package).
- `URDF_Exporter/package/` — ROS 1 package template.
- `Example/` — sample model carried over from the upstream repo.
- `tests/` — offline logic tests (no Fusion needed).

## Requirements

- Autodesk Fusion 360 with API scripts enabled.
- A model where each URDF link is a component; name the root chassis
  `base_link` when modelling cleanly (synthesized otherwise).
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

Optional: copy `URDF_Exporter/xacro_export.template.toml` to
`xacro_export.toml` next to `URDF_Exporter.py` and tune it.

## Use

1. Name the base component `base_link` and unground it if needed (or rely on
   synthetic-base for root-body imports).
2. In Fusion: Utilities → ADD-INS → Scripts → `URDF_Exporter` → Run.
3. Pick an output folder. The script creates `<robot>_description/` with
   `urdf/`, `meshes/`, `launch/`, `config/`, plus ROS 1 and/or ROS 2 package
   files per `ros_version`.
4. Move `<robot>_description` into a ROS workspace and build.

ROS 1:

```bash
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
roslaunch <robot>_description display.launch
```

ROS 2:

```bash
cd ~/ros2_ws/
colcon build
source install/setup.bash
ros2 launch <robot>_description display.launch.py
```

## Tests

```bash
python3 -m unittest discover -s tests -v
```

## Credits

Upstream work by Toshinori Kitamura and contributors under MIT
(see `LICENSE`). Ideas above are credited per-repo; implementations here are
original, reduced to this architecture. See `git log` for history.

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
