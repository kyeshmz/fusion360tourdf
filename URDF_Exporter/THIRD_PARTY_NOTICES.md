# Third-party code, ideas, and license notices

The exporter is distributed under [MIT](LICENSE). Its inherited foundation is Toshinori Kitamura's fusion2urdf and its contributors. Original copyright notices remain intact. This file and `licenses/` travel with the installed exporter, including installations made with the single-file CLI.

The table separates inherited code from documented implementation influences and comparison-only sources. Listing a project does not mean its entire implementation was copied or that it is a runtime dependency. For the influences, the local source comments establish design guidance, not line-by-line provenance; we retain their license texts conservatively rather than replacing attribution with a link alone.

All license files below were checked against these exact upstream revisions on October 19, 2026. The full upstream copyright, permission, and disclaimer text is preserved. Three repositories share the same Toshinori Kitamura license text, so they reference one copy.

| Project / pinned source license | Relationship to this repository | Copyright notice / retained license |
| --- | --- | --- |
| [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf/blob/21fd3050752c2baa62dfdc1bfa05ec51fc472122/LICENSE) | Inherited entry/module structure, extraction/export foundation, `Example/Basic_Robot.f3d`, and historical ROS sample | 2018 Toshinori Kitamura · [MIT](licenses/fusion2urdf-MIT.txt) |
| [haoyan-dev/fusion2urdf](https://github.com/haoyan-dev/fusion2urdf/blob/dc7a2e657ccf4f44b68e52e157067958f3de99c7/LICENSE) | Documented transform/Euler-angle approach in `utils/transforms.py`; not a wholesale parser import | 2018 Toshinori Kitamura · [MIT](licenses/fusion2urdf-MIT.txt) |
| [Adriaeik/fusion2URDF](https://github.com/Adriaeik/fusion2URDF/blob/4411ffbdffc0364dc6f5922c40b3e14fd7b642f8/LICENSE) | Relative-frame, visibility, topology/preflight, and closure-sidecar ideas in `utils/transforms.py`, `utils/utils.py`, and `utils/validate.py` | 2026 Adrian Valaker Eikeland · [MIT](licenses/Adriaeik-MIT.txt) |
| [newtonjeri/fusion2urdf-master](https://github.com/newtonjeri/fusion2urdf-master/blob/5238bd60e90d3996f06d18f78463c624c4f7d617/LICENSE) | Visibility-selection and small-inertia precision guidance in `utils/utils.py` | 2018 Toshinori Kitamura · [MIT](licenses/fusion2urdf-MIT.txt) |
| [ACDC4Robot/Fusion360](https://github.com/ACDC4Robot/Fusion360/blob/f1152d7b6fb067d25f82885f86ae7f79bbb45b5e/LICENSE) | Fail-closed preflight and ordinary/as-built discovery guidance; destructive flattening helper not incorporated | 2023 BionicDL@SUSTech · [MIT](licenses/ACDC4Robot-MIT.txt) |
| [dheena2k2/fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2/blob/8e883d2d66529a77c67edc28d29f2c3ba96169f6/LICENSE) | ROS 2 comparison/regression reference sharing the upstream lineage; no separate wholesale code import claimed | 2022 Dheenadhayalan R · [MIT](licenses/dheena2k2-MIT.txt) |
| [cadop/fusion360descriptor](https://github.com/cadop/fusion360descriptor/blob/4d28621a97e4e2c4be9a9dbecdfefb231813e9da/LICENSE.md) | Root/endpoint/fixed-edge comparison and acceptance cases; no direct parser import claimed | 2025 SiBORG Design Lab · [MIT](licenses/cadop-MIT.txt) |
| [runtimerobotics/fusion360-urdf-ros2](https://github.com/runtimerobotics/fusion360-urdf-ros2/blob/e317c5e00e1044996d19d5e4ff97431ed8816245/LICENSE) | Packaging, naming, and inertia comparison/regression reference; no wholesale import claimed | 2024 Lentin Joseph · [MIT](licenses/runtimerobotics-MIT.txt) |
| [david-dorf/ExportURDF](https://github.com/david-dorf/ExportURDF/blob/bbf2af2a19e73052aee16b37270ef7318e1e579e/LICENSE) | Fusion adapter comparison for as-built geometry and SI-unit cases; other adapters not incorporated | [2024] [David Dorf] · [MIT](licenses/david-dorf-MIT.txt) |

## Distribution boundaries

- Keep this file, `LICENSE`, and the `licenses/` directory together when copying or redistributing the exporter. Do not remove upstream headers when rewriting a module.
- The inherited historical example retains its own license. The new rotary-slide example is generated locally; it does not import another repository's CAD model.
- Autodesk Fusion and its `adsk` Python modules are proprietary external requirements, not redistributed or relicensed here. A Fusion license is still required.
- ROS 2, RViz, Blender, FFmpeg, Docker, Python, and optional test/media packages are separately installed tools with their own licenses. They are not bundled into the exporter or `.pyz`. Any future native binary or container redistribution must audit its bundled dependencies separately.
- A source comparison or an idea citation is not permission to import unrelated files later. New copied or adapted code must retain the applicable notices and identify its source revision and affected files.

Repository-level context and source-inspection paths are in `changelog.md`, `docs/related-work.md`, and `docs/fusion-repositories.json` in the checkout. This installed copy is self-contained so those documents are not needed to read the retained licenses.
