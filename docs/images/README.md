# README demo pictures

- `rotary-slide-fusion.png`: direct Fusion viewport capture of the verified native assembly.
- `rotary-slide-zero.png`: the actual exported URDF meshes at joint zero, rendered in Blender.
- `rotary-slide-moved.png`: the same export at 45 degrees of rotation and 25 mm of extension.

The Blender views share one camera and use neutral preview materials and studio lighting.
The robot meshes and transforms are not remodeled for the pictures. The renderer checks
their bounds against the independent expectations and does not save changes to the input `.blend`.
Images illustrate geometry, not material fidelity, controller execution, or physics.

`images.json` records image and source hashes. Native capture was checked for source-state preservation.
The published set was captured using Fusion 2705.1.30 and Blender 5.2.2 LTS.

Follow [the image refresh recipe](../demo-workflow.md#refresh-the-readme-pictures) to regenerate
and publish the complete PNG/metadata set. Fresh demo bundles capture `fusion.png` automatically;
`tools/verify_export.py` generates the matching exported-pose renders and checks the native screenshot hash.
