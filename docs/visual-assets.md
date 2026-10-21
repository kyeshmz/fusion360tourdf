# Portable visual asset backend

`URDF_Exporter.utils.visual_assets` provides read-only face snapshots and an OBJ,
MTL, PNG, and JPEG serializer. It preserves the selected diffuse RGBA and image UV
data separately for each selected face. Several body contexts can share one
visual asset without merging their colors.

The public exporter accepts `visual_materials = "strict"` or the explicitly
lossy `"diffuse-subset"`. The default `"legacy"` retains one-RGBA/STL behavior
without reading face tessellation. Both opt-in modes select the same immutable
visible direct bodies as CAD physics. Body context IDs use safe export aliases
and ordinals, not raw Unicode paths. Native faces tessellate in their native
basis and receive the selected occurrence transform exactly once.

`write_ros2_package(..., visual_assets={link: bundle})` accepts the same
SDK-free descriptor/files records. The existing batch `write_meshes` callback
still writes the original STL resources for collisions. OBJ/MTL/image bundles
are supplemental visuals, never a new source of mass or collision geometry.

## Separate files for each selected body

`visual_meshes = "combined"` is the default. Omission and explicit selection
preserve the combined bundle, sidecar schema, and package bytes. Multiple
materials in an OBJ do not mean multiple body files.

Select both options for one OBJ and one URDF `<visual>` per selected direct
BRep body context.

```toml
visual_meshes = "per_body"
visual_materials = "strict"
```

`diffuse-subset` also works with its explicit loss receipts. `per_body` with
`legacy` rejects rather than guessing a shader. Native MeshBody face/UV input
remains explicitly unsupported. No new native-render fidelity is claimed.

The same frozen visible direct-body selection supplies physics, combined STL
collision, and visual extraction. Hidden bodies produce no visual files.
Repeated instances keep their own body contexts and baked root-design poses.
Each physical link still has one aggregate mass, COM, and inertia. The combined
STL source collision bytes are unchanged. No source CAD geometry, material,
physics, or history is modified.

Every OBJ contains root-design millimeter vertices, including body placement.
Every visual uses its own URI, the link's inverse mesh origin, and `.001` scale.
All visuals on a link share that origin. Joint motion and explicit output
rotation apply once to every visual and collision piece. Context-specific OBJ
`o` records keep coincident bodies distinct. Identical MTL and image bytes can
still deduplicate safely.

The plain writer IR is `visual_assets={link: [bundle0, bundle1, ...]}`. Each
existing `fusion-visual-assets/1` descriptor gains one field in per-body mode.

```json
"body_context": {
  "id": "base_link_body_0",
  "source_path": "design_root",
  "selected_body_count": 2
}
```

The context ID is the link alias and zero-based selected-body ordinal.
`source_path` is the exact owning occurrence path or `design_root`, not a local
image path. The count binds every descriptor to the complete selection, so
dropping the last bundle fails. Pure callers declare that selection. Validation
checks the inertial source path when supplied, but does not certify external CAD.

The sidecar records `visual_meshes: "per_body"` and list-valued `visual_assets`.
Combined sidecars stay unchanged. `visual_descriptors(value)` owns iteration in
the writer, receipts, linter, loaders, Blender, and installed-resource checks.
All body contexts, paths, hashes, sizes, material policies, triangle ranges,
images, and UVs validate before publication. Mixed shapes, duplicate/foreign
contexts, missing second resources, nonplain descriptors, and dropped bundles
fail. A failed second-body read/write or cancellation leaves no new package
and never overwrites an existing one.

The existing optional registry provides `per_body` and `per_body_hull`.
Both have two multiface textured bodies on each of two physical links and a
nonzero `turn` sample. The latter combines an explicit triangle-vertex hull,
global rotation, and positive inertia floor with independent rotated literal
expectations. Asset bytes remain unchanged across the policy selection.

```bash
python3 -m unittest tests.test_per_body_visuals tests.test_per_body_hull -v
python3 tools/verify_loaders.py --output "$run/loaders" --fixture per_body_hull
bash tools/verify_cross_tools.sh "$run/cross" --fixture=per_body_hull
bash tools/verify_docker.sh humble "$run/humble" --fixture=per_body_hull
bash tools/verify_docker.sh jazzy "$run/jazzy" --fixture=per_body_hull
```

Use `per_body` to test separate visuals without hull/policy options. These are
generated-fixture checks. Live Fusion and actual loader/Blender/ROS results
remain separate evidence. ROS control must activate controllers, command the
nonzero sample, and check feedback and TF, not merely pass zero-pose display.

## Native read contract

```python
from URDF_Exporter.utils.visual_assets import read_native_body, build_obj_assets

snapshot = read_native_body(
    selected_body_context,
    owner=selected_occurrence,
    root_from_body_cm=native_body_to_root_matrix.asArray(),
    context_id="body-context-17",
    quality=adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh,
)
assets = build_obj_assets(snapshot)
```

The caller supplies the already selected direct body in its assembly context.
The backend does not discover additional bodies or descendants. The caller must
give each body instance a distinct `context_id`, even when instances share the
same native body. IDs accept ASCII letters, digits, underscores, periods,
colons, and hyphens. Empty IDs, `.` and `..`, and path separators are invalid.

`root_from_body_cm` is a finite row-major affine matrix with 16 elements. Its
translation and the native tessellation coordinates use centimeters. It maps
the native body's coordinate frame to the root design frame. The backend
applies this matrix once, then multiplies positions by 10 to produce
millimeters. It does not apply an occurrence transform a second time. It
transforms normals with the inverse transpose, normalizes them, and reverses
triangle winding for a mirrored matrix. Singular and nonaffine matrices fail.

The backend reads each selected face's `nativeObject` for tessellation. A native
face has a null `nativeObject`. A proxy with an unresolved native face fails
rather than risking double placement. Geometry comes from
`face.meshManager.createMeshCalculator().calculate()`. The adapter reads
`nodeCoordinates`, `nodeIndices`, `triangleCount`, `normalVectors`, and, for an
image, `textureCoordinates`. It never creates features, copies bodies, rolls
the timeline, or reads or changes mass properties.

`quality` defaults to `None`, which leaves the calculator's defaults intact.
An explicit value passes directly to `setQuality`. Supply a documented
`TriangleMeshQualityOptions` enum, not an integer interpreted as mesh LOD.
Changing the calculator's settings does not change the design's bodies.

### Appearance priority and alpha

A nonnull `owner.appearance` overrides every selected face. Without that
override, the backend reads the selected face's current appearance. A null face
appearance permits the body's current appearance. An unreadable or unsupported
appearance fails instead of falling through to a different material.

The selected body's `visibleOpacity` supplies the inherited opacity-control
factor. Do not multiply the occurrence's opacity into that factor again. For a
constant color, the output alpha is the native color's byte opacity divided by
255, multiplied by `visibleOpacity`. For an image, MTL `d` contains
`visibleOpacity`, and the copied PNG retains its own alpha pixels.

The caller must supply the effective occurrence override if an ancestor
override applies. The module does not traverse the occurrence tree. Native
tests must confirm inherited opacity and ancestor override behavior for the
integration's chosen context objects.

### Strict native shader subset

The default `shader_policy="strict"` accepts an appearance with exactly one `ColorProperty`.
Its ID must be `opaque_albedo` or `generic_diffuse`. Its `hasMultipleValues`
must be false. No other appearance properties are accepted in strict mode.
This conservative rule intentionally rejects many real Fusion appearances.
It avoids silently discarding their additional shader semantics.

For a constant diffuse color, `hasConnectedTexture` must be false and `value`
must be an `adsk::core::Color`. All four byte channels must be finite and within
0 to 255. The adapter does not substitute a compatibility color or silver.

For an image, `hasConnectedTexture` must be true and `connectedTexture.textureType`
must equal documented `TextureTypes.ImageTexture`, value 1. All texture
properties must be typed `FilenameProperty` objects. Unknown mapping,
procedural, scalar, or additional texture properties fail. The adapter does
not guess their UV transforms.

Automatic image selection requires exactly one filename property. Its `id`
must be a nonempty string, its `hasMultipleValues` must be false, and its
`value` must be a nonempty string. The property ID is not hardcoded. There is
no assumed `AppearanceTexture.imageFile` getter or universal bitmap ID.

An explicit `texture_sources={face_ordinal: local_image_path}` selects the image
instead of automatic filename discovery. This can resolve missing or ambiguous
filename properties. It does not permit procedural textures or unknown mapping
properties. The face ordinal is zero-based within the selected body's face
collection. A source for an unselected face, or for an untextured face, fails.

The supported texture's UVs must be finite and contain one pair per mesh vertex.
The serializer preserves each triangle's node-to-UV association without
flipping V, normalizing UV ranges, or inventing UV indices. UVs outside 0 to 1
remain unchanged. Matching those values to Fusion's rendered image mapping is
a separate native and loader verification requirement.

Metallic `metal_f0`, glass `transparent_color`, procedural textures 2 through 9,
unknown texture type 0, multi-value shaders, and arbitrary PBR semantics are
unsupported. Reflection, refraction, emission, normal maps, roughness, and
render equivalence are not promised by this diffuse asset format. A failure is
not a fidelity-preserving gray fallback.

### Explicit diffuse-channel selection

`shader_policy="diffuse-subset"` is an explicit lossy alternative to strict mode.
It accepts additional appearance properties only with portable per-face loss
receipts. It never labels an arbitrary shader faithfully preserved. The default
strict mode continues to reject the same multi-property appearances.

```python
snapshot = read_native_body(
    selected_body_context,
    root_from_body_cm=native_body_to_root_matrix.asArray(),
    context_id="body-context-17",
    shader_policy="diffuse-subset",
    diffuse_properties={0: "generic_diffuse"},
    texture_sources={0: selected_local_image},
)
```

The subset must find exactly one known `ColorProperty` with ID `opaque_albedo`
or `generic_diffuse`. It does not use `Appearance.color` to guess a channel.
When both known IDs occur, `diffuse_properties` must explicitly select one ID
for that zero-based face ordinal. Duplicate occurrences of the selected ID
still fail. Selectors for unselected faces, unknown IDs, and selectors in strict
mode fail. Omit the mapping when the known channel is unambiguous.

The selected property's value, connected texture, and UV data have the same
strict validation in both modes. A missing, unreadable, multi-value, procedural,
or untyped selected channel fails. Unknown image mapping properties also fail
in both modes, even when an image path was explicitly selected. No mapping
transforms, automatic image conversion, procedural baking, or ancestor override
precedence are invented.

Other appearance properties are not interpreted or copied as shader values.
Every omitted property produces an ordered receipt with its `id`, API `type`,
and reason `omitted-shader-property`. Their values are not read. This includes
omitted roughness, metallic, transmission, reflection, and other shader data;
those semantics are lost, not approximated. A connected texture on an omitted
channel is not exported or claimed supported. A procedural texture on the
selected diffuse channel is always an error.

Property IDs are limited to 1 through 256 ASCII letters, digits, underscores,
periods, colons, and hyphens, excluding `.` and `..`. API types must have the
form `adsk::core::<alphanumeric class name>`. Unreadable or nonportable IDs/types
fail rather than leak source paths. No property values, display names, or local
filenames enter receipts.

Each face carries an `appearance` receipt. It records `shader_policy`, the
selected property ID/type, `source` (`owner`, `face`, or `body`), `losses`, and
`warnings`. Subset mode always includes
`lossy-diffuse-subset-not-shader-equivalent`, even when no extra properties were
present. Textured faces in either mode include
`provided-uvs-not-native-render-proof`. These are serializable data warnings,
not console-only messages. The selected body context and face ordinal identify
the receipt. No new ancestor priority rules are introduced.

## Local image contract

`read_local_image(path, max_bytes=16777216)` reads only the selected local
regular file. It performs no directory search, image-library lookup, download,
or network request. URLs, UNC-style paths, final-component symlinks, special
files, empty files, and oversized files fail. On systems with `O_NOFOLLOW`, the
open also rejects a final-component symlink race. Parent-directory symlinks and
the provenance of an already mounted filesystem are not sandbox boundaries.

The reader supports noninterlaced PNG, including RGBA and palette transparency.
It checks chunk framing, CRCs, supported header values, palette and transparency
structure, and bounded scanline decompression. The encoded limit is 16 MiB.
The dimension limit is 16384 per axis and 67108864 pixels. The decompressed
scanline limit is 64 MiB. Animated PNG, interlaced PNG, and other image formats
fail. The reader does not convert pixels or apply color management.
PNG ancillary metadata remains byte-for-byte intact. Source filenames and
paths are not added to the asset files or descriptors.

Common 8-bit Huffman baseline (SOF0) and progressive (SOF2) JPEGs are also accepted
as opaque selected image bytes. Grayscale and three-component images are
supported. The encoded limit is 16 MiB, the dimension limit is 16384 per axis,
and width times height times component count must not exceed 64 MiB. The parser
checks segment bounds, quantization and Huffman-table structure, component and
scan references, progressive coefficient-scan order, byte stuffing, restart
marker sequence, and a final EOI without trailing bytes. At most 1024 scans are
accepted. Each coefficient must have an initial scan. CMYK, arithmetic coding,
lossless JPEG, 12-bit JPEG, 16-bit quantization tables, and other frame types
are explicit unsupported cases. JPEG has no alpha channel; MTL opacity is still
the selected body's visible opacity.

JPEG validation is structural, not a full Huffman/entropy decoder. It cannot
establish that entropy bytes decode to the declared number of blocks or pixels,
or reject every bit-level corruption or entropy truncation before an intact
EOI. The backend does not make an untrusted-decoder safety guarantee. Real
loader decoding remains a required integration gate. The asset descriptor
includes `jpeg-structure-only-not-entropy-decoded` whenever JPEG is present.

PNG/JPEG bytes, including metadata, remain unchanged. JPEG orientation and ICC
metadata are not interpreted or normalized. Native-render mapping and actual
color management remain unverified. Canonical output suffixes are `.png` and
`.jpg`. Recognized source image suffixes must agree with the byte format;
`.jpg`/`.jpeg`/`.jpe` denote JPEG. A conflicting `.png`, JPEG suffix, or known
unsupported image suffix fails. Extensionless or neutral suffixes such as
`.bin` are detected by bytes. The reader accepts no separate MIME declaration.
The snapshot's asserted `format` must agree with its image bytes.

The result is `{"format": "png" | "jpeg", "data": bytes}`. This byte record is separate
from the JSON-serializable geometry descriptor.

## Snapshot and serialization contracts

`read_native_body` returns this shape:

```text
{
  "descriptor": {
    "schema": "fusion-visual-snapshot/1", "frame": "root", "units": "mm",
    "surfaces": [{
      "context_id": str, "face_index": int,
      "vertices_mm": [[x, y, z], ...], "normals": [[x, y, z], ...],
      "uvs": [[u, v], ...], "triangles": [[i, j, k], ...],
      "material": {"rgba": [r, g, b, a], "image": sha256_or_null},
      "appearance": {
        "shader_policy": "strict" | "diffuse-subset", "source": "owner" | "face" | "body",
        "selected_property": {"id": str, "type": "adsk::core::ColorProperty"},
        "losses": [{"id": str, "type": str, "reason": "omitted-shader-property"}, ...],
        "warnings": [str, ...]
      }
    }, ...]
  },
  "images": {sha256: {"format": "png" | "jpeg", "data": bytes}, ...}
}
```

The descriptor contains only JSON values, with finite numbers and zero-based
triangle indices. It contains no SDK objects, source filenames, or physical
properties. The `images` mapping is the only snapshot field that contains
bytes. The serializer accepts a descriptor reconstructed through JSON.

`build_obj_assets(snapshot_or_sequence)` accepts one snapshot or a sequence for
several bodies in one physical link. It preserves the supplied body and face
order. It validates coordinates, attributes, nondegenerate triangles, indices,
RGBA ranges, image hashes, and selected-image membership. Duplicate
`context_id, face_index` pairs fail. Inputs are not mutated.

The result contains a JSON-serializable `descriptor` and a separate `files`
mapping from package-relative paths to bytes. The descriptor has schema
`fusion-visual-assets/1`. It records `mesh_path`, `format`, root frame,
millimeter units, `urdf_scale`, a null `urdf_material_override`, triangle count,
per-face triangle ranges with copied appearance receipts, and every file's path,
SHA-256, and byte size. It also records each image's `format` and validation
level, plus a deduplicated sorted warning list. Receipt fields and policy-warning
consistency are validated, not silently dropped. Legacy snapshots without a
receipt remain consumable but yield a null per-face `appearance` and a
`missing-appearance-provenance` warning. Missing provenance is not strict proof.

All generated file names contain the full SHA-256 of their content. OBJ and
MTL files live under `meshes/`. PNG/JPEG images live under `meshes/textures/`. An OBJ's
`mtllib` references its sibling MTL. MTL `map_Kd` paths are relative to that
MTL's directory. MTL uses `Kd`, `d`, and `illum 1`. OBJ vertex, normal, and UV
indices have independent offsets. Identical images and materials deduplicate.
Material identifiers use `mat_` plus lowercase, unpadded Base32 of the complete
SHA-256 digest: 56 ASCII characters, retaining all hash bits while fitting
Blender's 63-byte datablock-name limit. OBJ, MTL, descriptor, and bundle validation
derive the same identifier. This avoids importer truncation rather than guessing
which full material name a truncated prefix meant.
Conflicting image or material hashes fail instead of overwriting data. The
same ordered inputs produce identical bytes and paths.

The backend writes no files. Its relative paths contain no caller-supplied
names or path components. Package-contained paths do not replace the package
writer's responsibility to reject symlinks and collisions in its destination.

## Package integration contract

The opt-in package integration performs these steps:

1. Use the same immutable direct-body selection as physics extraction. Snapshot
   each body in its current context, with its native-to-root matrix.
2. Call `build_obj_assets` for each physical link before publishing any files.
   Keep the returned SDK-free descriptors in the writer's intermediate data.
   Persist all per-face loss receipts, image-validation levels, and warnings in
   the package provenance. Do not rename subset mode to shader fidelity.
3. Stage every entry of `files` in the writer's fresh package staging directory.
   Validate containment, duplicate destinations, sizes, and SHA-256 values.
   Reject an existing conflicting destination. Do not overwrite shared output.
4. Point the link's URDF visual at `package://<package>/<mesh_path>`. Use scale
   `0.001 0.001 0.001`. Keep the existing inverse-link-pose visual origin because
   the OBJ is in the root frame. Do not transform the mesh again.
5. Omit the URDF visual material override. A link-wide RGBA override can erase
   the embedded face colors and texture. Do not use this OBJ as a new physics
   source or change the existing collision policy.
6. Publish through the existing transactional writer only after all image,
   mesh, descriptor, and package checks succeed. Any preservation error aborts
   the opt-in export. Keep the current default behavior unchanged.

`validate_asset_bundles` validates plain descriptor schemas, policy receipts,
per-face material/UV association, positive triangle ranges, exact resource
membership, content-addressed contained paths, sizes, and SHA-256 values.
It accepts only the serializer's bounded OBJ/MTL/PNG/JPEG dialect. Each bundle
has at most 2048 files, each file at most 64 MiB, and package resources at most
128 MiB and 4096 files. Images retain their stricter 16 MiB limits. The writer
uses exclusive file creation in its fresh stage and rejects existing paths or
symlink destinations. A failure leaves the final destination unpublished.

`--fixture=materials` adds a fixed two-link, four-face-per-link material case
through the existing registry and wrappers. It includes diffuse alpha, PNG,
baseline JPEG, progressive JPEG, explicit omitted-property receipts, and literal
decoded-pixel expectations. It never becomes a native bundle. The independent
loader checks every material part, face/UV association, decoded image, and file
hash. Blender imports all OBJ objects with their intrinsic transforms and
materials. Neither verifier may coalesce away the all-elements assertion.

OBJ and MTL support differs among ROS visualization and mesh loaders. The
integrator must verify resource resolution, PNG alpha, JPEG decoding, MTL `d`, image UVs, and
root-frame placement with the actual loader and Blender. A passing byte decoder
does not prove those tools render the materials equivalently.

## Verification and evidence limits

Run the focused and full offline suites from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_visual_assets tests.test_fusion_materials -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```

The focused tests independently decode OBJ triangles, MTL colors and opacity,
and a two-pixel PNG. They check two offset bodies, face overrides, repeated
native geometry in different contexts, inverse-transpose normals, image bytes,
finite UVs, stable paths, source preservation, hash conflicts, and strict error
paths. `tests.test_visual_assets.write_evidence_scene(directory)` retains a
three-face analytic scene and an expected-value table for later loader checks.
Additional tests cover ordinary multi-property appearances, explicit diffuse
selection, per-face receipts, selected PNG/JPEG sources, and strict rejection
of unsupported mapping/procedural data. The embedded 2-by-2 baseline JPEG is
the parent's immutable fixture with SHA-256
`1bd8b829eee3160443ad3bada64f6ec0a0c76965c9b356421ba093db82947be8`.
The progressive fixture is an analytic 1-by-1 grayscale two-scan JPEG, expected
to decode to 128. Byte preservation is tested here, not full JPEG pixel decoding.

These are native API doubles and analytic byte-level tests. They do not prove
live Fusion tessellation frames, inherited appearances, native texture mapping,
history preservation, or actual viewer alpha. Full Fusion appearances commonly
contain extra properties and can fail strict mode. Explicit subset mode records
their omitted semantics instead of silently accepting them. The module
does not establish reproduction or completion of upstream issues 40 or 66, or
cadop issue 11. Public configuration, transactional glue, native tests, and
ROS 2 loader checks and live Fusion remain separate evidence requirements.

## Current official API references

The following Autodesk references were read on October 20, 2026:

- [TriangleMesh data and per-vertex texture coordinates](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_TriangleMesh.htm).
- [Selected body faces](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepBody_faces.htm) and [BRepFaces collection iteration](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepFaces.htm).
- [Face mesh manager](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepFace_meshManager.htm) and [mesh calculator creation](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_MeshManager_createMeshCalculator.htm).
- [TriangleMeshCalculator](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_TriangleMeshCalculator.htm) and [setQuality's enum parameter](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_TriangleMeshCalculator_setQuality.htm).
- [Face nativeObject](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepFace_nativeObject.htm) and [assemblyContext](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepFace_assemblyContext.htm).
- [Occurrence appearance override](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Occurrence_appearance.htm), [current face appearance](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepFace_appearance.htm), and [current body appearance](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepBody_appearance.htm).
- [Body opacity control](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepBody_opacity.htm) and [inherited visibleOpacity](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_BRepBody_visibleOpacity.htm).
- [Appearance primary color](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_Appearance_color.htm) and [ColorProperty texture access](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_ColorProperty.htm).
- [Color byte channels and opacity](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_Color.htm).
- [AppearanceTexture property collection](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_AppearanceTexture.htm), [TextureTypes](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_TextureTypes.htm), and [Properties collection iteration](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_Properties.htm).
- [Typed FilenameProperty](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_FilenameProperty.htm) and [its string value](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_FilenameProperty_value.htm).
