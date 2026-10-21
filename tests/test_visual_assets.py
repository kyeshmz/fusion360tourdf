import base64
import copy
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

from URDF_Exporter.utils.visual_assets import (
    VisualAssetError, build_obj_assets, read_local_image, read_native_body,
)


IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
JPEG_SHA256 = '1bd8b829eee3160443ad3bada64f6ec0a0c76965c9b356421ba093db82947be8'
KNOWN_JPEG = base64.b64decode(
    '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAIBAQEBAQIBAQECAgICAgQDAgICAgUEBAMEBgUGBgYFBgYGBwkIBgcJBwYGCAsICQoK'
    'CgoKBggLDAsKDAkKCgr/2wBDAQICAgICAgUDAwUKBwYHCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoK'
    'CgoKCgoKCgr/wAARCAACAAIDAREAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUF'
    'BAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVW'
    'V1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi'
    '4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAEC'
    'AxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVm'
    'Z2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq'
    '8vP09fb3+Pn6/9oADAMBAAIRAxEAPwD9YP8AgnB8CPgf4l/4J4/AXxF4i+DXhTUNQ1D4L+Frm/v73w7bSzXMz6Tas8kjshZ3ZiWL'
    'Ekkkk16Hij4e8A4XxMzuhQynDQhDGYlRiqFJJJVppJJQskloktEj4/EcF8HZ/XnmeZ5bh6+Jrt1KtWpRpzqVKk3zTqVJyi5TnOTc'
    'pzk3KUm222z/2Q==')


def progressive_gray_jpeg():
    def segment(marker, payload):
        return bytes((255, marker)) + struct.pack('>H', len(payload) + 2) + payload
    tables = segment(0xdb, b'\x00' + b'\x01' * 64)
    tables += segment(0xc4, b'\x00\x01' + b'\x00' * 15 + b'\x00')
    tables += segment(0xc4, b'\x10\x01' + b'\x00' * 15 + b'\x00')
    frame = segment(0xc2, b'\x08\x00\x01\x00\x01\x01\x01\x11\x00')
    dc = segment(0xda, b'\x01\x01\x00\x00\x00\x00') + b'\x7f'
    ac = segment(0xda, b'\x01\x01\x00\x01\x3f\x00') + b'\x7f'
    return b'\xff\xd8' + tables + frame + dc + ac + b'\xff\xd9'


def with_properties(appearance, *extras):
    return Frozen(appearanceProperties=Collection(appearance.appearanceProperties.values + extras))


class Frozen:
    def __init__(self, **attributes):
        self.__dict__.update(attributes)

    def __setattr__(self, name, value):
        raise AssertionError('Source mutation: ' + name)


class Collection(Frozen):
    def __init__(self, items):
        super().__init__(values=tuple(items), count=len(items))

    def item(self, index):
        return self.values[index]


def color_appearance(red, green, blue, alpha=255, identifier='opaque_albedo'):
    color = Frozen(objectType='adsk::core::Color', red=red, green=green, blue=blue, opacity=alpha)
    prop = Frozen(objectType='adsk::core::ColorProperty', id=identifier, value=color,
                  hasMultipleValues=False, hasConnectedTexture=False)
    return Frozen(appearanceProperties=Collection([prop]))


def image_appearance(path, *, texture_type=1, filename_properties=None, identifier='generic_diffuse'):
    if filename_properties is None:
        filename_properties = [Frozen(objectType='adsk::core::FilenameProperty', id='vendor-selected-image',
                                      hasMultipleValues=False, value=str(path))]
    texture = Frozen(textureType=texture_type, properties=Collection(filename_properties))
    prop = Frozen(objectType='adsk::core::ColorProperty', id=identifier,
                  hasMultipleValues=False, hasConnectedTexture=True, value=None, connectedTexture=texture)
    return Frozen(appearanceProperties=Collection([prop]))


def mesh(*, vertices=None, indices=None, normals=None, uvs=None):
    vertices = vertices if vertices is not None else [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
    indices = indices if indices is not None else [2, 0, 1]
    normals = normals if normals is not None else [(0, 0, 1)] * len(vertices)
    uvs = uvs if uvs is not None else [(0, 0), (1, 0), (0, 1)]
    return Frozen(nodeCoordinates=tuple(Frozen(x=x, y=y, z=z) for x, y, z in vertices),
                  nodeIndices=tuple(indices), triangleCount=len(indices) // 3,
                  normalVectors=tuple(Frozen(x=x, y=y, z=z) for x, y, z in normals),
                  textureCoordinates=tuple(Frozen(x=x, y=y) for x, y in uvs))


class Calculator:
    def __init__(self, data, quality_calls):
        self.data = data
        self.quality_calls = quality_calls

    def setQuality(self, quality):
        self.quality_calls.append(quality)
        return True

    def calculate(self):
        return self.data


class MeshManager(Frozen):
    def __init__(self, data, quality_calls=None):
        super().__init__(data=data, quality_calls=[] if quality_calls is None else quality_calls)

    def createMeshCalculator(self):
        return Calculator(self.data, self.quality_calls)


def face(appearance=None, data=None, *, native=None, quality_calls=None):
    return Frozen(appearance=appearance, nativeObject=native,
                  meshManager=MeshManager(mesh() if data is None else data, quality_calls))


def body(faces, appearance=None, opacity=1):
    return Frozen(faces=Collection(faces), appearance=appearance, visibleOpacity=opacity)


def snapshot(source, context='body:1', transform=None, **kwargs):
    return read_native_body(source, context_id=context,
                            root_from_body_cm=IDENTITY if transform is None else transform, **kwargs)


def png_rgba(width=2, height=1, pixels=b'\xff\x00\x00\xff\x00\xff\x00\x40'):
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data))
    rows = b''.join(b'\x00' + pixels[row*width*4:(row+1)*width*4] for row in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 6, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


def decode_obj(asset):
    vertices, uvs, normals, triangles, libraries = [], [], [], [], []
    material = None
    for line in asset.decode('ascii').splitlines():
        fields = line.split()
        if not fields:
            continue
        key, values = fields[0], fields[1:]
        if key == 'v':
            vertices.append(tuple(map(float, values)))
        elif key == 'vt':
            uvs.append(tuple(map(float, values)))
        elif key == 'vn':
            normals.append(tuple(map(float, values)))
        elif key == 'mtllib':
            libraries.append(values[0])
        elif key == 'usemtl':
            material = values[0]
        elif key == 'f':
            corners = []
            for value in values:
                v, vt, vn = value.split('/')
                corners.append((vertices[int(v)-1], uvs[int(vt)-1] if vt else None, normals[int(vn)-1]))
            triangles.append((material, corners))
    return {'vertices': vertices, 'uvs': uvs, 'normals': normals, 'triangles': triangles, 'libraries': libraries}


def decode_mtl(asset):
    materials = {}
    current = None
    for line in asset.decode('ascii').splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'newmtl':
            current = materials.setdefault(fields[1], {})
        else:
            current[fields[0]] = fields[1:]
    return materials


def decode_fixture_png(asset):
    chunks, offset, compressed = {}, 8, b''
    while offset < len(asset):
        length = struct.unpack('>I', asset[offset:offset+4])[0]
        kind = asset[offset+4:offset+8]
        payload = asset[offset+8:offset+8+length]
        chunks[kind] = payload
        if kind == b'IDAT':
            compressed += payload
        offset += 12 + length
    width, height = struct.unpack('>II', chunks[b'IHDR'][:8])
    raw = zlib.decompress(compressed)
    rows = []
    for y in range(height):
        row = raw[y*(1+width*4):(y+1)*(1+width*4)]
        if row[0] != 0:
            raise AssertionError('Expected independently decodable unfiltered fixture')
        rows.extend(tuple(row[i:i+4]) for i in range(1, len(row), 4))
    return width, height, rows


def decode_assets(assets):
    descriptor, files = assets['descriptor'], assets['files']
    obj_path = PurePosixPath(descriptor['mesh_path'])
    obj = decode_obj(files[str(obj_path)])
    mtl_path = obj_path.parent / obj['libraries'][0]
    return obj, decode_mtl(files[str(mtl_path)]), mtl_path.parent


def write_evidence_scene(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    image_path = directory / 'selected-source.png'
    image_path.write_bytes(png_rgba())
    red = color_appearance(255, 0, 0, 128)
    blue = color_appearance(0, 0, 255)
    native = face(red)
    blue_mesh = mesh(vertices=[(1, 0, 0), (2, 0, 0), (1, 1, 0)])
    first = snapshot(body([native, face(blue, blue_mesh)], opacity=0.5), context='body:red')
    transform = IDENTITY[:]
    transform[3], transform[7], transform[11] = 3, -2, 1
    second = snapshot(body([face(image_appearance(image_path), native=native)]),
                      context='body:image', transform=transform)
    assets = build_obj_assets([first, second])
    for path, data in assets['files'].items():
        target = directory / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    (directory / 'descriptor.json').write_text(json.dumps(assets['descriptor'], indent=2) + '\n')
    oracle = {'evidence_kind': 'analytic-native-api-doubles-not-live-fusion', 'body_contexts': 2,
              'faces': 3, 'triangles': 3, 'vertices': 9, 'image_pixels': 2,
              'bounds_mm': [[0, -20, 0], [40, 10, 10]],
              'triangle_rgba': [[1, 0, 0, 128/255*0.5], [0, 0, 1, 0.5], [1, 1, 1, 1]],
              'texture_pixels_rgba8': [[255, 0, 0, 255], [0, 255, 0, 64]],
              'source_image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest()}
    (directory / 'oracle.json').write_text(json.dumps(oracle, indent=2) + '\n')
    return assets


def write_subset_evidence_scene(directory):
    directory = Path(directory)
    source_directory = directory / 'source-inputs'
    source_directory.mkdir(parents=True, exist_ok=True)
    payloads = {'selected.png': png_rgba(), 'selected.jpg': KNOWN_JPEG,
                'progressive.jpg': progressive_gray_jpeg()}
    for name, data in payloads.items():
        (source_directory / name).write_bytes(data)
    extras = (Frozen(id='generic_roughness', objectType='adsk::core::FloatProperty'),
              Frozen(id='generic_is_metal', objectType='adsk::core::BooleanProperty'))
    appearances = [with_properties(color_appearance(255, 0, 0, 128), *extras)]
    appearances.extend(with_properties(image_appearance(source_directory / name), *extras)
                       for name in payloads)
    pose = IDENTITY[:]
    pose[3], pose[7], pose[11] = 3, -2, 1
    first = snapshot(body([face(a) for a in appearances[:2]], opacity=0.5),
                     context='subset:first', shader_policy='diffuse-subset')
    second = snapshot(body([face(a) for a in appearances[2:]]), context='subset:second',
                      transform=pose, shader_policy='diffuse-subset')
    assets = build_obj_assets([first, second])
    for path, data in assets['files'].items():
        target = directory / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    (directory / 'descriptor.json').write_text(json.dumps(assets['descriptor'], indent=2) + '\n')
    oracle = {'evidence_kind': 'analytic-native-api-doubles-not-live-fusion',
              'body_contexts': 2, 'faces': 4, 'triangles': 4, 'vertices': 12, 'uvs': 9,
              'loss_receipts': 8, 'image_count': 3, 'image_pixel_samples': 7,
              'bounds_mm': [[0, -20, 0], [40, 10, 10]],
              'constant_rgba': [1, 0, 0, 128/255*0.5],
              'png_rgba8': [[255, 0, 0, 255], [0, 255, 0, 64]],
              'baseline_rgb_parent_oracle': [[245, 2, 4], [3, 255, 2], [2, 0, 247], [255, 249, 250]],
              'progressive_gray8_analytic': [128],
              'jpeg_entropy_decode_by_worker': False,
              'source_hashes': {name: hashlib.sha256(data).hexdigest() for name, data in payloads.items()}}
    (directory / 'oracle.json').write_text(json.dumps(oracle, indent=2) + '\n')
    assert all((source_directory / name).read_bytes() == data for name, data in payloads.items())
    return assets


class VisualAssetsTest(unittest.TestCase):
    def test_legacy_missing_receipt_is_marked_unknown_not_claimed_strict(self):
        native = snapshot(body([face(color_appearance(1, 2, 3))]))
        del native['descriptor']['surfaces'][0]['appearance']
        assets = build_obj_assets(native)
        self.assertIsNone(assets['descriptor']['surfaces'][0]['appearance'])
        self.assertIn('missing-appearance-provenance', assets['descriptor']['warnings'])

    def test_jpeg_structural_validation_explicitly_does_not_certify_entropy(self):
        scan = KNOWN_JPEG.index(b'\xff\xda')
        entropy = scan + 2 + int.from_bytes(KNOWN_JPEG[scan+2:scan+4], 'big')
        opaque_data = KNOWN_JPEG[:entropy] + b'\x00\xff\xd9'
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'structure-only.jpg'
            path.write_bytes(opaque_data)
            assets = build_obj_assets(snapshot(body([face(image_appearance(path))])))
            self.assertIn('jpeg-structure-only-not-entropy-decoded', assets['descriptor']['warnings'])
            self.assertEqual(assets['descriptor']['images'][0]['validation'], 'structure-only-not-entropy-decoded')
            self.assertIn(opaque_data, assets['files'].values())

    def test_retained_subset_scene_has_exact_loss_counts_paths_and_source_copies(self):
        with tempfile.TemporaryDirectory() as temporary:
            assets = write_subset_evidence_scene(temporary)
            obj, mtl, parent = decode_assets(assets)
            self.assertEqual(len(obj['triangles']), 4)
            self.assertEqual(len(obj['vertices']), 12)
            self.assertEqual(len(obj['uvs']), 9)
            self.assertEqual(sum(len(s['appearance']['losses']) for s in assets['descriptor']['surfaces']), 8)
            self.assertEqual(len(assets['descriptor']['images']), 3)
            self.assertEqual([min(v[i] for v in obj['vertices']) for i in range(3)], [0, -20, 0])
            self.assertEqual([max(v[i] for v in obj['vertices']) for i in range(3)], [40, 10, 10])
            self.assertIn('jpeg-structure-only-not-entropy-decoded', assets['descriptor']['warnings'])
            for name, _ in obj['triangles'][1:]:
                self.assertIn(str(parent / mtl[name]['map_Kd'][0]), assets['files'])
            for entry in assets['descriptor']['files']:
                path = PurePosixPath(entry['path'])
                self.assertFalse(path.is_absolute())
                self.assertNotIn('..', path.parts)
                self.assertIn(entry['sha256'], path.name)

    def test_subset_missing_and_unreadable_selected_data_never_falls_back(self):
        class UnreadableValue:
            id = 'opaque_albedo'
            objectType = 'adsk::core::ColorProperty'
            hasMultipleValues = False
            hasConnectedTexture = False
            @property
            def value(self):
                raise RuntimeError('/private/source')
        unreadable = Frozen(appearanceProperties=Collection([UnreadableValue()]))
        source = body([face(color_appearance(0, 255, 0))], color_appearance(0, 0, 255))
        with self.assertRaisesRegex(VisualAssetError, '^unreadable-native-value$'):
            snapshot(source, owner=Frozen(appearance=unreadable), shader_policy='diffuse-subset')
        with self.assertRaisesRegex(VisualAssetError, '^unreadable-image-source$'):
            snapshot(body([face(image_appearance('/missing/selected.jpg'))]), shader_policy='diffuse-subset')
        with self.assertRaises(VisualAssetError):
            snapshot(source, shader_policy='diffuse-subset', diffuse_properties={0: []})

    def test_subset_selects_diffuse_from_ordinary_shader_and_retains_per_face_loss(self):
        extras = (Frozen(id='generic_roughness', objectType='adsk::core::FloatProperty'),
                  Frozen(id='generic_is_metal', objectType='adsk::core::BooleanProperty'))
        appearance = with_properties(color_appearance(12, 128, 240, 64), *extras)
        source = body([face(), face(appearance)], appearance, opacity=0.5)
        with self.assertRaisesRegex(VisualAssetError, 'shader'):
            snapshot(source)
        native = snapshot(source, shader_policy='diffuse-subset')
        before = copy.deepcopy(native)
        assets = build_obj_assets(native)
        self.assertEqual(before, native)
        self.assertEqual(assets, build_obj_assets(native))
        obj, mtl, _ = decode_assets(assets)
        for index, surface in enumerate(assets['descriptor']['surfaces']):
            receipt = surface['appearance']
            self.assertEqual(receipt['shader_policy'], 'diffuse-subset')
            self.assertEqual(receipt['source'], ('body', 'face')[index])
            self.assertEqual(receipt['selected_property'], {'id': 'opaque_albedo',
                                                          'type': 'adsk::core::ColorProperty'})
            self.assertEqual(receipt['losses'], [dict(id=p.id, type=p.objectType,
                                                       reason='omitted-shader-property') for p in extras])
            self.assertIn('lossy-diffuse-subset-not-shader-equivalent', receipt['warnings'])
        self.assertEqual(tuple(map(float, mtl[obj['triangles'][0][0]]['Kd'])), (12/255, 128/255, 240/255))
        self.assertEqual(float(mtl[obj['triangles'][0][0]]['d'][0]), 64/255*0.5)
        json.dumps(assets['descriptor'], allow_nan=False)

    def test_subset_ambiguity_needs_explicit_known_unique_diffuse_id(self):
        first = color_appearance(255, 0, 0).appearanceProperties.item(0)
        second = color_appearance(0, 0, 255, identifier='generic_diffuse').appearanceProperties.item(0)
        appearance = Frozen(appearanceProperties=Collection([first, second]))
        source = body([face(appearance)])
        with self.assertRaisesRegex(VisualAssetError, 'ambiguous-diffuse'):
            snapshot(source, shader_policy='diffuse-subset')
        native = snapshot(source, shader_policy='diffuse-subset', diffuse_properties={0: 'generic_diffuse'})
        self.assertEqual(native['descriptor']['surfaces'][0]['material']['rgba'], [0, 0, 1, 1])
        self.assertEqual(native['descriptor']['surfaces'][0]['appearance']['losses'][0]['id'], 'opaque_albedo')
        duplicate = Frozen(appearanceProperties=Collection([first, first]))
        with self.assertRaisesRegex(VisualAssetError, 'ambiguous-diffuse'):
            snapshot(body([face(duplicate)]), shader_policy='diffuse-subset',
                     diffuse_properties={0: 'opaque_albedo'})
        for kwargs in ({'shader_policy': 'anything'}, {'diffuse_properties': {0: 'opaque_albedo'}},
                       {'shader_policy': 'diffuse-subset', 'diffuse_properties': {0: 'metal_f0'}},
                       {'shader_policy': 'diffuse-subset', 'diffuse_properties': {1: 'opaque_albedo'}}):
            with self.subTest(kwargs=kwargs), self.assertRaises(VisualAssetError):
                snapshot(source, **kwargs)

    def test_subset_does_not_guess_missing_untyped_or_multivalue_diffuse(self):
        for appearance in (color_appearance(255, 0, 0, identifier='transparent_color'),
                           Frozen(appearanceProperties=Collection([])),
                           Frozen(appearanceProperties=Collection([
                               Frozen(id='opaque_albedo', objectType='adsk::core::FloatProperty')]))):
            with self.subTest(appearance=appearance), self.assertRaises(VisualAssetError):
                snapshot(body([face(appearance)]), shader_policy='diffuse-subset')
        prop = color_appearance(255, 0, 0).appearanceProperties.item(0)
        malformed = Frozen(**dict(vars(prop), hasMultipleValues=True))
        with self.assertRaises(VisualAssetError):
            snapshot(body([face(Frozen(appearanceProperties=Collection([malformed])))]),
                     shader_policy='diffuse-subset')

    def test_subset_requires_portable_readable_loss_ids_and_types_not_values(self):
        class Unreadable:
            objectType = 'adsk::core::FloatProperty'
            @property
            def id(self):
                raise RuntimeError('/private/source')
        for extra in (Frozen(id='/private/source', objectType='adsk::core::FloatProperty'),
                      Frozen(id='roughness', objectType='/private/type'),
                      Frozen(id=[], objectType='adsk::core::FloatProperty'), Unreadable()):
            with self.subTest(extra=extra):
                with self.assertRaises(VisualAssetError) as error:
                    snapshot(body([face(with_properties(color_appearance(1, 2, 3), extra))]),
                             shader_policy='diffuse-subset')
                self.assertNotIn('/private', str(error.exception))

    def test_subset_png_and_jpeg_keep_bytes_uvs_and_opaque_jpeg_opacity(self):
        with tempfile.TemporaryDirectory() as temporary:
            paths = [Path(temporary) / name for name in ('private.png', 'private.jpeg')]
            for path, data in zip(paths, (png_rgba(), KNOWN_JPEG)):
                path.write_bytes(data)
            extra = Frozen(id='generic_roughness', objectType='adsk::core::FloatProperty')
            appearances = [with_properties(image_appearance(path), extra) for path in paths]
            source = body([face(appearance) for appearance in appearances], opacity=0.5)
            native = snapshot(source, shader_policy='diffuse-subset')
            before = copy.deepcopy(native)
            assets = build_obj_assets(native)
            obj, mtl, parent = decode_assets(assets)
            for index, data in enumerate((png_rgba(), KNOWN_JPEG)):
                name, corners = obj['triangles'][index]
                relative = mtl[name]['map_Kd'][0]
                self.assertTrue(relative.endswith(('.png', '.jpg')[index]))
                self.assertEqual(assets['files'][str(parent / relative)], data)
                self.assertEqual(paths[index].read_bytes(), data)
                self.assertEqual([corner[1] for corner in corners], [(0, 1), (0, 0), (1, 0)])
                self.assertEqual(mtl[name]['d'], ['0.5'])
                receipt = assets['descriptor']['surfaces'][index]['appearance']
                self.assertIn('provided-uvs-not-native-render-proof', receipt['warnings'])
            self.assertEqual(native, before)
            self.assertNotIn(str(temporary), json.dumps(assets['descriptor']))
            reconstructed = {'descriptor': json.loads(json.dumps(native['descriptor'])), 'images': native['images']}
            self.assertEqual(assets, build_obj_assets(reconstructed))

    def test_subset_still_rejects_selected_procedural_and_unknown_texture_mapping(self):
        for texture_type in (0, 2, 3, 4, 5, 6, 7, 8, 9):
            with self.subTest(texture_type=texture_type), self.assertRaisesRegex(VisualAssetError, 'procedural'):
                snapshot(body([face(image_appearance('not-read', texture_type=texture_type))]),
                         shader_policy='diffuse-subset')
        mapping = Frozen(id='texture_scale', objectType='adsk::core::FloatProperty', value=2)
        with self.assertRaisesRegex(VisualAssetError, 'texture-properties'):
            snapshot(body([face(image_appearance(None, filename_properties=[mapping]))]),
                     shader_policy='diffuse-subset', texture_sources={0: 'not-read'})

    def test_loss_receipts_survive_serialization_and_invalid_receipts_fail(self):
        native = snapshot(body([face(color_appearance(1, 2, 3))]), shader_policy='diffuse-subset')
        valid = native['descriptor']['surfaces'][0]['appearance']
        for key, value in (('shader_policy', 'arbitrary-fidelity'), ('source', '/private/path'),
                           ('losses', [{'id': '/private/path', 'type': 'adsk::core::FloatProperty',
                                        'reason': 'omitted-shader-property'}]), ('warnings', [])):
            changed = copy.deepcopy(native)
            changed['descriptor']['surfaces'][0]['appearance'][key] = value
            with self.subTest(key=key), self.assertRaises(VisualAssetError):
                build_obj_assets(changed)
        self.assertEqual(build_obj_assets(native)['descriptor']['surfaces'][0]['appearance'], valid)

    def test_known_baseline_and_analytic_progressive_jpeg_are_byte_preserved(self):
        self.assertEqual(hashlib.sha256(KNOWN_JPEG).hexdigest(), JPEG_SHA256)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'source.jpg'
            for data in (KNOWN_JPEG, progressive_gray_jpeg()):
                with self.subTest(size=len(data)):
                    path.write_bytes(data)
                    self.assertEqual(read_local_image(path), {'format': 'jpeg', 'data': data})
                    assets = build_obj_assets(snapshot(body([face(image_appearance(path))])))
                    obj, mtl, parent = decode_assets(assets)
                    reference = mtl[obj['triangles'][0][0]]['map_Kd'][0]
                    self.assertEqual(reference, 'textures/' + hashlib.sha256(data).hexdigest() + '.jpg')
                    self.assertEqual(assets['files'][str(parent / reference)], data)

    def test_jpeg_truncation_markers_tables_dimensions_and_unsupported_modes_fail(self):
        frame = KNOWN_JPEG.index(b'\xff\xc0')
        scan = KNOWN_JPEG.index(b'\xff\xda')
        huffman = KNOWN_JPEG.index(b'\xff\xc4')
        quantization = KNOWN_JPEG.index(b'\xff\xdb')
        cases = [KNOWN_JPEG[:-1], KNOWN_JPEG + b'junk', KNOWN_JPEG[:scan] + b'\xff\xd9',
                 KNOWN_JPEG[:frame+5] + b'\x00\x00' + KNOWN_JPEG[frame+7:],
                 KNOWN_JPEG[:frame+5] + b'\xff\xff' + KNOWN_JPEG[frame+7:],
                 KNOWN_JPEG[:frame+1] + b'\xc1' + KNOWN_JPEG[frame+2:],
                 KNOWN_JPEG[:frame+4] + b'\x0c' + KNOWN_JPEG[frame+5:],
                 KNOWN_JPEG[:scan+4] + b'\x04' + KNOWN_JPEG[scan+5:],
                 KNOWN_JPEG[:scan+5] + b'\x09' + KNOWN_JPEG[scan+6:],
                 KNOWN_JPEG[:huffman+5] + b'\xff' + KNOWN_JPEG[huffman+6:],
                 KNOWN_JPEG[:quantization+5] + b'\x00' + KNOWN_JPEG[quantization+6:],
                 KNOWN_JPEG[:scan] + b'\xff\xda\x00\x01',
                 progressive_gray_jpeg()[:-3] + b'\xff\xd9',
                 b'\xff\xd8\xff\xdb\x00\x43\x00' + b'\x00'*64 + b'\xff\xd9']
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'source.jpg'
            for index, data in enumerate(cases):
                path.write_bytes(data)
                with self.subTest(index=index), self.assertRaises(VisualAssetError):
                    read_local_image(path)
            path.write_bytes(KNOWN_JPEG)
            with self.assertRaises(VisualAssetError):
                read_local_image(path, max_bytes=len(KNOWN_JPEG)-1)

    def test_declared_image_format_and_recognized_suffix_cannot_conflict(self):
        with tempfile.TemporaryDirectory() as temporary:
            for suffix, data in (('.png', KNOWN_JPEG), ('.jpg', png_rgba()), ('.gif', KNOWN_JPEG)):
                path = Path(temporary) / ('source' + suffix)
                path.write_bytes(data)
                with self.subTest(suffix=suffix), self.assertRaisesRegex(VisualAssetError, 'image-format'):
                    read_local_image(path)
            path = Path(temporary) / 'selected.jpg'
            path.write_bytes(KNOWN_JPEG)
            native = snapshot(body([face(image_appearance(path))]))
            next(iter(native['images'].values()))['format'] = 'png'
            with self.assertRaises(VisualAssetError):
                build_obj_assets(native)

    def test_empty_visual_is_not_a_successful_asset(self):
        with self.assertRaises(VisualAssetError):
            build_obj_assets({'descriptor': {'surfaces': []}, 'images': {}})

    def test_two_offset_bodies_face_override_alpha_and_exact_triangle_association(self):
        blue_mesh = mesh(vertices=[(1, 0, 0), (2, 0, 0), (1, 1, 0)])
        first = body([face(), face(color_appearance(0, 0, 255), blue_mesh)],
                     color_appearance(255, 0, 0, 128, identifier='generic_diffuse'), 0.5)
        pose = IDENTITY[:]
        pose[3], pose[7], pose[11] = 3, -2, 1
        second = body([face()], color_appearance(0, 255, 0, 64), 0.25)
        assets = build_obj_assets([snapshot(first, 'first'), snapshot(second, 'second', pose)])
        obj, mtl, _ = decode_assets(assets)
        self.assertEqual(len(obj['triangles']), 3)
        expected_colors = [(1, 0, 0, 128/255*0.5), (0, 0, 1, 0.5), (0, 1, 0, 64/255*0.25)]
        expected_triangles = [[(0, 10, 0), (0, 0, 0), (10, 0, 0)],
                              [(10, 10, 0), (10, 0, 0), (20, 0, 0)]]
        expected_triangles += [[(30, -10, 10), (30, -20, 10), (40, -20, 10)]]
        for (name, corners), rgba, triangle in zip(obj['triangles'], expected_colors, expected_triangles):
            self.assertEqual(tuple(map(float, mtl[name]['Kd'] + mtl[name]['d'])), rgba)
            self.assertEqual([corner[0] for corner in corners], triangle)
            self.assertEqual([corner[2] for corner in corners], [(0, 0, 1)] * 3)
        self.assertEqual(assets['descriptor']['urdf_scale'], [0.001] * 3)
        self.assertIsNone(assets['descriptor']['urdf_material_override'])
        self.assertEqual([p['first_triangle'] for p in assets['descriptor']['surfaces']], [0, 1, 2])

    def test_occurrence_override_replaces_all_face_and_body_colors(self):
        source = body([face(color_appearance(255, 0, 0))], color_appearance(0, 255, 0), 0.75)
        owner = Frozen(appearance=color_appearance(0, 0, 255, 128))
        obj, mtl, _ = decode_assets(build_obj_assets(snapshot(source, owner=owner)))
        self.assertEqual(tuple(map(float, mtl[obj['triangles'][0][0]]['Kd'])), (0, 0, 1))
        self.assertEqual(float(mtl[obj['triangles'][0][0]]['d'][0]), 128/255*0.75)

    def test_null_occurrence_override_preserves_contents(self):
        source = body([face(color_appearance(255, 0, 0))])
        result = snapshot(source, owner=Frozen(appearance=None))
        self.assertEqual(result['descriptor']['surfaces'][0]['material']['rgba'], [1, 0, 0, 1])

    def test_repeated_native_face_gets_each_context_pose_once(self):
        native = face(color_appearance(255, 0, 0))
        poisoned_proxy = mesh(vertices=[(999, 0, 0), (999, 1, 0), (999, 0, 1)])
        proxy = face(color_appearance(0, 255, 0), poisoned_proxy, native=native)
        first_pose = [0, -1, 0, 2, 1, 0, 0, 3, 0, 0, 1, 4, 0, 0, 0, 1]
        second_pose = IDENTITY[:]
        second_pose[3] = -5
        assets = build_obj_assets([snapshot(body([proxy]), 'repeat:1', first_pose),
                                   snapshot(body([proxy]), 'repeat:2', second_pose)])
        obj, _, _ = decode_assets(assets)
        self.assertEqual(obj['vertices'], [(20, 30, 40), (20, 40, 40), (10, 30, 40),
                                          (-50, 0, 0), (-40, 0, 0), (-50, 10, 0)])
        self.assertEqual([s['context_id'] for s in assets['descriptor']['surfaces']], ['repeat:1', 'repeat:2'])

    def test_normals_use_inverse_transpose_and_not_translation(self):
        source = body([face(color_appearance(255, 0, 0), mesh(normals=[(1, 1, 1)] * 3))])
        pose = [2, 0, 0, 50, 0, 3, 0, -20, 0, 0, 4, 60, 0, 0, 0, 1]
        obj, _, _ = decode_assets(build_obj_assets(snapshot(source, transform=pose)))
        expected = (6/ math.sqrt(61), 4 / math.sqrt(61), 3 / math.sqrt(61))
        for normal in obj['normals']:
            for actual, value in zip(normal, expected):
                self.assertAlmostEqual(actual, value)
        self.assertEqual(obj['vertices'], [(500, -200, 600), (520, -200, 600), (500, -170, 600)])

    def test_mirrored_transform_reverses_winding_to_match_normals(self):
        pose = IDENTITY[:]
        pose[0] = -1
        source = body([face(color_appearance(255, 0, 0))])
        obj, _, _ = decode_assets(build_obj_assets(snapshot(source, transform=pose)))
        triangle = obj['triangles'][0][1]
        self.assertEqual([c[0] for c in triangle], [(0, 10, 0), (-10, 0, 0), (0, 0, 0)])
        a, b, c = [corner[0] for corner in triangle]
        cross_z = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
        self.assertGreater(cross_z, 0)

    def test_image_bytes_pixels_relative_paths_and_uv_indices_survive(self):
        with tempfile.TemporaryDirectory() as temporary:
            selected = Path(temporary) / 'private source name.png'
            selected.write_bytes(png_rgba())
            original = selected.read_bytes()
            source = body([face(image_appearance(selected))], opacity=0.5)
            native = snapshot(source)
            json.dumps(native['descriptor'], allow_nan=False)
            assets = build_obj_assets(native)
            obj, mtl, directory = decode_assets(assets)
            name, corners = obj['triangles'][0]
            self.assertEqual([corner[1] for corner in corners], [(0, 1), (0, 0), (1, 0)])
            image_path = str(directory / mtl[name]['map_Kd'][0])
            self.assertEqual(assets['files'][image_path], original)
            self.assertEqual(decode_fixture_png(assets['files'][image_path]),
                             (2, 1, [(255, 0, 0, 255), (0, 255, 0, 64)]))
            self.assertEqual(float(mtl[name]['d'][0]), 0.5)
            self.assertEqual(selected.read_bytes(), original)
            for data in assets['files'].values():
                self.assertNotIn(str(selected).encode(), data)
                self.assertNotIn(b'private source name', data)
            self.assertEqual(build_obj_assets(native), assets)
            for path, data in assets['files'].items():
                self.assertFalse(PurePosixPath(path).is_absolute())
                self.assertNotIn('..', PurePosixPath(path).parts)
                self.assertIn(hashlib.sha256(data).hexdigest(), PurePosixPath(path).name)
            json.dumps(assets['descriptor'], allow_nan=False)

    def test_texture_can_follow_untextured_faces_with_independent_uv_offset(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'image.png'
            path.write_bytes(png_rgba())
            source = body([face(color_appearance(0, 0, 255)), face(image_appearance(path))])
            obj, _, _ = decode_assets(build_obj_assets(snapshot(source)))
            self.assertEqual([c[1] for c in obj['triangles'][0][1]], [None] * 3)
            self.assertEqual([c[1] for c in obj['triangles'][1][1]], [(0, 1), (0, 0), (1, 0)])
            self.assertEqual(len(obj['uvs']), 3)

    def test_explicit_selected_texture_source_resolves_ambiguous_filename_properties(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'selected.png'
            path.write_bytes(png_rgba())
            props = [Frozen(objectType='adsk::core::FilenameProperty', id=str(i),
                            hasMultipleValues=False, value='not-chosen') for i in range(2)]
            source = body([face(image_appearance(None, filename_properties=props))])
            with self.assertRaisesRegex(VisualAssetError, 'ambiguous-image-source'):
                snapshot(source)
            assets = build_obj_assets(snapshot(source, texture_sources={0: path}))
            self.assertEqual(len(assets['files']), 3)
            with self.assertRaisesRegex(VisualAssetError, 'unselected-texture-source'):
                snapshot(source, texture_sources={1: path})
            with self.assertRaisesRegex(VisualAssetError, 'texture-source-without-texture'):
                snapshot(body([face(color_appearance(0, 0, 255))]), texture_sources={0: path})

    def test_no_guessed_filename_id_or_image_file_getter_is_required(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'data.bin'
            path.write_bytes(png_rgba())
            for identifier in ('opaque_albedo', 'generic_diffuse'):
                source = body([face(image_appearance(path, identifier=identifier))])
                self.assertEqual(len(build_obj_assets(snapshot(source))['files']), 3)

    def test_textured_quad_keeps_two_exact_triangles_and_four_uvs(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'selected.png'
            path.write_bytes(png_rgba())
            data = mesh(vertices=[(0, 0, 0), (2, 0, 0), (2, 1, 0), (0, 1, 0)],
                        indices=[0, 1, 2, 0, 2, 3], uvs=[(0, 0), (1, 0), (1, 1), (0, 1)])
            assets = build_obj_assets(snapshot(body([face(image_appearance(path), data)])))
            obj, _, _ = decode_assets(assets)
            self.assertEqual([[c[1] for c in t[1]] for t in obj['triangles']],
                             [[(0, 0), (1, 0), (1, 1)], [(0, 0), (1, 1), (0, 1)]])
            self.assertEqual(assets['descriptor']['triangle_count'], 2)

    def test_identical_image_bytes_deduplicate_and_distinct_bytes_never_collide(self):
        with tempfile.TemporaryDirectory() as temporary:
            paths = [Path(temporary) / name for name in ('one.png', 'two.png', 'three.png')]
            for path in paths:
                path.write_bytes(png_rgba())
            paths[2].write_bytes(png_rgba(pixels=b'\x00\x00\xff\xff\xff\xff\x00\x00'))
            source = body([face(image_appearance(path)) for path in paths])
            assets = build_obj_assets(snapshot(source))
            self.assertEqual(len(assets['files']), 4)
            obj, _, _ = decode_assets(assets)
            self.assertEqual(obj['triangles'][0][0], obj['triangles'][1][0])
            self.assertNotEqual(obj['triangles'][1][0], obj['triangles'][2][0])

    def test_missing_ambiguous_multivalue_and_untyped_texture_sources_fail(self):
        cases = [[], [Frozen(objectType='adsk::core::StringProperty', id='filename', value='x')],
                 [Frozen(objectType='adsk::core::FilenameProperty', id='x', hasMultipleValues=True, value='x')],
                 [Frozen(objectType='adsk::core::FilenameProperty', id='x', hasMultipleValues=False, value='')],
                 [Frozen(objectType='adsk::core::FilenameProperty', id='x', hasMultipleValues=False, value=123)]]
        for props in cases:
            with self.subTest(properties=props), self.assertRaises(VisualAssetError):
                snapshot(body([face(image_appearance(None, filename_properties=props))]))
        with self.assertRaisesRegex(VisualAssetError, 'unreadable-image-source'):
            snapshot(body([face(image_appearance('/nonexistent/selected-only.png'))]))

    def test_every_procedural_texture_type_and_unknown_shader_is_rejected(self):
        for texture_type in (0, 2, 3, 4, 5, 6, 7, 8, 9):
            with self.subTest(texture_type=texture_type), self.assertRaisesRegex(VisualAssetError, 'procedural'):
                snapshot(body([face(image_appearance('unused', texture_type=texture_type))]))
        for identifier in ('metal_f0', 'transparent_color', 'unknown_shader'):
            with self.subTest(identifier=identifier), self.assertRaisesRegex(VisualAssetError, 'shader'):
                snapshot(body([face(color_appearance(255, 0, 0, identifier=identifier))]))
        appearance = color_appearance(255, 0, 0)
        multiple = Frozen(appearanceProperties=Collection(appearance.appearanceProperties.values * 2))
        with self.assertRaisesRegex(VisualAssetError, 'shader'):
            snapshot(body([face(multiple)]))
        with self.assertRaisesRegex(VisualAssetError, 'missing-appearance'):
            snapshot(body([face()]))
        prop = appearance.appearanceProperties.item(0)
        for attribute, value in (('hasMultipleValues', True), ('hasConnectedTexture', None)):
            malformed = Frozen(**dict(vars(prop), **{attribute: value}))
            with self.subTest(attribute=attribute), self.assertRaises(VisualAssetError):
                snapshot(body([face(Frozen(appearanceProperties=Collection([malformed])))]))

    def test_unsupported_owner_does_not_fall_through_to_supported_face(self):
        source = body([face(color_appearance(0, 255, 0))])
        with self.assertRaisesRegex(VisualAssetError, 'shader'):
            snapshot(source, owner=Frozen(appearance=color_appearance(255, 0, 0, identifier='metal_f0')))

    def test_unreadable_native_getter_is_not_an_absent_appearance(self):
        class Unreadable:
            @property
            def appearance(self):
                raise AttributeError('private source path should not leak')
        with self.assertRaisesRegex(VisualAssetError, '^unreadable-native-appearance$'):
            snapshot(body([face(color_appearance(0, 255, 0))]), owner=Unreadable())

    def test_quality_is_passed_as_enum_object_not_a_guessed_lod(self):
        calls, enum = [], object()
        source = body([face(color_appearance(255, 0, 0), quality_calls=calls)])
        snapshot(source, quality=enum)
        self.assertEqual(calls, [enum])
        snapshot(source)
        self.assertEqual(calls, [enum])

    def test_serializer_and_native_reads_preserve_inputs(self):
        source = body([face(color_appearance(1, 128, 255, 128))], opacity=0.25)
        native = snapshot(source)
        before = copy.deepcopy(native)
        assets = build_obj_assets(native)
        self.assertEqual(before, native)
        obj, mtl, _ = decode_assets(assets)
        self.assertEqual(tuple(map(float, mtl[obj['triangles'][0][0]]['Kd'])), (1/255, 128/255, 1))
        self.assertEqual(float(mtl[obj['triangles'][0][0]]['d'][0]), 128/255*0.25)
        self.assertEqual(source.visibleOpacity, 0.25)
        self.assertEqual(len(source.faces.values), 1)
        self.assertEqual(assets, build_obj_assets(native))

    def test_invalid_indices_nonfinite_geometry_uvs_normals_and_alpha_fail(self):
        valid = snapshot(body([face(color_appearance(255, 0, 0))]))
        modifications = [
            ('triangles', [[0, 1, 3]]), ('triangles', [[0, -1, 2]]), ('triangles', [[0, 0, 2]]),
            ('triangles', [[0, True, 2]]), ('triangles', [[0, 1.0, 2]]), ('triangles', [[0, 1]]),
            ('triangles', []), ('vertices_mm', [[0, 0, 0], [1, 0, 0], [2, 0, 0]]),
            ('vertices_mm', [[float('nan'), 0, 0]] * 3), ('vertices_mm', [[float('inf'), 0, 0]] * 3),
            ('normals', [[0, 0, 0]] * 3), ('normals', [[0, 0, float('inf')]] * 3),
            ('normals', [[0, 0, 1]]), ('uvs', [[0, float('nan')]] * 3), ('uvs', [[0, 0]]),
            ('material', {'rgba': [1, 0, 0, -0.1], 'image': None}),
            ('material', {'rgba': [1, 0, 0, float('nan')], 'image': None}),
        ]
        for key, value in modifications:
            with self.subTest(key=key, value=value), self.assertRaises(VisualAssetError):
                changed = copy.deepcopy(valid)
                changed['descriptor']['surfaces'][0][key] = value
                build_obj_assets(changed)

    def test_native_malformed_indices_and_nonfinite_values_are_rejected(self):
        for data in (mesh(indices=[0, 1, 9]), mesh(indices=[0, 1]),
                     mesh(vertices=[(float('nan'), 0, 0), (1, 0, 0), (0, 1, 0)]),
                     mesh(normals=[(0, 0, 0)] * 3)):
            with self.subTest(data=data), self.assertRaises(VisualAssetError):
                snapshot(body([face(color_appearance(255, 0, 0), data)]))
        for attribute in ('nodeCoordinates', 'normalVectors', 'nodeIndices'):
            broken = Frozen(**dict(vars(mesh()), **{attribute: None}))
            with self.subTest(attribute=attribute), self.assertRaises(VisualAssetError):
                snapshot(body([face(color_appearance(255, 0, 0), broken)]))

    def test_textures_require_finite_per_vertex_uvs(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'image.png'
            path.write_bytes(png_rgba())
            for uvs in ([], [(0, 0)], [(0, 0), (1, 0), (0, float('inf'))]):
                with self.subTest(uvs=uvs), self.assertRaises(VisualAssetError):
                    snapshot(body([face(image_appearance(path), mesh(uvs=uvs))]))

    def test_transform_units_context_and_duplicate_provenance_are_enforced(self):
        source = body([face(color_appearance(255, 0, 0))])
        for pose in ([0] * 16, [1] * 16, IDENTITY[:15], [float('nan')] + IDENTITY[1:]):
            with self.subTest(pose=pose), self.assertRaises(VisualAssetError):
                snapshot(source, transform=pose)
        for context in ('../escape', '/private/path', 'bad\nmtllib injected', '..', ''):
            with self.subTest(context=context), self.assertRaises(VisualAssetError):
                snapshot(source, context)
        native = snapshot(source)
        with self.assertRaisesRegex(VisualAssetError, 'duplicate-surface-context'):
            build_obj_assets([native, native])
        changed = copy.deepcopy(native)
        changed['descriptor']['units'] = 'm'
        with self.assertRaisesRegex(VisualAssetError, 'coordinate-contract'):
            build_obj_assets(changed)

    def test_invalid_visible_opacity_and_native_colors_are_not_clamped(self):
        transparent = snapshot(body([face(color_appearance(0, 0, 0, 0))], opacity=0))
        obj, mtl, _ = decode_assets(build_obj_assets(transparent))
        self.assertEqual(tuple(map(float, mtl[obj['triangles'][0][0]]['Kd'] + mtl[obj['triangles'][0][0]]['d'])),
                         (0, 0, 0, 0))
        for opacity in (-1, 2, float('nan'), None):
            with self.subTest(opacity=opacity), self.assertRaises(VisualAssetError):
                snapshot(body([face(color_appearance(255, 0, 0))], opacity=opacity))
        for value in (-1, 256, float('inf'), 'red'):
            with self.subTest(color=value), self.assertRaises(VisualAssetError):
                snapshot(body([face(color_appearance(value, 0, 0))]))

    def test_image_reader_is_bounded_regular_local_and_does_not_search_or_fetch(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'selected.png'
            path.write_bytes(png_rgba())
            self.assertEqual(read_local_image(path)['data'], png_rgba())
            for bound in (1, 0, -1, True, 17*1024*1024):
                with self.subTest(bound=bound), self.assertRaises(VisualAssetError):
                    read_local_image(path, max_bytes=bound)
            link = Path(temporary) / 'alias.png'
            link.symlink_to(path)
            for selected in (temporary, link, 'https://example.invalid/image.png', '//server/image.png',
                             'file:///private/image.png', '\\\\server\\image.png', '\x00bad'):
                with self.subTest(selected=selected), self.assertRaises(VisualAssetError):
                    read_local_image(selected)

    def test_png_framing_and_crc_reject_corrupt_truncated_or_other_formats(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'image.png'
            for data in (b'', b'\xff\xd8jpeg\xff\xd9', png_rgba()[:-1], png_rgba() + b'junk',
                         png_rgba()[:40] + b'x' + png_rgba()[41:]):
                with self.subTest(data=data), self.assertRaises(VisualAssetError):
                    path.write_bytes(data)
                    read_local_image(path)

    def test_hash_mismatch_missing_image_and_unselected_payload_are_rejected(self):
        valid = snapshot(body([face(color_appearance(255, 0, 0))]))
        for mode in ('unselected', 'hash', 'missing'):
            changed = copy.deepcopy(valid)
            digest = hashlib.sha256(png_rgba()).hexdigest()
            changed['images'][digest] = {'format': 'png', 'data': png_rgba()}
            if mode == 'hash':
                changed['images'] = {'0' * 64: changed['images'][digest]}
            if mode == 'missing':
                changed['images'] = {}
                changed['descriptor']['surfaces'][0]['material']['image'] = digest
            with self.subTest(mode=mode), self.assertRaises(VisualAssetError):
                build_obj_assets(changed)

    def test_content_hash_collision_is_an_error_not_an_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            paths = [Path(temporary) / str(i) for i in range(2)]
            paths[0].write_bytes(png_rgba())
            paths[1].write_bytes(png_rgba(pixels=b'\x00\x00\xff\xff\xff\xff\x00\x00'))
            snapshots = [snapshot(body([face(image_appearance(path))]), context=str(i))
                         for i, path in enumerate(paths)]
            first_digest = next(iter(snapshots[0]['images']))
            second_image = next(iter(snapshots[1]['images'].values()))
            snapshots[1]['images'] = {first_digest: second_image}
            snapshots[1]['descriptor']['surfaces'][0]['material']['image'] = first_digest
            with patch('URDF_Exporter.utils.visual_assets.hashlib.sha256',
                       return_value=SimpleNamespace(hexdigest=lambda: first_digest)):
                with self.assertRaisesRegex(VisualAssetError, 'image-hash-collision'):
                    build_obj_assets(snapshots)

    def test_material_hash_collision_never_recolors_another_face(self):
        source = body([face(color_appearance(255, 0, 0)), face(color_appearance(0, 255, 0))])
        native = snapshot(source)
        with patch('URDF_Exporter.utils.visual_assets.hashlib.sha256',
                   return_value=SimpleNamespace(hexdigest=lambda: '0' * 64)):
            with self.assertRaisesRegex(VisualAssetError, 'material-hash-collision'):
                build_obj_assets(native)

    def test_unknown_image_mapping_properties_are_not_silently_discarded(self):
        extra = Frozen(objectType='adsk::core::FloatProperty', id='unknown-uv-transform', value=2)
        source = body([face(image_appearance(None, filename_properties=[extra]))])
        with self.assertRaisesRegex(VisualAssetError, 'unsupported-image-texture-properties'):
            snapshot(source, texture_sources={0: 'not-read'})

    def test_proxy_without_resolved_native_geometry_fails_instead_of_double_placing(self):
        incomplete = Frozen(appearance=color_appearance(255, 0, 0), nativeObject=None,
                            assemblyContext=object(), meshManager=MeshManager(mesh()))
        with self.assertRaisesRegex(VisualAssetError, 'missing-native-face'):
            snapshot(body([incomplete]))

    def test_scanline_decompression_is_validated_even_with_correct_png_crcs(self):
        def with_idat(payload):
            data = png_rgba()
            start = data.index(b'IDAT') - 4
            length = struct.unpack('>I', data[start:start+4])[0]
            chunk = (struct.pack('>I', len(payload)) + b'IDAT' + payload
                     + struct.pack('>I', zlib.crc32(b'IDAT' + payload)))
            return data[:start] + chunk + data[start+12+length:]
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'selected.png'
            for payload in (b'invalid-compression', zlib.compress(b'\x00'),
                            zlib.compress(b'\x05' + b'\x00' * 8), zlib.compress(b'\x00' * 1000)):
                with self.subTest(payload=payload), self.assertRaises(VisualAssetError):
                    path.write_bytes(with_idat(payload))
                    read_local_image(path)

    def test_json_roundtrip_descriptor_keeps_identical_assets(self):
        native = snapshot(body([face(color_appearance(1, 128, 255, 128))]))
        reconstructed = {'descriptor': json.loads(json.dumps(native['descriptor'], allow_nan=False)),
                         'images': native['images']}
        self.assertEqual(build_obj_assets(native), build_obj_assets(reconstructed))

    def test_tessellation_failure_aborts_with_a_preservation_error(self):
        class BrokenManager:
            def createMeshCalculator(self):
                raise RuntimeError('private absolute source path')
        source_face = Frozen(appearance=color_appearance(255, 0, 0), nativeObject=None,
                             meshManager=BrokenManager())
        with self.assertRaisesRegex(VisualAssetError, '^native-tessellation-failed$'):
            snapshot(body([source_face]))

    def test_backend_imports_and_runs_without_sdk_or_third_party_dependencies(self):
        result = subprocess.run([sys.executable, '-c', '''
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in ('adsk', 'numpy', 'PIL', 'trimesh'):
        raise AssertionError(name)
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from tests.test_visual_assets import body, face, color_appearance, snapshot
from URDF_Exporter.utils.visual_assets import build_obj_assets
assert build_obj_assets(snapshot(body([face(color_appearance(255, 0, 0))])))['descriptor']['triangle_count'] == 1
'''], cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_retained_scene_is_independently_decodable(self):
        with tempfile.TemporaryDirectory() as temporary:
            assets = write_evidence_scene(temporary)
            descriptor = json.loads((Path(temporary) / 'descriptor.json').read_text())
            self.assertEqual(descriptor, assets['descriptor'])
            obj, mtl, parent = decode_assets(assets)
            self.assertEqual(len(obj['triangles']), 3)
            self.assertEqual([min(v[i] for v in obj['vertices']) for i in range(3)], [0, -20, 0])
            self.assertEqual([max(v[i] for v in obj['vertices']) for i in range(3)], [40, 10, 10])
            texture = mtl[obj['triangles'][2][0]]['map_Kd'][0]
            self.assertEqual(decode_fixture_png(assets['files'][str(parent / texture)])[2],
                             [(255, 0, 0, 255), (0, 255, 0, 64)])


if __name__ == '__main__':
    unittest.main()
