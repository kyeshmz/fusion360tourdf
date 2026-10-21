"""Reproductions of the independently reviewed appearance-boundary defects."""

import base64
import hashlib
import json
import struct
import unittest
import zlib

from tests.test_visual_assets import (
    Frozen, body, color_appearance, face, snapshot, with_properties,
)
from URDF_Exporter.utils.visual_assets import VisualAssetError, _validate_png, build_obj_assets


class ChunkKind(bytes):
    comparisons = 0

    def __eq__(self, other):
        type(self).comparisons += 1
        return bytes.__eq__(self, other)

    __hash__ = bytes.__hash__


class InstrumentedPNG(bytes):
    def __getitem__(self, key):
        value = super().__getitem__(key)
        return ChunkKind(value) if isinstance(value, bytes) and len(value) == 4 else value


def many_chunk_png(count):
    def chunk(kind, payload):
        return (struct.pack('>I', len(payload)) + kind + payload
                + struct.pack('>I', zlib.crc32(kind + payload)))
    header = chunk(b'IHDR', struct.pack('>IIBBBBB', 1, 1, 8, 6, 0, 0, 0))
    return (b'\x89PNG\r\n\x1a\n' + header + chunk(b'vpAg', b'') * count
            + chunk(b'IDAT', zlib.compress(b'\x00\xff\x00\x00\xff'))
            + chunk(b'IDAT', b'') * count + chunk(b'IEND', b''))


class AppearanceReviewTests(unittest.TestCase):
    def test_material_identifiers_fit_importer_names_without_losing_hash_bits(self):
        assets = build_obj_assets(snapshot(body([face(color_appearance(255, 0, 0)),
                                                 face(color_appearance(0, 255, 0))])))
        names = {surface['material'] for surface in assets['descriptor']['surfaces']}
        self.assertEqual(len(names), 2)
        self.assertTrue(all(name.isascii() and len(name) <= 63 for name in names))
        decoded = {base64.b32decode(name[4:].upper() + '=' * ((-len(name[4:])) % 8)).hex()
                   for name in names}
        expected = {hashlib.sha256(json.dumps({'rgba': rgba, 'image': None}, sort_keys=True,
                    separators=(',', ':')).encode()).hexdigest()
                    for rgba in ([1., 0., 0., 1.], [0., 1., 0., 1.])}
        self.assertEqual(decoded, expected)

    def test_png_chunk_type_comparisons_have_linear_work(self):
        count = 1000
        data = many_chunk_png(count)
        ChunkKind.comparisons = 0
        _validate_png(InstrumentedPNG(data))
        self.assertLess(ChunkKind.comparisons, (2 * count + 3) * 40)
        _validate_png(data)

    def test_selected_diffuse_id_duplicate_rejects_before_type_filtering(self):
        for other_type in ('FloatProperty', 'ColorProperty'):
            for selected in (None, 'opaque_albedo'):
                with self.subTest(other_type=other_type, selected=selected):
                    appearance = with_properties(color_appearance(255, 0, 0),
                        Frozen(id='opaque_albedo', objectType='adsk::core::' + other_type))
                    options = {} if selected is None else {'diffuse_properties': {0: selected}}
                    with self.assertRaisesRegex(VisualAssetError, 'ambiguous-diffuse-shader'):
                        snapshot(body([face(appearance)]), shader_policy='diffuse-subset', **options)


if __name__ == '__main__':
    unittest.main()
