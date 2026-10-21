import itertools
import copy
import importlib
import json
from collections import Counter
import math
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

from URDF_Exporter.utils.convex_collision import decompose_stl
from URDF_Exporter.utils.mesh_geometry import MeshGeometryError, ResourceLimitError, CancelledError
from tests.test_mesh_geometry import cell_surface, transformed, stl_bytes


class CollisionPackageTests(unittest.TestCase):
    def publish(self, output, config=None, cancelled=None, data=None):
        from URDF_Exporter.core.Write import write_ros2_package
        from tests.fixtures import model
        joints, links = model(fixed_only=True)
        source = data or stl_bytes(transformed(cell_surface([(0, 0, 0), (1, 0, 0), (0, 1, 0)]),
                                               scale=(10, 10, 10), translation=(180, -120, 300)))
        before = copy.deepcopy((joints, links))
        calls = []
        def meshes(targets):
            calls.append(set(targets))
            for path in targets.values():
                path.write_bytes(source)
        write_ros2_package(output, 'compound', joints, links, write_meshes=meshes,
                           config=config, cancelled=cancelled)
        self.assertEqual((joints, links), before)
        self.assertEqual(calls, [set(links)])
        return source

    def test_public_writer_all_pieces_uris_component_origins_and_unchanged_physics(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            visual, compound = output / 'visual_description', output / 'compound_description'
            source = self.publish(visual)
            self.publish(compound, {'collision_method': 'convex_decomposition', 'convex_decomposition': {'resolution': 4}})
            a = ET.parse(visual / 'urdf/compound.urdf').getroot()
            b = ET.parse(compound / 'urdf/compound.urdf').getroot()
            receipt = json.loads((compound / 'config/robot_data.yaml').read_text())['collision_decomposition']
            for name in ('base_link', 'mount'):
                left, right = a.find("link[@name='%s']" % name), b.find("link[@name='%s']" % name)
                self.assertEqual(ET.tostring(left.find('inertial')), ET.tostring(right.find('inertial')))
                self.assertEqual((compound / 'meshes' / (name + '.stl')).read_bytes(), source)
                self.assertEqual(ET.tostring(left.find('visual')).replace(b'visual_description', b'compound_description'),
                                 ET.tostring(right.find('visual')))
                collisions = right.findall('collision')
                self.assertGreater(len(collisions), 1)
                metadata = receipt['links'][name]
                self.assertEqual(metadata['piece_count'], len(collisions))
                self.assertGreater(metadata['hausdorff_bound_m'], 0)
                self.assertEqual(metadata['shell_policy'], 'single-embedded-shell')
                for collision, asset in zip(collisions, metadata['assets']):
                    self.assertEqual(collision.find('geometry/mesh').get('filename'), 'package://compound_description/' + asset)
                    self.assertEqual(collision.find('geometry/mesh').get('scale'), '0.001 0.001 0.001')
                    self.assertEqual(collision.find('origin').attrib, right.find('visual/origin').attrib)
                    path = compound / asset
                    self.assertTrue(path.resolve().is_relative_to(compound.resolve()))
                    self.assertEqual(len(read_piece(path.read_bytes())), 12)
                component = ET.parse(compound / 'urdf/components' / (name + '.urdf')).getroot().find('link')
                self.assertEqual(ET.tostring(component).strip(), ET.tostring(right).strip())
            self.assertEqual(receipt['visual_policy'], 'original-STL-bytes-unchanged')

    def test_public_fusion_entry_keeps_receipts_frames_visual_bytes_and_cad_mass(self):
        from tests.test_fusion_inputs import Body, design, hinge, occurrence, root
        body = Body('concave', 10, mass=7)
        body.mesh = stl_bytes(transformed(cell_surface([(0, 0, 0), (1, 0, 0), (0, 1, 0)]), scale=(10, 10, 10)))
        child = occurrence('café:1', [body])
        fixture, exported = design(root([child], joints=[hinge('drive', child, None)]))
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'entry_description'
            importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(fixture, target,
                config={'collision_method': 'convex_decomposition', 'convex_decomposition': {'resolution': 4}})
            robot = ET.parse(target / 'urdf/entry.urdf').getroot()
            self.assertEqual(list(robot.find("link[@name='base_link']")), [])
            physical = robot.find("link[@name='caf_1']")
            self.assertEqual(float(physical.find('inertial/mass').get('value')), 7)
            self.assertEqual((target / 'meshes/caf_1.stl').read_bytes()[84:], body.mesh[84:])
            self.assertGreater(len(physical.findall('collision')), 1)
            receipt = json.loads((target / 'config/robot_data.yaml').read_text())
            self.assertEqual(receipt['source_identifiers']['links']['caf_1'], 'café:1')
            self.assertEqual(receipt['link_sources']['base_link']['kind'], 'frame')
            self.assertEqual(exported, ['concave'])

    def test_budgets_geometry_cancellation_and_callback_errors_never_publish(self):
        controls = ({'max_pieces': 1}, {'max_cells': 1}, {'max_work': 1}, {'max_pair_checks': 1}, {'max_triangles': 1})
        for control in controls:
            with self.subTest(control=control), tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(ResourceLimitError):
                    self.publish(Path(directory) / 'compound_description',
                                 {'collision_method': 'convex_decomposition', 'convex_decomposition': control})
                self.assertEqual(list(Path(directory).iterdir()), [])
        for mode in ('cancel', 'late_cancel', 'callback', 'open', 'multi_shell', 'aggregate_pieces', 'second_link_failure'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                kwargs = {'config': {'collision_method': 'convex_decomposition', 'convex_decomposition': {'resolution': 4}}}
                expected_error = MeshGeometryError
                if mode == 'cancel':
                    kwargs['cancelled'] = lambda: True
                    expected_error = CancelledError
                elif mode == 'late_cancel':
                    counter = [0]
                    def late():
                        counter[0] += 1
                        return counter[0] > 70
                    kwargs['cancelled'] = late
                    expected_error = CancelledError
                elif mode == 'callback':
                    def fail():
                        raise RuntimeError('caller cancellation failure')
                    kwargs['cancelled'] = fail
                    expected_error = RuntimeError
                elif mode == 'open':
                    kwargs['data'] = stl_bytes(cell_surface([(0, 0, 0)])[:-1])
                elif mode == 'multi_shell':
                    cube = cell_surface([(0, 0, 0)])
                    kwargs['data'] = stl_bytes(cube + transformed(cube, translation=(3, 0, 0)))
                elif mode == 'aggregate_pieces':
                    kwargs['config']['convex_decomposition']['max_pieces'] = 40
                    expected_error = ResourceLimitError
                target = Path(directory) / 'compound_description'
                if mode == 'second_link_failure':
                    real = decompose_stl
                    count = [0]
                    def second(*args, **options):
                        count[0] += 1
                        if count[0] == 2:
                            raise MeshGeometryError('second link fails')
                        return real(*args, **options)
                    with patch('URDF_Exporter.core.Write.decompose_stl', side_effect=second), self.assertRaises(expected_error):
                        self.publish(target, **kwargs)
                    self.assertEqual(count[0], 2)
                else:
                    with self.assertRaises(expected_error):
                        self.publish(target, **kwargs)
                self.assertEqual(list(Path(directory).iterdir()), [])

    def test_configuration_rejects_unknown_noninteger_and_unbounded_controls(self):
        from URDF_Exporter.utils.config import load_config, COLLISION_MAXIMUMS
        for key, maximum in COLLISION_MAXIMUMS.items():
            for value in (0, -1, True, 1.5, '4', maximum + 1):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    load_config(overrides={'convex_decomposition': {key: value}})
        for value in ([], None, {'unknown': 1}):
            with self.assertRaises(ValueError):
                load_config(overrides={'convex_decomposition': value})

    def test_oversized_staged_input_rejects_before_reading_bytes(self):
        from URDF_Exporter.core.Write import write_ros2_package
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'bounded_description'
            def oversized(paths):
                with paths['base_link'].open('wb') as stream:
                    stream.truncate(84 + 12 * 50 + 1)
            with patch('URDF_Exporter.core.Write.read_binary_stl', side_effect=AssertionError('unbounded read')), self.assertRaises(ResourceLimitError):
                write_ros2_package(target, 'bounded', {}, {'base_link': {'mass': 1,
                    'center_of_mass': [0, 0, 0], 'inertia': [1, 1, 1, 0, 0, 0]}}, write_meshes=oversized,
                    config={'collision_method': 'convex_decomposition', 'convex_decomposition': {'max_triangles': 12}})
            self.assertEqual(list(Path(directory).iterdir()), [])


def read_piece(data):
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + count * 50
    return [tuple(tuple(row[k:k + 3]) for k in (3, 6, 9))
            for row in struct.iter_unpack('<12fH', data[84:])]


def bounds(triangles):
    points = [p for t in triangles for p in t]
    return tuple(min(p[i] for p in points) for i in range(3)), tuple(max(p[i] for p in points) for i in range(3))


def contains(boxes, p):
    return any(all(lo[i] - 1e-8 <= p[i] <= hi[i] + 1e-8 for i in range(3)) for lo, hi in boxes)


class CollisionDecompositionTests(unittest.TestCase):
    def assert_convex(self, pieces):
        for data in pieces:
            triangles = read_piece(data)
            self.assertEqual(len(triangles), 12)
            points = set(p for t in triangles for p in t)
            self.assertEqual(len(points), 8)
            edges = Counter((a, b) for tri in triangles for a, b in zip(tri, tri[1:] + tri[:1]))
            self.assertTrue(all(count == 1 and edges[b, a] == 1 for (a, b), count in edges.items()))
            for a, b, c in triangles:
                u = [b[i] - a[i] for i in range(3)]
                v = [c[i] - a[i] for i in range(3)]
                n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                self.assertGreater(sum(x * x for x in n), 0)
                self.assertTrue(all(sum(n[i] * (p[i] - a[i]) for i in range(3)) <= 1e-6 for p in points))

    def test_float32_outward_envelope_is_included_in_error_bound(self):
        shift, dimensions = (1e8, -1e8, 1e8), (8, 16, 24)
        source = stl_bytes(transformed(cell_surface([(0, 0, 0)]), scale=dimensions, translation=shift))
        result = decompose_stl(source, resolution=7)
        boxes = [bounds(read_piece(p)) for p in result['pieces']]
        self.assertGreater(result['metadata']['outward_rounding_l1_bound_m'], 0)
        for lo, hi in boxes:
            for p in itertools.product(*zip(lo, hi)):
                distance = math.sqrt(sum(max(shift[i] - p[i], 0, p[i] - shift[i] - dimensions[i]) ** 2
                                         for i in range(3)))
                self.assertLessEqual(distance / 1000, result['metadata']['hausdorff_bound_m'])
        for tri in read_piece(source):
            for p in tri:
                self.assertTrue(contains(boxes, p))
        self.assert_convex(result['pieces'])

    def test_L_U_multiple_convex_pieces_cover_solid_not_large_notch(self):
        scenes = [((0, 0, 0), (1, 0, 0), (0, 1, 0)),
                  tuple((x, y, 0) for x in range(5) for y in range(5) if x == 0 or x == 4 or y == 0)]
        for cells in scenes:
            with self.subTest(cells=len(cells)):
                source = stl_bytes(transformed(cell_surface(cells), scale=(10, 10, 10)))
                result = decompose_stl(source, resolution=10, max_pieces=4000)
                self.assertGreater(len(result['pieces']), 1)
                self.assert_convex(result['pieces'])
                boxes = [bounds(read_piece(p)) for p in result['pieces']]
                for cell in cells:
                    for fraction in itertools.product((0, .2, .5, .8, 1), repeat=3):
                        p = tuple(10 * (cell[i] + fraction[i]) for i in range(3))
                        self.assertTrue(contains(boxes, p), p)
                notch = (15, 15, 5) if len(cells) == 3 else (25, 35, 5)
                self.assertFalse(contains(boxes, notch))
                error_mm = result['metadata']['hausdorff_bound_m'] * 1000
                for lo, hi in boxes:
                    for fraction in itertools.product((0, .5, 1), repeat=3):
                        p = tuple(lo[i] + fraction[i] * (hi[i] - lo[i]) for i in range(3))
                        distance = min(math.sqrt(sum(max(10 * c[i] - p[i], 0, p[i] - 10 * (c[i] + 1)) ** 2
                                                     for i in range(3))) for c in cells)
                        self.assertLessEqual(distance, error_mm + 1e-10)
                self.assertEqual(result['metadata']['policy'], 'conservative-voxel-cells-v1')

    def test_rotated_translated_concavity_and_tetrahedron_interior(self):
        cells = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        rotation = ((0, -1, 0), (0, 0, 1), (-1, 0, 0))
        shift = (173, -91, 37)
        def place(p):
            return tuple(shift[i] + sum(rotation[i][j] * p[j] for j in range(3)) for i in range(3))
        data = stl_bytes(transformed(cell_surface(cells), scale=(10, 10, 10), rotation=rotation, translation=shift))
        result = decompose_stl(data, resolution=10)
        boxes = [bounds(read_piece(p)) for p in result['pieces']]
        for c in cells:
            for fraction in itertools.product((0, .25, .5, .75, 1), repeat=3):
                self.assertTrue(contains(boxes, place(tuple(10 * (c[i] + fraction[i]) for i in range(3)))))
        self.assertFalse(contains(boxes, place((15, 15, 5))))
        a, b, c, d = (0, 0, 0), (40, 0, 0), (0, 30, 0), (0, 0, 20)
        result = decompose_stl(stl_bytes([(a, c, b), (a, b, d), (a, d, c), (b, c, d)]), resolution=8)
        boxes = [bounds(read_piece(p)) for p in result['pieces']]
        for i in range(11):
            for j in range(11 - i):
                for k in range(11 - i - j):
                    self.assertTrue(contains(boxes, (4 * i, 3 * j, 2 * k)))
        self.assertFalse(contains(boxes, (35, 25, 15)))
        self.assert_convex(result['pieces'])

    def test_independent_error_bound_rotated_translated_box(self):
        angle = .37
        rotation = ((math.cos(angle), -math.sin(angle), 0), (math.sin(angle), math.cos(angle), 0), (0, 0, 1))
        shift = (1234.5, -876.25, 31.125)
        dimensions = (20, 11, 7)
        source_tri = transformed(cell_surface([(0, 0, 0)]), scale=dimensions, rotation=rotation, translation=shift)
        result = decompose_stl(stl_bytes(source_tri), resolution=7)
        boxes = [bounds(read_piece(p)) for p in result['pieces']]
        bound_mm = result['metadata']['hausdorff_bound_m'] * 1000
        for fraction in itertools.product((0, .2, .5, .8, 1), repeat=3):
            local = [fraction[i] * dimensions[i] for i in range(3)]
            p = [shift[i] + sum(rotation[i][j] * local[j] for j in range(3)) for i in range(3)]
            self.assertTrue(contains(boxes, p), p)
        for lo, hi in boxes:
            for p in itertools.product(*zip(lo, hi)):
                local = [sum(rotation[j][i] * (p[j] - shift[j]) for j in range(3)) for i in range(3)]
                distance = math.sqrt(sum(max(-v, 0, v - dimensions[i]) ** 2 for i, v in enumerate(local)))
                self.assertLessEqual(distance, bound_mm + .001)
        self.assert_convex(result['pieces'])

    def test_thin_oblique_feature_is_conservatively_covered(self):
        angle = .63
        r = ((math.cos(angle), 0, math.sin(angle)), (0, 1, 0), (-math.sin(angle), 0, math.cos(angle)))
        tri = transformed(cell_surface([(0, 0, 0)]), scale=(100, .001, .002), rotation=r)
        source = stl_bytes(tri)
        result = decompose_stl(source, resolution=9)
        boxes = [bounds(read_piece(p)) for p in result['pieces']]
        for a, b, c in read_piece(source):
            for i in range(9):
                for j in range(9 - i):
                    p = tuple((i * a[k] + j * b[k] + (8 - i - j) * c[k]) / 8 for k in range(3))
                    self.assertTrue(contains(boxes, p), p)
        self.assertGreater(len(boxes), 1)
        self.assert_convex(result['pieces'])

    def test_deterministic_bytes_order_and_source_unchanged(self):
        tri = cell_surface([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
        source = stl_bytes(tri)
        saved = bytes(source)
        a = decompose_stl(source, resolution=5)
        b = decompose_stl(stl_bytes(list(reversed(tri))), resolution=5)
        c = decompose_stl(stl_bytes([tuple(reversed(t)) for t in tri]), resolution=5)
        self.assertEqual(a['pieces'], b['pieces'])
        self.assertEqual(a['pieces'], c['pieces'])
        self.assertEqual(source, saved)
        self.assertEqual(a['metadata']['piece_count'], len(a['pieces']))

    def test_invalid_open_degenerate_and_nonmanifold_rejected(self):
        tri = cell_surface([(0, 0, 0)])
        for invalid in (tri[:-1], tri + [tri[0]], [((0, 0, 0),) * 3]):
            with self.assertRaises(MeshGeometryError):
                decompose_stl(stl_bytes(invalid))

    def test_controls_limits_cancel_and_callback_failure_return_nothing(self):
        data = stl_bytes(cell_surface([(0, 0, 0)]))
        for controls in ({'resolution': 0}, {'resolution': 65}, {'resolution': math.inf},
                         {'resolution': True}, {'max_work': 0}, {'max_pieces': 0}, {'max_cells': 0}):
            with self.subTest(controls=controls), self.assertRaises(ValueError):
                decompose_stl(data, **controls)
        for controls in ({'max_pieces': 1}, {'max_cells': 1}, {'max_work': 1}, {'max_pair_checks': 1}):
            with self.subTest(controls=controls), self.assertRaises(ResourceLimitError):
                decompose_stl(data, **controls)
        with self.assertRaises(CancelledError):
            decompose_stl(data, cancelled=lambda: True)
        calls = [0]
        def fail():
            calls[0] += 1
            if calls[0] > 20:
                raise RuntimeError('injected caller failure')
            return False
        with self.assertRaisesRegex(RuntimeError, 'injected caller failure'):
            decompose_stl(data, cancelled=fail)

    def test_exact_work_threshold_and_late_cancellation(self):
        data = stl_bytes(cell_surface([(0, 0, 0)]))
        calls = [0]
        def count_calls():
            calls[0] += 1
            return False
        result = decompose_stl(data, resolution=4, cancelled=count_calls)
        allowed_calls = calls[0] - 1
        calls[0] = 0
        def cancel_late():
            calls[0] += 1
            return calls[0] > allowed_calls
        with self.assertRaises(CancelledError):
            decompose_stl(data, resolution=4, cancelled=cancel_late)
        used = result['metadata']['work_used']
        self.assertEqual(decompose_stl(data, resolution=4, max_work=used)['pieces'], result['pieces'])
        with self.assertRaises(ResourceLimitError):
            decompose_stl(data, resolution=4, max_work=used - 1)


if __name__ == '__main__':
    unittest.main()
