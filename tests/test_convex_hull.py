import copy
from collections import Counter
from fractions import Fraction
import importlib
import itertools
import json
import math
from pathlib import Path
import random
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

from URDF_Exporter.utils.convex_hull import hull_stl
from URDF_Exporter.utils.mesh_geometry import MeshGeometryError, ResourceLimitError, CancelledError
from tests.test_mesh_geometry import cell_surface, transformed, stl_bytes


def decode(data):
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise AssertionError('Invalid STL length')
    return [tuple(tuple(Fraction(v) for v in row[k:k + 3]) for k in (3, 6, 9))
            for row in struct.iter_unpack('<12fH', data[84:])]


def determinant(a, b, c, origin):
    x, y, z = [tuple(p[i] - origin[i] for i in range(3)) for p in (a, b, c)]
    return (x[0] * (y[1] * z[2] - y[2] * z[1])
            - x[1] * (y[0] * z[2] - y[2] * z[0])
            + x[2] * (y[0] * z[1] - y[1] * z[0]))


def audit_hull(source, piece, expected_volume=None):
    faces = decode(piece)
    points = set(p for t in faces for p in t)
    original = set(p for t in decode(source) for p in t)
    assert points <= original, 'Hull moved or invented a source float32 vertex'
    edges = Counter((a, b) for t in faces for a, b in zip(t, t[1:] + t[:1]))
    assert all(n == 1 and edges[b, a] == 1 for (a, b), n in edges.items())
    assert len(points) - len(edges) // 2 + len(faces) == 2
    for a, b, c in faces:
        u, v = [tuple(p[i] - a[i] for i in range(3)) for p in (b, c)]
        assert any(u[i] * v[j] != u[j] * v[i] for i, j in ((0, 1), (0, 2), (1, 2)))
        assert all(determinant(b, c, p, a) <= 0 for p in original), 'Source outside exact hull face'
    origin = min(points)
    volume = sum(determinant(a, b, c, origin) for a, b, c in faces) / 6
    assert volume > 0
    if expected_volume is not None:
        assert volume == expected_volume, (volume, expected_volume)
    return {'source_vertices': len(original), 'hull_vertices': len(points), 'faces': len(faces),
            'halfspace_checks': len(original) * len(faces), 'directed_edges': len(edges),
            'volume_mm3_ratio': [volume.numerator, volume.denominator]}


def examples():
    a, b, c, d = (0, 0, 0), (40, 0, 0), (0, 30, 0), (0, 0, 20)
    tetra = [(a, c, b), (a, b, d), (a, d, c), (b, c, d)]
    cube = cell_surface([(0, 0, 0)])
    ell = cell_surface([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    ushape = cell_surface([(x, y, 0) for x in range(5) for y in range(5)
                          if x == 0 or x == 4 or y == 0])
    return [
        ('tetrahedron', tetra, Fraction(4000)),
        ('oblique_box', transformed(cube, scale=(10, 20, 30),
             rotation=((.6, -.8, 0), (.8, .6, 0), (0, 0, 1)), translation=(180, -120, 300)), Fraction(6000)),
        ('concave_l', ell, Fraction(7, 2)),
        ('oblique_u', transformed(ushape, rotation=((1, .5, 0), (0, 1, 0), (0, 0, 1))), Fraction(25)),
        ('coplanar_grid', cell_surface(itertools.product(range(3), range(2), range(2))), Fraction(12)),
        ('tiny', transformed(cube, scale=(2**-120, 2**-118, 2**-116)), Fraction(2)**-354),
        ('large', transformed(cube, scale=(2**102, 2**103, 2**104),
                              translation=(2**120, -2**120, 2**120)), Fraction(2)**309),
        ('subnormal', transformed(cube, scale=(2**-149, 2**-148, 2**-147)), Fraction(2)**-444),
    ]


def roof_mesh(rng, size=4):
    heights = {(x, y): rng.randrange(1, 8) for x in range(size + 1) for y in range(size + 1)}
    triangles = []
    for x, y in itertools.product(range(size), repeat=2):
        corners = ((x, y), (x + 1, y), (x + 1, y + 1), (x, y + 1))
        for z in (False, True):
            a, b, c, d = [(u, v, heights[u, v] if z else 0) for u, v in corners]
            triangles.extend(((a, b, c), (a, c, d)) if z else ((a, c, b), (a, d, c)))
    boundary = ([(i, 0) for i in range(size)] + [(size, i) for i in range(size)]
                + [(i, size) for i in range(size, 0, -1)] + [(0, i) for i in range(size, 0, -1)])
    for a, b in zip(boundary, boundary[1:] + boundary[:1]):
        low_a, low_b = (*a, 0), (*b, 0)
        high_a, high_b = (*a, heights[a]), (*b, heights[b])
        triangles.extend(((low_a, low_b, high_b), (low_a, high_b, high_a)))
    return triangles


class HullGeometryTests(unittest.TestCase):
    def test_exact_support_manifold_volume_and_source_vertex_serialization(self):
        for name, triangles, volume in examples():
            with self.subTest(case=name):
                data = stl_bytes(triangles)
                result = hull_stl(data)
                self.assertEqual(len(result['pieces']), 1)
                audit_hull(data, result['pieces'][0], volume)
                metadata = result['metadata']
                self.assertEqual(metadata['piece_count'], 1)
                self.assertEqual(metadata['serialization_bound_m'], 0)
                self.assertEqual(metadata['hull_volume_m3'], float(volume / 10**9))
                self.assertTrue(metadata['fills_concavities'])
                json.dumps(metadata, allow_nan=False)

    def test_concave_hulls_fill_notches_but_are_not_global_aabbs(self):
        for name, triangles, volume in examples()[2:4]:
            with self.subTest(case=name):
                data = stl_bytes(triangles)
                result = hull_stl(data)
                faces = decode(result['pieces'][0])
                points = set(p for t in faces for p in t)
                aabb_volume = math.prod(max(p[i] for p in points) - min(p[i] for p in points) for i in range(3))
                self.assertLess(volume, aabb_volume)
                notch = (Fraction(5, 4), Fraction(5, 4), Fraction(1, 2)) if name == 'concave_l' else (3, 3, Fraction(1, 2))
                self.assertTrue(all(determinant(b, c, notch, a) <= 0 for a, b, c in faces))

    def test_input_order_winding_vertex_rotation_and_signed_zero_are_deterministic(self):
        rng = random.Random(716)
        for name, triangles, _ in examples()[:5]:
            expected = hull_stl(stl_bytes(triangles))['pieces']
            for repeat in range(4):
                reordered = list(triangles)
                rng.shuffle(reordered)
                reordered = [t[repeat % 3:] + t[:repeat % 3] for t in reordered]
                if repeat % 2:
                    reordered = [tuple(reversed(t)) for t in reordered]
                reordered = [tuple(tuple(-0.0 if v == 0 else v for v in p) for p in t) for t in reordered]
                self.assertEqual(hull_stl(stl_bytes(reordered))['pieces'], expected, name)

    def test_oblique_thin_and_nearly_coplanar_float32_shells(self):
        cube = cell_surface([(0, 0, 0)])
        for scale, rotation in (((100, 2**-20, 2**-18), ((1, .5, .25), (0, 1, .5), (0, 0, 1))),
                                ((10, 7, .01), ((math.cos(.37), -math.sin(.37), 0),
                                               (math.sin(.37), math.cos(.37), 0), (0, 0, 1)))):
            data = stl_bytes(transformed(cube, scale=scale, rotation=rotation))
            audit_hull(data, hull_stl(data)['pieces'][0])

    def test_seeded_nonconvex_roofs_with_many_coplanar_and_interior_vertices(self):
        rng = random.Random(111716)
        for index in range(24):
            triangles = roof_mesh(rng)
            data = stl_bytes(transformed(triangles, rotation=((1, .5, .25), (0, 1, .5), (0, 0, 1)),
                                         translation=(index * 100, -index * 10, index)))
            with self.subTest(case=index):
                result = hull_stl(data)
                audit_hull(data, result['pieces'][0])
                rng.shuffle(triangles)
                reordered = stl_bytes(transformed(triangles, rotation=((1, .5, .25), (0, 1, .5), (0, 0, 1)),
                                                  translation=(index * 100, -index * 10, index)))
                self.assertEqual(hull_stl(reordered)['pieces'], result['pieces'])

    def test_exact_work_face_byte_vertex_thresholds(self):
        data = stl_bytes(cell_surface([(0, 0, 0)]))
        result = hull_stl(data)
        metadata = result['metadata']
        controls = {'max_work': metadata['work_used'], 'max_vertices': 8,
                    'max_faces': 12, 'max_output_bytes': len(result['pieces'][0]), 'max_pieces': 1}
        self.assertEqual(hull_stl(data, **controls)['pieces'], result['pieces'])
        for key in ('max_work', 'max_vertices', 'max_faces', 'max_output_bytes'):
            with self.subTest(control=key), self.assertRaises(ResourceLimitError):
                hull_stl(data, **dict(controls, **{key: controls[key] - 1}))

    def test_invalid_meshes_limits_and_cancellation_return_no_result(self):
        cube = cell_surface([(0, 0, 0)])
        invalid = (cube[:-1], cube + [cube[0]], [((0, 0, 0),) * 3],
                   cube + transformed(cube, translation=(3, 0, 0)))
        for triangles in invalid:
            with self.assertRaises(MeshGeometryError):
                hull_stl(stl_bytes(triangles))
        data = stl_bytes(cube)
        for controls in ({'max_work': 1}, {'max_pair_checks': 1}, {'max_triangles': 1}):
            with self.assertRaises(ResourceLimitError):
                hull_stl(data, **controls)
        for source in (b'', data[:-1], 'not bytes', cube):
            with self.assertRaises(MeshGeometryError):
                hull_stl(source)
        for offset in (84, 96):
            nonfinite = bytearray(data)
            struct.pack_into('<f', nonfinite, offset, math.nan)
            with self.assertRaisesRegex(MeshGeometryError, 'Nonfinite'):
                hull_stl(nonfinite)
        with self.assertRaises(CancelledError):
            hull_stl(data, cancelled=lambda: True)
        calls = [0]
        def count():
            calls[0] += 1
            return False
        hull_stl(data, cancelled=count)
        final_call = calls[0]
        calls[0] = 0
        def late():
            calls[0] += 1
            return calls[0] == final_call
        with self.assertRaises(CancelledError):
            hull_stl(data, cancelled=late)
        def fail():
            raise RuntimeError('caller failure')
        with self.assertRaisesRegex(RuntimeError, 'caller failure'):
            hull_stl(data, cancelled=fail)


class HullPackageTests(unittest.TestCase):
    def publish(self, output, config=None, cancelled=None, data=None):
        from URDF_Exporter.core.Write import write_ros2_package
        from tests.fixtures import model
        joints, links = model(fixed_only=True)
        source = data if data is not None else stl_bytes(transformed(
            cell_surface([(0, 0, 0), (1, 0, 0), (0, 1, 0)]),
            scale=(10, 10, 10), translation=(180, -120, 300)))
        before = copy.deepcopy((joints, links))
        calls = []
        def meshes(targets):
            calls.append(set(targets))
            for path in targets.values():
                path.write_bytes(source)
        write_ros2_package(output, 'single', joints, links, write_meshes=meshes,
                           config=config, cancelled=cancelled)
        self.assertEqual((joints, links), before)
        self.assertEqual(calls, [set(links)])
        return source

    def test_public_writer_one_hull_component_uri_origin_visual_and_physics_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            visual = Path(directory) / 'visual_description'
            convex = Path(directory) / 'hull_description'
            source = self.publish(visual)
            with patch('URDF_Exporter.core.Write.hull_stl', wraps=hull_stl) as compute:
                self.publish(convex, {'collision_method': 'convex_hull'})
            self.assertEqual(compute.call_count, 2)
            a = ET.parse(visual / 'urdf/single.urdf').getroot()
            b = ET.parse(convex / 'urdf/single.urdf').getroot()
            receipt = json.loads((convex / 'config/robot_data.yaml').read_text())['convex_hull']
            for name in ('base_link', 'mount'):
                left, right = [root.find("link[@name='%s']" % name) for root in (a, b)]
                self.assertEqual(ET.tostring(left.find('inertial')), ET.tostring(right.find('inertial')))
                self.assertEqual(ET.tostring(left.find('visual')).replace(b'visual_description', b'hull_description'),
                                 ET.tostring(right.find('visual')))
                self.assertEqual((convex / 'meshes' / (name + '.stl')).read_bytes(), source)
                self.assertEqual(len(right.findall('collision')), 1)
                collision = right.find('collision')
                self.assertEqual(collision.find('origin').attrib, right.find('visual/origin').attrib)
                self.assertEqual(collision.find('geometry/mesh').get('scale'), '0.001 0.001 0.001')
                asset, = receipt['links'][name]['assets']
                self.assertEqual(collision.find('geometry/mesh').get('filename'), 'package://hull_description/' + asset)
                audit_hull(source, (convex / asset).read_bytes(), Fraction(3500))
                component = ET.parse(convex / 'urdf/components' / (name + '.urdf')).getroot().find('link')
                self.assertEqual(ET.tostring(component).strip(), ET.tostring(right).strip())
            self.assertEqual(receipt['visual_policy'], 'original-STL-bytes-unchanged')

    def test_default_off_existing_modes_and_normalized_config_stay_unchanged(self):
        from URDF_Exporter.utils.config import load_config
        self.assertEqual(load_config()['collision_method'], 'visual')
        self.assertNotIn('convex_hull', load_config())
        for method in ('visual', 'primitive_box', 'convex_decomposition', 'convex_hull'):
            config = load_config(overrides={'collision_method': method})
            self.assertEqual(load_config(overrides=config), config)
        for method in ('visual', 'primitive_box'):
            with tempfile.TemporaryDirectory() as directory:
                a, b = [Path(directory) / p / 'same_description' for p in ('a', 'b')]
                self.publish(a, {'collision_method': method})
                self.publish(b, {'collision_method': method, 'convex_hull': {'max_work': 1}})
                self.assertEqual({p.relative_to(a): p.read_bytes() for p in a.rglob('*') if p.is_file()},
                                 {p.relative_to(b): p.read_bytes() for p in b.rglob('*') if p.is_file()})

    def test_strict_config_all_controls_and_unknowns_abort_before_callback(self):
        from URDF_Exporter.utils.config import HULL_MAXIMUMS, load_config
        for key, maximum in HULL_MAXIMUMS.items():
            for value in (0, -1, True, 1.5, '4', maximum + 1, math.inf, math.nan):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    load_config(overrides={'convex_hull': {key: value}})
                with self.assertRaises(MeshGeometryError):
                    hull_stl(stl_bytes(cell_surface([(0, 0, 0)])), **{key: value})
        for value in (None, [], {'unknown': 1}, {'resolution': 4}):
            with self.assertRaises(ValueError):
                load_config(overrides={'convex_hull': value})
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                self.publish(Path(directory) / 'bad_description', {'convex_hull': {'unknown': 1}})
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_resource_and_geometry_failures_are_transactional(self):
        for control in ({'max_vertices': 1}, {'max_faces': 1}, {'max_work': 1},
                        {'max_triangles': 1}, {'max_pair_checks': 1},
                        {'max_pieces': 1}, {'max_output_bytes': 683}):
            with self.subTest(control=control), tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(ResourceLimitError):
                    self.publish(Path(directory) / 'bad_description',
                                 {'collision_method': 'convex_hull', 'convex_hull': control})
                self.assertEqual(list(Path(directory).iterdir()), [])
        source = stl_bytes(transformed(cell_surface([(0, 0, 0), (1, 0, 0), (0, 1, 0)]),
                                       scale=(10, 10, 10), translation=(180, -120, 300)))
        metadata = hull_stl(source)['metadata']
        for control in ({'max_work': metadata['work_used'] * 2 - 1},
                        {'max_output_bytes': metadata['output_bytes'] * 2 - 1}):
            with self.subTest(aggregate=control), tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(ResourceLimitError):
                    self.publish(Path(directory) / 'bad_description',
                                 {'collision_method': 'convex_hull', 'convex_hull': control})
                self.assertEqual(list(Path(directory).iterdir()), [])
        cube = cell_surface([(0, 0, 0)])
        for data in (stl_bytes(cube[:-1]), stl_bytes(cube + transformed(cube, translation=(3, 0, 0)))):
            with tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(MeshGeometryError):
                    self.publish(Path(directory) / 'bad_description', {'collision_method': 'convex_hull'}, data=data)
                self.assertEqual(list(Path(directory).iterdir()), [])

    def test_cancellation_callback_errors_and_second_link_failure_do_not_publish(self):
        for mode in ('early', 'late', 'callback', 'second'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                calls = [0]
                def cancel():
                    calls[0] += 1
                    if mode == 'callback':
                        raise RuntimeError('caller failure')
                    return mode == 'early' or (mode == 'late' and calls[0] > 90)
                count = [0]
                def compute(*args, **kwargs):
                    count[0] += 1
                    if mode == 'second' and count[0] == 2:
                        raise MeshGeometryError('second link failure')
                    return hull_stl(*args, **kwargs)
                error = MeshGeometryError if mode == 'second' else RuntimeError if mode == 'callback' else CancelledError
                with patch('URDF_Exporter.core.Write.hull_stl', side_effect=compute), self.assertRaises(error):
                    self.publish(Path(directory) / 'bad_description', {'collision_method': 'convex_hull'}, cancelled=cancel)
                self.assertEqual(list(Path(directory).iterdir()), [])
                if mode == 'second':
                    self.assertEqual(count[0], 2)

    def test_oversized_input_is_rejected_before_unbounded_read(self):
        from URDF_Exporter.core.Write import write_ros2_package
        with tempfile.TemporaryDirectory() as directory:
            def meshes(paths):
                with paths['base_link'].open('wb') as stream:
                    stream.truncate(84 + 12 * 50 + 1)
            with patch('URDF_Exporter.core.Write.read_binary_stl', side_effect=AssertionError('unbounded read')), self.assertRaises(ResourceLimitError):
                write_ros2_package(Path(directory) / 'bounded_description', 'bounded', {},
                    {'base_link': {'mass': 1, 'center_of_mass': [0, 0, 0], 'inertia': [1, 1, 1, 0, 0, 0]}},
                    write_meshes=meshes, config={'collision_method': 'convex_hull', 'convex_hull': {'max_triangles': 12}})
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_cancellation_at_the_last_publication_check_leaves_no_destination(self):
        calls = [0]
        def count():
            calls[0] += 1
            return False
        with tempfile.TemporaryDirectory() as directory:
            self.publish(Path(directory) / 'count_description', {'collision_method': 'convex_hull'}, cancelled=count)
        final_call = calls[0]
        calls[0] = 0
        def final():
            calls[0] += 1
            return calls[0] == final_call
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(CancelledError):
                self.publish(Path(directory) / 'last_description', {'collision_method': 'convex_hull'}, cancelled=final)
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_public_fusion_entry_frame_and_source_preservation(self):
        from tests.test_fusion_inputs import Body, design, hinge, occurrence, root
        body = Body('concave', 10, mass=7)
        body.mesh = stl_bytes(cell_surface([(0, 0, 0), (1, 0, 0), (0, 1, 0)]))
        child = occurrence('caf\u00e9:1', [body])
        fixture, exported = design(root([child], joints=[hinge('drive', child, None)]))
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'entry_description'
            importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(fixture, target,
                config={'collision_method': 'convex_hull'})
            robot = ET.parse(target / 'urdf/entry.urdf').getroot()
            self.assertEqual(list(robot.find("link[@name='base_link']")), [])
            physical = robot.find("link[@name='caf_1']")
            self.assertEqual(float(physical.find('inertial/mass').get('value')), 7)
            self.assertEqual(len(physical.findall('collision')), 1)
            self.assertEqual((target / 'meshes/caf_1.stl').read_bytes()[84:], body.mesh[84:])
            receipt = json.loads((target / 'config/robot_data.yaml').read_text())
            self.assertEqual(receipt['source_identifiers']['links']['caf_1'], 'caf\u00e9:1')
            self.assertEqual(receipt['link_sources']['base_link']['kind'], 'frame')
            self.assertEqual(set(receipt['convex_hull']['links']), {'caf_1'})
            self.assertEqual(exported, ['concave'])


def triangle_vertex_examples():
    cube = cell_surface([(0, 0, 0)])
    pair = transformed(cube, scale=(10, 20, 30)) + transformed(
        cube, scale=(10, 20, 30), translation=(40, 0, 0))
    return [
        ('offset_boxes', pair, Fraction(30000)),
        ('oblique_offset_boxes', transformed(pair,
             rotation=((1, .5, .25), (0, 1, .5), (0, 0, 1)),
             translation=(180, -120, 300)), Fraction(30000)),
        ('open_tetrahedron', examples()[0][1][:-1], Fraction(4000)),
        ('nested_shells', transformed(cube, scale=(10, 10, 10)) + transformed(
             cube, scale=(2, 2, 2), translation=(4, 4, 4)), Fraction(1000)),
    ]


def offset_body_fixture():
    from tests.test_fusion_inputs import Body, design, root
    from tests.test_exporter_logic import FakePoint
    bodies = []
    for name, mass, center in (('left', 1, (-10, -5, 2)), ('right', 3, (10, 15, 8))):
        body = Body(name, center[0], mass=mass)
        x, y, z = center
        body.properties.centerOfMass = FakePoint(center)
        moments = (True, mass * (2 / 3 + y*y + z*z), mass * (2 / 3 + x*x + z*z),
                   mass * (2 / 3 + x*x + y*y), -mass*x*y, -mass*y*z, -mass*x*z)
        body.properties.getXYZMomentsOfInertia = lambda values=moments: values
        body.boundingBox.minPoint = FakePoint([v - 1 for v in center])
        body.boundingBox.maxPoint = FakePoint([v + 1 for v in center])
        body.mesh = stl_bytes(transformed(cell_surface([(0, 0, 0)]), scale=(20, 20, 20),
            translation=tuple(10 * v - 10 for v in center)))
        bodies.append(body)
    fixture, exported = design(root(bodies=bodies))
    return fixture, exported, bodies


class TriangleVertexHullTests(unittest.TestCase):
    def test_explicit_policy_exact_multishell_open_and_cavity_hulls(self):
        for name, triangles, volume in triangle_vertex_examples():
            with self.subTest(case=name):
                data = stl_bytes(triangles)
                with self.assertRaises(MeshGeometryError):
                    hull_stl(data)
                result = hull_stl(data, source_policy='triangle_vertices', max_pair_checks=1)
                audit_hull(data, result['pieces'][0], volume)
                metadata = result['metadata']
                self.assertEqual(metadata['source_policy'], 'triangle_vertices')
                self.assertEqual(metadata['source_validation'], 'finite-nonzero-area-triangles')
                self.assertEqual(metadata['shell_policy'], 'topology-not-verified')
                self.assertFalse(metadata['source_topology_verified'])
                self.assertEqual(metadata['validation_pair_checks'], 0)
                self.assertEqual(metadata['source_triangle_count'], len(triangles))
                self.assertEqual(metadata['source_vertex_count'], len(set(p for t in decode(data) for p in t)))
                self.assertEqual(metadata['serialization_bound_m'], 0)
                self.assertTrue(metadata['fills_gaps_and_cavities'])
                self.assertFalse(metadata['decomposition'])
                self.assertFalse(metadata['topology_preservation'])
                if name == 'oblique_offset_boxes':
                    points = set(p for t in decode(data) for p in t)
                    self.assertLess(volume, math.prod(max(p[i] for p in points) - min(p[i] for p in points)
                                                      for i in range(3)))
                json.dumps(metadata, allow_nan=False)

    def test_duplicates_winding_order_and_extreme_float32_vertices(self):
        rng = random.Random(211)
        for _, triangles, volume in triangle_vertex_examples() + examples()[-3:]:
            data = stl_bytes(triangles)
            expected = hull_stl(data, source_policy='triangle_vertices')['pieces']
            audit_hull(data, expected[0], volume)
            repeated = triangles + [tuple(reversed(t)) for t in triangles]
            rng.shuffle(repeated)
            result = hull_stl(stl_bytes(repeated), source_policy='triangle_vertices')
            self.assertEqual(result['pieces'], expected)
            self.assertEqual(result['metadata']['source_triangle_count'], len(repeated))

    def test_source_policy_config_is_shared_strict_and_default_bytes_unchanged(self):
        from URDF_Exporter.utils.config import load_config
        from URDF_Exporter.utils.convex_hull import HULL_SOURCE_POLICIES
        self.assertEqual(HULL_SOURCE_POLICIES, ('closed_shell', 'triangle_vertices'))
        data = stl_bytes(cell_surface([(0, 0, 0)]))
        self.assertEqual(hull_stl(data), hull_stl(data, source_policy='closed_shell'))
        for policy in HULL_SOURCE_POLICIES:
            config = load_config(overrides={'collision_method': 'convex_hull',
                                            'convex_hull': {'source_policy': policy}})
            self.assertEqual(load_config(overrides=config), config)
        self.assertEqual(load_config(overrides={'collision_method': 'convex_hull'}),
                         load_config(overrides={'collision_method': 'convex_hull',
                                                'convex_hull': {'source_policy': 'closed_shell'}}))
        for policy in (None, True, 1, [], {}, 'vertices', 'TRIANGLE_VERTICES'):
            with self.subTest(policy=policy), self.assertRaises(MeshGeometryError):
                hull_stl(data, source_policy=policy)
            with self.assertRaises(ValueError):
                load_config(overrides={'convex_hull': {'source_policy': policy}})

    def test_nondegenerate_framing_and_finite_validation_precede_hull(self):
        tetra = examples()[0][1]
        degenerate = [((0, 0, 0), (1, 1, 1), (2, 2, 2))]
        coplanar = [((0, 0, 0), (1, 0, 0), (0, 1, 0)),
                    ((1, 0, 0), (1, 1, 0), (0, 1, 0))]
        for triangles in (degenerate, tetra + degenerate, coplanar):
            with self.subTest(triangles=triangles), self.assertRaises(MeshGeometryError):
                hull_stl(stl_bytes(triangles), source_policy='triangle_vertices')
        data = stl_bytes(tetra)
        invalid = [b'', data[:-1], data + b'x', b'\0' * 84]
        for offset in (84, 96):
            for value in (math.nan, math.inf, -math.inf):
                changed = bytearray(data)
                struct.pack_into('<f', changed, offset, value)
                invalid.append(changed)
        changed = bytearray(data)
        struct.pack_into('<I', changed, 80, 1)
        invalid.append(changed)
        for source in invalid:
            with self.subTest(source=bytes(source)[:12]), self.assertRaises(MeshGeometryError):
                hull_stl(source, source_policy='triangle_vertices')

    def test_exact_resource_limits_and_early_late_cancellation(self):
        data = stl_bytes(triangle_vertex_examples()[0][1])
        result = hull_stl(data, source_policy='triangle_vertices')
        metadata = result['metadata']
        controls = {'max_work': metadata['work_used'], 'max_vertices': metadata['source_vertex_count'],
                    'max_faces': 32, 'max_output_bytes': metadata['output_bytes'],
                    'max_triangles': metadata['source_triangle_count'], 'max_pieces': 1, 'max_pair_checks': 1}
        self.assertEqual(hull_stl(data, source_policy='triangle_vertices', **controls)['pieces'], result['pieces'])
        for key in ('max_work', 'max_vertices', 'max_output_bytes', 'max_triangles'):
            with self.subTest(control=key), self.assertRaises(ResourceLimitError):
                hull_stl(data, source_policy='triangle_vertices', **dict(controls, **{key: controls[key] - 1}))
        with self.assertRaises(ResourceLimitError):
            hull_stl(data, source_policy='triangle_vertices', max_faces=1)
        calls = [0]
        def count():
            calls[0] += 1
            return False
        hull_stl(data, source_policy='triangle_vertices', cancelled=count)
        final = calls[0]
        for stop in (1, 3, final):
            calls[0] = 0
            def cancel():
                calls[0] += 1
                return calls[0] == stop
            with self.subTest(stop=stop), self.assertRaises(CancelledError):
                hull_stl(data, source_policy='triangle_vertices', cancelled=cancel)

    def test_actual_combined_bodies_one_physical_link_preserve_full_physics(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory) / name / 'pair_description' for name in ('visual', 'hull')]
            for path, config in zip(paths, (None, {'collision_method': 'convex_hull',
                    'convex_hull': {'source_policy': 'triangle_vertices'}})):
                fixture, exported, bodies = offset_body_fixture()
                entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
                entry.export_design(fixture, path, config=config)
                self.assertEqual(exported, ['left', 'right'])
                source = (path / 'meshes/base_link.stl').read_bytes()
                self.assertEqual(source[84:], bodies[0].mesh[84:] + bodies[1].mesh[84:])
            self.assertEqual((paths[0] / 'meshes/base_link.stl').read_bytes(), source)
            roots = [ET.parse(path / 'urdf/pair.urdf').getroot() for path in paths]
            left, right = [root.find('link') for root in roots]
            self.assertEqual([link.get('name') for link in roots[1].findall('link')], ['base_link'])
            self.assertEqual(ET.tostring(left.find('inertial')), ET.tostring(right.find('inertial')))
            self.assertEqual(ET.tostring(left.find('visual')), ET.tostring(right.find('visual')))
            self.assertEqual(float(right.find('inertial/mass').get('value')), 4)
            for actual, expected in zip(map(float, right.find('inertial/origin').get('xyz').split()), (.05, .10, .065)):
                self.assertAlmostEqual(actual, expected, places=12)
            inertia = right.find('inertial/inertia')
            for key, expected in zip(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'),
                    (989 / 30000, 989 / 30000, 1808 / 30000, -.03, -.009, -.009)):
                self.assertAlmostEqual(float(inertia.get(key)), expected, places=12)
            collision, = right.findall('collision')
            self.assertEqual(collision.find('origin').attrib, right.find('visual/origin').attrib)
            self.assertEqual(collision.find('geometry/mesh').get('scale'), '0.001 0.001 0.001')
            receipt = json.loads((paths[1] / 'config/robot_data.yaml').read_text())['convex_hull']
            asset, = receipt['links']['base_link']['assets']
            self.assertEqual(collision.find('geometry/mesh').get('filename'), 'package://pair_description/' + asset)
            audit_hull(source, (paths[1] / asset).read_bytes(), Fraction(192000))
            component = ET.parse(paths[1] / 'urdf/components/base_link.urdf').getroot().find('link')
            self.assertEqual(ET.tostring(component).strip(), ET.tostring(right).strip())

    def test_public_failures_are_transactional_under_new_policy(self):
        publish = HullPackageTests().publish
        pair = stl_bytes(triangle_vertex_examples()[0][1])
        config = {'collision_method': 'convex_hull', 'convex_hull': {'source_policy': 'triangle_vertices'}}
        bad = stl_bytes([((0, 0, 0), (1, 0, 0), (0, 1, 0))])
        nonfinite = bytearray(pair)
        struct.pack_into('<f', nonfinite, 84, math.inf)
        cases = [(pair, {'collision_method': 'convex_hull'}, None, MeshGeometryError),
                 (bad, config, None, MeshGeometryError), (bytes(nonfinite), config, None, ValueError),
                 (stl_bytes([((0, 0, 0),) * 3]), config, None, MeshGeometryError),
                 (pair[:-1], config, None, ValueError),
                 (pair, config, lambda: True, CancelledError)]
        for key, value in (('max_vertices', 1), ('max_triangles', 1), ('max_work', 1),
                           ('max_faces', 1), ('max_output_bytes', 683), ('max_pieces', 1)):
            cases.append((pair, dict(config, convex_hull=dict(config['convex_hull'], **{key: value})),
                          None, ResourceLimitError))
        for data, options, cancel, error in cases:
            with self.subTest(options=options), tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(error):
                    publish(Path(directory) / 'bad_description', options, cancelled=cancel, data=data)
                self.assertEqual(list(Path(directory).iterdir()), [])
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                publish(Path(directory) / 'bad_description', {'convex_hull': {'source_policy': []}}, data=pair)
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_new_policy_aggregate_budgets_and_final_publication_cancellation(self):
        publish = HullPackageTests().publish
        source = stl_bytes(triangle_vertex_examples()[0][1])
        metadata = hull_stl(source, source_policy='triangle_vertices')['metadata']
        options = {'collision_method': 'convex_hull', 'convex_hull': {'source_policy': 'triangle_vertices'}}
        for key, limit in (('max_work', 2 * metadata['work_used']),
                           ('max_output_bytes', 2 * metadata['output_bytes'])):
            for adjustment in (0, -1):
                with self.subTest(key=key, adjustment=adjustment), tempfile.TemporaryDirectory() as directory:
                    config = dict(options, convex_hull=dict(options['convex_hull'], **{key: limit + adjustment}))
                    if adjustment:
                        with self.assertRaises(ResourceLimitError):
                            publish(Path(directory) / 'bad_description', config, data=source)
                        self.assertEqual(list(Path(directory).iterdir()), [])
                    else:
                        publish(Path(directory) / 'good_description', config, data=source)
        calls = [0]
        def count():
            calls[0] += 1
            return False
        with tempfile.TemporaryDirectory() as directory:
            publish(Path(directory) / 'count_description', options, cancelled=count, data=source)
        final = calls[0]
        calls[0] = 0
        def cancel():
            calls[0] += 1
            return calls[0] == final
        def fail():
            raise RuntimeError('caller failure')
        for callback, error in ((cancel, CancelledError), (fail, RuntimeError)):
            with tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(error):
                    publish(Path(directory) / 'bad_description', options, cancelled=callback, data=source)
                self.assertEqual(list(Path(directory).iterdir()), [])


def retain_triangle_vertex_packages(directory):
    from URDF_Exporter.core.Write import write_ros2_package
    from tests.fixtures import ZERO_POSES
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    config = {'collision_method': 'convex_hull', 'convex_hull': {'source_policy': 'triangle_vertices'}}
    cases = []
    for name, triangles, volume in triangle_vertex_examples():
        source = stl_bytes(triangles)
        result = hull_stl(source, source_policy='triangle_vertices')
        (directory / (name + '_source.stl')).write_bytes(source)
        (directory / (name + '_hull.stl')).write_bytes(result['pieces'][0])
        cases.append(dict(case=name, **audit_hull(source, result['pieces'][0], volume), metadata=result['metadata']))
    fixture, exported, bodies = offset_body_fixture()
    package = directory / 'combined_bodies_description'
    importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(fixture, package, config=config)
    assert exported == ['left', 'right']
    source = (package / 'meshes/base_link.stl').read_bytes()
    assert source[84:] == bodies[0].mesh[84:] + bodies[1].mesh[84:]
    receipt = json.loads((package / 'config/robot_data.yaml').read_text())['convex_hull']['links']['base_link']
    cases.append(dict(case='actual_combined_bodies',
        **audit_hull(source, (package / receipt['assets'][0]).read_bytes(), Fraction(192000)), metadata=receipt))
    expected = {'zero': {'positions': {}, 'link_poses': {'base_link': ZERO_POSES['base_link']},
        'visual_poses': {'base_link': ZERO_POSES['base_link']},
        'bounds': {'base_link': [[-.110, -.060, .010], [.110, .160, .090]]}}}
    (directory / 'combined_bodies_expected.json').write_text(json.dumps(expected, indent=2) + '\n')
    oblique = stl_bytes(triangle_vertex_examples()[1][1])
    def meshes(paths):
        paths['base_link'].write_bytes(oblique)
    write_ros2_package(directory / 'oblique_bodies_description', 'oblique_bodies', {},
        {'base_link': {'mass': 4, 'center_of_mass': [.05, .10, .065],
                       'inertia': [989 / 30000, 989 / 30000, 1808 / 30000, -.03, -.009, -.009]}},
        write_meshes=meshes, config=config)
    expected['zero']['bounds']['base_link'] = [[.180, -.120, .300], [.2475, -.085, .330]]
    (directory / 'oblique_bodies_expected.json').write_text(json.dumps(expected, indent=2) + '\n')
    report = {'method': 'independent decoded float32 Fraction determinants, analytic swept-box volumes, literal bounds and physical sums',
        'cases': cases, 'case_count': len(cases), 'halfspace_checks': sum(c['halfspace_checks'] for c in cases),
        'combined_physics': {'mass_kg': 4, 'center_of_mass_m': [.05, .10, .065],
            'inertia_kg_m2_order': ['ixx', 'iyy', 'izz', 'ixy', 'iyz', 'ixz'],
            'inertia_kg_m2': [989 / 30000, 989 / 30000, 1808 / 30000, -.03, -.009, -.009]},
        'evidence_kind': 'offline-Fusion-shaped-fixture-not-native-CAD'}
    (directory / 'triangle-vertex-oracles.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def retain_packages(directory):
    from URDF_Exporter.core.Write import write_ros2_package
    from tests.fixtures import model, oracle, MESH_BOUNDS_MM
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    cases = []
    geometry_cases = examples()
    rng = random.Random(111716)
    for index in range(24):
        triangles = roof_mesh(rng)
        geometry_cases.append(('roof_%02d' % index, transformed(triangles,
            rotation=((1, .5, .25), (0, 1, .5), (0, 0, 1)),
            translation=(index * 100, -index * 10, index)), None))
        rng.shuffle(triangles)
    for name, triangles, volume in geometry_cases:
        source = stl_bytes(triangles)
        result = hull_stl(source)
        (directory / (name + '_source.stl')).write_bytes(source)
        (directory / (name + '_hull.stl')).write_bytes(result['pieces'][0])
        cases.append(dict(case=name, analytic_volume=volume is not None,
                          **audit_hull(source, result['pieces'][0], volume), metadata=result['metadata']))
    for variant in ('fixed', 'articulated'):
        joints, links = model(fixed_only=variant == 'fixed')
        def meshes(paths):
            for name, path in paths.items():
                low, high = MESH_BOUNDS_MM[name]
                path.write_bytes(stl_bytes(transformed(cell_surface([(0, 0, 0), (1, 0, 0), (0, 1, 0)]),
                    scale=((high[0] - low[0]) / 2, (high[1] - low[1]) / 2, high[2] - low[2]), translation=low)))
        output = directory / (variant + '_hull_description')
        write_ros2_package(output, variant + '_hull', joints, links, write_meshes=meshes,
                           config={'collision_method': 'convex_hull'})
        (directory / (variant + '_expected.json')).write_text(json.dumps(oracle(variant == 'fixed'), indent=2) + '\n')
    (directory / 'numeric-oracles.json').write_text(json.dumps({'cases': cases,
        'method': 'independent Fraction determinants from decoded emitted binary STL; analytic prism/tetrahedron volumes for eight named cases',
        'case_count': len(cases), 'analytic_volume_cases': sum(c['analytic_volume'] for c in cases),
        'halfspace_checks': sum(c['halfspace_checks'] for c in cases)}, indent=2) + '\n')
    return cases


if __name__ == '__main__':
    import sys
    if len(sys.argv) == 3 and sys.argv[1] == '--artifacts':
        print(json.dumps(retain_packages(sys.argv[2]), indent=2))
    elif len(sys.argv) == 3 and sys.argv[1] == '--triangle-vertex-artifacts':
        print(json.dumps(retain_triangle_vertex_packages(sys.argv[2]), indent=2))
    else:
        unittest.main()
