"""Independent cyclic-rotation oracle for multibody visuals, hulls and inertia floor."""
import copy
from fractions import Fraction
import itertools
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from tests.test_per_body_visuals import (CONFIG as VISUAL_CONFIG, export_fixture as export_visuals,
    oracle as visual_oracle, decode_descriptor, assembly, ENTRY, source_state)
from tests.test_fusion_inputs import design
from tests.test_convex_hull import audit_hull, decode
from tests.test_public_motion import matrix_origin
from tests.test_public_export_policies import pose_matrix
from tests.test_generalized_motion import mm
from tools.verify_loaders import asset_receipts, mesh_path
from tools.verify_ros import fixture_requires_control
from URDF_Exporter.utils.mesh_geometry import CancelledError, MeshGeometryError

ROTATION = [0, 0, 1, 1, 0, 0, 0, 1, 0]
HULL_CONFIG = dict(VISUAL_CONFIG, collision_method='convex_hull',
                   convex_hull={'source_policy': 'triangle_vertices'})
CONFIG = dict(HULL_CONFIG, output_rotation=ROTATION, minimum_principal_inertia=.001)
IDENTITY = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def rotate_point(p):
    return [p[2], p[0], p[1]]


def rotate_pose(p):
    return [list(p[2]), list(p[0]), list(p[1]), [0, 0, 0, 1]]


def rotate_bounds(b):
    return [rotate_point(p) for p in b]


def oracle():
    cases = copy.deepcopy(visual_oracle())
    for values in cases.values():
        values['link_poses'] = {n: rotate_pose(p) for n, p in values['link_poses'].items()}
        values['link_poses']['export_output_reference'] = copy.deepcopy(IDENTITY)
        values['link_kinds'] = {'base_link': 'physical', 'repeat_1': 'physical', 'export_output_reference': 'frame'}
        values['visual_poses'] = {n: rotate_pose(p) for n, p in values['visual_poses'].items()}
        values['bounds'] = {n: rotate_bounds(b) for n, b in values['bounds'].items()}
        values['visual_descriptor_bounds'] = {n: [rotate_bounds(b) for b in parts]
                                              for n, parts in values['visual_descriptor_bounds'].items()}
        for physics in values['physics'].values():
            physics['inertia'] = [.001, .030266666666666667, .030266666666666667, 0., 0., 0.]
    return cases


def export_fixture(output, robot_name='fixture_per_body_hull'):
    return export_visuals(output, robot_name, config=CONFIG)


def world_points(pose, points):
    return {tuple(round(sum(pose[i][k]*p[k]/1000 for k in range(3))+pose[i][3], 10)
                  for i in range(3)) for p in points}


def corners(bounds):
    return {tuple(round(v, 10) for v in p) for p in itertools.product(*zip(*bounds))}


class PerBodyHullTests(unittest.TestCase):
    def test_all_visuals_hulls_literal_motion_full_physics_floor_and_unchanged_asset_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            baseline = export_visuals(directory/'plain', 'same', config=HULL_CONFIG)
            package = export_fixture(directory/'rotated', 'same')
            assets = lambda p: {f.relative_to(p/'meshes').as_posix(): f.read_bytes()
                                for f in (p/'meshes').rglob('*') if f.is_file()}
            self.assertEqual(assets(baseline), assets(package))
            root = ET.parse(package/'urdf/same.urdf').getroot()
            receipt = json.loads((package/'config/robot_data.yaml').read_text())
            descriptors = asset_receipts(package)
            self.assertEqual(receipt['root_frame']['output_reference_link'], 'export_output_reference')
            self.assertEqual(receipt['export_policies']['mesh_coordinates'], 'unchanged-source-design-millimeters')
            self.assertEqual(receipt['convex_hull']['physics_policy'], 'mass-unchanged; output-inertia-explicit-spectral-floor')
            self.assertTrue(any('topology is not verified' in w for w in receipt['warnings']))
            self.assertFalse(any('visual and CAD physics are unchanged' in w for w in receipt['warnings']))
            self.assertEqual(len(root.findall('link/inertial')), 2)
            self.assertEqual(len(root.findall('link/visual')), 4)
            self.assertEqual(len(root.findall('link/collision')), 2)
            for name in ('base_link', 'repeat_1'):
                link = root.find("link[@name='%s']" % name)
                self.assertEqual(float(link.find('inertial/mass').get('value')), 4.)
                for actual, expected in zip(map(float, link.find('inertial/origin').get('xyz').split()),
                                             oracle()['zero']['physics'][name]['com']):
                    self.assertAlmostEqual(actual, expected, places=12)
                for key, expected in zip(('ixx', 'iyy', 'izz', 'ixy', 'iyz', 'ixz'),
                                         oracle()['zero']['physics'][name]['inertia']):
                    self.assertAlmostEqual(float(link.find('inertial/inertia').get(key)), expected, places=12)
                floor = receipt['export_policies']['receipts']['inertia'][name]
                self.assertTrue(floor['enabled'] and floor['changed'])
                self.assertEqual(floor['threshold_kg_m2'], .001)
                metadata = receipt['convex_hull']['links'][name]
                self.assertFalse(metadata['source_topology_verified'])
                self.assertEqual(metadata['validation_pair_checks'], 0)
                self.assertEqual(metadata['source_triangle_count'], 24)
                self.assertEqual(metadata['source_vertex_count'], 16)
                collision, = link.findall('collision')
                asset, = metadata['assets']
                self.assertEqual(collision.find('geometry/mesh').get('filename'), 'package://same_description/'+asset)
                audit_hull((package/('meshes/'+name+'.stl')).read_bytes(), (package/asset).read_bytes(), Fraction(88000))
                self.assertEqual(len(descriptors[name]), 2)
                for index, (element, descriptor) in enumerate(zip(link.findall('visual'), descriptors[name])):
                    self.assertEqual(element.find('geometry/mesh').get('filename'), 'package://same_description/'+descriptor['mesh_path'])
                    self.assertEqual(element.find('geometry/mesh').get('scale'), '0.001 0.001 0.001')
                    self.assertEqual(element.find('origin').attrib, collision.find('origin').attrib)
                    self.assertEqual(descriptor['body_context']['selected_body_count'], 2)
                    self.assertEqual(descriptor['body_context']['id'], '%s_body_%d' % (name, index))
                    self.assertEqual(len(descriptor['images']), 3)
                    obj, materials, _ = decode_descriptor(package, descriptor)
                    self.assertEqual(len(obj['triangles']), 12)
                    self.assertEqual(len(materials), 5)
                    self.assertEqual(len([1 for m, _ in obj['triangles'] if 'map_Kd' in materials[m]]), 6)
                    self.assertEqual({uv for _, triangle in obj['triangles'] for _, uv, _ in triangle if uv is not None},
                                     {(-.25, .125), (1.25, .125), (1.25, .875), (-.25, .875)})
                self.assertEqual(collision.find('geometry/mesh').get('scale'), '0.001 0.001 0.001')
            for label, expected in oracle().items():
                poses = {n: pose_matrix(root, n, expected['positions']) for n in expected['link_poses']}
                for name, wanted in expected['link_poses'].items():
                    for actual_row, wanted_row in zip(poses[name], wanted):
                        for a, b in zip(actual_row, wanted_row):
                            self.assertAlmostEqual(a, b, places=12)
                for name in descriptors:
                    link = root.find("link[@name='%s']" % name)
                    for index, (element, descriptor) in enumerate(zip(link.findall('visual'), descriptors[name])):
                        obj, _, _ = decode_descriptor(package, descriptor)
                        action = mm(poses[name], matrix_origin(element.find('origin')))
                        self.assertEqual(world_points(action, obj['vertices']), corners(expected['visual_descriptor_bounds'][name][index]))
                    collision = link.find('collision')
                    action = mm(poses[name], matrix_origin(collision.find('origin')))
                    path = mesh_path(collision.find('geometry/mesh').get('filename'), package, package/'urdf/same.urdf')
                    self.assertEqual(world_points(action, [p for t in decode(Path(path).read_bytes()) for p in t]), corners(expected['bounds'][name]))

    def test_public_source_and_prior_package_survive_invalid_policy_late_failure_and_final_cancel(self):
        component, _ = assembly()
        fixture, _ = design(component)
        before = source_state(fixture)
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            prior = ENTRY.export_design(fixture, directory/'prior_description', config=CONFIG)
            prior_bytes = {p.relative_to(prior): p.read_bytes() for p in prior.rglob('*') if p.is_file()}
            calls = [0]
            def count():
                calls[0] += 1
                return False
            ENTRY.export_design(fixture, directory/'count_description', config=CONFIG, cancelled=count)
            last = calls[0]
            for mode in ('invalid_policy', 'closed_default', 'late_hull', 'bad_second_stl', 'cancel', 'callback_error'):
                destination = directory/(mode+'_description')
                if mode == 'invalid_policy':
                    with self.assertRaises(ValueError):
                        ENTRY.export_design(fixture, destination, config=dict(CONFIG, convex_hull={'source_policy': []}))
                elif mode == 'closed_default':
                    with self.assertRaises(MeshGeometryError):
                        ENTRY.export_design(fixture, destination, config=dict(CONFIG, convex_hull={}))
                elif mode == 'late_hull':
                    from URDF_Exporter.core import Write
                    original = Write.hull_stl
                    seen = [0]
                    def fail_second(*args, **kwargs):
                        seen[0] += 1
                        if seen[0] == 2:
                            raise RuntimeError('second hull failed')
                        return original(*args, **kwargs)
                    with patch.object(Write, 'hull_stl', fail_second), self.assertRaisesRegex(RuntimeError, 'second hull'):
                        ENTRY.export_design(fixture, destination, config=CONFIG)
                    self.assertEqual(seen[0], 2)
                elif mode == 'bad_second_stl':
                    from URDF_Exporter.utils import utils
                    original = utils.export_stl
                    def corrupt_second(design, targets, **kwargs):
                        original(design, targets, **kwargs)
                        data = bytearray(targets['repeat_1'].read_bytes())
                        struct.pack_into('<f', data, 96, math.nan)
                        targets['repeat_1'].write_bytes(data)
                    with patch.object(utils, 'export_stl', corrupt_second), self.assertRaises(ValueError):
                        ENTRY.export_design(fixture, destination, config=CONFIG)
                else:
                    calls[0] = 0
                    def cancel():
                        calls[0] += 1
                        if calls[0] == last and mode == 'callback_error':
                            raise RuntimeError('caller cancellation failed')
                        return calls[0] == last
                    with self.assertRaises(RuntimeError if mode == 'callback_error' else CancelledError):
                        ENTRY.export_design(fixture, destination, config=CONFIG, cancelled=cancel)
                self.assertFalse(destination.exists())
                self.assertEqual(source_state(fixture), before)
                self.assertEqual({p.relative_to(prior): p.read_bytes() for p in prior.rglob('*') if p.is_file()}, prior_bytes)
            self.assertEqual(sorted(p.name for p in directory.iterdir()), ['count_description', 'prior_description'])

    def test_registry_control_routing_uses_oracle_commands_not_variant_names(self):
        from tests.fixtures import fixture_oracle, VARIANTS, OPTIONAL_VARIANTS
        for variant in VARIANTS+OPTIONAL_VARIANTS:
            expected = fixture_oracle(variant)
            with self.subTest(variant=variant):
                self.assertEqual(fixture_requires_control(expected), bool(expected['zero']['positions']))
        for variant in ('per_body', 'per_body_hull'):
            self.assertTrue(fixture_requires_control(fixture_oracle(variant)))
            self.assertEqual(fixture_oracle(variant)['moved']['positions'], {'turn': math.pi/2})
        self.assertTrue(fixture_requires_control({'zero': {'positions': {'future_joint': 0}},
                                                'moved': {'positions': {'future_joint': .3}}}))
        for bad in ({}, {'moved': {'positions': {'turn': 0}}}, {'moved': {'positions': {'turn': math.nan}}}):
            with self.assertRaises(AssertionError):
                fixture_requires_control(dict({'zero': {'positions': {'turn': 0}}}, **bad))
        with self.assertRaisesRegex(AssertionError, 'declared zero-sample'):
            fixture_requires_control({'zero': {'positions': {}}, 'moved': {'positions': {'turn': .3}}})

    def test_ros_main_routes_new_moving_cases_to_control_and_fixed_cases_to_display_only(self):
        from contextlib import redirect_stdout
        import io
        import sys
        from types import ModuleType
        from unittest.mock import Mock
        from tools import verify_ros
        modules = {n: ModuleType(n) for n in ('ament_index_python', 'ament_index_python.packages',
                                             'rclpy', 'rclpy.utilities')}
        modules['rclpy'].init = Mock()
        modules['rclpy'].shutdown = Mock()
        modules['rclpy.utilities'].get_rmw_implementation_identifier = lambda: 'offline-routing-test'
        for variant in ('per_body', 'per_body_hull', 'materials', 'surface'):
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as directory:
                install = Path(directory)/'install'
                output = Path(directory)/'output'
                modules['ament_index_python.packages'].get_package_share_directory = lambda name: str(install/name)
                launch = Mock(return_value={'routing_only': True})
                with patch.dict(sys.modules, modules), patch.object(sys, 'argv', ['verify_ros.py',
                        '--output', str(output), '--install', str(install), '--fixture', variant]), \
                        patch.object(verify_ros, 'check_installed', return_value=['routing-only']), \
                        patch.object(verify_ros, 'verify_launch', launch), redirect_stdout(io.StringIO()):
                    verify_ros.main()
                self.assertEqual([call.args[1] for call in launch.call_args_list],
                    ['display', 'control'] if variant.startswith('per_body') else ['display'])
                if variant.startswith('per_body'):
                    self.assertEqual(launch.call_args_list[1].args[2]['moved']['positions'], {'turn': math.pi/2})

    def test_native_mesh_per_body_materials_reject_before_brep_face_collection(self):
        from tests.test_mesh_inputs import mesh_root, snapshot_design
        from tests.test_mesh_geometry import cell_surface
        component, _ = mesh_root(cell_surface([(0, 0, 0)]))
        fixture, _ = snapshot_design(component)
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/'unsupported_description'
            with patch('URDF_Exporter.utils.visual_assets.read_native_body',
                       side_effect=AssertionError('MeshBody reached BRep collector')):
                with self.assertRaisesRegex(ValueError, 'MESH_BODY_VISUAL_MATERIALS_UNSUPPORTED'):
                    ENTRY.export_design(fixture, target, config=dict(CONFIG, mesh_physics={'density_kg_m3': 2000}))
            self.assertEqual(list(Path(directory).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
