"""Public per-body exports with literal geometry, material and physics oracles."""

import copy
import hashlib
import importlib
import itertools
import json
import math
from pathlib import Path
import re
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakeCollection, FakePoint
from tests.test_fusion_inputs import Body, design, hinge, occurrence, root
from tests.test_visual_assets import (color_appearance, face, mesh, decode_assets, image_appearance,
                                     png_rgba, KNOWN_JPEG, progressive_gray_jpeg, with_properties, Frozen)
from URDF_Exporter.core.Write import write_ros2_package
from URDF_Exporter.utils.config import load_config
from URDF_Exporter.utils import utils
from URDF_Exporter.utils.mesh_geometry import CancelledError

ENTRY = importlib.import_module('URDF_Exporter.URDF_Exporter')
CONFIG = {'visual_meshes': 'per_body', 'visual_materials': 'strict'}


def cube_body(name, x, mass, colors, images=()):
    result = Body(name, x, mass=mass)
    vertices = [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    normals = ((0, 0, -1), (0, 0, 1), (0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0))
    result.faces = FakeCollection([face(image_appearance(images[index-3]) if index >= 3 and images
                                       else color_appearance(*colors[index % len(colors)]),
        mesh(vertices=[(x + vertices[i][0], vertices[i][1], vertices[i][2]) for i in indices],
             normals=[normals[index]] * 4, indices=[0, 1, 2, 0, 2, 3],
             uvs=[(-.25, .125), (1.25, .125), (1.25, .875), (-.25, .875)]))
        for index, indices in enumerate(faces)])
    result.visibleOpacity, result.appearance = .5, None
    return result


def selected_bodies(images=()):
    return [cube_body('red', -10, 1, [(255, 0, 0, 128), (255, 128, 0, 255)], images),
            cube_body('blue', 10, 3, [(0, 0, 255, 255), (0, 255, 255, 128)], images)]


def assembly(images=()):
    bodies = selected_bodies(images)
    hidden = cube_body('hidden', 100, 99, [(255, 255, 255, 255)])
    hidden.isVisible = False
    base = occurrence('base:1', [*bodies, hidden], component='base_link')
    base.appearance = None
    proxies = [copy.copy(b) for b in bodies]
    for proxy, native in zip(proxies, bodies):
        proxy.nativeObject = native
        x, y, z, mass = 20, 30 + native.properties.centerOfMass.asArray()[0], 40, native.properties.mass
        proxy.properties = NS(mass=mass, centerOfMass=FakePoint([x, y, z]),
            getXYZMomentsOfInertia=lambda x=x, y=y, z=z, m=mass: (True,
                m*(2/3+y*y+z*z), m*(2/3+x*x+z*z), m*(2/3+x*x+y*y), -m*x*y, -m*y*z, -m*x*z))
        proxy.boundingBox = NS(minPoint=FakePoint([x-1, y-1, z-1]), maxPoint=FakePoint([x+1, y+1, z+1]))
    moved = occurrence('repeat:1', proxies, matrix=[0, -1, 0, 20, 1, 0, 0, 30, 0, 0, 1, 40, 0, 0, 0, 1])
    moved.appearance = None
    joint = hinge('turn', moved, base)
    return root([base, moved], joints=[joint]), bodies


def export_fixture(output, robot_name='fixture_per_body', *, config=None):
    sources = Path(output) / 'per_body_sources'
    sources.mkdir(parents=True, exist_ok=True)
    images = []
    for name, data in (('shared.png', png_rgba()), ('baseline.jpg', KNOWN_JPEG), ('progressive.jpg', progressive_gray_jpeg())):
        path = sources / name
        if path.exists():
            assert path.read_bytes() == data
        else:
            path.write_bytes(data)
        images.append(path)
    component, _ = assembly(images)
    fixture, _ = design(component)
    return ENTRY.export_design(fixture, Path(output) / (robot_name + '_description'),
                               config=CONFIG if config is None else config)


def oracle():
    identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    cases = {
        'zero': {'positions': {'turn': 0.}, 'masses': {'base_link': 4., 'repeat_1': 4.},
                 'link_poses': {'base_link': identity, 'repeat_1': [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]},
                 'visual_poses': {'base_link': identity, 'repeat_1': identity},
                 'bounds': {'base_link': [[-.11, -.01, -.01], [.11, .01, .01]],
                            'repeat_1': [[.19, .19, .39], [.21, .41, .41]]}},
        'moved': {'positions': {'turn': math.pi / 2}, 'masses': {'base_link': 4., 'repeat_1': 4.},
                  'link_poses': {'base_link': identity, 'repeat_1': [[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]},
                  'visual_poses': {'base_link': identity, 'repeat_1': [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]},
                  'bounds': {'base_link': [[-.11, -.01, -.01], [.11, .01, .01]],
                             'repeat_1': [[-.41, .19, .39], [-.19, .21, .41]]}},
    }
    for case, values in cases.items():
        values['physics'] = {
            'base_link': {'com': [.05, 0., 0.], 'inertia': [.0002666666666666667, .030266666666666667, .030266666666666667, 0., 0., 0.]},
            'repeat_1': {'com': [.35, -.2, .4], 'inertia': [.0002666666666666667, .030266666666666667, .030266666666666667, 0., 0., 0.]}}
        values['materials'] = {name: {'visuals': [dict(triangles=12, materials=5,
            png_rgba8=[[255, 0, 0, 255], [0, 255, 0, 64]],
            jpeg_rgb8=[[245, 2, 4], [3, 255, 2], [2, 0, 247], [255, 249, 250]],
            progressive_gray8=[128]) for _ in range(2)]} for name in ('base_link', 'repeat_1')}
        values['visual_descriptor_bounds'] = {
            'base_link': [[[-.11, -.01, -.01], [-.09, .01, .01]], [[.09, -.01, -.01], [.11, .01, .01]]],
            'repeat_1': ([[[.19, .19, .39], [.21, .21, .41]], [[.19, .39, .39], [.21, .41, .41]]]
                         if case == 'zero' else
                         [[[-.21, .19, .39], [-.19, .21, .41]], [[-.41, .19, .39], [-.39, .21, .41]]])}
    return cases


def read_assets(package):
    return json.loads((package / 'config/robot_data.yaml').read_text())['visual_assets']


def decode_descriptor(package, descriptor):
    return decode_assets({'descriptor': descriptor,
                          'files': {e['path']: (package / e['path']).read_bytes() for e in descriptor['files']}})


def pure_inputs():
    from URDF_Exporter.core import Link
    sources = utils.link_sources(root(bodies=selected_bodies()))
    inertial, status = Link.make_inertial_dict(sources, 'ok')
    assert status == 'ok'
    return inertial, utils.collect_visual_assets(sources, CONFIG)


def source_state(value):
    if isinstance(value, (str, bytes, int, float, bool, type(None))):
        return value
    if callable(value):
        return ('callable', id(value))
    if isinstance(value, (list, tuple)):
        return tuple(source_state(item) for item in value)
    if isinstance(value, dict):
        return {key: source_state(item) for key, item in value.items()}
    return (type(value).__name__, source_state(vars(value)))


class PerBodyVisualTests(unittest.TestCase):
    def test_coincident_bodies_still_have_distinct_visual_files(self):
        bodies = [cube_body(name, 0, 1, [(255, 0, 0, 255)]) for name in ('one', 'two')]
        fixture, _ = design(root(bodies=bodies))
        with tempfile.TemporaryDirectory() as directory:
            package = ENTRY.export_design(fixture, Path(directory) / 'coincident_description', config=CONFIG)
            descriptors = read_assets(package)['base_link']
            self.assertEqual(len({d['mesh_path'] for d in descriptors}), 2)
            self.assertEqual(len(list((package / 'meshes').glob('*.mtl'))), 1)
            for index, descriptor in enumerate(descriptors):
                self.assertIn(('o base_link_body_%d\n' % index).encode(), (package / descriptor['mesh_path']).read_bytes())

    def test_subset_retains_second_body_losses_and_unsupported_inputs_do_not_fall_back(self):
        bodies = selected_bodies()
        selected = bodies[1].faces[0]
        bodies[1].faces[0] = face(with_properties(selected.appearance,
            Frozen(id='roughness', objectType='adsk::core::FloatProperty')), selected.meshManager.data)
        fixture, _ = design(root(bodies=bodies))
        before = source_state(bodies)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'unsupported-or-ambiguous-shader'):
                ENTRY.export_design(fixture, Path(directory) / 'strict_description', config=CONFIG)
            self.assertEqual(list(Path(directory).iterdir()), [])
            package = ENTRY.export_design(fixture, Path(directory) / 'subset_description',
                config=dict(CONFIG, visual_materials='diffuse-subset'))
            second = read_assets(package)['base_link'][1]
            self.assertEqual(second['surfaces'][0]['appearance']['losses'],
                [{'id': 'roughness', 'type': 'adsk::core::FloatProperty', 'reason': 'omitted-shader-property'}])
            self.assertEqual(source_state(bodies), before)
            fixture.rootComponent.meshBodies = [NS(name='scan', isVisible=True)]
            with self.assertRaisesRegex(ValueError, 'MESH_PHYSICS_REQUIRED.*scan'):
                ENTRY.export_design(fixture, Path(directory) / 'unsupported_description', config=CONFIG)
            self.assertFalse((Path(directory) / 'unsupported_description').exists())

    def test_config_is_explicit_and_incompatible_legacy_rejects(self):
        self.assertEqual(load_config().get('visual_meshes', 'combined'), 'combined')
        self.assertEqual(load_config(overrides=CONFIG)['visual_meshes'], 'per_body')
        for bad in ({'visual_meshes': 'per_body'}, {'visual_meshes': True},
                    {'visual_meshes': 'separate'}, {'visual_meshes': []}):
            with self.subTest(bad=bad), self.assertRaisesRegex(ValueError, 'visual_meshes'):
                load_config(overrides=bad)

    def test_two_offset_multiface_bodies_one_link_and_aggregate_physics_collision(self):
        bodies = selected_bodies()
        fixture, exported = design(root(bodies=bodies))
        before = source_state(bodies)
        with tempfile.TemporaryDirectory() as directory:
            per = ENTRY.export_design(fixture, Path(directory) / 'per_description', config=CONFIG)
            combined = ENTRY.export_design(fixture, Path(directory) / 'combined_description',
                                           config={'visual_materials': 'strict'})
            robot = ET.parse(per / 'urdf/per.urdf').getroot()
            self.assertEqual(len(robot.findall('link')), 1)
            link = robot.find('link')
            self.assertEqual(len(link.findall('visual')), 2)
            self.assertEqual(len(link.findall('collision')), 1)
            self.assertEqual((per / 'meshes/base_link.stl').read_bytes(),
                             (combined / 'meshes/base_link.stl').read_bytes())
            self.assertEqual(float(link.find('inertial/mass').get('value')), 4.)
            self.assertEqual(list(map(float, link.find('inertial/origin').get('xyz').split())), [.05, 0., 0.])
            expected = {'ixx': .0002666666666666667, 'iyy': .030266666666666667,
                        'izz': .030266666666666667, 'ixy': 0., 'iyz': 0., 'ixz': 0.}
            for key, value in expected.items():
                self.assertAlmostEqual(float(link.find('inertial/inertia').get(key)), value, places=12)
            descriptors = read_assets(per)['base_link']
            self.assertEqual(len(descriptors), 2)
            paths = [v.find('geometry/mesh').get('filename') for v in link.findall('visual')]
            self.assertEqual(len(set(p.casefold() for p in paths)), 2)
            for index, (descriptor, lo, hi, colors) in enumerate(zip(descriptors, [-110., 90.], [-90., 110.],
                    [([1., 0., 0.], [1., 128/255, 0.]), ([0., 0., 1.], [0., 1., 1.])])):
                self.assertTrue(paths[index].endswith('/' + descriptor['mesh_path']))
                self.assertEqual(descriptor['body_context'], {'id': 'base_link_body_%d' % index,
                    'source_path': 'design_root', 'selected_body_count': 2})
                self.assertEqual(descriptor['triangle_count'], 12)
                self.assertEqual(len(descriptor['surfaces']), 6)
                obj, materials, _ = decode_descriptor(per, descriptor)
                self.assertEqual(min(v[0] for v in obj['vertices']), lo)
                self.assertEqual(max(v[0] for v in obj['vertices']), hi)
                self.assertEqual(obj['vertices'][0], (lo, -10., -10.))
                self.assertEqual(set(obj['vertices']), set(itertools.product((lo, hi), (-10., 10.), (-10., 10.))))
                self.assertEqual({tuple(map(float, m['Kd'])) for m in materials.values()}, set(map(tuple, colors)))
                self.assertEqual({float(m['d'][0]) for m in materials.values()}, {.5, 128/255*.5})
                self.assertEqual({s['context_id'] for s in descriptor['surfaces']}, {'base_link_body_%d' % index})
                self.assertEqual([s['face_index'] for s in descriptor['surfaces']], list(range(6)))
                for face_index, surface in enumerate(descriptor['surfaces']):
                    material = materials[surface['material']]
                    self.assertEqual(list(map(float, material['Kd'])), list(colors[face_index % 2]))
                    alpha = ([128/255*.5, .5] if index == 0 else [.5, 128/255*.5])[face_index % 2]
                    self.assertEqual(float(material['d'][0]), alpha)
                    self.assertEqual([m for m, _ in obj['triangles'][2*face_index:2*face_index+2]], [surface['material']]*2)
            self.assertEqual(exported, ['red', 'blue', 'red', 'blue'])
            self.assertEqual(source_state(bodies), before)

    def test_repeated_instances_hidden_selection_and_literal_baked_pose(self):
        component, originals = assembly()
        fixture, exported = design(component)
        with tempfile.TemporaryDirectory() as directory:
            package = ENTRY.export_design(fixture, Path(directory) / 'repeated_description', config=CONFIG)
            descriptors = read_assets(package)
            self.assertEqual(set(descriptors), {'base_link', 'repeat_1'})
            self.assertEqual([len(v) for v in descriptors.values()], [2, 2])
            for descriptor, wanted in zip(descriptors['repeat_1'], [(210., 190., 390.), (210., 390., 390.)]):
                obj, _, _ = decode_descriptor(package, descriptor)
                self.assertEqual(obj['vertices'][0], wanted)
                x, y, z = wanted
                self.assertEqual((-y/1000, x/1000, z/1000),
                                 (-.19, .21, .39) if x == 210 and y == 190 else (-.39, .21, .39))
            self.assertEqual(exported, ['red', 'blue', 'red', 'blue'])
            self.assertEqual([b.name for b in originals], ['red', 'blue'])

    def test_all_visual_origins_follow_literal_joint_motion(self):
        from tests.test_generalized_motion import mm
        from tests.test_public_motion import xml_fk, matrix_origin
        component, _ = assembly()
        fixture, _ = design(component)
        with tempfile.TemporaryDirectory() as directory:
            package = ENTRY.export_design(fixture, Path(directory) / 'motion_description', config=CONFIG)
            robot = ET.parse(package / 'urdf/motion.urdf').getroot()
            for q, expected in ((0., [( .21, .19, .39), (.21, .39, .39)]),
                                (math.pi/2, [(-.19, .21, .39), (-.39, .21, .39)])):
                poses = xml_fk(robot, {'turn': q})
                for index, (visual, descriptor, wanted) in enumerate(zip(robot.findall("link[@name='repeat_1']/visual"),
                                                      read_assets(package)['repeat_1'], expected)):
                    obj, _, _ = decode_descriptor(package, descriptor)
                    action = mm(poses['repeat_1'], matrix_origin(visual.find('origin')))
                    point = [v/1000 for v in obj['vertices'][0]]
                    actual = [sum(action[i][k]*point[k] for k in range(3))+action[i][3] for i in range(3)]
                    for a, b in zip(actual, wanted):
                        self.assertAlmostEqual(a, b, places=12)
                    axes = (((.19, .21), (.19, .21) if index == 0 else (.39, .41), (.39, .41)) if q == 0 else
                            ((-.21, -.19) if index == 0 else (-.41, -.39), (.19, .21), (.39, .41)))
                    all_world = {tuple(round(sum(action[i][k]*vertex[k]/1000 for k in range(3))+action[i][3], 12)
                                       for i in range(3)) for vertex in obj['vertices']}
                    self.assertEqual(all_world, set(itertools.product(*axes)))

    def test_invalid_second_bundle_and_mixed_shapes_reject_before_transaction(self):
        inertial, assets = pure_inputs()
        for mode in ('combined-shape', 'mixed', 'duplicate', 'context', 'path', 'extra', 'missing', 'drop-last',
                     'triangle', 'uv', 'policy', 'shape', 'plain', 'bool-index', 'budget'):
            changed = copy.deepcopy(assets)
            second = changed['base_link'][1]
            if mode == 'combined-shape':
                changed['base_link'] = second
            elif mode == 'mixed':
                changed['base_link'][1] = [second]
            elif mode == 'duplicate':
                changed['base_link'][1] = copy.deepcopy(changed['base_link'][0])
            elif mode == 'context':
                second['descriptor']['surfaces'][0]['context_id'] = 'unselected_body_1'
            elif mode == 'path':
                second['descriptor']['body_context']['source_path'] = 'unselected:1'
            elif mode == 'extra':
                second['files']['meshes/extra.obj'] = b'not selected'
            elif mode == 'missing':
                second['files'].pop(second['descriptor']['mesh_path'])
            elif mode == 'drop-last':
                changed['base_link'].pop()
            elif mode == 'triangle':
                second['descriptor']['triangle_count'] += 1
            elif mode == 'uv':
                second['descriptor']['surfaces'][0]['material'] = 'unknown'
            elif mode == 'policy':
                second['descriptor']['surfaces'][0]['appearance']['shader_policy'] = 'diffuse-subset'
            elif mode == 'shape':
                second['descriptor']['images'] = {}
            elif mode == 'plain':
                second['descriptor']['surfaces'] = tuple(second['descriptor']['surfaces'])
            elif mode == 'bool-index':
                second['descriptor']['surfaces'][0]['first_triangle'] = False
            else:
                second['descriptor']['files'] *= 2049
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                destination = Path(directory) / 'bad_description'
                with self.assertRaises(ValueError):
                    write_ros2_package(destination, 'bad', {}, inertial, config=CONFIG, visual_assets=changed,
                                       write_meshes=lambda _: self.fail('Invalid assets reached staging'))
                self.assertEqual(list(Path(directory).iterdir()), [])

    def test_second_body_read_write_and_cancel_failures_preserve_source_and_prior_output(self):
        component = root(bodies=selected_bodies())
        fixture, _ = design(component)
        before = source_state(component)
        with tempfile.TemporaryDirectory() as directory:
            prior = ENTRY.export_design(fixture, Path(directory) / 'prior_description')
            prior_bytes = {p.relative_to(prior): p.read_bytes() for p in prior.rglob('*') if p.is_file()}
            for mode in ('read', 'write', 'cancel'):
                destination = Path(directory) / 'failed_description'
                if mode == 'read':
                    second = component.bRepBodies[1]
                    old = second.faces
                    del second.faces
                    try:
                        with self.assertRaises(ValueError):
                            ENTRY.export_design(fixture, destination, config=CONFIG)
                    finally:
                        second.faces = old
                elif mode == 'write':
                    original = Path.open
                    count = [0]
                    def failed_open(path, *args, **kwargs):
                        if path.suffix == '.obj' and args and args[0] == 'xb':
                            count[0] += 1
                            if count[0] == 2:
                                raise OSError('second body write failed')
                        return original(path, *args, **kwargs)
                    with patch.object(Path, 'open', failed_open), self.assertRaises(OSError):
                        ENTRY.export_design(fixture, destination, config=CONFIG)
                    self.assertEqual(count[0], 2)
                else:
                    checks = [0]
                    def cancelled():
                        checks[0] += 1
                        return checks[0] == 5
                    with self.assertRaises(CancelledError):
                        ENTRY.export_design(fixture, destination, config=CONFIG, cancelled=cancelled)
                self.assertFalse(destination.exists())
                self.assertEqual([p.name for p in Path(directory).iterdir()], ['prior_description'])
                self.assertEqual(source_state(component), before)
                self.assertEqual({p.relative_to(prior): p.read_bytes() for p in prior.rglob('*') if p.is_file()}, prior_bytes)

    def test_receipts_and_linter_check_every_visual_and_missing_second_resource(self):
        from tools.verify_loaders import asset_receipts
        from tools.validate_urdf import lint_file
        with tempfile.TemporaryDirectory() as directory:
            package = export_fixture(directory)
            descriptors = asset_receipts(package)
            self.assertEqual([len(v) for v in descriptors.values()], [2, 2])
            urdf = package / 'urdf/fixture_per_body.urdf'
            self.assertEqual(lint_file(urdf, package / 'meshes'), ([], []))
            path = package / descriptors['repeat_1'][1]['mesh_path']
            path.rename(path.with_suffix('.missing'))
            with self.assertRaisesRegex(AssertionError, 'Missing'):
                asset_receipts(package)
            self.assertTrue(lint_file(urdf, package / 'meshes')[0])

    def test_shared_images_deduplicate_without_losing_body_uv_or_resources(self):
        from tools.verify_loaders import asset_receipts
        with tempfile.TemporaryDirectory() as directory:
            package = export_fixture(directory)
            descriptors = asset_receipts(package)
            images = {e['path'] for value in descriptors.values() for d in value for e in d['images']}
            self.assertEqual(len(images), 3)
            self.assertEqual(len(list((package / 'meshes/textures').iterdir())), 3)
            for value in descriptors.values():
                for descriptor in value:
                    obj, materials, _ = decode_descriptor(package, descriptor)
                    self.assertEqual(len(descriptor['images']), 3)
                    self.assertEqual(len(obj['triangles']), 12)
                    textured = [(name, corners) for name, corners in obj['triangles'] if 'map_Kd' in materials[name]]
                    self.assertEqual(len(textured), 6)
                    self.assertEqual({uv for _, corners in textured for _, uv, _ in corners},
                                     {(-.25, .125), (1.25, .125), (1.25, .875), (-.25, .875)})
            source_hashes = {hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(directory) / 'per_body_sources').iterdir()}
            self.assertEqual(source_hashes, {hashlib.sha256((package / p).read_bytes()).hexdigest() for p in images})

    def test_second_body_missing_uv_with_valid_hashes_rejects(self):
        from URDF_Exporter.core import Link
        with tempfile.TemporaryDirectory() as directory:
            images = []
            for name, data in (('a.png', png_rgba()), ('b.jpg', KNOWN_JPEG), ('c.jpg', progressive_gray_jpeg())):
                path = Path(directory) / name
                path.write_bytes(data)
                images.append(path)
            sources = utils.link_sources(root(bodies=selected_bodies(images)))
            inertial, _ = Link.make_inertial_dict(sources, 'ok')
            assets = utils.collect_visual_assets(sources, CONFIG)
            second = assets['base_link'][1]
            old_path = second['descriptor']['mesh_path']
            data = re.sub(rb'(\d+)/\d+/(\d+)', rb'\1//\2', second['files'].pop(old_path))
            new_path = 'meshes/visual_' + hashlib.sha256(data).hexdigest() + '.obj'
            second['files'][new_path] = data
            second['descriptor']['mesh_path'] = new_path
            for entry in second['descriptor']['files']:
                if entry['path'] == old_path:
                    entry.update(path=new_path, sha256=hashlib.sha256(data).hexdigest(), size=len(data))
            with self.assertRaisesRegex(ValueError, 'uv-mismatch'):
                write_ros2_package(Path(directory) / 'bad_description', 'bad', {}, inertial, config=CONFIG,
                    visual_assets=assets, write_meshes=lambda _: self.fail('Missing UV reached staging'))
            self.assertFalse((Path(directory) / 'bad_description').exists())

    def test_combined_explicit_option_is_byte_identical(self):
        fixture, _ = design(root(bodies=selected_bodies()))
        with tempfile.TemporaryDirectory() as directory:
            for policy in ('legacy', 'strict'):
                packages = []
                for parent, options in (('implicit', {}), ('explicit', {'visual_meshes': 'combined'})):
                    packages.append(ENTRY.export_design(fixture, Path(directory) / policy / parent / 'same_description',
                        config=dict(visual_materials=policy, **options)))
                actual = [{p.relative_to(package).as_posix(): p.read_bytes() for p in package.rglob('*') if p.is_file()}
                          for package in packages]
                self.assertEqual(actual[0], actual[1])


if __name__ == '__main__':
    unittest.main()
