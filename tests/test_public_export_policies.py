"""Public XML/STL checks with an independent source-space screw-motion oracle."""

import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

from URDF_Exporter.core.Write import write_ros2_package
from URDF_Exporter.utils import config
from URDF_Exporter.utils.mesh_geometry import CancelledError
from tests.fixtures import write_tetrahedron
from tests.test_package import model as legacy_model, write_meshes as legacy_meshes


IDENTITY = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
Y_UP = [1., 0., 0., 0., 0., -1., 0., 1., 0.]
SAMPLES = ({'turn': 0., 'slide': 0.}, {'turn': .43, 'slide': -.07},
           {'turn': -.27, 'slide': .12})
BOUNDS = {'base_link': ((-310, 240, -170), (-190, 580, 390)),
          'arm': ((110, -230, 310), (230, 110, 870)),
          'carriage': ((-70, 130, -210), (50, 470, 350))}


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def scale(a, s):
    return [x*s for x in a]


def mv(m, v):
    return [sum(m[3*i+j]*v[j] for j in range(3)) for i in range(3)]


def mm(a, b):
    return [sum(a[3*i+k]*b[3*k+j] for k in range(3)) for i in range(3) for j in range(3)]


def transpose(a):
    return [a[3*j+i] for i in range(3) for j in range(3)]


def euler(v, angles):
    x, y, z = v
    r, p, w = angles
    y, z = math.cos(r)*y-math.sin(r)*z, math.sin(r)*y+math.cos(r)*z
    x, z = math.cos(p)*x+math.sin(p)*z, -math.sin(p)*x+math.cos(p)*z
    return [math.cos(w)*x-math.sin(w)*y, math.sin(w)*x+math.cos(w)*y, z]


def rotation(angles):
    axes = [euler(v, angles) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return [axes[j][i] for i in range(3) for j in range(3)]


def unit(v):
    return scale(v, 1/math.hypot(*v))


def screw(v, axis, angle):
    x, y, z = axis
    cross = [y*v[2]-z*v[1], z*v[0]-x*v[2], x*v[1]-y*v[0]]
    return add(add(scale(v, math.cos(angle)), scale(cross, math.sin(angle))),
               scale(axis, sum(a*b for a, b in zip(axis, v))*(1-math.cos(angle))))


def tensor(six):
    a, b, c, d, e, f = six
    return [a, d, f, d, b, e, f, e, c]


def scene():
    joints = {
        'slide': dict(parent='arm', child='carriage', type='prismatic',
                      xyz=[-.2, .6, -.1], rpy=[-.52, .43, -.24], axis=[-1., .2, .3],
                      lower_limit=-.2, upper_limit=.2),
        'turn': dict(parent='base_link', child='arm', type='revolute',
                     xyz=[.1, -.4, .7], rpy=[.32, -.21, .48], axis=[.2, 1., -.3],
                     lower_limit=-1., upper_limit=1., provenance={'raw': 'pivot'}),
    }
    inertial = {}
    for index, (name, (lo, hi)) in enumerate(BOUNDS.items()):
        r = rotation([.21+index*.1, -.43, .37])
        t = mm(mm(r, [.002, 0, 0, 0, .003, 0, 0, 0, .004]), transpose(r))
        inertial[name] = dict(mass=1.25+index, center_of_mass=[-.31+index*.12, .24-index*.1, -.17],
                              inertia=[t[i] for i in (0, 4, 8, 1, 5, 2)],
                              bbox_center_m=[(a+b)/2000 for a, b in zip(lo, hi)],
                              bbox_size_m=[(b-a)/1000 for a, b in zip(lo, hi)], source_path=name)
    return joints, inertial


def write_scene_meshes(targets):
    for name, target in targets.items():
        write_tetrahedron(target, BOUNDS[name])


def source_world(v, link, sample, direction=False):
    joints, _ = scene()
    turn, slide = joints['turn'], joints['slide']
    first_r = rotation(turn['rpy'])
    slide_axis = unit(mv(mm(first_r, rotation(slide['rpy'])), slide['axis']))
    if link == 'carriage' and not direction:
        v = add(v, scale(slide_axis, sample['slide']))
    if link != 'base_link':
        pivot = turn['xyz']
        v = screw(v if direction else add(v, scale(pivot, -1)),
                  unit(mv(first_r, turn['axis'])), sample['turn'])
        if not direction:
            v = add(v, pivot)
    return v


def vector(node, key):
    return [float(v) for v in node.get(key).split()]


def xml_world(robot, link, v, sample, direction=False):
    incoming = {j.find('child').get('link'): j for j in robot.findall('joint')}
    while link in incoming:
        joint = incoming[link]
        kind, q = joint.get('type'), sample.get(joint.get('name'), 0.)
        if kind != 'fixed':
            axis = unit(vector(joint.find('axis'), 'xyz'))
            if kind == 'prismatic':
                if not direction:
                    v = add(v, scale(axis, q))
            else:
                v = screw(v, axis, q)
        origin = joint.find('origin')
        v = euler(v, vector(origin, 'rpy'))
        if not direction:
            v = add(v, vector(origin, 'xyz'))
        link = joint.find('parent').get('link')
    return v


def xml_pose_world(robot, link, origin, v, sample):
    return xml_world(robot, link, add(vector(origin, 'xyz'), euler(v, vector(origin, 'rpy'))), sample)


def stl_points(path):
    data = Path(path).read_bytes()
    return sorted({tuple(struct.unpack_from('<3f', data, 84+50*i+12+12*j))
                   for i in range(struct.unpack_from('<I', data, 80)[0]) for j in range(3)})


def pose_matrix(robot, link, sample, origin=None):
    point = (lambda v: xml_pose_world(robot, link, origin, v, sample)) if origin is not None else (
        lambda v: xml_world(robot, link, v, sample))
    t = point([0, 0, 0])
    columns = [add(point(v), scale(t, -1)) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return [[columns[j][i] for j in range(3)]+[t[i]] for i in range(3)]+[[0, 0, 0, 1]]


def expected_link_pose(link, sample, g, dh):
    joints, _ = scene()
    frames = {'base_link': ([0, 0, 0], IDENTITY)}
    r1 = rotation(joints['turn']['rpy'])
    r2 = mm(r1, rotation(joints['slide']['rpy']))
    p, q = joints['turn']['xyz'], joints['slide']['xyz']
    z, w = unit(mv(r1, joints['turn']['axis'])), unit(mv(r2, joints['slide']['axis']))
    if dh:
        difference = add(q, scale(p, -1))
        dot = lambda a, b: sum(x*y for x, y in zip(a, b))
        b = dot(z, w)
        s = (dot(difference, z)-b*dot(difference, w))/(1-b*b)
        t = (b*dot(difference, z)-dot(difference, w))/(1-b*b)
        first = add(p, scale(z, s))
        x = unit(add(add(q, scale(w, t)), scale(first, -1)))
        terminal_x = unit(add([1, 0, 0], scale(w, -w[0])))
        cross = lambda a, b: [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
        basis = lambda x, z: [a[i] for i in range(3) for a in (x, cross(z, x), z)]
        frames['arm'] = first, basis(x, z)
        frames['carriage'] = q, basis(terminal_x, w)
    else:
        frames['arm'] = p, r1
        frames['carriage'] = q, r2
    if link not in frames:
        return [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    point, r = frames[link]
    t = mv(g, source_world(point, link, sample))
    columns = [mv(g, source_world([r[i] for i in (j, j+3, j+6)], link, sample, True)) for j in range(3)]
    return [[columns[j][i] for j in range(3)]+[t[i]] for i in range(3)]+[[0, 0, 0, 1]]


def retained_fixture(output, name, options):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    package = write_ros2_package(output / (name+'_description'), name, *scene(),
                                 write_meshes=write_scene_meshes, config=options)
    robot = ET.parse(package / 'urdf' / (name+'.urdf')).getroot()
    receipt = json.loads((package / 'config/robot_data.yaml').read_text())
    g = options.get('output_rotation', Y_UP if options.get('output_rotation_preset') else IDENTITY)
    cases = {}
    for label, sample in zip(('zero', 'moved', 'negative'), SAMPLES):
        bounds, collision_bounds, visual_poses = {}, {}, {}
        for link, (lo, hi) in BOUNDS.items():
            mesh = [lo, (hi[0], lo[1], lo[2]), (lo[0], hi[1], lo[2]), (lo[0], lo[1], hi[2])]
            points = [mv(g, source_world(scale(v, .001), link, sample)) for v in mesh]
            bounds[link] = [[f(p[i] for p in points) for i in range(3)] for f in (min, max)]
            corners = [mv(g, source_world(scale(v, .001), link, sample))
                       for v in itertools.product(*zip(lo, hi))]
            collision_bounds[link] = ([[f(p[i] for p in corners) for i in range(3)] for f in (min, max)]
                                      if options.get('collision_method') == 'primitive_box' else bounds[link])
            t = mv(g, source_world([0, 0, 0], link, sample))
            columns = [mv(g, source_world(v, link, sample, True)) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
            visual_poses[link] = [[columns[j][i] for j in range(3)]+[t[i]] for i in range(3)]+[[0, 0, 0, 1]]
        cases[label] = dict(positions=sample, link_poses={link.get('name'): expected_link_pose(
                               link.get('name'), sample, g, options.get('frame_convention') is not None)
                           for link in robot.findall('link')}, visual_poses=visual_poses, bounds=bounds,
                           visual_bounds=bounds, collision_bounds=collision_bounds,
                           link_kinds={n: v['kind'] for n, v in receipt['link_sources'].items()})
    expected = output / (name+'.expected.json')
    expected.write_text(json.dumps(cases, indent=2)+'\n')
    return package, expected


class PublicExportPoliciesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def export(self, options=None, model=None, callback=write_scene_meshes, **kwargs):
        return write_ros2_package(self.root/'policy_description', 'policy', *(model or scene()),
                                  write_meshes=callback, config=options, **kwargs)

    def assertVector(self, actual, expected, tolerance=5e-10):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def test_public_xml_geometry_com_tensor_and_boxes_at_three_motion_samples(self):
        configurations = [({'output_rotation_preset': 'y_up_to_z_up'}, Y_UP),
                          ({'output_rotation': rotation([-.37, .28, -.19])}, rotation([-.37, .28, -.19])),
                          ({'frame_convention': 'dh_common_normal'}, IDENTITY),
                          ({'output_rotation_preset': 'y_up_to_z_up', 'frame_convention': 'dh_common_normal'}, Y_UP)]
        for index, (options, g) in enumerate(configurations):
            with self.subTest(options=options):
                self.root = Path(self.tmp.name)/str(index)
                options = dict(options, collision_method='primitive_box')
                joints, inertial = scene()
                before = copy.deepcopy((joints, inertial, options))
                package = self.export(options, (joints, inertial))
                self.assertEqual((joints, inertial, options), before)
                robot = ET.parse(package/'urdf/policy.urdf').getroot()
                for sample in SAMPLES:
                    for name, entry in inertial.items():
                        for actual, expected in zip(pose_matrix(robot, name, sample), expected_link_pose(
                                name, sample, g, options.get('frame_convention') is not None)):
                            self.assertVector(actual, expected)
                        link = robot.find("link[@name='%s']" % name)
                        visual = link.find('visual')
                        self.assertVector(vector(visual.find('geometry/mesh'), 'scale'), [.001]*3)
                        self.assertEqual((package/'meshes'/ (name+'.stl')).read_bytes(),
                                         self._source_mesh(name))
                        for point in stl_points(package/'meshes'/(name+'.stl')):
                            actual = xml_pose_world(robot, name, visual.find('origin'), scale(point, .001), sample)
                            self.assertVector(actual, mv(g, source_world(scale(point, .001), name, sample)))
                        physics = link.find('inertial')
                        self.assertEqual(float(physics.find('mass').get('value')), entry['mass'])
                        self.assertVector(xml_world(robot, name, vector(physics.find('origin'), 'xyz'), sample),
                                          mv(g, source_world(entry['center_of_mass'], name, sample)))
                        local = tensor([float(physics.find('inertia').get(k))
                                        for k in ('ixx', 'iyy', 'izz', 'ixy', 'iyz', 'ixz')])
                        r = [xml_world(robot, name, a, sample, True) for a in ((1,0,0),(0,1,0),(0,0,1))]
                        r = [r[j][i] for i in range(3) for j in range(3)]
                        source_r = [mv(g, source_world(a, name, sample, True)) for a in ((1,0,0),(0,1,0),(0,0,1))]
                        source_r = [source_r[j][i] for i in range(3) for j in range(3)]
                        self.assertVector(mm(mm(r, local), transpose(r)),
                                          mm(mm(source_r, tensor(entry['inertia'])), transpose(source_r)))
                        collision = link.find('collision')
                        size = vector(collision.find('geometry/box'), 'size')
                        self.assertVector(size, entry['bbox_size_m'], 1e-14)
                        for signs in itertools.product((-1, 1), repeat=3):
                            corner = [s*d/2 for s, d in zip(signs, size)]
                            source = add(entry['bbox_center_m'], corner)
                            self.assertVector(xml_pose_world(robot, name, collision.find('origin'), corner, sample),
                                              mv(g, source_world(source, name, sample)))

    def _source_mesh(self, name):
        path = Path(self.tmp.name)/(name+'.source.stl')
        write_tetrahedron(path, BOUNDS[name])
        return path.read_bytes()

    def test_y_up_is_upright_in_real_output_reference_and_receipts_are_coherent(self):
        package = self.export({'output_rotation_preset': 'y_up_to_z_up'})
        robot = ET.parse(package/'urdf/policy.urdf').getroot()
        receipt = json.loads((package/'config/robot_data.yaml').read_text())
        root = receipt['root_frame']
        self.assertEqual(root['source_root_link'], 'base_link')
        reference = robot.find("link[@name='%s']" % root['output_reference_link'])
        self.assertIsNotNone(reference)
        self.assertEqual(list(reference), [])
        self.assertEqual(receipt['link_sources'][reference.get('name')]['kind'], 'frame')
        self.assertIsNone(receipt['link_sources'][reference.get('name')]['path'])
        self.assertFalse((package/'meshes'/(reference.get('name')+'.stl')).exists())
        self.assertIn('Fixed Frame: '+reference.get('name'), (package/'viewers/display.rviz').read_text())
        self.assertVector(xml_world(robot, 'base_link', [0, 1, 0], {}), [0, 0, 1])
        self.assertVector(vector(robot.find("link[@name='base_link']/visual/origin"), 'rpy'), [0, 0, 0])
        self.assertTrue(receipt['export_policies']['receipts']['coordinate_rotation']['enabled'])
        self.assertEqual(set(receipt['links']), {e.get('name') for e in robot.findall('link')})
        self.assertEqual(set(receipt['joints']), {e.get('name') for e in robot.findall('joint')})
        for name, relative in receipt['component_urdfs'].items():
            component = ET.parse(package/relative).getroot()
            self.assertEqual(component.find('link').get('name'), name)
            self.assertEqual(len(component.findall('link')), 1)
            for mesh in component.findall('.//mesh'):
                self.assertTrue((package/mesh.get('filename').split('policy_description/')[1]).is_file())

    def test_disabled_defaults_match_exact_seed_xml_and_sidecar(self):
        for index, options in enumerate((None, {'output_rotation': None, 'output_rotation_preset': None,
                                              'frame_convention': None, 'minimum_principal_inertia': None})):
            package = write_ros2_package(Path(self.tmp.name)/str(index)/'seed_description', 'bot', *legacy_model(),
                                         write_meshes=legacy_meshes, config=options)
            for path, expected in [('urdf/bot.urdf', '7ded7372be2cd1a80322abc19dc4ba88cebe99886ccf9bcf9bbf26d190042ef1'),
                                   ('config/robot_data.yaml', '0168f271031cb63f1cd1d3694ead555d5c9adc92f5c614ca341f0b700830b8d1')]:
                self.assertEqual(hashlib.sha256((package/path).read_bytes()).hexdigest(), expected)

    def test_disabled_path_accepts_unused_unpaired_boxes_large_and_tiny_inertia(self):
        joints, inertial = legacy_model()
        inertial['base_link']['bbox_center_m'] = [1, 2, 3]
        inertial['tool']['bbox_size_m'] = [.1, .2, .3]
        inertial['base_link']['inertia'] = [2e-30, 3e-30, 4e-30, 0, 0, 0]
        inertial['tool']['inertia'] = [1.2e308, 1.2e308, 1.2e308, 3e307, 3e307, 3e307]
        before = copy.deepcopy((joints, inertial))
        package = self.export(model=(joints, inertial), callback=legacy_meshes)
        self.assertEqual((joints, inertial), before)
        robot = ET.parse(package/'urdf/policy.urdf').getroot()
        self.assertEqual(float(robot.find("link[@name='base_link']/inertial/inertia").get('ixx')), 2e-30)

    def test_zero_floor_is_identity_for_legacy_unpaired_boxes_and_large_tensor(self):
        joints, inertial = legacy_model()
        inertial['base_link']['bbox_center_m'] = [1, 2, 3]
        inertial['tool']['bbox_size_m'] = [.1, .2, .3]
        inertial['tool']['inertia'] = [1.2e308, 1.2e308, 1.2e308, 3e307, 3e307, 3e307]
        before = copy.deepcopy((joints, inertial))
        first = write_ros2_package(self.root/'plain'/'same_description', 'same', joints, inertial,
                                   write_meshes=legacy_meshes)
        second = write_ros2_package(self.root/'zero'/'same_description', 'same', joints, inertial,
                                    write_meshes=legacy_meshes, config={'minimum_principal_inertia': 0})
        self.assertEqual((first/'urdf/same.urdf').read_bytes(), (second/'urdf/same.urdf').read_bytes())
        receipt = json.loads((second/'config/robot_data.yaml').read_text())['export_policies']['receipts']['inertia']['tool']
        self.assertTrue(receipt['enabled'])
        self.assertFalse(receipt['changed'])
        self.assertEqual(receipt['original'], inertial['tool']['inertia'])
        self.assertEqual(receipt['final'], inertial['tool']['inertia'])
        self.assertEqual(receipt['principal_moment_scale_kg_m2'], 1.2e308)
        self.assertVector(sorted(receipt['principal_moments_before']), [.75, .75, 1.5])
        self.assertEqual((joints, inertial), before)

    def test_config_normalizes_toml_and_overrides_without_mutation(self):
        options = {'output_rotation_preset': 'y_up_to_z_up', 'minimum_principal_inertia': 0,
                   'frame_convention': 'dh_common_normal'}
        before = copy.deepcopy((options, config.DEFAULTS))
        (self.root/'xacro_export.toml').write_text('output_rotation_preset = "y_up_to_z_up"\n'
            'frame_convention = "dh_common_normal"\nminimum_principal_inertia = 0.0\n')
        self.assertEqual(config.load_config(self.root), config.load_config(overrides=options))
        self.assertEqual((options, config.DEFAULTS), before)
        self.assertVector(config.load_config(overrides={'output_rotation': tuple(Y_UP)})['output_rotation'], Y_UP)
        normalized = config.load_config(overrides=options)
        self.assertEqual(config.load_config(overrides=normalized), normalized)
        with self.assertRaisesRegex(ValueError, 'mutually exclusive'):
            config.load_config(self.root, overrides={'output_rotation': IDENTITY})
        self.assertEqual(config.load_config(self.root, overrides={'output_rotation_preset': None,
                                                                 'output_rotation': IDENTITY})['output_rotation'], IDENTITY)
        for text in ('minimum_principal_inertia = true', 'output_rotation_preset = "auto"',
                     'output_rotation = [1, 0, 0]', 'frame_convention = "dh"',
                     'minimum_principal_inertia = nan'):
            (self.root/'xacro_export.toml').write_text(text)
            with self.subTest(toml=text), self.assertRaises(ValueError):
                config.load_config(self.root)
            with self.subTest(overridden_invalid_toml=text), self.assertRaises(ValueError):
                config.load_config(self.root, overrides=options)
        for key in ('output_rotation', 'output_rotation_preset', 'frame_convention', 'minimum_principal_inertia'):
            self.assertNotIn(key, config.load_config())

    def test_invalid_policies_reject_before_callback_and_publication(self):
        invalid = [{'output_rotation_preset': v} for v in ('auto', '', False, 1)]
        invalid += [{'frame_convention': v} for v in ('dh', 'standard_dh', '', False)]
        invalid += [{'minimum_principal_inertia': v} for v in (-1, True, '0', float('inf'), float('nan'))]
        invalid += [{'output_rotation': v} for v in ([1]*9, [1]*8, 'y_up', [True]+IDENTITY[1:],
                                                     [-1,0,0,0,1,0,0,0,1])]
        invalid += [{'output_rotation': IDENTITY, 'output_rotation_preset': 'y_up_to_z_up'}]
        for options in invalid:
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.export(options, callback=lambda _: self.fail('invalid policy reached callback'))
            self.assertFalse((self.root/'policy_description').exists())
            self.assertEqual(list(self.root.iterdir()), [])

    def test_floor_changes_only_inertia_and_persists_source_physics_and_warning(self):
        joints, inertial = scene()
        package = self.export({'minimum_principal_inertia': .0035})
        robot = ET.parse(package/'urdf/policy.urdf').getroot()
        receipt = json.loads((package/'config/robot_data.yaml').read_text())
        for name, entry in inertial.items():
            floor = receipt['export_policies']['receipts']['inertia'][name]
            self.assertEqual(floor['original'], entry['inertia'])
            self.assertTrue(floor['enabled'] and floor['changed'])
            self.assertEqual(floor['threshold_kg_m2'], .0035)
            self.assertVector(sorted(floor['principal_moments_before']), [.002, .003, .004])
            self.assertVector(sorted(floor['principal_moments_after']), [.0035, .0035, .004])
            self.assertIn('stability is not established', floor['warning'])
            link = robot.find("link[@name='%s']" % name)
            self.assertEqual(float(link.find('inertial/mass').get('value')), entry['mass'])
            self.assertEqual((package/'meshes'/(name+'.stl')).read_bytes(), self._source_mesh(name))
            index = list(inertial).index(name)
            r = rotation([.21+index*.1, -.43, .37])
            expected = mm(mm(r, [.0035, 0, 0, 0, .0035, 0, 0, 0, .004]), transpose(r))
            local = tensor([float(link.find('inertial/inertia').get(k))
                            for k in ('ixx', 'iyy', 'izz', 'ixy', 'iyz', 'ixz')])
            basis = pose_matrix(robot, name, SAMPLES[0])
            basis = [basis[i][j] for i in range(3) for j in range(3)]
            self.assertVector(mm(mm(basis, local), transpose(basis)), expected)
        self.assertEqual(receipt['joints'], dict((n, dict(j, effort=100., velocity=100.)) for n, j in joints.items()))
        self.assertTrue(any('Physical inertia properties changed' in warning for warning in receipt['warnings']))

    def test_invalid_source_inertia_cannot_be_healed_by_floor(self):
        joints, inertial = scene()
        inertial['arm']['inertia'] = [1, 1, 3, 0, 0, 0]
        with self.assertRaisesRegex(ValueError, 'triangle'):
            self.export({'minimum_principal_inertia': 100}, (joints, inertial),
                        callback=lambda _: self.fail('invalid physics reached callback'))
        self.assertFalse((self.root/'policy_description').exists())

    def test_closure_global_rotation_preserves_full_original_and_transforms_known_fields(self):
        joints, inertial = scene()
        joints['closure'] = dict(joints['turn'], parent='carriage', child='base_link',
                                source='original CAD', custom={'constraint': [1, 2, 3]})
        original = copy.deepcopy(joints['closure'])
        package = self.export({'output_rotation_preset': 'y_up_to_z_up', 'minimum_principal_inertia': .001},
                              (joints, inertial))
        receipt = json.loads((package/'config/robot_data.yaml').read_text())
        closure = receipt['closing_joints'][0]
        self.assertVector(closure['xyz'], mv(Y_UP, original['xyz']))
        self.assertEqual(closure['rpy'], original['rpy'])
        self.assertEqual(closure['axis'], original['axis'])
        self.assertEqual(closure['custom'], original['custom'])
        source = receipt['export_policies']['source_closing_joints']['closure']
        self.assertEqual(source, dict(original, effort=100., velocity=100.))
        self.assertEqual(joints['closure'], original)

    def test_dh_unsupported_topologies_reject_explicitly_before_callback(self):
        cases = []
        joints, inertial = scene(); joints['slide']['type'] = 'fixed'; cases.append((joints, inertial, 'serial'))
        joints, inertial = scene(); joints['slide']['parent'] = 'base_link'; cases.append((joints, inertial, 'branch'))
        joints, inertial = scene(); joints['closure'] = dict(joints['turn'], parent='carriage', child='base_link')
        cases.append((joints, inertial, 'clos'))
        joints, inertial = scene()
        joints['turn'].update(rpy=[0, 0, 0], axis=[0, 0, 1])
        joints['slide'].update(rpy=[0, 0, 0], axis=[0, .5e-8, 1])
        cases.append((joints, inertial, 'near-parallel'))
        joints, inertial = scene(); joints['slide']['type'] = 'spherical'
        cases.append((joints, inertial, 'unsupported'))
        for joints, inertial, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                self.export({'frame_convention': 'dh_common_normal'}, (joints, inertial),
                            callback=lambda _: self.fail('unsupported DH reached callback'))
            self.assertFalse((self.root/'policy_description').exists())

    def test_failure_and_cancellation_never_publish_enabled_policy(self):
        def failure(_):
            raise RuntimeError('mesh failure')
        for callback, cancelled, error in ((failure, None, RuntimeError),
                                           (write_scene_meshes, lambda: True, CancelledError)):
            with self.assertRaises(error):
                self.export({'output_rotation_preset': 'y_up_to_z_up'}, callback=callback, cancelled=cancelled)
            self.assertFalse((self.root/'policy_description').exists())
            self.assertEqual(list(self.root.iterdir()), [])
        completed = []
        def callback(targets):
            write_scene_meshes(targets)
            completed.append(True)
        with self.assertRaises(CancelledError):
            self.export({'output_rotation_preset': 'y_up_to_z_up'}, callback=callback,
                        cancelled=lambda: bool(completed))
        self.assertEqual(completed, [True])
        self.assertEqual(list(self.root.iterdir()), [])

    def test_enabled_policies_preserve_existing_frame_only_source_links(self):
        joints, inertial = scene()
        inertial['arm'] = {'kind': 'frame', 'source_path': 'empty CAD context'}
        seen = []
        def callback(targets):
            seen.append(set(targets))
            write_scene_meshes(targets)
        package = self.export({'output_rotation_preset': 'y_up_to_z_up'}, (joints, inertial), callback=callback)
        robot = ET.parse(package/'urdf/policy.urdf').getroot()
        self.assertEqual(list(robot.find("link[@name='arm']")), [])
        self.assertEqual(seen, [{'base_link', 'carriage'}])
        receipt = json.loads((package/'config/robot_data.yaml').read_text())
        self.assertEqual(receipt['link_sources']['arm'], {'kind': 'frame', 'path': 'empty CAD context'})
        self.assertEqual(receipt['root_frame']['source_root_link'], 'base_link')

    def test_reserved_reference_names_and_source_identifiers_survive(self):
        class SourceJoints(dict):
            pass
        joints, inertial = scene()
        joints = SourceJoints(joints)
        joints.source_identifiers = {'raw/cad': {'exported': 'base_link', 'kind': 'physical'}}
        identifiers = copy.deepcopy(joints.source_identifiers)
        inertial['export_output_reference'] = inertial.pop('carriage')
        joints['slide']['child'] = 'export_output_reference'
        package = self.export({'output_rotation_preset': 'y_up_to_z_up'}, (joints, inertial), callback=legacy_meshes)
        receipt = json.loads((package/'config/robot_data.yaml').read_text())
        self.assertEqual(receipt['root_frame']['output_reference_link'], 'export_output_reference_2')
        self.assertEqual(receipt['source_identifiers'], identifiers)
        self.assertEqual(joints.source_identifiers, identifiers)
        self.assertEqual(receipt['link_sources']['export_output_reference']['kind'], 'physical')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixtures', type=Path)
    args = parser.parse_args()
    if args.fixtures:
        for name, options in [('policy_rotation', {'output_rotation_preset': 'y_up_to_z_up'}),
                              ('policy_boxes_dh', {'output_rotation_preset': 'y_up_to_z_up',
                                                   'frame_convention': 'dh_common_normal',
                                                   'collision_method': 'primitive_box'}),
                              ('policy_floor', {'minimum_principal_inertia': .0035})]:
            print(*retained_fixture(args.fixtures, name, options))
    else:
        unittest.main(argv=[__file__])
