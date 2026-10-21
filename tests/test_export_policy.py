"""Independent analytic checks of the opt-in policy boundary, not Fusion exports."""

from copy import deepcopy
import itertools
import math
import struct
import unittest

from URDF_Exporter.utils.export_policy import apply_export_policies, regularize_inertia, Y_UP_TO_Z_UP


I = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
SAMPLES = ({}, {'j1': .43, 'j2': -.07, 'j3': .18, 'j4': -.32, 'j5': .24},
           {'j1': -.27, 'j2': .12, 'j3': -.16, 'j4': .21, 'j5': -.13})


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def scale(a, n):
    return [x*n for x in a]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def unit(a):
    return scale(a, 1/math.hypot(*a))


def apply(matrix, point):
    return [sum(matrix[i*3+j]*point[j] for j in range(3)) for i in range(3)]


def transpose(matrix):
    return [matrix[j*3+i] for i in range(3) for j in range(3)]


def multiply(a, b):
    return [sum(a[3*i+k]*b[3*k+j] for k in range(3)) for i in range(3) for j in range(3)]


def euler(point, angles):
    x, y, z = point
    r, p, w = angles
    y, z = math.cos(r)*y-math.sin(r)*z, math.sin(r)*y+math.cos(r)*z
    x, z = math.cos(p)*x+math.sin(p)*z, -math.sin(p)*x+math.cos(p)*z
    return [math.cos(w)*x-math.sin(w)*y, math.sin(w)*x+math.cos(w)*y, z]


def rotation(angles):
    columns = [euler(axis, angles) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return [columns[j][i] for i in range(3) for j in range(3)]


def rodrigues(point, axis, angle):
    return add(add(scale(point, math.cos(angle)), scale(cross(axis, point), math.sin(angle))),
               scale(axis, dot(axis, point)*(1-math.cos(angle))))


def six(matrix):
    return [matrix[i] for i in (0, 4, 8, 1, 5, 2)]


def tensor(values):
    a, b, c, d, e, f = values
    return [a, d, f, d, b, e, f, e, c]


def anisotropic(principal, angles):
    axes = [euler(axis, angles) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return [sum(principal[k]*axes[k][i]*axes[k][j] for k in range(3))
            for i in range(3) for j in range(3)]


def body(index):
    return {'mass': 1.25 + index, 'center_of_mass': [-.31+index*.12, .24-index*.1, -.17],
            'inertia': six(anisotropic([.002, .003, .004], [.21+index*.1, -.43, .37])),
            'bbox_center_m': [.11, -.23+index*.12, .31], 'bbox_size_m': [.12, .34, .56],
            'source_notes': {'unchanged': [index]}}


def moving_scene():
    root = {'t': [.4, -.3, .2], 'R': rotation([.23, -.31, .42])}
    j1rpy, j2rpy = [.32, -.21, .48], [-.52, .43, -.24]
    r1 = multiply(root['R'], rotation(j1rpy))
    r2 = multiply(r1, rotation(j2rpy))
    joints = {
        'j2': {'parent': 'arm', 'child': 'slide', 'type': 'prismatic',
               'xyz': [-.2, .6, -.1], 'rpy': j2rpy, 'axis': [-1., .2, .3]},
        'j1': {'parent': 'base_link', 'child': 'arm', 'type': 'revolute',
               'xyz': [.1, -.4, .7], 'rpy': j1rpy, 'axis': [.2, 1., -.3]},
    }
    inertial = {name: body(i) for i, name in enumerate(('base_link', 'arm', 'slide'))}
    screws = [('j1', 'arm', [joints['j1']['xyz'], unit(apply(r1, joints['j1']['axis']))], 'revolute'),
              ('j2', 'slide', [joints['j2']['xyz'], unit(apply(r2, joints['j2']['axis']))], 'prismatic')]
    return joints, inertial, root, screws


def common_normal_scene():
    w = unit([0., 1., 1.])
    definitions = [([0., 0., 2.], [0., 0., 1.]), ([2., 3., 4.], w),
                   ([4., 5., 8.], w), (add([4., 5., 8.], scale(w, 2)), scale(w, -1)),
                   ([4., -1.5, 1.5], [0., 0., 1.])]
    joints, inertial, screws = {}, {'base_link': body(0)}, []
    parent = 'base_link'
    for i, (point, axis) in enumerate(definitions, 1):
        name, child = 'j%d' % i, 'link%d' % i
        joints[name] = {'parent': parent, 'child': child, 'type': 'revolute',
                        'xyz': point, 'rpy': [0., 0., 0.], 'axis': axis}
        inertial[child] = body(i)
        screws.append((name, child, [point, axis], 'revolute'))
        parent = child
    return joints, inertial, {'t': [0., 0., 0.], 'R': I}, screws


def analytic_world(point, link, sample, screws, direction=False):
    ancestors = []
    for screw in screws:
        ancestors.append(screw)
        if screw[1] == link:
            break
    if link == 'base_link':
        ancestors = []
    for name, _, (pivot, axis), kind in reversed(ancestors):
        q = sample.get(name, 0.)
        if kind == 'prismatic':
            if not direction:
                point = add(point, scale(axis, q))
        else:
            point = rodrigues(point if direction else sub(point, pivot), axis, q)
            if not direction:
                point = add(pivot, point)
    return point


def emitted_world(result, link, point, sample, direction=False):
    incoming = {j['child']: (name, j) for name, j in result['joints'].items()}
    while link in incoming:
        name, joint = incoming[link]
        origin = result['emission']['joint_origins'][name]
        q, axis = sample.get(name, 0.), unit(joint.get('axis', [0., 0., 1.]))
        if joint['type'] == 'prismatic':
            if not direction:
                point = add(point, scale(axis, q))
        elif joint['type'] != 'fixed':
            point = rodrigues(point, axis, q)
        point = apply(origin['R'], point)
        if not direction:
            point = add(origin['t'], point)
        link = joint['parent']
    root = result['emission']['root_pose']
    point = apply(root['R'], point)
    return point if direction else add(root['t'], point)


def local_pose(pose, point):
    return add(pose['t'], apply(pose['R'], point))


def corners(entry):
    return [add(entry['bbox_center_m'], [s*d/2 for s, d in zip(signs, entry['bbox_size_m'])])
            for signs in itertools.product((-1, 1), repeat=3)]


class ExportPolicyTests(unittest.TestCase):
    def assertVector(self, actual, expected, tolerance=3e-11):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def test_disabled_keeps_tiny_inertia_exact(self):
        tensor = [2e-30, 3e-30, 4e-30, 0, 0, 0]
        self.assertEqual(regularize_inertia(tensor)['final'], tensor)

    def test_default_off_is_owned_exact_and_source_unchanged(self):
        joints, inertial, root, _ = moving_scene()
        inertial['arm']['inertia'] = [2e-30, 3e-30, 4e-30, 0, 0, 0]
        before = deepcopy((joints, inertial, root))
        result = apply_export_policies(joints, inertial, source_root_frame=root)
        self.assertEqual(result['joints'], joints)
        self.assertEqual(result['inertial'], inertial)
        self.assertEqual((joints, inertial, root), before)
        self.assertFalse(result['receipts']['coordinate_rotation']['enabled'])
        self.assertIsNone(result['receipts']['frame_convention']['selected'])
        self.assertEqual(result['warnings'], [])
        result['joints']['j1']['xyz'][0] = 123
        result['inertial']['arm']['source_notes']['unchanged'].append(999)
        result['emission']['root_pose']['t'][0] = 321
        self.assertEqual((joints, inertial, root), before)

    def check_scene(self, factory, rotations, conventions):
        joints, inertial, root, screws = factory()
        before = deepcopy((joints, inertial, root))
        mesh_points = [[-.12, .37, -.26], [.84, -.49, .13], [.23, -.11, .58], [-.41, -.32, -.17]]
        raw_mesh_points = struct.pack('<12d', *(v*1000 for point in mesh_points for v in point))
        for global_rotation, convention in itertools.product(rotations, conventions):
            g = global_rotation if global_rotation is not None else I
            result = apply_export_policies(joints, inertial, source_root_frame=root,
                                           output_rotation=global_rotation, frame_convention=convention)
            self.assertEqual((joints, inertial, root), before)
            self.assertEqual(raw_mesh_points, struct.pack('<12d', *(v*1000 for point in mesh_points for v in point)))
            for name, joint in result['joints'].items():
                self.assertVector(rotation(joint['rpy']), result['emission']['joint_origins'][name]['R'])
                if convention is None:
                    self.assertEqual(joint['axis'], joints[name]['axis'])
            for sample, link in itertools.product(SAMPLES, inertial):
                with self.subTest(scene=factory.__name__, rotation=g, convention=convention, sample=sample, link=link):
                    mesh = result['emission']['mesh_origins'][link]
                    for millimeters, source_point in zip(struct.iter_unpack('<3d', raw_mesh_points), mesh_points):
                        scaled = [v*s for v, s in zip(millimeters, result['emission']['mesh_scale'])]
                        local = local_pose(mesh, scaled)
                        actual = emitted_world(result, link, local, sample)
                        expected = apply(g, analytic_world(source_point, link, sample, screws))
                        self.assertVector(actual, expected)
                    box = result['emission']['boxes'][link]
                    self.assertEqual(box['size'], inertial[link]['bbox_size_m'])
                    local_corners = [[s*d/2 for s, d in zip(signs, box['size'])]
                                     for signs in itertools.product((-1, 1), repeat=3)]
                    for source_point, corner in zip(corners(inertial[link]), local_corners):
                        actual = emitted_world(result, link, local_pose(box, corner), sample)
                        expected = apply(g, analytic_world(source_point, link, sample, screws))
                        self.assertVector(actual, expected)
                    physics = result['emission']['inertials'][link]
                    self.assertEqual(physics['mass'], inertial[link]['mass'])
                    actual_com = emitted_world(result, link, physics['center_of_mass'], sample)
                    expected_com = apply(g, analytic_world(inertial[link]['center_of_mass'], link, sample, screws))
                    self.assertVector(actual_com, expected_com)
                    actual_columns = [emitted_world(result, link, axis, sample, True)
                                      for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
                    actual_basis = [actual_columns[j][i] for i in range(3) for j in range(3)]
                    actual_tensor = multiply(multiply(actual_basis, tensor(physics['inertia'])), transpose(actual_basis))
                    expected_columns = [apply(g, analytic_world(axis, link, sample, screws, True))
                                        for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
                    basis = [expected_columns[j][i] for i in range(3) for j in range(3)]
                    expected_tensor = multiply(multiply(basis, tensor(inertial[link]['inertia'])), transpose(basis))
                    self.assertVector(actual_tensor, expected_tensor, 3e-13)

    def test_global_rotation_mesh_box_com_tensor_and_nonzero_fk(self):
        self.assertVector(apply(Y_UP_TO_Z_UP, [0, 1, 0]), [0, 0, 1])
        self.assertVector(apply(Y_UP_TO_Z_UP, [0, 0, 1]), [0, -1, 0])
        self.check_scene(moving_scene, [None, Y_UP_TO_Z_UP, rotation([.57, -.46, .83])], [None, 'dh_common_normal'])

    def test_common_normal_feet_skew_parallel_coincident_intersecting(self):
        joints, inertial, root, _ = common_normal_scene()
        result = apply_export_policies(joints, inertial, frame_convention='dh_common_normal')
        expected = {
            'j1': ('skew', [0., 0., 1.], [2., 0., 1.]),
            'j2': ('parallel', [2., -.5, .5], [4., -1.5, 1.5]),
            'j3': ('coincident', [4., -1.5, 1.5], [4., -1.5, 1.5]),
            'j4': ('intersecting', [4., -1.5, 1.5], [4., -1.5, 1.5]),
            'j5': ('terminal_source_pivot', [4., -1.5, 1.5], None),
        }
        for name, (case, origin, other) in expected.items():
            receipt = result['receipts']['frame_convention']['joints'][name]
            self.assertEqual(receipt['rule'], case)
            self.assertVector(receipt['origin_on_current_axis'], origin)
            if other is None:
                self.assertIsNone(receipt['foot_on_next_axis'])
            else:
                self.assertVector(receipt['foot_on_next_axis'], other)
            self.assertVector(result['joints'][name]['axis'], [0, 0, 1])
        self.assertVector([result['link_frames']['link1']['R'][i] for i in (0, 3, 6)], [1, 0, 0])
        self.assertVector([result['link_frames']['link2']['R'][i] for i in (0, 3, 6)], unit([2, -1, 1]))
        self.check_scene(common_normal_scene, [None, rotation([.51, -.37, .64])], ['dh_common_normal'])

    def test_common_normal_reference_rotates_with_source_root(self):
        joints = {'j1': {'parent': 'base_link', 'child': 'a', 'type': 'revolute',
                          'xyz': [0, 0, 2], 'axis': [0, 0, 1]},
                  'j2': {'parent': 'a', 'child': 'b', 'type': 'prismatic',
                          'xyz': [0, 0, 7], 'axis': [0, 0, -1]}}
        inertia = {name: body(i) for i, name in enumerate(('base_link', 'a', 'b'))}
        result = apply_export_policies(joints, inertia, frame_convention='dh_common_normal', output_rotation=Y_UP_TO_Z_UP)
        self.assertEqual(result['receipts']['frame_convention']['joints']['j1']['rule'], 'coincident')
        self.assertVector(result['joints']['j1']['xyz'], [0, 0, 0])
        self.assertVector(result['link_frames']['a']['R'], list(Y_UP_TO_Z_UP))

    def test_terminal_reference_uses_root_y_when_root_x_is_axis(self):
        joints = {'j': {'parent': 'base_link', 'child': 'a', 'type': 'continuous',
                        'xyz': [1, 2, 3], 'axis': [1, 0, 0]}}
        inertial = {'base_link': body(0), 'a': body(1)}
        result = apply_export_policies(joints, inertial, frame_convention='dh_common_normal')
        frame = result['link_frames']['a']
        self.assertVector([frame['R'][i] for i in (0, 3, 6)], [0, 1, 0])
        self.assertVector([frame['R'][i] for i in (2, 5, 8)], [1, 0, 0])
        self.assertVector(frame['t'], [1, 2, 3])

    def test_regularization_rotated_anisotropic_basis_and_provenance(self):
        for factor in (1., 1e-24, 1e18):
            for angles in ([0., 0., 0.], [.23, -.51, .77], [1.1, .4, -.9]):
                original = six(anisotropic(scale([2., 3., 4.], factor), angles))
                before = deepcopy(original)
                receipt = regularize_inertia(original, 3.5*factor)
                expected = six(anisotropic(scale([3.5, 3.5, 4.], factor), angles))
                self.assertVector(scale(receipt['final'], 1/factor), scale(expected, 1/factor), 2e-13)
                self.assertEqual(original, before)
                self.assertEqual(receipt['original'], original)
                self.assertEqual(receipt['threshold_kg_m2'], 3.5*factor)
                self.assertTrue(receipt['changed'])
                self.assertIn('Physical inertia properties changed', receipt['warning'])
                self.assertIn('stability is not established', receipt['warning'])
                self.assertEqual(regularize_inertia(receipt['final'], 0)['warning'], None)

    def test_basis_equivariance_of_floor_and_global_rotation(self):
        original = six(anisotropic([2e-8, 3e-8, 4e-8], [.3, -.6, .9]))
        g = rotation([-.7, .2, .4])
        rotated = six(multiply(multiply(g, tensor(original)), transpose(g)))
        first = regularize_inertia(original, 3.5e-8)['final']
        second = regularize_inertia(rotated, 3.5e-8)['final']
        self.assertVector(second, six(multiply(multiply(g, tensor(first)), transpose(g))), 2e-21)
        joints, inertia, root, _ = moving_scene()
        inertia['arm']['inertia'] = original
        before = deepcopy(inertia)
        result = apply_export_policies(joints, inertia, output_rotation=g, minimum_principal_inertia=3.5e-8,
                                       frame_convention='dh_common_normal', source_root_frame=root)
        self.assertEqual(inertia, before)
        self.assertVector(result['inertial']['arm']['inertia'], second, 2e-21)
        self.assertEqual(result['inertial']['arm']['mass'], inertia['arm']['mass'])
        self.assertEqual(result['receipts']['inertia']['arm']['original'], original)

    def test_tiny_boundary_zero_and_default_off_physics(self):
        for principal in ([0., 0., 0.], [0., 1., 1.], [1., 2., 3.], [2., 2., 2.]):
            for factor in (1e-250, 1., 1e100):
                original = six(anisotropic(scale(principal, factor), [.31, -.27, .49]))
                receipt = regularize_inertia(original)
                self.assertEqual(receipt['final'], original)
                self.assertFalse(receipt['changed'])
                self.assertIsNone(receipt['warning'])
                final = regularize_inertia(original, 1e-4)['final']
                regularize_inertia(final)
        unchanged = regularize_inertia([1., 2., 3., 0., 0., 0.], 1.)
        self.assertFalse(unchanged['changed'])
        self.assertEqual(unchanged['original'], unchanged['final'])

    def test_invalid_physics_rejected_before_large_floor(self):
        bad = ([1., 1., 3., 0., 0., 0.], [1., 1., 1., 2., 0., 0.],
               [-1e-250, 1e-250, 1e-250, 0., 0., 0.],
               [1e-250, 1e-250, 3e-250, 0., 0., 0.],
               [1., 2., math.inf, 0., 0., 0.], [1., 2., math.nan, 0., 0., 0.], [1, 2])
        for original, threshold in itertools.product(bad, (None, 100.)):
            with self.subTest(original=original, threshold=threshold), self.assertRaises(ValueError):
                regularize_inertia(original, threshold)
        for threshold in (-1., math.inf, math.nan, True, '1e-4'):
            with self.subTest(threshold=threshold), self.assertRaises(ValueError):
                regularize_inertia([1., 2., 3., 0., 0., 0.], threshold)
        for principal in ([1., 1., 3.], [-1., 2., 2.]):
            for angles in ([.7, .8, .9], [.2, -.4, .6]):
                with self.subTest(principal=principal, angles=angles), self.assertRaises(ValueError):
                    regularize_inertia(six(anisotropic(principal, angles)), 100.)

    def test_invalid_shapes_rotation_topology_and_inputs_are_unchanged(self):
        joints, inertia, root, _ = moving_scene()
        mutations = [lambda j, b: j['j1'].update(xyz=[1, 2]),
                     lambda j, b: j['j1'].update(axis=[0, 0, 0]),
                     lambda j, b: j['j1'].update(rpy=[0, math.nan, 0]),
                     lambda j, b: j['j1'].update(parent='missing'),
                     lambda j, b: j['j1'].update(child='base_link'),
                     lambda j, b: j['j1'].update(parent='slide'),
                     lambda j, b: j['j2'].update(child='arm'),
                     lambda j, b: b.update(isolated=body(9)),
                     lambda j, b: b['arm'].update(mass=0),
                     lambda j, b: b['arm'].update(center_of_mass=[0, 0, math.inf]),
                     lambda j, b: b['arm'].update(bbox_size_m=[1, -1, 1]),
                     lambda j, b: b['arm'].pop('bbox_center_m')]
        for mutate in mutations:
            j, b = deepcopy(joints), deepcopy(inertia)
            mutate(j, b)
            before = repr((j, b))
            with self.assertRaises(ValueError):
                apply_export_policies(j, b, source_root_frame=root)
            self.assertEqual(repr((j, b)), before)
        for matrix in ([1]*9, [1, 0, 0, 0, 1, 0, 0, 0, -1], [0]*8, [math.nan]*9):
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                apply_export_policies(joints, inertia, output_rotation=matrix)
        with self.assertRaises(ValueError):
            apply_export_policies(joints, inertia, source_root_frame={'t': [0, 0, 0], 'R': [1]*9})

    def test_explicit_dh_rejections_do_not_weaken_default_tree_support(self):
        joints, inertia, _, _ = moving_scene()
        joints['j2']['parent'] = 'base_link'
        apply_export_policies(joints, inertia)
        with self.assertRaisesRegex(ValueError, 'branching'):
            apply_export_policies(joints, inertia, frame_convention='dh_common_normal')
        joints['j2']['parent'] = 'arm'
        joints['j2']['type'] = 'fixed'
        apply_export_policies(joints, inertia)
        with self.assertRaisesRegex(ValueError, 'serial'):
            apply_export_policies(joints, inertia, frame_convention='dh_common_normal')
        with self.assertRaisesRegex(ValueError, 'frame_convention'):
            apply_export_policies(joints, inertia, frame_convention=True)
        j, b, _, _ = common_normal_scene()
        j['j2']['axis'] = unit([0., 1e-10, 1.])
        with self.assertRaisesRegex(ValueError, 'near-parallel'):
            apply_export_policies(j, b, frame_convention='dh_common_normal')

    def test_frame_only_is_explicit_and_has_no_mesh_mass_or_box(self):
        joints = {'j1': {'parent': 'base_link', 'child': 'helper', 'type': 'fixed', 'xyz': [1, 2, 3]}}
        inertial = {'base_link': body(0), 'helper': {'kind': 'frame'}}
        result = apply_export_policies(joints, inertial, output_rotation=Y_UP_TO_Z_UP)
        for key in ('mesh_origins', 'boxes', 'inertials'):
            self.assertNotIn('helper', result['emission'][key])
        self.assertIn('helper', result['link_frames'])
        inertial['helper']['mass'] = 1
        with self.assertRaisesRegex(ValueError, 'frame-only'):
            apply_export_policies(joints, inertial)

    def test_invalid_threshold_is_not_hidden_by_frame_only_model(self):
        for threshold in (-1., True, math.inf, 10**1000):
            with self.subTest(threshold=threshold), self.assertRaises(ValueError):
                apply_export_policies({}, {'base_link': {'kind': 'frame'}},
                                      minimum_principal_inertia=threshold)

    def test_no_policy_single_link_and_omitted_rpy_are_exact(self):
        inertia = {'base_link': body(0)}
        result = apply_export_policies({}, inertia)
        self.assertEqual(result['inertial'], inertia)
        self.assertEqual(result['joints'], {})
        self.assertEqual(result['link_frames']['base_link'], {'t': [0, 0, 0], 'R': I})
        joints = {'j': {'parent': 'base_link', 'child': 'a', 'type': 'continuous',
                         'xyz': [1, 2, 3], 'axis': [0, -1, 0]}}
        inertia['a'] = body(1)
        result = apply_export_policies(joints, inertia)
        self.assertEqual(result['joints'], joints)
        self.assertNotIn('rpy', result['joints']['j'])

    def test_unavailable_box_from_existing_extractor_is_not_fabricated(self):
        inertia = {'base_link': body(0)}
        inertia['base_link'].update(bbox_center_m=None, bbox_size_m=None)
        before = deepcopy(inertia)
        for output in (None, Y_UP_TO_Z_UP):
            result = apply_export_policies({}, inertia, output_rotation=output)
            self.assertEqual(result['emission']['boxes'], {})
            self.assertIsNone(result['inertial']['base_link']['bbox_center_m'])
            self.assertIsNone(result['inertial']['base_link']['bbox_size_m'])
        self.assertEqual(inertia, before)
        inertia['base_link']['bbox_center_m'] = [0, 0, 0]
        with self.assertRaises(ValueError):
            apply_export_policies({}, inertia)


if __name__ == '__main__':
    unittest.main()
