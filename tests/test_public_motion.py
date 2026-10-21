"""Public package oracles. Native doubles are not live Fusion evidence."""
import copy
import importlib
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

from tests.test_fusion_inputs import Body, design, fixed, group, hinge, occurrence, root
from tests.test_exporter_logic import FakeMatrix, identity_at
from tests.test_generalized_motion import I, cm, inverse, limits, mm, motion, shift, turn
from URDF_Exporter.core.Write import write_ros2_package
from tests.fixtures import model, write_tetrahedron, MESH_BOUNDS_MM


ENTRY = importlib.import_module('URDF_Exporter.URDF_Exporter')
KINDS = {'cylindrical': 3, 'pin_slot': 4, 'planar': 5, 'ball': 6}


def matrix_origin(element):
    xyz = list(map(float, element.get('xyz', '0 0 0').split()))
    r, p, y = map(float, element.get('rpy', '0 0 0').split())
    return mm(shift(xyz), mm(mm(turn((0, 0, 1), y), turn((0, 1, 0), p)), turn((1, 0, 0), r)))


def xml_fk(robot, positions):
    poses, pending = {'base_link': I}, list(robot.findall('joint'))
    values = dict(positions)
    mimics = {j.get('name'): j.find('mimic') for j in pending if j.find('mimic') is not None}
    while mimics:
        before = len(mimics)
        for name, mimic in list(mimics.items()):
            if mimic.get('joint') in values:
                values[name] = float(mimic.get('multiplier', '1'))*values[mimic.get('joint')] + float(mimic.get('offset', '0'))
                del mimics[name]
        assert len(mimics) < before
    while pending:
        before = len(pending)
        for joint in pending[:]:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in poses:
                continue
            action = I
            if joint.get('type') != 'fixed':
                q = values.get(joint.get('name'), 0.)
                axis = list(map(float, joint.find('axis').get('xyz').split()))
                action = shift([q*v for v in axis]) if joint.get('type') == 'prismatic' else turn(axis, q)
            poses[child] = mm(mm(poses[parent], matrix_origin(joint.find('origin'))), action)
            pending.remove(joint)
        assert len(pending) < before
    return poses


def assembly(kind, reverse=False, q0=(.43, 2.5, -1.7)):
    base = occurrence('base:1', [Body('base', 0)], component='base_link')
    pose = mm(shift((.37, -.12, .41)), turn((1, 2, 3), .61))
    child = occurrence('body:1', [Body('body', 10, mass=3)], matrix=cm(pose))
    joint = fixed('compound', base if reverse else child, child if reverse else base, (20, -10, 30))
    joint.jointMotion = motion(kind, q0)
    joint.jointMotion.jointType = KINDS[kind]
    joint.assemblyContext = None
    joint.nativeObject = None
    return root([base, child], joints=[joint]), joint, pose


def native_link(one, two=None, ids=(3, 4), reversed=False):
    return NS(name='native-link', isValid=True, isSuppressed=False, assemblyContext=None,
              jointOne=one, jointTwo=two, motionOne=ids[0], motionTwo=ids[1],
              valueOne=NS(value=2.), valueTwo=NS(value=5.), isReversed=reversed)


class PublicMotionTest(unittest.TestCase):
    def export(self, component, config=None):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        target = Path(temporary.name) / 'public_description'
        fixture, exported = design(component)
        ENTRY.export_design(fixture, target, config=config)
        return target, ET.parse(target / 'urdf/public.urdf').getroot(), exported

    def assertPose(self, actual, expected):
        for a, b in zip(sum(actual, ()), sum(expected, ())):
            self.assertAlmostEqual(a, b, places=10)

    def test_public_compound_fk_current_zero_nonzero_and_whole_chain_reversal(self):
        for kind in ('cylindrical', 'pin_slot', 'planar'):
            for reverse in (False, True):
                for current in ((0, 0, 0), (.43, 2.5, -1.7)):
                    component, source, pose = assembly(kind, reverse, current)
                    before = vars(source.jointMotion).copy()
                    target, robot, exported = self.export(component, {'generalized_joints': 'decompose'})
                    count = 3 if kind == 'planar' else 2
                    self.assertEqual(len(robot.findall('link')), 2+count)
                    self.assertEqual(len(robot.findall('joint')), count+1)
                    self.assertEqual([float(m.get('value')) for m in robot.findall('link/inertial/mass')], [1., 3.])
                    self.assertEqual(exported, ['base', 'body'])
                    for frame in robot.findall('link'):
                        if frame.get('name').startswith('fusion_helper_'):
                            self.assertEqual(list(frame), [])
                    sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
                    self.assertEqual(len(sidecar['independent_joints']), count)
                    pivot = shift((.2, -.1, .3))
                    for r, s, t in ((0, 0, 0), (.4, -.03, .02), (-.5, .04, -.03)):
                        rotate = turn((0, 0, 1), r)
                        if kind == 'cylindrical':
                            factors = mm(rotate, shift((0, 0, s)))
                            positions = {'rotate': r, 'slide': s}
                        elif kind == 'pin_slot':
                            factors = mm(shift((s, 0, 0)), rotate)
                            positions = {'rotate': r, 'slide': s}
                        else:
                            factors = mm(shift((s, -t, 0)), rotate)
                            positions = {'rotate': r, 'primary': s, 'secondary': t}
                        action = mm(mm(pivot, factors), inverse(pivot))
                        expected = mm(inverse(action) if reverse else action, pose)
                        names = {'fusion_helper_compound__'+n: v for n, v in positions.items()}
                        self.assertPose(xml_fk(robot, names)['body_1'], expected)
                    self.assertEqual(vars(source.jointMotion), before)

    def test_defaults_ball_optin_bounds_and_unbounded_travel_fail_before_meshes(self):
        for kind in KINDS:
            component, joint, _ = assembly(kind)
            with self.assertRaisesRegex(ValueError, 'UNSUPPORTED_FUSION_JOINT_TYPE'):
                self.export(component)
        component, joint, _ = assembly('ball')
        with self.assertRaisesRegex(ValueError, 'BALL_CHART_REQUIRED'):
            self.export(component, {'generalized_joints': 'decompose'})
        config = {'generalized_joints': 'decompose', 'ball_chart': 'export-local-rx-ry-rz'}
        _, robot, _ = self.export(component, config)
        self.assertEqual(len(robot.findall('ros2_control/joint/command_interface')), 3)
        joint.jointMotion.pitchLimits = limits(-1, 1)
        with self.assertRaisesRegex(ValueError, 'BALL_NATIVE_BOUNDS_UNPROVEN'):
            self.export(component, config)
        for bounds in (limits(), limits(-1, None)):
            component, joint, _ = assembly('cylindrical')
            joint.jointMotion.slideLimits = bounds
            fixture, exported = design(component)
            with tempfile.TemporaryDirectory() as temporary:
                target = Path(temporary) / 'blocked_description'
                with self.assertRaisesRegex(ValueError, 'finite lower_limit'):
                    ENTRY.export_design(fixture, target, config={'generalized_joints': 'decompose'})
                self.assertEqual(exported, [])
                self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_compound_cannot_be_welded_by_rigid_group_and_exact_dof_settings(self):
        component, _, _ = assembly('cylindrical')
        component.allRigidGroups = [group('clamp', list(component.allOccurrences))]
        with self.assertRaisesRegex(ValueError, 'RIGID_GROUP_INTERNAL_MOTION'):
            self.export(component, {'generalized_joints': 'decompose'})
        component.allRigidGroups = []
        config = {'generalized_joints': 'decompose', 'joint_limits': {'fusion_helper_compound__slide': {'effort': 7., 'velocity': .2}}}
        _, robot, _ = self.export(component, config)
        self.assertEqual(robot.find("joint[@name='fusion_helper_compound__slide']/limit").get('effort'), '7')
        config['joint_limits'] = {'compound': {'effort': 3}}
        with self.assertRaisesRegex(ValueError, 'DOF_SELECTOR'):
            self.export(component, config)

    def test_native_same_joint_mimic_source_phase_limits_and_independent_commands(self):
        for reverse in (False, True):
            component, joint, _ = assembly('cylindrical', reverse)
            component.motionLinks = [native_link(joint, reversed=True)]
            target, robot, _ = self.export(component, {'generalized_joints': 'decompose', 'motion_links': 'mimic'})
            driver, follower = 'fusion_helper_compound__rotate', 'fusion_helper_compound__slide'
            mimic = robot.find("joint[@name='%s']/mimic" % follower)
            self.assertEqual(mimic.attrib, {'joint': driver, 'multiplier': '-0.025', 'offset': '0'})
            control = robot.find('ros2_control')
            self.assertEqual([j.get('name') for j in control.findall('joint') if j.find('command_interface') is not None], [driver])
            self.assertIsNotNone(control.find("joint[@name='%s']/state_interface" % follower))
            sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
            plan = sidecar['motion_coupling']
            self.assertEqual(plan['relations'][0]['provenance']['current_positions_si'], [.43, .025])
            self.assertEqual(plan['independent_joints'], [driver])
            self.assertEqual(plan['source_limits'][follower]['lower_limit'], -.225)
            self.assertEqual(sidecar['joints'][follower]['lower_limit'], -.225)
            controllers = json.loads((target / 'config/controllers.yaml').read_text())
            self.assertEqual(controllers['joint_trajectory_controller']['ros__parameters']['joints'], [driver])

    def test_native_ordinary_and_asbuilt_bind_actual_objects_not_tokens(self):
        for asbuilt in (False, True):
            base = occurrence('base:1', [Body('base', 0)], component='base_link')
            a, b = occurrence('a:1', [Body('a', 10)]), occurrence('b:1', [Body('b', 20)])
            first, second = hinge('first', a, base), hinge('second', b, base)
            for joint in (first, second):
                joint.nativeObject, joint.assemblyContext, joint.entityToken = None, None, 'same-unreliable-token'
            component = root([base, a, b], joints=[] if asbuilt else [first, second])
            if asbuilt:
                component.allAsBuiltJoints = [first, second]
                first.transform = second.transform = FakeMatrix(identity_at([0, 0, 0]))
            component.motionLinks = [native_link(first, second, (10, 10))]
            _, robot, _ = self.export(component, {'motion_links': 'mimic'})
            self.assertEqual(robot.find("joint[@name='second']/mimic").get('joint'), 'first')
            self.assertEqual(len(robot.findall('ros2_control/joint/command_interface')), 1)
            component.motionLinks[0].jointTwo = NS(nativeObject=None, assemblyContext=None, entityToken=first.entityToken)
            with self.assertRaisesRegex(ValueError, 'BINDING'):
                self.export(component, {'motion_links': 'mimic'})

    def test_removed_macro_dofs_leave_no_helpers_and_coupling_to_closure_rejects(self):
        component, joint, _ = assembly('cylindrical')
        base, child = component.allOccurrences
        component.allJoints.insert(0, hinge('a_tree_edge', child, base))
        target, robot, _ = self.export(component, {'generalized_joints': 'decompose'})
        self.assertEqual(len(robot.findall('link')), 2)
        self.assertEqual([j.get('name') for j in robot.findall('joint')], ['a_tree_edge'])
        sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
        self.assertEqual(sidecar['closing_joints'][0]['joint'], 'compound')
        component.motionLinks = [native_link(joint)]
        with self.assertRaisesRegex(ValueError, 'REMOVED_CLOSURE_DOF'):
            self.export(component, {'generalized_joints': 'decompose', 'motion_links': 'mimic'})

    def test_pure_writer_explicit_mimic_common_initial_limits_and_invalid_no_publication(self):
        joints, inertial = model(True)
        joints['a'] = dict(parent='mount', child='arm', type='revolute', xyz=[.2, .1, .3], axis=[0, 0, 1],
                           lower_limit=-10., upper_limit=10., effort=9., velocity=8.)
        joints['b'] = dict(parent='arm', child='carriage', type='prismatic', xyz=[.3, .1, .3], axis=[1, 0, 0],
                           lower_limit=2., upper_limit=4., effort=7., velocity=2.,
                           mimic={'joint': 'a', 'multiplier': -2., 'offset': 1.})
        inertial.update(model()[1])
        del inertial['wheel']
        before = copy.deepcopy(joints)
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / 'pure_description'
            write_ros2_package(target, 'pure', joints, inertial,
                               write_meshes=lambda paths: [write_tetrahedron(p, MESH_BOUNDS_MM[n]) for n, p in paths.items()])
            sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
            self.assertEqual(sidecar['joints']['a']['lower_limit'], -1.5)
            self.assertEqual(sidecar['joints']['a']['upper_limit'], -.5)
            self.assertEqual(sidecar['joints']['a']['velocity'], 1.)
            self.assertEqual(sidecar['initial_positions'], {'a': -.5, 'b': 2.})
            self.assertEqual(joints, before)
            for bad in ({'joint': 'missing'}, {'joint': 'b'}, {'joint': 'a', 'multiplier': float('nan')}, {'joint': 'a', 'unknown': 1}):
                changed = copy.deepcopy(joints)
                changed['b']['mimic'] = bad
                with self.assertRaises(ValueError):
                    write_ros2_package(Path(temporary) / 'blocked_description', 'blocked', changed, inertial,
                                       write_meshes=lambda _: self.fail('Invalid mimic reached mesh export'))
            self.assertEqual(sorted(p.name for p in Path(temporary).iterdir()), ['pure_description'])

    def test_optional_registry_fixtures_have_independent_pose_mass_and_command_oracles(self):
        from tests.fixtures import fixture_oracle, fixture_variants, generate_fixture
        for variant in ('generalized', 'mimic'):
            self.assertEqual(fixture_variants(variant), (variant,))
            with tempfile.TemporaryDirectory() as temporary:
                package = generate_fixture(temporary, variant)
                robot = ET.parse(package / ('urdf/fixture_'+variant+'.urdf')).getroot()
                for values in fixture_oracle(variant).values():
                    poses = xml_fk(robot, values['positions'])
                    self.assertEqual(set(poses), set(values['link_poses']))
                    for name, expected in values['link_poses'].items():
                        self.assertPose(poses[name], expected)
                    for name, mass in values['masses'].items():
                        self.assertEqual(float(robot.find("link[@name='%s']/inertial/mass" % name).get('value')), mass)
                commands = {j.get('name') for j in robot.findall('ros2_control/joint') if j.find('command_interface') is not None}
                self.assertEqual(commands, set(fixture_oracle(variant)['moved']['positions']))

    def test_component_local_motion_links_keep_repeated_actual_contexts_distinct(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        native = NS()
        component = root([base])
        shared_component = NS(name='module', motionLinks=[native_link(native)])
        for side in ('left', 'right'):
            context = occurrence(side+':1')
            context.component = shared_component
            child = occurrence(side+':1+body:1', [Body(side, 10)])
            joint = fixed('drive', child, base)
            joint.jointMotion = motion('cylindrical')
            joint.jointMotion.jointType = 3
            joint.nativeObject, joint.assemblyContext = native, context
            component.allOccurrences.extend([context, child])
            component.allJoints.append(joint)
        _, robot, _ = self.export(component, {'generalized_joints': 'decompose', 'motion_links': 'mimic'})
        mimics = {j.get('name'): j.find('mimic').get('joint') for j in robot.findall('joint') if j.find('mimic') is not None}
        self.assertEqual(mimics, {'fusion_helper_left_1_drive__slide': 'fusion_helper_left_1_drive__rotate',
                                  'fusion_helper_right_1_drive__slide': 'fusion_helper_right_1_drive__rotate'})

    def test_suppressed_excluded_crossing_and_unknown_native_constraints_are_distinct(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        a, b = occurrence('hidden_a:1', [Body('a', 10)]), occurrence('hidden_b:1', [Body('b', 20)])
        a.isVisible = b.isVisible = False
        one, two = hinge('excluded_one', b, a), hinge('excluded_two', a, b)
        component = root([base, a, b], joints=[one, two])
        class Suppressed:
            name, isValid, isSuppressed = 'suppressed', True, True
            def __getattr__(self, field):
                raise AssertionError('Suppressed active metadata read: ' + field)
        component.motionLinks = [native_link(one, two, (10, 10)), Suppressed()]
        target, _, _ = self.export(component, {'motion_links': 'mimic'})
        sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
        self.assertEqual([r['status'] for r in sidecar['motion_coupling']['relations']], ['excluded', 'suppressed'])
        child = occurrence('selected:1', [Body('selected', 30)])
        selected = hinge('selected', child, base)
        component.allOccurrences.append(child)
        component.allJoints.append(selected)
        component.motionLinks[0].jointTwo = selected
        with self.assertRaisesRegex(ValueError, 'SELECTED_EXCLUDED'):
            self.export(component, {'motion_links': 'mimic'})
        component.motionLinks[0].jointOne = object()
        with self.assertRaisesRegex(ValueError, 'BINDING'):
            self.export(component, {'motion_links': 'mimic'})

    def test_compound_after_offset_ordinary_frame_preserves_physical_zero_and_motion(self):
        component, joint, pose = assembly('pin_slot')
        base, child = component.allOccurrences
        mount = occurrence('mount:1', [Body('mount', -10)], matrix=[0, -1, 0, 30, 1, 0, 0, 40, 0, 0, 1, 50, 0, 0, 0, 1])
        joint.occurrenceTwo = mount
        component.allOccurrences.append(mount)
        component.allJoints.insert(0, fixed('mount_fixed', mount, base, (5, 8, 13)))
        _, robot, _ = self.export(component, {'generalized_joints': 'decompose'})
        self.assertPose(xml_fk(robot, {})['body_1'], pose)
        pivot = shift((.2, -.1, .3))
        action = mm(mm(pivot, mm(shift((.03, 0, 0)), turn((0, 0, 1), .2))), inverse(pivot))
        self.assertPose(xml_fk(robot, {'fusion_helper_compound__slide': .03,
                                      'fusion_helper_compound__rotate': .2})['body_1'], mm(action, pose))

    def test_public_writer_rejects_sdk_state_without_deepcopy_and_closure_settings_survive(self):
        class Native:
            def __deepcopy__(self, memo):
                raise AssertionError('SDK deep copy attempted')
        joints, inertial = model(True)
        joints['mount_fixed']['sdk'] = Native()
        with tempfile.TemporaryDirectory() as temporary, self.assertRaisesRegex(ValueError, 'PLAIN_DATA'):
            write_ros2_package(Path(temporary) / 'blocked_description', 'blocked', joints, inertial,
                               write_meshes=lambda _: self.fail('Invalid native state reached mesh staging'))
        component, compound, _ = assembly('cylindrical')
        base, child = component.allOccurrences
        component.allJoints.insert(0, hinge('a_tree_edge', child, base))
        target, _, _ = self.export(component, {'generalized_joints': 'decompose',
            'joint_limits': {'fusion_helper_compound__slide': {'effort': 6., 'velocity': .3}}})
        sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
        self.assertEqual(sidecar['closing_joints'][0]['removed_dofs']['fusion_helper_compound__slide']['velocity'], .3)


if __name__ == '__main__':
    unittest.main()
