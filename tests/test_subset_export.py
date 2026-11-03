"""Config-driven subset export: include/exclude/frame-only selection, travel overrides, root origin."""
import copy
from fractions import Fraction
import importlib
import json
import math
from pathlib import Path
import struct
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.fixtures import model, write_tetrahedron, MESH_BOUNDS_MM
from tests.test_exporter_logic import FakePoint, identity_at
from tests.test_fusion_inputs import Body, design, first_vertex_fk, fixed, group, hinge, occurrence, root
from URDF_Exporter.core.Write import write_ros2_package
from URDF_Exporter.utils import config as config_mod
from URDF_Exporter.utils import utils
from URDF_Exporter.utils.export_policy import apply_export_policies, Y_UP_TO_Z_UP

ENTRY = importlib.import_module('URDF_Exporter.URDF_Exporter')
IDENTITY = [1., 0., 0., 0., 1., 0., 0., 0., 1.]


def slider(name, child, parent, xyz=(0, 0, 0), bounds=None, position=0):
    joint = fixed(name, child, parent, xyz)
    limits = types.SimpleNamespace(isMinimumValueEnabled=bounds is not None, isMaximumValueEnabled=bounds is not None)
    if bounds is not None:
        limits.minimumValue, limits.maximumValue = bounds
    joint.jointMotion = types.SimpleNamespace(jointType=2, slideDirectionVector=FakePoint([1, 0, 0]),
                                              slideValue=position, slideLimits=limits)
    return joint


def bounded_hinge(name, child, parent, bounds, position=0, xyz=(0, 0, 0)):
    joint = fixed(name, child, parent, xyz)
    joint.jointMotion = types.SimpleNamespace(jointType=1, rotationAxisVector=FakePoint([0, 0, 1]),
        rotationValue=position, rotationLimits=types.SimpleNamespace(isMinimumValueEnabled=True,
            isMaximumValueEnabled=True, minimumValue=bounds[0], maximumValue=bounds[1]))
    return joint


def origin_of(element):
    origin = element.find('origin')
    return tuple(map(float, origin.get('xyz').split())), tuple(map(float, origin.get('rpy').split()))


def rotate(matrix, vector):
    return [sum(matrix[3*i+k]*vector[k] for k in range(3)) for i in range(3)]


class CloseMixin:
    def assertClose(self, actual, expected, places=12):
        if isinstance(expected, dict):
            self.assertEqual(set(actual), set(expected))
            for key in expected:
                self.assertClose(actual[key], expected[key], places)
        elif isinstance(expected, (list, tuple)):
            self.assertEqual(len(actual), len(expected))
            for a, b in zip(actual, expected):
                self.assertClose(a, b, places)
        elif isinstance(expected, float):
            self.assertAlmostEqual(actual, expected, places=places)
        else:
            self.assertEqual(actual, expected)


class ExportMixin(CloseMixin):
    def export(self, component, config=None, *, name='subset'):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        destination = Path(temporary.name) / (name + '_description')
        fixture, exported = design(component)
        ENTRY.export_design(fixture, destination, config=config)
        robot = ET.parse(destination / ('urdf/%s.urdf' % name)).getroot()
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        return robot, sidecar, destination, exported

    def assert_rejected(self, component, config, pattern):
        fixture, exported = design(component)
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'rejected_description'
            with self.assertRaisesRegex(ValueError, pattern):
                ENTRY.export_design(fixture, destination, config=config)
            self.assertFalse(destination.exists())
        self.assertEqual(exported, [])

    def assert_limits(self, robot, name, kind, lower, upper):
        joint = robot.find("joint[@name='%s']" % name)
        self.assertEqual(joint.get('type'), kind)
        self.assertAlmostEqual(float(joint.find('limit').get('lower')), lower, places=12)
        self.assertAlmostEqual(float(joint.find('limit').get('upper')), upper, places=12)


class SelectionConfigTest(unittest.TestCase):
    def test_lists_must_be_unique_exact_strings_and_consistent_across_options(self):
        cases = (
            ({'include_occurrences': 'arm:1'}, 'include_occurrences must be an array'),
            ({'exclude_occurrences': ['']}, 'exclude_occurrences must be an array'),
            ({'frame_only_occurrences': [1]}, 'frame_only_occurrences must be an array'),
            ({'include_occurrences': ['arm:1', 'arm:1']}, 'include_occurrences lists a duplicate'),
            ({'include_occurrences': ['arm:1'], 'exclude_occurrences': ['arm:1']}, 'both list'),
            ({'exclude_occurrences': ['arm:1'], 'frame_only_occurrences': ['arm:1']}, 'both list'),
            ({'exclude_occurrences': ['arm:1'], 'frame_only_occurrences': ['arm:1+clip:1']}, 'inside an exclude'),
            ({'exclude_occurrences': ['arm:1'], 'include_occurrences': ['arm:1+clip:1']}, 'inside an exclude'),
            ({'root_origin': 'world'}, 'root_origin must be one of'),
        )
        for overrides, pattern in cases:
            with self.subTest(overrides=overrides), self.assertRaisesRegex(ValueError, pattern):
                config_mod.load_config(overrides=overrides)
        config = config_mod.load_config(overrides={'include_occurrences': ('arm:1',),
                                                   'exclude_occurrences': ['arm:1+clip:1'],
                                                   'frame_only_occurrences': ['arm:1+pad:1']})
        self.assertEqual(config['include_occurrences'], ['arm:1'])
        self.assertNotIn('include_occurrences', config_mod.load_config())

    def test_travel_bounds_need_both_finite_ordered_values(self):
        for fields, pattern in (({'lower': -1.0}, 'both lower and upper'), ({'upper': 1.0}, 'both lower and upper'),
                                ({'lower': 1.0, 'upper': 1.0}, 'less than upper'),
                                ({'lower': 2.0, 'upper': 1.0}, 'less than upper'),
                                ({'lower': float('nan'), 'upper': 1.0}, 'finite number'),
                                ({'lower': True, 'upper': 1.0}, 'finite number'),
                                ({'lower': -1.0, 'upper': float('inf')}, 'finite number')):
            with self.subTest(fields=fields), self.assertRaisesRegex(ValueError, 'drive.*' + pattern):
                config_mod.load_config(overrides={'joint_limits': {'drive': fields}})
        config = config_mod.load_config(overrides={'joint_limits': {'drive': {'lower': -1, 'upper': 2, 'effort': 4}}})
        self.assertEqual(config['joint_limits'], {'drive': {'effort': 4.0, 'lower': -1.0, 'upper': 2.0}})


class IncludeExcludeTest(ExportMixin, unittest.TestCase):
    def assembly(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        module = occurrence('module:1')
        arm = occurrence('module:1+arm:1', [Body('arm', 10), Body('arm_hidden', 15, visible=False)],
                         matrix=identity_at([10, 0, 0]))
        lift = occurrence('lift:1', [Body('lift', 20)], matrix=identity_at([20, 0, 0]))
        clip = occurrence('clip:1', [Body('clip', 30)], matrix=identity_at([30, 0, 0]))
        joints = [hinge('drive', arm, base), fixed('lift_mount', lift, base, (20, 0, 0))]
        groups = [group('clip_clamp', [arm, clip, lift])]
        return root([base, module, arm, lift, clip], joints=joints, groups=groups), lift, arm, clip

    def assert_subset(self, robot, sidecar, destination, exported, option):
        self.assertEqual({link.get('name') for link in robot.findall('link')}, {'base_link', 'module_1_arm_1', 'clip_1'})
        joints = {j.get('name'): (j.get('type'), j.find('parent').get('link'), j.find('child').get('link'))
                  for j in robot.findall('joint')}
        self.assertEqual(joints, {'drive': ('continuous', 'base_link', 'module_1_arm_1'),
                                  'rigid_attachment_clip_1': ('fixed', 'module_1_arm_1', 'clip_1')})
        self.assertEqual(exported, ['base', 'arm', 'clip'])
        self.assertEqual(sorted(p.name for p in (destination / 'meshes').iterdir()),
                         ['base_link.stl', 'clip_1.stl', 'module_1_arm_1.stl'])
        notes = {note['name']: note for note in sidecar['source_notes']}
        self.assertEqual(notes['lift_mount']['reason'], 'Joint endpoint lift:1 is excluded by ' + option)
        self.assertEqual(notes['clip_clamp']['excluded'], {'lift:1': option})
        self.assertEqual(sidecar['joints']['rigid_attachment_clip_1']['rigid_constraints'][0]['members'],
                         ['clip_1', 'module_1_arm_1'])

    def test_include_selects_listed_subtrees_drops_crossing_joint_and_welds_remaining_group(self):
        component, lift, arm, _ = self.assembly()
        arm.isVisible = lift.isVisible = False
        config = {'include_occurrences': ['base:1', 'module:1', 'clip:1']}
        result = self.export(component, config)
        self.assert_subset(*result, 'include_occurrences')
        self.assertEqual(result[1]['selection'], config)

    def test_include_keeps_bodies_of_hidden_parts_by_their_own_light_bulb(self):
        component, lift, arm, _ = self.assembly()
        arm.isVisible = lift.isVisible = False
        arm_body = utils.collection_items(arm.bRepBodies)[0]
        arm_body.isVisible, arm_body.isLightBulbOn = False, True
        config = {'include_occurrences': ['base:1', 'module:1', 'clip:1']}
        self.assert_subset(*self.export(component, config), 'include_occurrences')
        arm_body.isLightBulbOn = False
        self.assert_rejected(component, config, r'drive.*CONSTRAINT_TARGET_EXCLUDED.*module:1\+arm:1')

    def test_exclude_without_include_keeps_light_bulb_selection_for_other_parts(self):
        component, lift, _, clip = self.assembly()
        self.assert_subset(*self.export(component, {'exclude_occurrences': ['lift:1']}), 'exclude_occurrences')
        clip.isVisible = False
        self.assert_rejected(component, {'exclude_occurrences': ['lift:1']},
                             r'CONSTRAINT_TARGET_EXCLUDED.*clip_clamp.*clip:1')

    def test_exclude_wins_inside_an_included_subtree_and_both_excluded_joints_keep_existing_note(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        lift = occurrence('lift:1', [Body('lift', 20)])
        pad = occurrence('lift:1+pad:1', [Body('pad', 30)])
        component = root([base, lift, pad], joints=[fixed('lift_mount', lift, base), fixed('lift_pad', pad, lift)])
        robot, sidecar, _, exported = self.export(component, {'include_occurrences': ['base:1', 'lift:1'],
                                                              'exclude_occurrences': ['lift:1+pad:1']})
        self.assertEqual({link.get('name') for link in robot.findall('link')}, {'base_link', 'lift_1'})
        self.assertEqual(exported, ['base', 'lift'])
        self.assertEqual({n['name']: n['reason'] for n in sidecar['source_notes']},
                         {'lift_pad': 'Joint endpoint lift:1+pad:1 is excluded by exclude_occurrences'})
        robot, sidecar, _, exported = self.export(component, {'include_occurrences': ['base:1']})
        self.assertEqual([link.get('name') for link in robot.findall('link')], ['base_link'])
        self.assertEqual(exported, ['base'])
        self.assertEqual({n['name']: n['reason'] for n in sidecar['source_notes']},
                         {'lift_mount': 'Joint endpoint lift:1 is excluded by include_occurrences',
                          'lift_pad': 'All joint endpoints are excluded'})

    def test_light_bulb_hidden_parts_without_lists_still_fail_by_name(self):
        component, lift, _, _ = self.assembly()
        lift.isVisible = False
        self.assert_rejected(component, None, r'lift_mount.*CONSTRAINT_TARGET_EXCLUDED.*lift:1')
        component.allJoints = component.allJoints[:1]
        self.assert_rejected(component, None, r'CONSTRAINT_TARGET_EXCLUDED.*clip_clamp.*lift:1')

    def test_unknown_listed_paths_fail_before_export(self):
        component, _, _, _ = self.assembly()
        for option in config_mod.SELECTION_OPTIONS:
            with self.subTest(option=option):
                self.assert_rejected(component, {option: ['arm:1']}, 'OCCURRENCE_NOT_FOUND.*' + option + '.*arm:1')

    def test_grounding_to_config_excluded_parent_is_dropped_with_note(self):
        component, _, arm, _ = self.assembly()
        component.allJoints, component.allRigidGroups = component.allJoints[:1], []
        arm.isGroundToParent = True
        robot, sidecar, _, _ = self.export(component, {'include_occurrences': ['base:1', 'module:1+arm:1']})
        self.assertEqual([j.get('name') for j in robot.findall('joint')], ['drive'])
        [note] = [n for n in sidecar['source_notes'] if n.get('source') == 'Fusion grounding']
        self.assertEqual((note['name'], note['reason']), ('module:1+arm:1.isGroundToParent',
                         'Grounding endpoint module:1 is excluded by include_occurrences'))


class FrameOnlyTest(ExportMixin, unittest.TestCase):
    def assembly(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        finger = occurrence('finger:1', [Body('finger', 10)], matrix=identity_at([10, 0, 0]))
        clip = occurrence('clip:1', [Body('clip', 20)], matrix=identity_at([20, 0, 0]))
        joints = [fixed('finger_mount', finger, base, (10, 0, 0)),
                  slider('slide', clip, finger, (20, 0, 0), bounds=(0, 20), position=5)]
        return root([base, finger, clip], joints=joints)

    def test_frame_only_part_keeps_its_slider_without_mesh_or_mass(self):
        robot, _, destination, exported = self.export(self.assembly())
        self.assertIsNotNone(robot.find("link[@name='clip_1']/inertial"))
        self.assertTrue((destination / 'meshes/clip_1.stl').exists())
        self.assertEqual(exported, ['base', 'finger', 'clip'])
        robot, sidecar, destination, exported = self.export(self.assembly(), {'frame_only_occurrences': ['clip:1']})
        self.assertEqual(list(robot.find("link[@name='clip_1']")), [])
        self.assertFalse((destination / 'meshes/clip_1.stl').exists())
        self.assertEqual(exported, ['base', 'finger'])
        slide = robot.find("joint[@name='slide']")
        self.assertEqual((slide.find('parent').get('link'), slide.find('child').get('link')), ('finger_1', 'clip_1'))
        self.assert_limits(robot, 'slide', 'prismatic', -.05, .15)
        self.assertEqual(sidecar['link_sources']['clip_1'], {'kind': 'frame', 'path': 'clip:1'})
        self.assertEqual(sidecar['selection'], {'frame_only_occurrences': ['clip:1']})
        self.assertEqual(sidecar['independent_joints'], ['slide'])

    def test_unreferenced_frame_only_part_adds_no_link(self):
        component = self.assembly()
        component.allJoints = component.allJoints[:1]
        robot, _, _, exported = self.export(component, {'frame_only_occurrences': ['clip:1']})
        self.assertEqual({link.get('name') for link in robot.findall('link')}, {'base_link', 'finger_1'})
        self.assertEqual(exported, ['base', 'finger'])


class FrameOnlyRootTest(ExportMixin, unittest.TestCase):
    def test_bodyless_root_with_excluded_children_is_a_frame_base_link_with_revolute_child(self):
        arm = occurrence('arm:1')
        base_asm = occurrence('arm:1+base_asm:1', matrix=identity_at([30, 10, 80]))
        plate = occurrence('arm:1+base_asm:1+plate:1', [Body('plate', 0)], matrix=identity_at([30, 10, 80]))
        motor = occurrence('arm:1+base_asm:1+motor:1', [Body('motor', 5)], matrix=identity_at([35, 10, 80]))
        link1 = occurrence('arm:1+link1:1', [Body('link1', 10)], matrix=identity_at([30, 10, 90]))
        lift = occurrence('lift:1', [Body('lift', 50)])
        joints = [bounded_hinge('J1', link1, base_asm, (-1, 2), xyz=(30, 10, 90)),
                  fixed('plate_mount', plate, link1, (30, 10, 85)), fixed('lift_mount', base_asm, lift, (30, 10, 0))]
        component = root([arm, base_asm, plate, motor, link1, lift], joints=joints)
        config = {'root_occurrence': 'arm:1+base_asm:1', 'include_occurrences': ['arm:1'],
                  'exclude_occurrences': ['arm:1+base_asm:1+plate:1', 'arm:1+base_asm:1+motor:1']}
        robot, sidecar, destination, exported = self.export(component, config)
        self.assertEqual({link.get('name') for link in robot.findall('link')}, {'base_link', 'arm_1_link1_1'})
        self.assertEqual(list(robot.find("link[@name='base_link']")), [])
        self.assertFalse((destination / 'meshes/base_link.stl').exists())
        self.assertEqual(exported, ['link1'])
        j1 = robot.find("joint[@name='J1']")
        self.assertEqual((j1.find('parent').get('link'), j1.find('child').get('link')), ('base_link', 'arm_1_link1_1'))
        self.assert_limits(robot, 'J1', 'revolute', -1, 2)
        self.assertEqual(sidecar['link_sources']['base_link'], {'kind': 'frame', 'path': 'arm:1+base_asm:1'})
        self.assertEqual({n['name']: n['reason'] for n in sidecar['source_notes']}, {
            'plate_mount': 'Joint endpoint arm:1+base_asm:1+plate:1 is excluded by exclude_occurrences',
            'lift_mount': 'Joint endpoint lift:1 is excluded by include_occurrences'})
        self.assert_rejected(component, {'root_occurrence': 'arm:1+base_asm:1', 'exclude_occurrences': ['arm:1']},
                             r'ROOT_NOT_FOUND.*arm:1\+base_asm:1')


class TravelOverrideTest(ExportMixin, unittest.TestCase):
    def assembly(self, *, cad_bounds=False):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        arm = occurrence('arm:1', [Body('arm', 10)], matrix=identity_at([10, 0, 0]))
        tool = occurrence('tool:1', [Body('tool', 20)], matrix=identity_at([20, 0, 0]))
        drive = bounded_hinge('drive', arm, base, (-1, 2), position=.25) if cad_bounds else hinge('drive', arm, base)
        lift = slider('lift', tool, arm, (20, 0, 0), bounds=(0, 20) if cad_bounds else None, position=5)
        return root([base, arm, tool], joints=[drive, lift])

    def test_unbounded_cad_joints_fail_without_and_export_with_configured_travel(self):
        self.assert_rejected(self.assembly(), None, r'SLIDER_TRAVEL_REQUIRED.*lift.*joint_limits')
        self.assert_rejected(self.assembly(), {'joint_limits': {'drive': {'lower': -1, 'upper': 2}}},
                             'SLIDER_TRAVEL_REQUIRED.*lift')
        robot, sidecar, _, _ = self.export(self.assembly(), {'joint_limits': {'drive': {'lower': -1, 'upper': 2},
                                                                              'lift': {'lower': -.01, 'upper': .03}}})
        self.assert_limits(robot, 'drive', 'revolute', -1, 2)
        self.assert_limits(robot, 'lift', 'prismatic', -.01, .03)
        self.assertEqual(sidecar['joints']['drive']['travel_override'], {'source': 'joint_limits', 'replaced': {
            'type': 'continuous', 'lower_limit': None, 'upper_limit': None}})
        self.assertEqual(sidecar['joints']['lift']['travel_override'], {'source': 'joint_limits', 'replaced': {
            'type': 'prismatic', 'lower_limit': None, 'upper_limit': None}})
        self.assertEqual(sidecar['initial_positions'], {'drive': 0., 'lift': 0.})
        self.assertEqual({j.get('name') for j in robot.findall('ros2_control/joint')}, {'drive', 'lift'})

    def test_configured_travel_replaces_cad_bounds_and_records_them(self):
        robot, _, _, _ = self.export(self.assembly(cad_bounds=True))
        self.assert_limits(robot, 'drive', 'revolute', -1.25, 1.75)
        robot, sidecar, _, _ = self.export(self.assembly(cad_bounds=True), {'joint_limits': {
            'drive': {'lower': -.5, 'upper': .5, 'effort': 4}, 'lift': {'effort': 6}}})
        self.assert_limits(robot, 'drive', 'revolute', -.5, .5)
        self.assertEqual(float(robot.find("joint[@name='drive']/limit").get('effort')), 4)
        self.assertClose(sidecar['joints']['drive']['travel_override'],
                         {'source': 'joint_limits', 'replaced': {'type': 'revolute', 'lower_limit': -1.25, 'upper_limit': 1.75}})
        self.assert_limits(robot, 'lift', 'prismatic', -.05, .15)
        self.assertNotIn('travel_override', sidecar['joints']['lift'])
        self.assert_rejected(self.assembly(cad_bounds=True), {'joint_limits': {'missing': {'lower': -1, 'upper': 1}}},
                             'joint_limits.*missing.*does not match')

    def test_mimic_follower_and_leader_travel_compose_through_the_coupling_planner(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        negative = occurrence('negative:1', [Body('negative', 10)], matrix=identity_at([10, 0, 0]))
        positive = occurrence('positive:1', [Body('positive', -10)], matrix=identity_at([-10, 0, 0]))
        leader = slider('negative_slide', negative, base, (10, 0, 0))
        follower = slider('positive_slide', positive, base, (-10, 0, 0))
        for joint in (leader, follower):
            joint.nativeObject = joint.assemblyContext = None
        component = root([base, negative, positive], joints=[leader, follower])
        component.motionLinks = [types.SimpleNamespace(name='fingers', isValid=True, isSuppressed=False,
            assemblyContext=None, jointOne=leader, jointTwo=follower, motionOne=11, motionTwo=11,
            valueOne=types.SimpleNamespace(value=1.), valueTwo=types.SimpleNamespace(value=1.), isReversed=True)]
        bounds = {'lower': -.00519, 'upper': .03241}
        self.assert_rejected(component, {'motion_links': 'mimic'}, 'SLIDER_TRAVEL_REQUIRED')
        robot, sidecar, _, _ = self.export(component, {'motion_links': 'mimic', 'joint_limits': {
            'negative_slide': dict(bounds), 'positive_slide': dict(bounds)}})
        mimic = robot.find("joint[@name='positive_slide']/mimic")
        self.assertEqual((mimic.get('joint'), float(mimic.get('multiplier'))), ('negative_slide', -1.))
        self.assert_limits(robot, 'positive_slide', 'prismatic', bounds['lower'], bounds['upper'])
        # Follower bounds divided by the -1 multiplier intersect the leader's own bounds.
        lower = max(Fraction(bounds['lower']), -Fraction(bounds['upper']))
        upper = min(Fraction(bounds['upper']), -Fraction(bounds['lower']))
        self.assert_limits(robot, 'negative_slide', 'prismatic', float(lower), float(upper))
        self.assertEqual(sidecar['independent_joints'], ['negative_slide'])
        self.assertEqual(sidecar['motion_coupling']['effective_limits']['negative_slide'],
                         {'lower_limit': float(lower), 'upper_limit': float(upper), 'velocity': 100.})
        self.assertEqual([j.get('name') for j in robot.findall('ros2_control/joint')
                          if j.find('command_interface') is not None], ['negative_slide'])


class RootOriginTest(ExportMixin, unittest.TestCase):
    ORIGIN_CM = [30, 10, 80]

    def assembly(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link', matrix=identity_at(self.ORIGIN_CM))
        arm = occurrence('arm:1', [Body('arm', 10)], matrix=[0, -1, 0, 40, 1, 0, 0, 10, 0, 0, 1, 80, 0, 0, 0, 1])
        tool = occurrence('tool:1', [Body('tool', 20)], matrix=identity_at([60, 10, 80]))
        joints = [bounded_hinge('drive', arm, base, (-2, 2), xyz=(40, 10, 80)),
                  slider('lift', tool, arm, (60, 10, 80), bounds=(0, 20))]
        return root([base, arm, tool], joints=joints)

    def test_root_occurrence_origin_shifts_only_base_link_frame_data(self):
        shift = [v / 100 for v in self.ORIGIN_CM]
        design_robot, design_sidecar, design_destination, _ = self.export(self.assembly())
        robot, sidecar, destination, _ = self.export(self.assembly(), {'root_origin': 'root_occurrence'})
        self.assertNotIn('root_origin', design_sidecar)
        self.assertEqual(sidecar['root_origin'], {'selected': 'root_occurrence', 'translation_m': shift, 'axes': 'design'})
        self.assertEqual(sidecar['joints'], design_sidecar['joints'])
        for name in ('drive', 'lift'):
            before, after = (r.find("joint[@name='%s']" % name) for r in (design_robot, robot))
            (xyz_before, rpy_before), (xyz_after, rpy_after) = origin_of(before), origin_of(after)
            self.assertEqual(rpy_after, rpy_before)
            self.assertEqual(after.find('axis').get('xyz'), before.find('axis').get('xyz'))
            expected = [a - b for a, b in zip(xyz_before, shift)] if name == 'drive' else list(xyz_before)
            self.assertClose(list(xyz_after), expected)
        self.assertEqual(ET.tostring(robot.find("joint[@name='lift']")), ET.tostring(design_robot.find("joint[@name='lift']")))
        for name in ('arm_1', 'tool_1'):
            self.assertEqual(ET.tostring(robot.find("link[@name='%s']" % name)),
                             ET.tostring(design_robot.find("link[@name='%s']" % name)))
        for tag in ('visual', 'collision', 'inertial'):
            before = origin_of(design_robot.find("link[@name='base_link']/" + tag))
            after = origin_of(robot.find("link[@name='base_link']/" + tag))
            self.assertEqual(after[1], before[1])
            self.assertClose(list(after[0]), [a - b for a, b in zip(before[0], shift)])
        self.assertEqual((destination / 'meshes/base_link.stl').read_bytes(),
                         (design_destination / 'meshes/base_link.stl').read_bytes())
        for q in (0., .7):
            moved = first_vertex_fk(robot, destination, 'drive', q)
            reference = first_vertex_fk(design_robot, design_destination, 'drive', q)
            self.assertClose(moved, [a - b for a, b in zip(reference, shift)])

    def test_design_root_as_root_is_a_zero_shift(self):
        component = root([occurrence('arm:1', [Body('arm', 10)])], bodies=[Body('chassis', 0)])
        component.allJoints = [fixed('mount', component.allOccurrences[0], None, (10, 0, 0))]
        design_robot, _, _, _ = self.export(component)
        robot, sidecar, _, _ = self.export(component, {'root_origin': 'root_occurrence'})
        self.assertEqual(sidecar['root_origin']['translation_m'], [0., 0., 0.])
        self.assertEqual(ET.tostring(robot), ET.tostring(design_robot))


class RootOriginPolicyTest(CloseMixin, unittest.TestCase):
    ORIGIN = [.2, -.1, .3]

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.parent = Path(temporary.name)

    def write(self, name, config, joints=None, inertials=None, **kwargs):
        if joints is None:
            joints, inertials = model()
        destination = self.parent / (name + '_description')
        write_ros2_package(destination, name, joints, inertials, config=config,
                           write_meshes=lambda paths: [write_tetrahedron(p, MESH_BOUNDS_MM[l]) for l, p in paths.items()],
                           **kwargs)
        return (ET.parse(destination / ('urdf/%s.urdf' % name)).getroot(),
                json.loads((destination / 'config/robot_data.yaml').read_text()), destination)

    def test_pure_policy_origin_moves_root_pose_and_mesh_contract_only(self):
        joints, inertials = model()
        root_frame = {'t': self.ORIGIN, 'R': IDENTITY}
        plain = apply_export_policies(joints, inertials, output_rotation=Y_UP_TO_Z_UP, source_root_frame=root_frame)
        moved = apply_export_policies(joints, inertials, output_rotation=Y_UP_TO_Z_UP, source_root_frame=root_frame,
                                      source_origin=self.ORIGIN)
        for key in ('joint_origins', 'mesh_origins', 'inertials', 'boxes'):
            self.assertClose(moved['emission'][key], plain['emission'][key])
        rotated_origin = rotate(Y_UP_TO_Z_UP, self.ORIGIN)
        self.assertClose(plain['emission']['root_pose']['t'], rotated_origin)
        self.assertClose(moved['emission']['root_pose'], {'t': [0., 0., 0.], 'R': list(Y_UP_TO_Z_UP)})
        self.assertClose(moved['mesh_source_to_output'], {'t': [-v for v in rotated_origin], 'R': list(Y_UP_TO_Z_UP)})
        for name, frame in moved['link_frames'].items():
            self.assertClose(frame['t'], [a - b for a, b in zip(plain['link_frames'][name]['t'], rotated_origin)])
        with self.assertRaisesRegex(ValueError, 'source_origin'):
            apply_export_policies(joints, inertials, source_origin=[0., float('nan'), 0.])

    def test_root_origin_composes_with_output_rotation_without_moving_the_reference(self):
        config = {'output_rotation_preset': 'y_up_to_z_up'}
        rotated, _, _ = self.write('rotated', config)
        robot, sidecar, destination = self.write('moved', dict(config, root_origin='root_occurrence'),
                                                 root_origin_m=self.ORIGIN)
        reference = robot.find("joint[@name='export_output_to_base_link']")
        self.assertEqual(origin_of(reference)[0], (0., 0., 0.))
        self.assertClose(origin_of(reference)[1], origin_of(rotated.find("joint[@name='export_output_to_base_link']"))[1])
        self.assertEqual(sidecar['root_frame']['source_root_pose_in_output']['t'], [0., 0., 0.])
        self.assertEqual(sidecar['root_origin']['translation_m'], self.ORIGIN)
        self.assertClose(origin_of(robot.find("joint[@name='mount_fixed']")), ((0., 0., 0.), (0., 0., math.pi / 2)))
        for name in ('shoulder', 'slide', 'spin'):
            self.assertClose(origin_of(robot.find("joint[@name='%s']" % name)),
                             origin_of(rotated.find("joint[@name='%s']" % name)))
        for name in ('mount', 'arm', 'carriage', 'wheel'):
            for tag in ('visual', 'collision', 'inertial'):
                self.assertClose(origin_of(robot.find("link[@name='%s']/%s" % (name, tag))),
                                 origin_of(rotated.find("link[@name='%s']/%s" % (name, tag))))
        visual = robot.find("link[@name='base_link']/visual")
        self.assertEqual(origin_of(visual), ((-.2, .1, -.3), (0., 0., 0.)))
        vertex = struct.unpack_from('<3f', (destination / 'meshes/base_link.stl').read_bytes(), 96)
        world = rotate(Y_UP_TO_Z_UP, [v/1000 - o for v, o in zip(vertex, self.ORIGIN)])
        local = [v/1000 + o for v, o in zip(vertex, origin_of(visual)[0])]
        self.assertClose(rotate(Y_UP_TO_Z_UP, local), world, places=9)

    def test_root_origin_composes_with_dh_frames_and_requires_the_root_origin(self):
        joints = {'shoulder': dict(parent='base_link', child='arm', type='revolute', xyz=[.2, .1, .3], rpy=[0, 0, 0],
                                   axis=[0, 0, 1], lower_limit=-1., upper_limit=1., effort=2., velocity=3.),
                  'slide': dict(parent='arm', child='carriage', type='prismatic', xyz=[.3, .1, .3], rpy=[0, 0, 0],
                                axis=[0, 1, 0], lower_limit=-.1, upper_limit=.1, effort=2., velocity=3.)}
        inertials = {name: model()[1][name] for name in ('base_link', 'arm', 'carriage')}
        config = {'frame_convention': 'dh_common_normal'}
        dh, _, _ = self.write('dh', config, copy.deepcopy(joints), copy.deepcopy(inertials))
        robot, sidecar, _ = self.write('moved', dict(config, root_origin='root_occurrence'), copy.deepcopy(joints),
                                       copy.deepcopy(inertials), root_origin_m=self.ORIGIN)
        self.assertIsNone(robot.find("link[@name='export_output_reference']"))
        self.assertEqual(sidecar['export_policies']['receipts']['frame_convention']['joints']['shoulder']['rule'], 'skew')
        self.assertClose(origin_of(robot.find("joint[@name='slide']")), origin_of(dh.find("joint[@name='slide']")))
        before, after = origin_of(dh.find("joint[@name='shoulder']")), origin_of(robot.find("joint[@name='shoulder']"))
        self.assertClose(after[1], before[1])
        self.assertClose(list(after[0]), [a - b for a, b in zip(before[0], self.ORIGIN)])
        with self.assertRaisesRegex(ValueError, 'LINK_FRAME_UNAVAILABLE.*root_origin_m'):
            self.write('missing', {'root_origin': 'root_occurrence'})
        self.assertFalse((self.parent / 'missing_description').exists())


if __name__ == '__main__':
    unittest.main()
