import copy
import importlib
import json
from fractions import Fraction
from pathlib import Path
import re
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakePoint
from tests.test_fusion_inputs import Body, design, fixed, group, occurrence, root
from tests.test_package import model, write_meshes
from URDF_Exporter.core.Write import write_ros2_package
from URDF_Exporter.utils import config as config_mod


def scene(*, swap_collections=False):
    base = occurrence('base:1', [Body('base', 0)], component='base_link')
    arm = occurrence('arm:1', [Body('arm', 10)])
    tool = occurrence('tool:1', [Body('tool', 20)])
    drive = fixed('drive pin', arm, base)
    drive.jointMotion = types.SimpleNamespace(jointType=1,
        rotationAxisVector=FakePoint([0, 0, 1]), rotationValue=.25,
        rotationLimits=types.SimpleNamespace(isMinimumValueEnabled=True,
            isMaximumValueEnabled=True, minimumValue=-1, maximumValue=2))
    lift = fixed('lift', tool, arm)
    lift.jointMotion = types.SimpleNamespace(jointType=2,
        slideDirectionVector=FakePoint([1, 0, 0]), slideValue=5,
        slideLimits=types.SimpleNamespace(isMinimumValueEnabled=True,
            isMaximumValueEnabled=True, minimumValue=0, maximumValue=20))
    component = root([base, arm, tool], joints=[lift if swap_collections else drive])
    component.allAsBuiltJoints = [drive if swap_collections else lift]
    return component, drive, lift


class JointSettingsConfigTest(unittest.TestCase):
    def test_normalizes_real_values_without_mutating_input_or_defaults(self):
        defaults = copy.deepcopy(config_mod.DEFAULTS)
        options = {'effort': 50, 'joint_limits': {
            'drive_pin': {'effort': 4, 'velocity': Fraction(4, 5)}, 'lift': {}}}
        original = copy.deepcopy(options)
        result = config_mod.load_config(overrides=options)
        self.assertEqual(result['joint_limits'], {
            'drive_pin': {'effort': 4.0, 'velocity': .8}, 'lift': {}})
        self.assertIsInstance(result['joint_limits']['drive_pin']['effort'], float)
        self.assertEqual(options, original)
        result['joint_limits']['drive_pin']['effort'] = 99
        result['joint_limits']['lift']['velocity'] = 2
        self.assertEqual(options, original)
        self.assertEqual(config_mod.DEFAULTS, defaults)
        self.assertEqual(config_mod.load_config(), defaults)

    def test_each_load_owns_its_nested_settings(self):
        options = {'joint_limits': {'drive_pin': {'effort': 4}}}
        first = config_mod.load_config(overrides=options)
        second = config_mod.load_config(overrides=options)
        first['joint_limits']['drive_pin']['effort'] = 9
        self.assertEqual(second['joint_limits']['drive_pin']['effort'], 4)
        self.assertEqual(options['joint_limits']['drive_pin']['effort'], 4)

    def test_rejects_bad_tables_selectors_and_unknown_fields(self):
        cases = [
            (None, 'joint_limits'), ([], 'joint_limits'), (True, 'joint_limits'),
            ({1: {'effort': 4}}, 'selector'), ({'': {'effort': 4}}, 'selector'),
            ({'drive_pin': 4}, 'drive_pin'), ({'drive_pin': []}, 'drive_pin'),
            ({'drive_pin': {'lower': -1}}, 'lower'),
            ({'drive_pin': {'upper_limit': 2}}, 'upper_limit'),
            ({'drive_pin': {'effrot': 4}}, 'effrot'),
        ]
        for values, diagnostic in cases:
            with self.subTest(values=values), self.assertRaisesRegex(ValueError, diagnostic):
                config_mod.load_config(overrides={'joint_limits': values})

    def test_rejects_nonpositive_nonfinite_and_nonreal_fields(self):
        for field in ('effort', 'velocity'):
            for value in (True, False, '4', None, [], 0, -1, float('nan'),
                          float('inf'), -float('inf'), complex(1, 0), 10**1000):
                with self.subTest(field=field, value=value):
                    with self.assertRaisesRegex(ValueError, 'drive_pin.*' + field + '.*finite positive'):
                        config_mod.load_config(overrides={'joint_limits': {'drive_pin': {field: value}}})

    def test_invalid_toml_joint_settings_are_not_hidden_by_overrides(self):
        for content in ('[joint_limits.drive_pin]\neffort = "4"\n',
                        '[joint_limits.drive_pin]\nlower = -1\n'):
            with self.subTest(content=content), tempfile.TemporaryDirectory() as directory:
                (Path(directory) / config_mod.CONFIG_FILENAME).write_text(content)
                with self.assertRaisesRegex(ValueError, 'joint_limits.*drive_pin'):
                    config_mod.load_config(directory, {'joint_limits': {'drive_pin': {'effort': 4}}})


class JointSettingsExportTest(unittest.TestCase):
    def export(self, component, options=None):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        destination = Path(temporary.name) / 'settings_description'
        fixture, exported = design(component)
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        self.assertEqual(entry.export_design(fixture, destination, config=options), destination)
        self.assertEqual(exported, ['base', 'arm', 'tool'])
        return destination

    def assert_limits(self, destination, expected, *, continuous=False):
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        controllers = json.loads((destination / 'config/controllers.yaml').read_text())
        self.assertEqual(set(controllers['joint_trajectory_controller']['ros__parameters']['joints']),
                         {'drive_pin', 'lift'})
        for extension in ('urdf', 'xacro'):
            robot = ET.parse(destination / ('urdf/settings.' + extension)).getroot()
            self.assertEqual({joint.get('name') for joint in robot.findall('ros2_control/joint')},
                             {'drive_pin', 'lift'})
            for name, (effort, velocity, lower, upper) in expected.items():
                joint = robot.find("joint[@name='%s']" % name)
                limits = joint.find('limit')
                self.assertEqual(float(limits.get('effort')), effort)
                self.assertEqual(float(limits.get('velocity')), velocity)
                self.assertEqual(sidecar['joints'][name]['effort'], effort)
                self.assertEqual(sidecar['joints'][name]['velocity'], velocity)
                if continuous and name == 'drive_pin':
                    self.assertEqual(joint.get('type'), 'continuous')
                    self.assertNotIn('lower', limits.attrib)
                    self.assertNotIn('upper', limits.attrib)
                else:
                    self.assertAlmostEqual(float(limits.get('lower')), lower)
                    self.assertAlmostEqual(float(limits.get('upper')), upper)
                    self.assertAlmostEqual(sidecar['joints'][name]['lower_limit'], lower)
                    self.assertAlmostEqual(sidecar['joints'][name]['upper_limit'], upper)

    def test_distinct_ordinary_and_as_built_settings_preserve_source_bounds(self):
        for swap in (False, True):
            with self.subTest(swap_collections=swap):
                component, drive, lift = scene(swap_collections=swap)
                root_before = vars(component).copy()
                options = {'joint_limits': {'drive_pin': {'effort': 4, 'velocity': .8},
                                            'lift': {'effort': 6, 'velocity': .2}}}
                before = copy.deepcopy(options)
                destination = self.export(component, options)
                self.assert_limits(destination, {'drive_pin': (4, .8, -1.25, 1.75),
                                                 'lift': (6, .2, -.05, .15)})
                self.assertEqual(options, before)
                self.assertEqual(vars(component), root_before)
                self.assertEqual(drive.jointMotion.rotationValue, .25)
                self.assertEqual(vars(drive.jointMotion.rotationLimits), {
                    'isMinimumValueEnabled': True, 'isMaximumValueEnabled': True,
                    'minimumValue': -1, 'maximumValue': 2})
                self.assertEqual(lift.jointMotion.slideValue, 5)
                self.assertEqual(vars(lift.jointMotion.slideLimits), {
                    'isMinimumValueEnabled': True, 'isMaximumValueEnabled': True,
                    'minimumValue': 0, 'maximumValue': 20})

    def test_partial_fields_inherit_custom_globals(self):
        component, _, _ = scene()
        destination = self.export(component, {'effort': 50, 'velocity': 3,
            'joint_limits': {'drive_pin': {'effort': 4}, 'lift': {'velocity': .8}}})
        self.assert_limits(destination, {'drive_pin': (4, 3, -1.25, 1.75),
                                         'lift': (50, .8, -.05, .15)})

    def test_unmentioned_joints_inherit_globals(self):
        component, _, _ = scene()
        destination = self.export(component, {'effort': 20, 'velocity': 2,
            'joint_limits': {'drive_pin': {'effort': 4}}})
        self.assert_limits(destination, {'drive_pin': (4, 2, -1.25, 1.75),
                                         'lift': (20, 2, -.05, .15)})

    def test_absent_and_empty_settings_preserve_defaults(self):
        for options in (None, {'joint_limits': {}}, {'joint_limits': {'drive_pin': {}}}):
            with self.subTest(options=options):
                component, _, _ = scene()
                destination = self.export(component, options)
                self.assert_limits(destination, {'drive_pin': (100, 100, -1.25, 1.75),
                                                 'lift': (100, 100, -.05, .15)})

    def test_continuous_joints_accept_effort_and_velocity_without_travel_bounds(self):
        component, drive, _ = scene()
        drive.jointMotion.rotationLimits.isMinimumValueEnabled = False
        drive.jointMotion.rotationLimits.isMaximumValueEnabled = False
        destination = self.export(component, {'joint_limits': {'drive_pin': {'effort': 4, 'velocity': .8}}})
        self.assert_limits(destination, {'drive_pin': (4, .8, None, None),
                                         'lift': (100, 100, -.05, .15)}, continuous=True)

    def test_toml_and_call_overrides_merge_per_joint_and_field(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / config_mod.CONFIG_FILENAME).write_text(
                'effort = 40\nvelocity = 2\n'
                '[joint_limits.drive_pin]\neffort = 4\nvelocity = 0.8\n'
                '[joint_limits.lift]\neffort = 6\n')
            overrides = {'velocity': 3, 'joint_limits': {'drive_pin': {'effort': 5},
                                                       'lift': {'velocity': .2}}}
            before = copy.deepcopy(overrides)
            options = config_mod.load_config(directory, overrides)
            self.assertEqual(options['joint_limits'], {
                'drive_pin': {'effort': 5, 'velocity': .8}, 'lift': {'effort': 6, 'velocity': .2}})
            component, _, _ = scene()
            self.assert_limits(self.export(component, options), {
                'drive_pin': (5, .8, -1.25, 1.75), 'lift': (6, .2, -.05, .15)})
            self.assertEqual(overrides, before)

    def assert_rejected(self, component, options, diagnostic):
        fixture, exported = design(component)
        before = vars(component).copy()
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'invalid_description'
            sentinel = Path(directory) / 'keep.txt'
            sentinel.write_bytes(b'untouched')
            with self.assertRaisesRegex(ValueError, diagnostic):
                importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(
                    fixture, destination, config=options)
            self.assertFalse(destination.exists())
            self.assertEqual(list(Path(directory).iterdir()), [sentinel])
            self.assertEqual(sentinel.read_bytes(), b'untouched')
        self.assertEqual(exported, [])
        self.assertEqual(vars(component), before)

    def test_unknown_unsanitized_and_suppressed_selectors_fail_before_publication(self):
        for selector in ('missing', 'drive pin', 'Drive_pin', 'drive*', ' drive_pin'):
            with self.subTest(selector=selector):
                component, _, _ = scene()
                self.assert_rejected(component, {'joint_limits': {selector: {'effort': 4}}},
                                     'joint_limits.*' + re.escape(selector))
        component, drive, _ = scene()
        drive.isSuppressed = True
        self.assert_rejected(component, {'joint_limits': {'drive_pin': {'effort': 4}}},
                             'joint_limits.*drive_pin')

    def test_fixed_locked_and_generated_rigid_targets_fail_before_publication(self):
        for kind in ('fixed', 'locked', 'attachment'):
            with self.subTest(kind=kind):
                component, drive, _ = scene()
                selector = 'drive_pin'
                if kind == 'fixed':
                    drive.jointMotion = types.SimpleNamespace(jointType=0)
                elif kind == 'locked':
                    drive.isLocked = True
                else:
                    component.allJoints = []
                    component.allRigidGroups = [group('mount', component.allOccurrences[:2])]
                    selector = 'rigid_attachment_arm_1'
                self.assert_rejected(component, {'joint_limits': {selector: {'effort': 4}}},
                                     'joint_limits.*' + selector + '.*fixed')

    def test_sanitized_name_collision_is_not_a_selector_tiebreak(self):
        component, drive, lift = scene()
        drive.name, lift.name = 'drive pin', 'drive_pin'
        self.assert_rejected(component, {'joint_limits': {'drive_pin': {'effort': 4}}},
                             'joint_limits.*drive_pin.*does not match an exported joint')

    def test_invalid_numeric_and_travel_fields_fail_before_publication(self):
        for values in ({'effort': True}, {'velocity': '0.8'}, {'effort': float('nan')},
                       {'velocity': 0}, {'lower': -1}, {'upper': 2}):
            with self.subTest(values=values):
                component, _, _ = scene()
                self.assert_rejected(component, {'joint_limits': {'drive_pin': values}},
                                     'joint_limits.*drive_pin')


class JointSettingsWriterTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.parent = Path(temporary.name)
        self.destination = self.parent / 'settings_description'
        self.joints, self.inertial = model()

    def export(self, options=None):
        before = copy.deepcopy((self.joints, self.inertial, options))
        callbacks = []

        def meshes(targets):
            self.assertFalse(self.destination.exists())
            self.assertEqual((self.joints, self.inertial, options), before)
            callbacks.append(set(targets))
            write_meshes(targets)

        result = write_ros2_package(self.destination, 'settings', self.joints, self.inertial,
                                    config=options, write_meshes=meshes)
        self.assertEqual(result, self.destination)
        self.assertEqual(callbacks, [set(self.inertial)])
        self.assertEqual((self.joints, self.inertial, options), before)
        return json.loads((result / 'config/robot_data.yaml').read_text())

    def assert_limits(self, sidecar, effort, velocity):
        for extension in ('urdf', 'xacro'):
            joint = ET.parse(self.destination / ('urdf/settings.' + extension)).find(
                "joint[@name='shoulder']")
            self.assertEqual({key: float(value) for key, value in joint.find('limit').items()},
                             {'effort': effort, 'velocity': velocity, 'lower': -1, 'upper': 1})
        self.assertEqual(sidecar['joints']['shoulder'],
                         dict(self.joints['shoulder'], effort=effort, velocity=velocity))

    def assert_rejected(self, options, diagnostic):
        before = copy.deepcopy((self.joints, self.inertial, options))
        callbacks = []
        sentinel = self.parent / 'keep.txt'
        sentinel.write_bytes(b'untouched')
        with self.assertRaisesRegex(ValueError, diagnostic):
            write_ros2_package(self.destination, 'settings', self.joints, self.inertial,
                                config=options, write_meshes=callbacks.append)
        self.assertEqual(callbacks, [])
        self.assertFalse(self.destination.exists())
        self.assertEqual(list(self.parent.iterdir()), [sentinel])
        self.assertEqual(sentinel.read_bytes(), b'untouched')
        self.assertEqual((self.joints, self.inertial, options), before)

    def test_named_fields_override_explicit_joint_values(self):
        sidecar = self.export({'effort': 50, 'velocity': 3,
            'joint_limits': {'shoulder': {'effort': 4, 'velocity': .8}}})
        self.assert_limits(sidecar, 4, .8)

    def test_partial_named_fields_preserve_other_explicit_joint_values(self):
        for field, expected in (('effort', (4, 2)), ('velocity', (12, 4))):
            with self.subTest(field=field):
                self.destination = self.parent / (field + '_description')
                sidecar = self.export({'effort': 50, 'velocity': 3,
                    'joint_limits': {'shoulder': {field: 4}}})
                self.assert_limits(sidecar, *expected)

    def test_absent_and_empty_settings_preserve_explicit_fields_over_globals(self):
        for index, limits in enumerate((None, {}, {'shoulder': {}})):
            with self.subTest(limits=limits):
                self.destination = self.parent / ('case_%d_description' % index)
                options = {'effort': 50, 'velocity': 3}
                if limits is not None:
                    options['joint_limits'] = limits
                self.assert_limits(self.export(options), 12, 2)

    def test_missing_fields_use_global_fallbacks_in_urdf_and_sidecar(self):
        del self.joints['shoulder']['effort']
        del self.joints['shoulder']['velocity']
        for index, (limits, expected) in enumerate((({}, (50, 3)),
                ({'shoulder': {'effort': 4}}, (4, 3)),
                ({'shoulder': {'velocity': .8}}, (50, .8)))):
            with self.subTest(limits=limits):
                self.destination = self.parent / ('case_%d_description' % index)
                sidecar = self.export({'effort': 50, 'velocity': 3, 'joint_limits': limits})
                self.assert_limits(sidecar, *expected)

    def test_unknown_and_fixed_targets_reject_before_callback_or_publication(self):
        for selector, diagnostic in (('missing', 'does not match'), ('tool_mount', 'fixed')):
            for fields in ({}, {'effort': 4}):
                with self.subTest(selector=selector, fields=fields):
                    self.assert_rejected({'joint_limits': {'shoulder': {'velocity': .8},
                        selector: fields}}, 'joint_limits.*' + selector + '.*' + diagnostic)

    def test_invalid_options_reject_without_mutating_inputs(self):
        for fields in ({'effort': True}, {'velocity': '0.8'}, {'effort': 0},
                       {'velocity': float('inf')}, {'lower_limit': -2}):
            with self.subTest(fields=fields):
                self.assert_rejected({'joint_limits': {'shoulder': fields}}, 'joint_limits.*shoulder')

    def test_closing_joint_settings_are_bound_before_reduction_and_retained(self):
        self.joints['loop'] = dict(self.joints['shoulder'], parent='tool', child='base_link')
        del self.joints['loop']['velocity']
        sidecar = self.export({'velocity': 3, 'joint_limits': {'loop': {'effort': 4}}})
        self.assert_limits(sidecar, 12, 2)
        self.assertEqual(sidecar['closing_joints'], [dict(self.joints['loop'], effort=4,
            velocity=3, joint='loop', source='auto_detected')])
        for extension in ('urdf', 'xacro'):
            robot = ET.parse(self.destination / ('urdf/settings.' + extension))
            self.assertIsNone(robot.find("joint[@name='loop']"))
            self.assertIsNone(robot.find("ros2_control/joint[@name='loop']"))
        controllers = json.loads((self.destination / 'config/controllers.yaml').read_text())
        self.assertEqual(controllers['joint_trajectory_controller']['ros__parameters']['joints'],
                         ['shoulder'])

    def test_fixed_closing_joint_selector_is_not_hidden_by_reduction(self):
        self.joints['loop'] = dict(self.joints['tool_mount'], parent='tool', child='base_link')
        self.assert_rejected({'joint_limits': {'loop': {'effort': 4}}}, 'joint_limits.*loop.*fixed')

    def test_override_is_validated_instead_of_replaced_explicit_field(self):
        self.joints['shoulder']['effort'] = -1
        self.assert_limits(self.export({'joint_limits': {'shoulder': {'effort': 4}}}), 4, 2)


if __name__ == '__main__':
    unittest.main()
