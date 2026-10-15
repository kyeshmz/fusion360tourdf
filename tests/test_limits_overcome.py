"""Limit-overcoming tests: orientations, loops, ROS2, materials, preflight."""
import math
import os
import sys
import tempfile
import types
import unittest

# Stub Fusion API before importing exporter code (mirrors test_exporter_logic).
adsk = types.ModuleType('adsk')
adsk_core = types.ModuleType('adsk.core')
adsk_fusion = types.ModuleType('adsk.fusion')


class _FakeCalcAccuracy:
    VeryHighCalculationAccuracy = 1


adsk_fusion.CalculationAccuracy = _FakeCalcAccuracy
adsk_fusion.MeshRefinementSettings = types.SimpleNamespace(
    MeshRefinementLow=0, MeshRefinementMedium=1, MeshRefinementHigh=2)
adsk_fusion.JointOrigin = type('JointOrigin', (), {})
adsk.core = adsk_core
adsk.fusion = adsk_fusion
sys.modules.setdefault('adsk', adsk)
sys.modules.setdefault('adsk.core', adsk_core)
sys.modules.setdefault('adsk.fusion', adsk_fusion)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from URDF_Exporter.utils import transforms as tf  # noqa: E402
from URDF_Exporter.utils import validate as val  # noqa: E402
from URDF_Exporter.utils import config as cfg  # noqa: E402
from URDF_Exporter.utils import materials as matlib  # noqa: E402
from URDF_Exporter.utils import utils  # noqa: E402
from URDF_Exporter.core import Joint, Link, Write  # noqa: E402


def yaw90_at(xyz):
    # Rz(90deg): x->y, y->-x. Row-major with translation in cm.
    return [0, -1, 0, xyz[0],
            1, 0, 0, xyz[1],
            0, 0, 1, xyz[2],
            0, 0, 0, 1]


def identity_at(xyz):
    return [1, 0, 0, xyz[0],
            0, 1, 0, xyz[1],
            0, 0, 1, xyz[2],
            0, 0, 0, 1]


class FakePoint:
    def __init__(self, xyz):
        self._xyz = list(xyz)

    def asArray(self):
        return list(self._xyz)


class FakeMatrix:
    def __init__(self, array16):
        self._array = list(array16)
        self.translation = FakePoint([array16[3], array16[7], array16[11]])

    def asArray(self):
        return list(self._array)


class FakeOccurrence:
    def __init__(self, name, component_name, matrix, visible=True, bodies=1):
        self.name = name
        self.component = types.SimpleNamespace(
            name=component_name,
            bRepBodies=types.SimpleNamespace(count=bodies),
            isLightBulbOn=visible,
        )
        self.transform2 = FakeMatrix(matrix)
        self.isLightBulbOn = visible


class FakeJointMotion:
    def __init__(self, joint_type, axis=None):
        self.jointType = joint_type
        if axis is not None:
            self.rotationAxisVector = FakePoint(axis)
            self.slideDirectionVector = FakePoint(axis)
        limits = types.SimpleNamespace(isMaximumValueEnabled=False,
                                       isMinimumValueEnabled=False)
        self.rotationLimits = limits
        self.slideLimits = limits


class FakeJoint:
    def __init__(self, name, occ_one, occ_two, one_xyz, two_xyz, motion=None):
        self.name = name
        self.occurrenceOne = occ_one
        self.occurrenceTwo = occ_two
        self.geometryOrOriginOne = types.SimpleNamespace(origin=FakePoint(one_xyz))
        self.geometryOrOriginTwo = types.SimpleNamespace(origin=FakePoint(two_xyz))
        self.jointMotion = motion or FakeJointMotion(0)


class FakeRoot:
    def __init__(self, joints, occurrences=(), as_built=()):
        self.joints = joints
        self.asBuiltJoints = list(as_built)
        self._occs = list(occurrences)
        self.allOccurrences = list(occurrences)
        self.occurrences = list(occurrences)


class TransformsTest(unittest.TestCase):
    def test_rpy_roundtrip(self):
        for rpy in ([0, 0, 0], [math.pi / 2, 0, 0], [0, math.pi / 4, 0],
                    [0, 0, -math.pi / 2], [0.3, -0.2, 1.1]):
            m = tf.mat3_from_rpy(rpy)
            back = tf.rpy_from_mat3(m)
            m2 = tf.mat3_from_rpy(back)
            for a, b in zip(m, m2):
                self.assertAlmostEqual(a, b, places=9)

    def test_relative_pose_parent_rotated(self):
        parent = yaw90_at([0, 0, 0])
        child = yaw90_at([100, 0, 0])
        t, rpy = tf.relative_pose(parent, child)
        # Same orientation -> zero relative rotation; translation of the
        # 100cm offset expressed in the parent frame (x->y).
        self.assertAlmostEqual(rpy[2], 0.0, places=6)
        self.assertAlmostEqual(t[0], 0.0, places=6)
        self.assertAlmostEqual(t[1], -1.0, places=6)

    def test_inverse_pose_roundtrip(self):
        t, r = [0.1, -0.2, 1.05], [0.0, 0.0, math.pi / 2]
        ti, ri = tf.inverse_pose(t, r)
        # Applying the inverse then the pose returns to origin.
        m = tf.mat3_from_rpy(r)
        mit = tf.mat3_from_rpy(ri)
        back = [mit[0] * t[0] + mit[1] * t[1] + mit[2] * t[2] + ti[0],
                mit[3] * t[0] + mit[4] * t[1] + mit[5] * t[2] + ti[1],
                mit[6] * t[0] + mit[7] * t[1] + mit[8] * t[2] + ti[2]]
        for v in back:
            self.assertAlmostEqual(v, 0.0, places=9)
        identity = tf.mat3_mul(mit, m)
        for a, b in zip(identity, tf.IDENTITY_3X3):
            self.assertAlmostEqual(a, b, places=9)

    def test_format_float_preserves_tiny(self):
        self.assertNotEqual(tf.format_float(1e-9), '0')
        self.assertEqual(tf.format_float(0.0), '0')
        self.assertEqual(float(tf.format_float(0.123456789)), 0.123456789)


class ValidateTest(unittest.TestCase):
    def test_closing_joint_on_shared_child(self):
        joints = {
            'j1': {'parent': 'base_link', 'child': 'a', 'type': 'fixed'},
            'j2': {'parent': 'a', 'child': 'b', 'type': 'revolute'},
            'j3': {'parent': 'base_link', 'child': 'b', 'type': 'revolute'},
        }
        tree, closing, warnings = val.detect_closing_joints(joints)
        self.assertIn('j1', tree)
        self.assertIn('j2', tree)
        self.assertNotIn('j3', tree)
        self.assertEqual(len(closing), 1)
        self.assertEqual(closing[0]['joint'], 'j3')
        self.assertTrue(warnings)

    def test_cycle_detection(self):
        joints = {
            'j1': {'parent': 'a', 'child': 'b', 'type': 'fixed'},
            'j2': {'parent': 'b', 'child': 'a', 'type': 'fixed'},
        }
        tree, closing, _ = val.detect_closing_joints(joints)
        self.assertEqual(len(tree), 1)
        self.assertEqual(len(closing), 1)

    def test_validate_catches_missing_child_and_bad_limits(self):
        joints = {
            'j1': {'parent': 'base_link', 'child': 'ghost', 'type': 'fixed'},
            'j2': {'parent': 'base_link', 'child': 'arm', 'type': 'revolute',
                   'axis': [0, 0, 1], 'upper_limit': -1.0, 'lower_limit': 1.0},
            'j3': {'parent': 'base_link', 'child': 'arm2', 'type': 'revolute',
                   'axis': [0, 0, 0], 'upper_limit': 1.0, 'lower_limit': -1.0},
        }
        inertial = {'base_link': {}, 'arm': {}, 'arm2': {}}
        errors, warnings = val.validate_export(joints, inertial)
        self.assertTrue(any('ghost' in e for e in errors))
        self.assertTrue(any('inverted' in e for e in errors))
        self.assertTrue(any('zero axis' in w for w in warnings))


class ConfigTest(unittest.TestCase):
    def test_defaults_and_validation(self):
        config = cfg.load_config(None, None)
        self.assertEqual(config['ros_version'], 'both')
        bad = cfg.load_config(None, {'ros_version': 'ros3', 'effort': 'x',
                                     'collision_method': 'mesh'})
        self.assertEqual(bad['ros_version'], 'both')
        self.assertEqual(bad['collision_method'], 'visual')
        self.assertEqual(bad['effort'], 100.0)
        self.assertTrue(cfg.wants_ros1({'ros_version': 'both'}))
        self.assertTrue(cfg.wants_ros2({'ros_version': 'both'}))
        self.assertFalse(cfg.wants_ros2({'ros_version': '1'}))

    def test_toml_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, 'xacro_export.toml'), 'w') as fh:
                fh.write('ros_version = "2"\ncollision_method = "primitive_box"\n')
            config = cfg.load_config(tmp, None)
            self.assertEqual(config['ros_version'], '2')
            self.assertEqual(config['collision_method'], 'primitive_box')


class MaterialsTest(unittest.TestCase):
    def test_appearance_255_to_urdf(self):
        diffuse = types.SimpleNamespace(red=255, green=0, blue=0, opacity=1.0)
        appearance = types.SimpleNamespace(diffuse=diffuse)
        rgba = matlib.appearance_to_rgba(appearance)
        self.assertEqual(rgba, (1.0, 0.0, 0.0, 1.0))
        self.assertEqual(matlib.rgba_to_urdf_string(rgba), '1.0 0.0 0.0 1.0')


class JointOrientationTest(unittest.TestCase):
    def test_joint_rpy_and_axis_in_parent_frame(self):
        parent = FakeOccurrence('base:1', 'base_link', identity_at([0, 0, 0]))
        child = FakeOccurrence('arm:1', 'arm', yaw90_at([100, 0, 0]))
        motion = FakeJointMotion(1, axis=[1, 0, 0])
        motion.rotationLimits = types.SimpleNamespace(
            isMaximumValueEnabled=True, isMinimumValueEnabled=True,
            maximumValue=1.0, minimumValue=-1.0)
        joint = FakeJoint('J1', child, parent, [0, 0, 0], [0, 0, 0], motion)
        joints_dict, msg = Joint.make_joints_dict(FakeRoot([joint]), 'ok')
        self.assertEqual(msg, 'ok')
        # Child yawed 90deg relative to identity parent.
        self.assertAlmostEqual(joints_dict['J1']['rpy'][2], math.pi / 2, places=5)
        # Global X axis expressed in the identity parent frame stays X...
        # (parent is identity here) so check rotation handling via helper:
        axis = Joint._axis_in_parent_frame([1, 0, 0], child)
        # Child yawed 90deg: global X in child frame is [0, -1, 0].
        self.assertAlmostEqual(axis[0], 0.0, places=5)
        self.assertAlmostEqual(axis[1], -1.0, places=5)


class WritersTest(unittest.TestCase):
    def _design(self, ros_version='both', collision='visual'):
        joints = {
            'J1': {'parent': 'base_link', 'child': 'arm', 'type': 'revolute',
                   'axis': [0, 0, 1], 'upper_limit': 1.0, 'lower_limit': -1.0,
                   'xyz': [0.0, 0.0, 1.0], 'rpy': [0, 0, math.pi / 2],
                   'effort': 50.0, 'velocity': 2.0},
        }
        inertial = {
            'base_link': {'center_of_mass': [0, 0, 0.5], 'mass': 1.0,
                          'inertia': [1e-9, 2e-9, 3e-9, 0, 0, 0],
                          'color_rgba': (1.0, 0.0, 0.0, 1.0),
                          'bbox_center_m': [0, 0, 0.5], 'bbox_size_m': [0.2, 0.2, 1.0]},
            'arm': {'center_of_mass': [0, 0, 1.5], 'mass': 2.0,
                    'inertia': [0.2] * 6, 'color_rgba': None,
                    'bbox_center_m': [0, 0, 1.5], 'bbox_size_m': [0.1, 0.1, 1.0]},
        }
        return joints, inertial

    def test_joint_xml_has_rpy_and_config_effort(self):
        joints, inertial = self._design()
        with tempfile.TemporaryDirectory() as tmp:
            save_dir = os.path.join(tmp, 'bot_description')
            os.makedirs(save_dir)
            Write.write_urdf(joints, {}, inertial, 'bot_description', 'bot', save_dir,
                             config={'effort': 50.0, 'velocity': 2.0,
                                     'collision_method': 'visual'})
            Write.write_materials_xacro(joints, {}, inertial, 'bot_description', 'bot', save_dir)
            with open(os.path.join(save_dir, 'urdf', 'bot.xacro')) as fh:
                text = fh.read()
            self.assertIn('rpy="0 0 1.570796"', text)
            self.assertIn('effort="50"', text)
            # Tiny inertia survives instead of truncating to 0.
            self.assertIn('1e-09', text)
            # Per-link color material emitted.
            with open(os.path.join(save_dir, 'urdf', 'materials.xacro')) as fh:
                mats = fh.read()
            self.assertIn('base_link_material', mats)

    def test_primitive_box_collision(self):
        joints, inertial = self._design()
        with tempfile.TemporaryDirectory() as tmp:
            save_dir = os.path.join(tmp, 'bot_description')
            os.makedirs(save_dir)
            Write.write_urdf(joints, {}, inertial, 'bot_description', 'bot', save_dir,
                             config={'collision_method': 'primitive_box',
                                     'effort': 100.0, 'velocity': 100.0})
            with open(os.path.join(save_dir, 'urdf', 'bot.xacro')) as fh:
                text = fh.read()
            self.assertIn('<box size=', text)

    def test_ros2_and_sidecar_files(self):
        joints, _ = self._design()
        joints['Jdup'] = {'parent': 'base_link', 'child': 'arm', 'type': 'fixed',
                          'axis': [0, 0, 0], 'upper_limit': 0.0, 'lower_limit': 0.0,
                          'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
        tree, closing, warnings = val.detect_closing_joints(joints)
        with tempfile.TemporaryDirectory() as tmp:
            save_dir = os.path.join(tmp, 'bot_description')
            os.makedirs(save_dir)
            Write.write_ros2_control_xacro(tree, 'bot_description', 'bot', save_dir)
            Write.write_ros2_controllers_yaml('bot_description', 'bot', save_dir, tree)
            Write.write_ros2_display_launch_py('bot_description', 'bot', save_dir)
            Write.write_ros2_gazebo_launch_py('bot_description', 'bot', save_dir)
            Write.write_ros2_package_files('bot_description', 'bot', save_dir)
            Write.write_robot_data_yaml('bot_description', 'bot', save_dir, tree,
                                        closing, warnings)
            for rel in ['urdf/bot.ros2_control.xacro', 'config/controllers.yaml',
                        'launch/display.launch.py', 'launch/gazebo.launch.py',
                        'package.xml', 'CMakeLists.txt', 'config/robot_data.yaml']:
                self.assertTrue(os.path.exists(os.path.join(save_dir, rel)), rel)
            with open(os.path.join(save_dir, 'config', 'robot_data.yaml')) as fh:
                sidecar = fh.read()
            self.assertIn('closing_joints:', sidecar)
            self.assertIn('Jdup', sidecar)


class OccurrenceWalkTest(unittest.TestCase):
    def test_flattens_nested_and_skips_hidden(self):
        leaf = FakeOccurrence('arm:1', 'arm', identity_at([0, 0, 0]), visible=True, bodies=2)
        hidden = FakeOccurrence('leg:1', 'leg', identity_at([0, 0, 0]), visible=False, bodies=2)
        container = FakeOccurrence('asm:1', 'asm', identity_at([0, 0, 0]), visible=True, bodies=0)
        root = FakeRoot([], occurrences=[leaf, hidden, container])
        names = [o.name for o in utils.iter_link_occurrences(root)]
        self.assertIn('arm:1', names)
        self.assertNotIn('leg:1', names)
        self.assertNotIn('asm:1', names)


if __name__ == '__main__':
    unittest.main()
