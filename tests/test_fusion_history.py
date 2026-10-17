import math
from pathlib import Path
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakeJoint, FakeMatrix, FakeOccurrence, FakeRoot, identity_at
from tests.test_package import write_meshes
from URDF_Exporter.core import Joint, Write


class DirectJoint(FakeJoint):
    @property
    def geometryOrOriginOne(self):
        raise RuntimeError('Original history geometry is unavailable')

    @geometryOrOriginOne.setter
    def geometryOrOriginOne(self, value):
        pass

    @property
    def geometryOrOriginTwo(self):
        raise RuntimeError('Original history geometry is unavailable')

    @geometryOrOriginTwo.setter
    def geometryOrOriginTwo(self, value):
        pass

    @property
    def timelineObject(self):
        raise AssertionError('Export must not access or roll the joint timeline')


class FusionHistoryTest(unittest.TestCase):
    def joint(self, cls=FakeJoint):
        base = FakeOccurrence('base:1', 'base_link', [100, 0, 0])
        child = FakeOccurrence('arm:1', 'arm', [10, 20, 30])
        joint = cls('mount', child, base, [10, 0, 0], [5, 0, 0])
        joint.geometryTwoTransform = FakeMatrix(identity_at([123, 20, 30]))
        return joint

    def test_current_transform_overrides_old_history_geometry(self):
        joints, status = Joint.make_joints_dict(FakeRoot([self.joint()]), 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(joints['mount']['xyz'], [1.23, .2, .3])

    def test_direct_design_without_original_geometry_does_not_touch_timeline(self):
        joint = self.joint(DirectJoint)
        before = joint.occurrenceOne.transform2.asArray()
        joints, status = Joint.make_joints_dict(FakeRoot([joint]), 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(joints['mount']['xyz'], [1.23, .2, .3])
        self.assertEqual(joint.occurrenceOne.transform2.asArray(), before)

    def test_rigid_as_built_needs_no_joint_geometry(self):
        joint = self.joint(DirectJoint)
        joint.geometry = None
        joint.transform = None
        joint.occurrenceOne.transform2 = FakeMatrix([
            0, -1, 0, 10, 1, 0, 0, 20, 0, 0, 1, 30, 0, 0, 0, 1])
        joints, status = Joint.make_joints_dict(FakeRoot([], as_built=[joint]), 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(joints['mount']['type'], 'fixed')
        self.assertEqual(joints['mount']['xyz'], [.1, .2, .3])
        self.assertAlmostEqual(joints['mount']['rpy'][2], math.pi / 2)
        inertial = {name: {'mass': 1.0, 'center_of_mass': [0, 0, 0],
                           'inertia': [.1, .1, .1, 0, 0, 0]}
                    for name in ('base_link', 'arm_1')}
        with tempfile.TemporaryDirectory() as directory:
            package = Write.write_ros2_package(Path(directory) / 'history_description',
                'history', joints, inertial, write_meshes=write_meshes)
            robot = ET.parse(package / 'urdf/history.xacro').getroot()
            self.assertEqual(robot.find('joint').get('type'), 'fixed')
            self.assertEqual(robot.find('joint/origin').get('xyz'), '0.1 0.2 0.3')

    def test_as_built_prefers_current_transform_over_obsolete_geometry(self):
        joint = self.joint()
        joint.geometry = joint.geometryOrOriginTwo
        joint.transform = FakeMatrix(identity_at([40, 50, 60]))
        joints, status = Joint.make_joints_dict(FakeRoot([], as_built=[joint]), 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(joints['mount']['xyz'], [.4, .5, .6])

    def test_suppressed_joint_is_not_exported_but_hidden_marker_is(self):
        suppressed = self.joint()
        suppressed.name = 'suppressed'
        suppressed.isSuppressed = True
        active = self.joint()
        active.isSuppressed = False
        active.isLightBulbOn = False
        joints, status = Joint.make_joints_dict(FakeRoot([suppressed, active]), 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(set(joints), {'mount'})
        self.assertEqual(joints['mount']['xyz'], [1.23, .2, .3])

    def test_geometry_one_current_transform_survives_missing_geometry_two(self):
        joint = self.joint(DirectJoint)
        joint.geometryTwoTransform = None
        joint.geometryOneTransform = FakeMatrix(identity_at([40, 50, 60]))
        root = FakeRoot([joint])
        root.allJoints.insert(0, types.SimpleNamespace(name='suppressed_unreadable', isSuppressed=True))
        joints, status = Joint.make_joints_dict(root, 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(joints['mount']['xyz'], [.4, .5, .6])
        self.assertEqual(set(joints), {'mount'})

    def test_moving_joint_without_current_frame_is_named_failure(self):
        for as_built in (False, True):
            with self.subTest(as_built=as_built):
                joint = self.joint(DirectJoint)
                joint.geometryTwoTransform = None
                joint.geometryOneTransform = None
                joint.transform = None
                joint.jointMotion = types.SimpleNamespace(jointType=1,
                    rotationAxisVector=types.SimpleNamespace(asArray=lambda: [0, 0, 1]),
                    rotationValue=0, rotationLimits=types.SimpleNamespace(
                        isMaximumValueEnabled=False, isMinimumValueEnabled=False))
                root = FakeRoot([] if as_built else [joint], as_built=[joint] if as_built else [])
                joints, status = Joint.make_joints_dict(root, 'ok')
                self.assertEqual(joints, {})
                self.assertIn('JOINT_FRAME_UNAVAILABLE: mount', status)
