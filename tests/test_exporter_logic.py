"""Offline tests for fusion360tourdf exporter logic (no Fusion required)."""
import os
import sys
import tempfile
import types
import unittest

# Stub the Fusion `adsk` modules before importing exporter code.
adsk = types.ModuleType('adsk')
adsk_core = types.ModuleType('adsk.core')
adsk_fusion = types.ModuleType('adsk.fusion')


class _FakeCalcAccuracy:
    VeryHighCalculationAccuracy = 1


adsk_fusion.CalculationAccuracy = _FakeCalcAccuracy
adsk_fusion.MeshRefinementSettings = types.SimpleNamespace(MeshRefinementLow=0)
adsk_fusion.JointOrigin = type('JointOrigin', (), {})
adsk.core = adsk_core
adsk.fusion = adsk_fusion
sys.modules['adsk'] = adsk
sys.modules['adsk.core'] = adsk_core
sys.modules['adsk.fusion'] = adsk_fusion

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from URDF_Exporter.utils import utils  # noqa: E402
from URDF_Exporter.core import Joint, Link, Write  # noqa: E402


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


def identity_at(xyz):
    return [
        1, 0, 0, xyz[0],
        0, 1, 0, xyz[1],
        0, 0, 1, xyz[2],
        0, 0, 0, 1,
    ]


class FakeOccurrence:
    def __init__(self, name, component_name, xyz):
        self.name = name
        self.component = types.SimpleNamespace(name=component_name)
        self.transform2 = FakeMatrix(identity_at(xyz))


class FakeJointMotion:
    def __init__(self, joint_type):
        self.jointType = joint_type


class FakeJoint:
    def __init__(self, name, occ_one, occ_two, one_xyz, two_xyz):
        self.name = name
        self.occurrenceOne = occ_one
        self.occurrenceTwo = occ_two
        self.geometryOrOriginOne = types.SimpleNamespace(origin=FakePoint(one_xyz))
        self.geometryOrOriginTwo = types.SimpleNamespace(origin=FakePoint(two_xyz))
        self.jointMotion = FakeJointMotion(0)


class FakeRoot:
    def __init__(self, joints, as_built=()):
        self.joints = joints
        self.asBuiltJoints = list(as_built)


class ExporterLogicTest(unittest.TestCase):
    def test_sanitize_name(self):
        self.assertEqual(utils.sanitize_name('base_link (1):1'), 'base_link_1_1')
        self.assertEqual(utils.sanitize_name('base-link/foo'), 'base_link_foo')
        self.assertEqual(utils.sanitize_name('base_link'), 'base_link')

    def test_transform_point_identity(self):
        out = utils.transform_point(identity_at([10, 20, 30]), [1, 2, 3])
        self.assertEqual(out, [11, 22, 33])

    def test_occurrence_helpers_prefer_transform2(self):
        occ = FakeOccurrence('a:1', 'a', [1, 2, 3])
        # Legacy `transform` returns the wrong answer; transform2 must win.
        occ.transform = FakeMatrix(identity_at([99, 99, 99]))
        self.assertEqual(utils.occurrence_translation(occ), [1, 2, 3])
        self.assertEqual(utils.occurrence_matrix_array(occ)[:4], [1, 0, 0, 1])

    def test_make_joints_dict_uses_transform2(self):
        occ_one = FakeOccurrence('child:1', 'child', [0, 0, 0])
        occ_two = FakeOccurrence('parent:1', 'parent', [100, 0, 0])
        # Joint origins differ so the occurrence transform must compose them:
        # 100cm (occurrence) + 5cm (joint in occurrence frame) = 105cm.
        joint = FakeJoint('Rev1', occ_one, occ_two, [10, 0, 0], [5, 0, 0])
        # Force legacy transform to be wrong to prove transform2 is used.
        occ_two.transform = FakeMatrix(identity_at([0, 0, 0]))
        joints_dict, msg = Joint.make_joints_dict(FakeRoot([joint]), 'ok')
        self.assertEqual(msg, 'ok')
        self.assertIn('Rev1', joints_dict)
        self.assertEqual(joints_dict['Rev1']['xyz'], [1.05, 0.0, 0.0])
        self.assertEqual(joints_dict['Rev1']['type'], 'fixed')

    def test_make_joints_dict_rejects_ball(self):
        occ_one = FakeOccurrence('child:1', 'child', [0, 0, 0])
        occ_two = FakeOccurrence('parent:1', 'parent', [0, 0, 0])
        joint = FakeJoint('Ball1', occ_one, occ_two, [0, 0, 0], [0, 0, 0])
        joint.jointMotion = FakeJointMotion(6)
        _, msg = Joint.make_joints_dict(FakeRoot([joint]), 'ok')
        self.assertIn('Ball', msg)

    def test_origin2center_of_mass(self):
        out = utils.origin2center_of_mass([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0)
        # Ixx about CoM should drop by y^2+z^2 = 0 in this setup for y/z terms.
        self.assertEqual(len(out), 6)

    def test_write_urdf_roundtrip(self):
        joints_dict = {
            'Rev1': {
                'type': 'revolute', 'axis': [0, 0, 1],
                'upper_limit': 1.0, 'lower_limit': -1.0,
                'parent': 'base_link', 'child': 'link1',
                'xyz': [0.0, 0.0, 1.0],
            }
        }
        inertial_dict = {
            'base_link': {'center_of_mass': [0, 0, 0.5], 'mass': 1.0,
                          'inertia': [0.1] * 6},
            'link1': {'center_of_mass': [0, 0, 1.5], 'mass': 2.0,
                      'inertia': [0.2] * 6},
        }
        with tempfile.TemporaryDirectory() as tmp:
            save_dir = os.path.join(tmp, 'bot_description')
            os.makedirs(save_dir)
            Write.write_urdf(joints_dict, {}, inertial_dict,
                             'bot_description', 'bot', save_dir)
            xacro = os.path.join(save_dir, 'urdf', 'bot.xacro')
            self.assertTrue(os.path.exists(xacro))
            with open(xacro) as fh:
                text = fh.read()
            self.assertIn('joint name="Rev1"', text)
            self.assertIn('link name="link1"', text)


if __name__ == '__main__':
    unittest.main()
