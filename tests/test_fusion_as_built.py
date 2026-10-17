import importlib
import math
from pathlib import Path
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakeMatrix, FakePoint, identity_at
from tests.test_fusion_inputs import Body, design, fixed, occurrence, root


class FusionAsBuiltMotionTest(unittest.TestCase):
    def test_supported_ordinary_and_as_built_motions_preserve_frames_limits_and_pose(self):
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        for as_built in (False, True):
            for kind, bounded, expected_type, position, expected_limits in (
                    (1, True, 'revolute', .25, [-1.25, 1.75]),
                    (1, False, 'continuous', .25, None),
                    (2, True, 'prismatic', 5.0, [-.05, .15])):
                with self.subTest(as_built=as_built, kind=expected_type):
                    base = occurrence('base:1', [Body('base', 0)], component='base_link')
                    arm = occurrence('arm:1', [Body('arm', 10)], matrix=[
                        0, -1, 0, 10, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
                    joint = fixed('motion', arm, base)
                    current = FakeMatrix(identity_at([30, 40, 50]))
                    joint.geometryTwoTransform = current
                    joint.transform = current
                    joint.geometry = None
                    limits = types.SimpleNamespace(isMaximumValueEnabled=bounded,
                        isMinimumValueEnabled=bounded, minimumValue=-1 if kind == 1 else 0,
                        maximumValue=2 if kind == 1 else 20)
                    if kind == 1:
                        joint.jointMotion = types.SimpleNamespace(jointType=kind,
                            rotationAxisVector=FakePoint([1, 0, 0]), rotationValue=position,
                            rotationLimits=limits)
                    else:
                        joint.jointMotion = types.SimpleNamespace(jointType=kind,
                            slideDirectionVector=FakePoint([1, 0, 0]), slideValue=position,
                            slideLimits=limits)
                    component = root([base, arm], joints=[] if as_built else [joint])
                    component.allAsBuiltJoints = [joint] if as_built else []
                    fixture, _ = design(component)
                    with tempfile.TemporaryDirectory() as directory:
                        package = entry.export_design(fixture, Path(directory) / 'motion_description')
                        robot = ET.parse(package / 'urdf/motion.xacro').getroot()
                        result = robot.find('joint')
                        self.assertEqual(result.get('type'), expected_type)
                        self.assertEqual(result.find('origin').get('xyz'), '0.3 0.4 0.5')
                        self.assertAlmostEqual(float(result.find('origin').get('rpy').split()[2]), math.pi / 2)
                        self.assertEqual(result.find('axis').get('xyz'), '0 -1 0')
                        if expected_limits is not None:
                            for key, value in zip(('lower', 'upper'), expected_limits):
                                self.assertAlmostEqual(float(result.find('limit').get(key)), value)
                        else:
                            self.assertNotIn('lower', result.find('limit').attrib)
                            self.assertNotIn('upper', result.find('limit').attrib)
                        actual_position = (joint.jointMotion.rotationValue if kind == 1
                                           else joint.jointMotion.slideValue)
                        self.assertEqual(actual_position, position)
