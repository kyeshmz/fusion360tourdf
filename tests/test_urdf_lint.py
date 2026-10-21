"""Tests for the offline URDF linter + generated-file validation."""
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from tests.test_package import write_meshes

adsk = types.ModuleType('adsk')
adsk_core = types.ModuleType('adsk.core')
adsk_fusion = types.ModuleType('adsk.fusion')


class _FakeCalcAccuracy:
    VeryHighCalculationAccuracy = 1


adsk_fusion.CalculationAccuracy = _FakeCalcAccuracy
adsk_fusion.SearchRefinementSettings = types.SimpleNamespace(MeshRefinementLow=0)
adsk_fusion.JointOrigin = type('JointOrigin', (), {})
adsk.core = adsk_core
adsk.fusion = adsk_fusion
sys.modules.setdefault('adsk', adsk)
sys.modules.setdefault('adsk.core', adsk_core)
sys.modules.setdefault('adsk.fusion', adsk_fusion)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

sys.path.insert(0, os.path.join(REPO_ROOT, 'tools'))
import validate_urdf  # noqa: E402
from URDF_Exporter.core import Write  # noqa: E402


class LinterTest(unittest.TestCase):
    def test_example_model_passes(self):
        xacro = os.path.join(REPO_ROOT, 'Example', 'Basic_Robot_description',
                             'urdf', 'Basic_Robot.xacro')
        meshes = os.path.join(REPO_ROOT, 'Example', 'Basic_Robot_description', 'meshes')
        errors, warnings = validate_urdf.lint_file(xacro, meshes)
        self.assertEqual(errors, [])

    def test_generated_output_passes(self):
        joints = {
            'J1': {'parent': 'base_link', 'child': 'arm', 'type': 'revolute',
                   'axis': [0, 0, 1], 'upper_limit': 1.0, 'lower_limit': -1.0,
                   'xyz': [0.0, 0.0, 1.0], 'rpy': [0, 0, 0]},
        }
        inertial = {
            'base_link': {'center_of_mass': [0, 0, 0.5], 'mass': 1.0,
                          'inertia': [0.1, 0.1, 0.1, 0, 0, 0], 'color_rgba': None,
                          'bbox_center_m': None, 'bbox_size_m': None},
            'arm': {'center_of_mass': [0, 0, 1.5], 'mass': 2.0,
                    'inertia': [0.2, 0.2, 0.2, 0, 0, 0], 'color_rgba': None,
                    'bbox_center_m': None, 'bbox_size_m': None},
        }
        with tempfile.TemporaryDirectory() as tmp:
            save_dir = Path(tmp) / 'bot_description'
            Write.write_ros2_package(save_dir, 'bot', joints, inertial, write_meshes=write_meshes)
            errors, _ = validate_urdf.lint_file(
                os.path.join(save_dir, 'urdf', 'bot.xacro'),
                os.path.join(save_dir, 'meshes'))
            self.assertEqual(errors, [])

    def test_linter_catches_broken_tree(self):
        with tempfile.TemporaryDirectory() as tmp:
            bad = os.path.join(tmp, 'bad.urdf')
            with open(bad, 'w') as fh:
                fh.write('<robot name="b"><link name="a"/>'
                         '<joint name="j" type="fixed">'
                         '<parent link="a"/><child link="ghost"/>'
                         '<origin xyz="0 0 0" rpy="0 0 0"/></joint></robot>')
            errors, _ = validate_urdf.lint_file(bad)
            self.assertTrue(any('ghost' in e for e in errors))


if __name__ == '__main__':
    unittest.main()
