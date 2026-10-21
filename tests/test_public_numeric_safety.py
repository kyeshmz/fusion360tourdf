"""Public-writer regressions for the final review's numeric safety findings."""

import copy
from fractions import Fraction
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from tests.test_package import write_meshes
from URDF_Exporter.core import coupling
from URDF_Exporter.core.Write import write_ros2_package
from URDF_Exporter.utils import transforms as tf
from tools.validate_urdf import lint_file


def model(axis=(0., 0., 1.)):
    joints = {'turn': dict(parent='base_link', child='arm', type='revolute', xyz=[.01, -.02, .03],
                           rpy=[0., 0., 0.], axis=list(axis), lower_limit=-1., upper_limit=1.)}
    links = {name: dict(mass=1., center_of_mass=[.02, -.01, .04],
                       inertia=[.002, .003, .004, 0., 0., 0.]) for name in ('base_link', 'arm')}
    return joints, links


def reject_json_constant(value):
    raise AssertionError('Nonfinite JSON constant: ' + value)


class PublicNumericSafetyTests(unittest.TestCase):
    def test_large_finite_axes_emit_the_literal_unit_direction(self):
        cases = (((1e155, 0., 0.), (1., 0., 0.)),
                 ((0., -1e200, 0.), (0., -1., 0.)),
                 ((0., 0., 1e300), (0., 0., 1.)),
                 ((1e308, -1e308, 1e308), (1/math.sqrt(3), -1/math.sqrt(3), 1/math.sqrt(3))))
        policies = ({}, {'output_rotation_preset': 'y_up_to_z_up'},
                    {'minimum_principal_inertia': .005}, {'frame_convention': 'dh_common_normal'})
        with tempfile.TemporaryDirectory() as directory:
            for index, (axis, wanted) in enumerate(cases):
                for policy_index, config in enumerate(policies):
                    with self.subTest(axis=axis, policy=config):
                        joints, links = model(axis)
                        original = copy.deepcopy((joints, links, config))
                        package = Path(directory)/('axis_%d_%d_description' % (index, policy_index))
                        write_ros2_package(package, 'axis', joints, links, write_meshes=write_meshes, config=config)
                        root = ET.parse(package/'urdf/axis.urdf')
                        actual = list(map(float, root.find("joint[@name='turn']/axis").get('xyz').split()))
                        expected = (0., 0., 1.) if config.get('frame_convention') else wanted
                        self.assertAlmostEqual(math.hypot(*actual), 1., places=14)
                        for a, b in zip(actual, expected):
                            self.assertAlmostEqual(a, b, places=14)
                        self.assertFalse(lint_file(package/'urdf/axis.urdf', package/'meshes')[0])
                        self.assertEqual((joints, links, config), original)

    def test_normal_axes_preserve_arithmetic_and_degeneracy_threshold(self):
        for axis in ((1., 2., 3.), (.5, .5, .5), (0., math.sqrt(.5), math.sqrt(.5)), (0., 0., 1.)):
            old_norm = math.sqrt(sum(v*v for v in axis))
            self.assertEqual(tf.normalize(axis), [v/old_norm for v in axis])
        for axis in ((0., 0., 0.), (1e-13, 0., 0.), (float('inf'), 0., 0.), (float('nan'), 0., 0.)):
            self.assertEqual(tf.normalize(axis), [0., 0., 0.])

    def test_nonfinite_numbers_are_not_serializable_numeric_fields(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'finite'):
                tf.format_float(value)
        for value in (0., -0., 1e-200, -1e-200, 1e308):
            self.assertEqual(float(tf.format_float(value)), value)

    def test_unrepresentable_derived_origin_never_publishes(self):
        for config in ({}, {'minimum_principal_inertia': 0.},
                       {'minimum_principal_inertia': .005}, {'output_rotation_preset': 'y_up_to_z_up'}):
            with self.subTest(config=config), tempfile.TemporaryDirectory() as directory:
                joints = {'mount': dict(parent='base_link', child='arm', type='fixed', xyz=[-1e308, 0., 0.])}
                links = {name: dict(mass=1., center_of_mass=[1e308 if name == 'arm' else 0., 0., 0.],
                                   inertia=[1., 1., 1., 0., 0., 0.]) for name in ('base_link', 'arm')}
                before = copy.deepcopy((joints, links, config))
                target = Path(directory)/'overflow_description'
                with self.assertRaisesRegex(ValueError, 'finite|range'):
                    write_ros2_package(target, 'overflow', joints, links, write_meshes=write_meshes, config=config)
                self.assertFalse(target.exists())
                self.assertEqual(list(Path(directory).iterdir()), [])
                self.assertEqual((joints, links, config), before)

    def test_cancelling_mimic_initialization_uses_finite_exact_result(self):
        for sign in (1., -1.):
            with self.subTest(sign=sign), tempfile.TemporaryDirectory() as directory:
                multiplier, offset = sign*1e308, -sign*1e308
                joints = {'driver': dict(parent='base_link', child='arm', type='revolute', xyz=[0., 0., 0.],
                    axis=[0., 0., 1.], lower_limit=2., upper_limit=2.25),
                    'follower': dict(parent='base_link', child='slide', type='prismatic', xyz=[.02, .03, .04],
                    axis=[1., 0., 0.], lower_limit=-1.5e308, upper_limit=1.5e308,
                    mimic={'joint': 'driver', 'multiplier': multiplier, 'offset': offset})}
                links = {name: dict(mass=1., center_of_mass=[0., 0., 0.], inertia=[.02, .03, .04, 0., 0., 0.])
                         for name in ('base_link', 'arm', 'slide')}
                before = copy.deepcopy((joints, links))
                package = Path(directory)/'initial_description'
                write_ros2_package(package, 'initial', joints, links, write_meshes=write_meshes)
                data = json.loads((package/'config/robot_data.yaml').read_text(), parse_constant=reject_json_constant)
                wanted = float(Fraction(multiplier)*2 + Fraction(offset))
                self.assertEqual(data['initial_positions'], {'driver': 2., 'follower': wanted})
                root = ET.parse(package/'urdf/initial.urdf')
                actual = root.find("ros2_control/joint[@name='follower']/state_interface[@name='position']/param").text
                self.assertTrue(math.isfinite(float(actual)))
                self.assertEqual(float(actual), wanted)
                self.assertEqual((joints, links), before)

    def test_affine_evaluation_checks_final_range_without_intermediate_overflow(self):
        for a, q, b in ((1e308, 2., -1e308), (-1e308, 2., 1e308), (0., 1e308, 3.), (2., .25, -.5)):
            with self.subTest(a=a, q=q, b=b):
                self.assertEqual(coupling.evaluate_affine(a, q, b), float(Fraction(a)*Fraction(q)+Fraction(b)))
        for values in ((1e308, 2., 0.), (float('inf'), 0., 0.), (True, 1., 0.)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                coupling.evaluate_affine(*values)

    def test_nonfinite_opaque_metadata_cannot_publish_nonstandard_json(self):
        for value in (float('nan'), float('inf')):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as directory:
                joints, links = model()
                links['base_link']['source_notes'] = [{'name': 'source', 'reason': 'opaque metadata', 'value': value}]
                target = Path(directory)/'metadata_description'
                with self.assertRaisesRegex(ValueError, 'finite|range|JSON'):
                    write_ros2_package(target, 'metadata', joints, links, write_meshes=write_meshes)
                self.assertFalse(target.exists())
                self.assertEqual(list(Path(directory).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
