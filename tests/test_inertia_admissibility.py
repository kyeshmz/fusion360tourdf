import importlib
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakePoint
from tests.test_fusion_inputs import Body, design, fixed, occurrence, root
from tests.test_package import write_meshes
from URDF_Exporter.core import Write


INVALID_TENSORS = {
    'indefinite': [1, 1, 1, 2, 0, 0],
    'principal_triangle': [1, 1, 3, 0, 0, 0],
    'rotated_principal_triangle': [5 / 3, 5 / 3, 5 / 3, 2 / 3, 2 / 3, 2 / 3],
}
SCALES = (1, 1e-12, 1e-200, 1e200)


def body_with_properties(name, center_cm, central_kg_cm2):
    body = Body(name, center_cm[0])
    x, y, z = center_cm
    shift = [y*y + z*z, x*x + z*z, x*x + y*y, -x*y, -y*z, -x*z]
    origin = [value + term for value, term in zip(central_kg_cm2, shift)]
    body.properties.centerOfMass = FakePoint(center_cm)
    body.properties.getXYZMomentsOfInertia = lambda: (True, *origin)
    return body


class InertiaAdmissibilityTest(unittest.TestCase):
    def assertTensor(self, path, link, expected, relative=2e-13):
        tensor = ET.parse(path).find("link[@name='%s']/inertial/inertia" % link)
        actual = [float(tensor.get(key)) for key in ('ixx', 'iyy', 'izz', 'ixy', 'iyz', 'ixz')]
        for value, want in zip(actual, expected):
            if want == 0:
                self.assertEqual(value, 0)
            else:
                self.assertTrue(math.isclose(value, want, rel_tol=relative, abs_tol=0),
                                (value, want))

    def test_writer_rejects_nonphysical_tensors_before_meshes_or_publication(self):
        for label, tensor in INVALID_TENSORS.items():
            for scale in SCALES:
                with self.subTest(tensor=label, scale=scale), tempfile.TemporaryDirectory() as directory:
                    destination = Path(directory) / 'bad_description'
                    calls = []

                    def meshes(targets):
                        calls.append(True)
                        write_meshes(targets)

                    with self.assertRaisesRegex(ValueError, 'base_link.*inertia'):
                        Write.write_ros2_package(destination, 'bad', {}, {
                            'base_link': {'mass': 1, 'center_of_mass': [0, 0, 0],
                                          'inertia': [v * scale for v in tensor]}}, write_meshes=meshes)
                    self.assertEqual(calls, [])
                    self.assertEqual(list(Path(directory).iterdir()), [])

    def test_export_rejects_nonphysical_cad_tensors_before_meshes_or_publication(self):
        exporter = importlib.import_module('URDF_Exporter.URDF_Exporter')
        for label, tensor in INVALID_TENSORS.items():
            for scale in SCALES:
                with self.subTest(tensor=label, scale=scale), tempfile.TemporaryDirectory() as directory:
                    body = body_with_properties('invalid', [0, 0, 0], [v * scale for v in tensor])
                    fixture, exported = design(root(bodies=[body]))
                    with self.assertRaisesRegex(ValueError, 'base_link.*inertia'):
                        exporter.export_design(fixture, Path(directory) / 'bad_description')
                    self.assertEqual(exported, [])
                    self.assertEqual(list(Path(directory).iterdir()), [])

    def test_writer_preserves_tiny_rotated_all_six_terms(self):
        for scale in (1e-12, 1e-200):
            with self.subTest(scale=scale), tempfile.TemporaryDirectory() as directory:
                tensor = [v * scale / 9 for v in (47, 41, 38, -8, -10, -2)]
                entry = {'mass': 1, 'center_of_mass': [0, 0, 0], 'inertia': tensor}
                destination = Path(directory) / 'tiny_description'
                Write.write_ros2_package(destination, 'tiny', {
                    'mount': {'type': 'fixed', 'parent': 'base_link', 'child': 'tiny',
                              'xyz': [0, 0, 0], 'rpy': [0, 0, math.pi / 2]}},
                    {'base_link': entry, 'tiny': entry}, write_meshes=write_meshes)
                self.assertTensor(destination / 'urdf/tiny.urdf', 'base_link', tensor)
                self.assertTensor(destination / 'urdf/tiny.urdf', 'tiny',
                                  [v * scale / 9 for v in (41, 47, 38, 8, 2, -10)])

    def test_export_preserves_tiny_si_conversion_and_all_six_terms(self):
        exporter = importlib.import_module('URDF_Exporter.URDF_Exporter')
        with tempfile.TemporaryDirectory() as directory:
            tensor = [v * 1e-12 / 9 for v in (47, 41, 38, -8, -10, -2)]
            body = body_with_properties('tiny', [0, 0, 0], [v * 10000 for v in tensor])
            fixture, _ = design(root(bodies=[body]))
            destination = Path(directory) / 'tiny_description'
            exporter.export_design(fixture, destination)
            self.assertTensor(destination / 'urdf/tiny.urdf', 'base_link', tensor)

    def test_degenerate_and_triangle_boundary_tensors_remain_valid(self):
        for tensor in ([0, 1e-200, 1e-200, 0, 0, 0], [1e-12, 2e-12, 3e-12, 0, 0, 0],
                       [8e-12 / 9, 5e-12 / 9, 5e-12 / 9, -2e-12 / 9, -4e-12 / 9, -2e-12 / 9]):
            with self.subTest(tensor=tensor), tempfile.TemporaryDirectory() as directory:
                destination = Path(directory) / 'boundary_description'
                Write.write_ros2_package(destination, 'boundary', {}, {
                    'base_link': {'mass': 1, 'center_of_mass': [0, 0, 0], 'inertia': tensor}},
                    write_meshes=write_meshes)
                self.assertTensor(destination / 'urdf/boundary.urdf', 'base_link', tensor)

    def test_far_offset_preserves_central_moments_representable_in_cad_result(self):
        exporter = importlib.import_module('URDF_Exporter.URDF_Exporter')
        for offset, central in ((1024.0, 2.0**-20), (1048576.0, 2.0**-10)):
            with self.subTest(offset_cm=offset), tempfile.TemporaryDirectory() as directory:
                tensor = [central, central, central, 0, 0, 0]
                fixture, _ = design(root(bodies=[body_with_properties('offset', [offset, 0, 0], tensor)]))
                destination = Path(directory) / 'offset_description'
                exporter.export_design(fixture, destination)
                self.assertTensor(destination / 'urdf/offset.urdf', 'base_link',
                                  [v / 10000 for v in tensor])

    def test_far_offset_aggregation_uses_local_body_displacements(self):
        exporter = importlib.import_module('URDF_Exporter.URDF_Exporter')
        central = 2.0**-20
        bodies = [body_with_properties(str(index), [1024 + index / 32, 0, 0],
                                       [central, central, central, 0, 0, 0]) for index in (0, 1)]
        with tempfile.TemporaryDirectory() as directory:
            fixture, _ = design(root(bodies=bodies))
            destination = Path(directory) / 'aggregate_description'
            exporter.export_design(fixture, destination)
            transverse = (2 * central + 2 * (1 / 64)**2) / 10000
            self.assertTensor(destination / 'urdf/aggregate.urdf', 'base_link',
                              [2 * central / 10000, transverse, transverse, 0, 0, 0])
            origin = ET.parse(destination / 'urdf/aggregate.urdf').find('link/inertial/origin')
            self.assertEqual([float(v) for v in origin.get('xyz').split()], [1024.015625 / 100, 0, 0])

    def test_export_combines_si_conversion_aggregation_shift_and_frame_rotation(self):
        exporter = importlib.import_module('URDF_Exporter.URDF_Exporter')
        unit = 2.0**-20
        tensor = [value * unit for value in (5, 6, 7, 1, 1, 1)]
        bodies = [body_with_properties(str(index), [1 + index * 2.0**-10, 2, 3], tensor)
                  for index in (0, 1)]
        child = occurrence('tiny:1', bodies, matrix=[0, -1, 0, 0, 1, 0, 0, 0,
                                                   0, 0, 1, 0, 0, 0, 0, 1])
        joint = fixed('mount', child, None)
        fixture, _ = design(root([child], bodies=[Body('base', 0)], joints=[joint]))
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'combined_description'
            exporter.export_design(fixture, destination)
            self.assertTensor(destination / 'urdf/combined.urdf', 'tiny_1',
                              [value * unit / 10000 for value in (12.5, 10, 14.5, -2, -2, 2)])
            origin = ET.parse(destination / 'urdf/combined.urdf').find("link[@name='tiny_1']/inertial/origin")
            for actual, expected in zip(map(float, origin.get('xyz').split()),
                                        [.02, -(1 + 2.0**-11) / 100, .03]):
                self.assertTrue(math.isclose(actual, expected, rel_tol=2e-13, abs_tol=0))


if __name__ == '__main__':
    unittest.main()
