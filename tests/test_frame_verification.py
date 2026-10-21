import copy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from tests.fixtures import VARIANTS, fixture_oracle, fixture_variants, generate_fixture
from tools.validate_urdf import lint_file
from tools.verify_loaders import expected_link_kinds, mesh_path


class FrameVerificationTests(unittest.TestCase):
    def test_optional_fixture_preserves_default_contract_and_literal_motion(self):
        self.assertEqual(VARIANTS, ('articulated', 'fixed', 'boxes'))
        self.assertEqual(fixture_variants(), VARIANTS)
        self.assertEqual(fixture_variants('frames'), ('frames',))
        with tempfile.TemporaryDirectory() as directory:
            package = generate_fixture(directory, 'frames')
            path = package / 'urdf/fixture_frames.urdf'
            robot = ET.parse(path).getroot()
            expected = fixture_oracle('frames')
            kinds = expected_link_kinds(robot, package, expected['zero'])
            self.assertEqual(kinds, {'base_link': 'frame', 'arm': 'frame', 'mount': 'physical',
                                     'carriage': 'physical', 'wheel': 'physical'})
            self.assertEqual(len(robot.findall('ros2_control/joint')), 3)
            self.assertEqual(expected['moved']['link_poses']['arm'],
                             [[0, 0, 1, .2], [0, -1, 0, .1], [1, 0, 0, .3], [0, 0, 0, 1]])
            self.assertEqual(set(expected['moved']['bounds']), {'mount', 'carriage', 'wheel'})
            self.assertEqual(sorted(p.name for p in (package / 'meshes').iterdir()),
                             ['carriage.stl', 'mount.stl', 'wheel.stl'])
            self.assertEqual(lint_file(path, package / 'meshes', link_kinds=kinds), ([], []))
            self.assertTrue(lint_file(path, package / 'meshes')[1])

    def test_missing_physical_geometry_and_frame_physics_do_not_pass_as_frames(self):
        for mutation in ('physical_visual', 'physical_collision', 'physical_inertia', 'frame_mass', 'frame_visual'):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                package = generate_fixture(directory, 'frames')
                path = package / 'urdf/fixture_frames.urdf'
                robot = ET.parse(path).getroot()
                if mutation.startswith('physical'):
                    tag = {'physical_visual': 'visual', 'physical_collision': 'collision', 'physical_inertia': 'inertial'}[mutation]
                    link = robot.find("link[@name='mount']")
                    link.remove(link.find(tag))
                else:
                    frame = robot.find("link[@name='arm']")
                    frame.append(copy.deepcopy(robot.find("link[@name='mount']/" + ('inertial' if mutation == 'frame_mass' else 'visual'))))
                with self.assertRaises(AssertionError):
                    expected_link_kinds(robot, package, fixture_oracle('frames')['zero'])
                ET.ElementTree(robot).write(path)
                errors, warnings = lint_file(path, package / 'meshes', link_kinds=fixture_oracle('frames')['zero']['link_kinds'])
                self.assertTrue(errors or warnings)

    def test_missing_second_collision_resource_is_checked_with_explicit_frame_kinds(self):
        with tempfile.TemporaryDirectory() as directory:
            package = generate_fixture(directory, 'frames')
            path = package / 'urdf/fixture_frames.urdf'
            robot = ET.parse(path).getroot()
            link = robot.find("link[@name='mount']")
            collision = copy.deepcopy(link.find('collision'))
            uri = 'package://fixture_frames_description/meshes/missing-second.stl'
            collision.find('geometry/mesh').set('filename', uri)
            link.append(collision)
            ET.ElementTree(robot).write(path)
            errors, _ = lint_file(path, package / 'meshes', link_kinds=fixture_oracle('frames')['zero']['link_kinds'])
            self.assertTrue(any('missing-second' in error for error in errors))
            with self.assertRaisesRegex(AssertionError, 'Missing or external mesh'):
                mesh_path(uri, package, path)

    def test_component_frame_receipt_and_assembly_membership_are_strict(self):
        with tempfile.TemporaryDirectory() as directory:
            package = generate_fixture(directory, 'frames')
            component = ET.parse(package / 'urdf/components/arm.urdf').getroot()
            self.assertEqual(expected_link_kinds(component, package), {'arm': 'frame'})
            robot = ET.parse(package / 'urdf/fixture_frames.urdf').getroot()
            robot.remove(robot.find("link[@name='mount']"))
            with self.assertRaisesRegex(AssertionError, 'membership'):
                expected_link_kinds(robot, package)

    def test_native_analytic_verifier_requires_independent_frame_pose_and_all_resources(self):
        from tests.test_fusion_verifier import literal_package
        from tools.verify_fusion.verification import QUARTER_TURN, verify_package
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            path = package / 'urdf/literal.xacro'
            robot = ET.parse(path).getroot()
            arm = robot.find("link[@name='arm']")
            for element in list(arm):
                arm.remove(element)
            (package / 'meshes/arm.stl').unlink()
            ET.ElementTree(robot).write(path)
            frames = {'arm': (QUARTER_TURN, (.1, .2, .3))}
            boxes.pop('arm')
            cad.pop('arm')
            evidence = verify_package(package, boxes, cad, joints, frame_poses=frames)
            self.assertTrue(any('frame rotation' in item['name'] for item in evidence))
            ET.SubElement(arm, 'inertial')
            ET.ElementTree(robot).write(path)
            with self.assertRaises(AssertionError):
                verify_package(package, boxes, cad, joints, frame_poses=frames)
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            path = package / 'urdf/literal.xacro'
            robot = ET.parse(path).getroot()
            link = robot.find("link[@name='arm']")
            second = copy.deepcopy(link.find('collision'))
            second.find('geometry/mesh').set('filename', 'package://literal_description/meshes/missing.stl')
            link.append(second)
            ET.ElementTree(robot).write(path)
            with self.assertRaises(FileNotFoundError):
                verify_package(package, boxes, cad, joints)
