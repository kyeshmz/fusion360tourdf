import copy
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from URDF_Exporter.core import Write
from URDF_Exporter.utils import config


def mesh_bytes():
    return (b'fusion360tourdf test'.ljust(80, b'\0') + struct.pack('<I', 1)
            + struct.pack('<12fH', 0, 0, 1, 0, 0, 0, 100, 0, 0, 0, 100, 0, 0))


def write_meshes(targets):
    for target in targets.values():
        target.write_bytes(mesh_bytes())


def model():
    joints = {
        'shoulder': {'parent': 'base_link', 'child': 'arm_1', 'type': 'revolute',
                     'xyz': [1, 2, 3], 'rpy': [0, 0, math.pi / 2],
                     'axis': [0, -1, 0], 'lower_limit': -1, 'upper_limit': 1,
                     'effort': 12, 'velocity': 2},
        'tool_mount': {'parent': 'arm_1', 'child': 'tool', 'type': 'fixed',
                       'xyz': [1, 3, 3], 'rpy': [0, 0, 0]},
    }
    inertial = {
        'base_link': {'mass': 2, 'center_of_mass': [0, 0, 0],
                      'inertia': [1e-9, 2e-9, 3e-9, 0, 0, 0]},
        'arm_1': {'mass': 3, 'center_of_mass': [2, 2, 3],
                  'inertia': [2, 3, 4, 0.2, 0.4, 0.3],
                  'color_rgba': [1, 0.2, 0.1, 1],
                  'bbox_center_m': [2, 2, 3], 'bbox_size_m': [0.2, 0.4, 0.6]},
        'tool': {'mass': 1, 'center_of_mass': [1, 3, 3],
                 'inertia': [0.1, 0.1, 0.1, 0, 0, 0]},
    }
    return joints, inertial


class PackageTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.destination = Path(self.tmp.name) / 'bot_description'
        self.joints, self.inertial = model()

    def export(self, **kwargs):
        return Write.write_ros2_package(
            self.destination, 'bot', self.joints, self.inertial,
            write_meshes=kwargs.pop('write_meshes', write_meshes), **kwargs)

    def vector(self, element, key, expected):
        actual = [float(v) for v in element.get(key).split()]
        self.assertEqual(len(actual), len(expected))
        for value, want in zip(actual, expected):
            self.assertAlmostEqual(value, want, places=12)

    def test_complete_package_and_control_membership(self):
        requests = []

        def callback(targets):
            self.assertFalse(self.destination.exists())
            requests.append({name: path.name for name, path in targets.items()})
            self.assertTrue(all(isinstance(path, Path) for path in targets.values()))
            write_meshes(targets)

        result = self.export(write_meshes=callback)
        self.assertEqual(result, self.destination)
        self.assertEqual(requests, [{'base_link': 'base_link.stl',
                                     'arm_1': 'arm_1.stl', 'tool': 'tool.stl'}])
        self.assertEqual({str(p.relative_to(result)) for p in result.rglob('*') if p.is_file()}, {
            'urdf/bot.xacro', 'meshes/base_link.stl', 'meshes/arm_1.stl', 'meshes/tool.stl',
            'viewers/display.launch.py', 'launch/control.launch.py', 'viewers/display.rviz',
            'config/controllers.yaml', 'config/robot_data.yaml', 'package.xml', 'CMakeLists.txt',
        })
        robot = ET.parse(result / 'urdf/bot.xacro').getroot()
        self.assertEqual(robot.tag, 'robot')
        self.assertEqual([e.get('name') for e in robot.findall('link/collision/..')],
                         ['base_link', 'arm_1', 'tool'])
        self.assertEqual(robot.findall('collision'), [])
        self.assertEqual(robot.findall('.//{http://www.ros.org/wiki/xacro}include'), [])
        for mesh in robot.findall('.//mesh'):
            self.assertTrue(mesh.get('filename').startswith('package://bot_description/meshes/'))
            self.assertTrue((result / mesh.get('filename').split('bot_description/')[1]).is_file())
        self.assertEqual(robot.findtext('ros2_control/hardware/plugin'), 'mock_components/GenericSystem')
        self.assertEqual([j.get('name') for j in robot.findall('ros2_control/joint')], ['shoulder'])
        self.assertEqual(robot.findtext("ros2_control/joint/state_interface[@name='position']/param[@name='initial_value']"), '0')
        controllers = json.loads((result / 'config/controllers.yaml').read_text())
        self.assertEqual(controllers['joint_trajectory_controller']['ros__parameters']['joints'], ['shoulder'])
        self.assertEqual(controllers['joint_trajectory_controller']['ros__parameters']['command_interfaces'], ['position'])
        self.assertEqual(controllers['controller_manager']['ros__parameters']['joint_trajectory_controller'],
                         {'type': 'joint_trajectory_controller/JointTrajectoryController'})
        metadata = ET.parse(result / 'package.xml').getroot()
        self.assertEqual(metadata.findtext('name'), 'bot_description')
        self.assertEqual(metadata.findtext('export/build_type'), 'ament_cmake')
        self.assertIn('install(DIRECTORY urdf meshes launch config viewers DESTINATION share/${PROJECT_NAME})',
                      (result / 'CMakeLists.txt').read_text())
        deps = [e.text for e in metadata.findall('exec_depend')]
        self.assertIn('joint_state_publisher', deps)
        self.assertIn('controller_manager', deps)
        self.assertFalse(any('gazebo' in d or 'ros_gz' in d for d in deps))
        display = (result / 'viewers/display.launch.py').read_text()
        control = (result / 'launch/control.launch.py').read_text()
        compile(display, 'display.launch.py', 'exec')
        compile(control, 'control.launch.py', 'exec')
        self.assertIn("UnlessCondition(LaunchConfiguration('gui'))", display)
        self.assertIn("IfCondition(LaunchConfiguration('rviz'))", display)
        for launch in (display, control):
            self.assertIn("os.path.join(pkg, 'viewers', 'display.rviz')", launch)
            self.assertNotIn("os.path.join(pkg, 'config', 'display.rviz')", launch)
        self.assertNotIn('joint_state_publisher', control)
        self.assertIn("('~/robot_description', '/robot_description')", control)
        self.assertIn("('robot_description', '/robot_description')", control)

    def test_frames_com_inertia_and_axes_against_literals(self):
        result = self.export()
        robot = ET.parse(result / 'urdf/bot.xacro').getroot()
        arm = robot.find("link[@name='arm_1']")
        self.vector(arm.find('visual/origin'), 'xyz', [-2, 1, -3])
        self.vector(arm.find('visual/origin'), 'rpy', [0, 0, -math.pi / 2])
        self.vector(arm.find('collision/origin'), 'xyz', [-2, 1, -3])
        self.vector(arm.find('inertial/origin'), 'xyz', [0, -1, 0])
        tensor = arm.find('inertial/inertia')
        for key, expected in {'ixx': 3, 'iyy': 2, 'izz': 4, 'ixy': -0.2, 'iyz': -0.3, 'ixz': 0.4}.items():
            self.assertAlmostEqual(float(tensor.get(key)), expected, places=12)
        self.vector(robot.find("joint[@name='shoulder']/axis"), 'xyz', [0, -1, 0])
        self.vector(robot.find("joint[@name='tool_mount']/origin"), 'xyz', [1, 0, 0])
        self.assertEqual(float(robot.find("link[@name='base_link']/inertial/inertia").get('ixx')), 1e-9)

    def test_primitive_collision_is_nested_and_root_box_is_rotated_into_link(self):
        result = self.export(config={'collision_method': 'primitive_box'})
        arm = ET.parse(result / 'urdf/bot.xacro').find("link[@name='arm_1']")
        self.vector(arm.find('collision/origin'), 'xyz', [0, -1, 0])
        self.vector(arm.find('collision/origin'), 'rpy', [0, 0, -math.pi / 2])
        self.vector(arm.find('collision/geometry/box'), 'size', [0.2, 0.4, 0.6])

    def test_fixed_only_and_single_link_have_no_control_assets(self):
        for single in (False, True):
            with self.subTest(single=single):
                self.joints['shoulder']['type'] = 'fixed'
                if single:
                    self.joints = {}
                    self.inertial = {'base_link': self.inertial['base_link']}
                self.destination = Path(self.tmp.name) / ('single_description' if single else 'fixed_description')
                result = self.export()
                robot = ET.parse(result / 'urdf/bot.xacro').getroot()
                self.assertEqual(len(robot.findall('link')), 1 if single else 3)
                self.assertIsNone(robot.find('ros2_control'))
                self.assertFalse((result / 'launch/control.launch.py').exists())
                self.assertFalse((result / 'config/controllers.yaml').exists())
                self.assertEqual({p.name for p in (result / 'viewers').iterdir()},
                                 {'display.launch.py', 'display.rviz'})
                self.assertEqual(list((result / 'launch').iterdir()), [])
                self.assertEqual({p.name for p in (result / 'config').iterdir()}, {'robot_data.yaml'})

    def test_closing_joints_keep_full_metadata_outside_tree(self):
        self.joints['loop'] = dict(self.joints['shoulder'], parent='tool', child='base_link')
        result = self.export()
        robot = ET.parse(result / 'urdf/bot.xacro').getroot()
        self.assertEqual([e.get('name') for e in robot.findall('joint')], ['shoulder', 'tool_mount'])
        sidecar = json.loads((result / 'config/robot_data.yaml').read_text())
        self.assertEqual(sidecar['closing_joints'][0]['joint'], 'loop')
        self.assertEqual(sidecar['closing_joints'][0]['axis'], [0, -1, 0])
        self.assertEqual(sidecar['closing_joints'][0]['xyz'], [1, 2, 3])

    def test_loop_selection_remains_root_connected_in_reverse_input_order(self):
        closing = dict(self.joints['shoulder'], parent='tool')
        self.joints = dict(loop=closing, **self.joints)
        result = self.export()
        robot = ET.parse(result / 'urdf/bot.xacro').getroot()
        self.assertEqual([e.get('name') for e in robot.findall('joint')], ['shoulder', 'tool_mount'])
        sidecar = json.loads((result / 'config/robot_data.yaml').read_text())
        self.assertEqual([j['joint'] for j in sidecar['closing_joints']], ['loop'])

    def test_destination_created_during_callback_is_not_replaced(self):
        def callback(targets):
            write_meshes(targets)
            self.destination.mkdir()
            (self.destination / 'keep.txt').write_text('created while exporting')

        with self.assertRaises(FileExistsError):
            self.export(write_meshes=callback)
        self.assertEqual((self.destination / 'keep.txt').read_text(), 'created while exporting')
        self.assertEqual(list(Path(self.tmp.name).iterdir()), [self.destination])

    def test_incomplete_or_disconnected_models_are_rejected_before_callback(self):
        cases = []
        joints, inertial = model()
        inertial['loose'] = copy.deepcopy(inertial['tool'])
        cases.append((joints, inertial))
        joints, inertial = model()
        joints['shoulder']['parent'] = 'ghost'
        cases.append((joints, inertial))
        joints, inertial = model()
        del joints['shoulder']['child']
        cases.append((joints, inertial))
        joints, inertial = model()
        del inertial['arm_1']['inertia']
        cases.append((joints, inertial))
        joints, inertial = model()
        joints['shoulder']['axis'] = [0, 0, 0]
        cases.append((joints, inertial))
        for joints, inertial in cases:
            with self.subTest(joints=joints, inertial=inertial):
                self.joints, self.inertial = joints, inertial
                with self.assertRaises(ValueError):
                    self.export(write_meshes=lambda _: self.fail('invalid model reached mesh export'))
                self.assertFalse(self.destination.exists())

    def test_existing_destination_is_preserved(self):
        self.destination.mkdir()
        sentinel = self.destination / 'keep.txt'
        sentinel.write_text('original')
        with self.assertRaises(FileExistsError):
            self.export()
        self.assertEqual(sentinel.read_text(), 'original')
        self.assertEqual(list(self.destination.iterdir()), [sentinel])

    def test_callback_failure_missing_mesh_and_corrupt_binary_never_publish(self):
        def failure(_):
            raise RuntimeError('Fusion refused export')

        def corrupt(targets):
            for target in targets.values():
                target.write_bytes(mesh_bytes()[:-1])

        def nonfinite(targets):
            for target in targets.values():
                data = bytearray(mesh_bytes())
                struct.pack_into('<f', data, 96, float('nan'))
                target.write_bytes(data)

        for callback, error in ((failure, RuntimeError), (lambda _: None, ValueError),
                                (corrupt, ValueError), (nonfinite, ValueError),
                                (lambda ts: [p.write_bytes(b'') for p in ts.values()], ValueError)):
            with self.subTest(callback=callback):
                with self.assertRaises(error):
                    self.export(write_meshes=callback)
                self.assertFalse(self.destination.exists())
                self.assertEqual(list(Path(self.tmp.name).iterdir()), [])

    def test_invalid_names_and_malformed_config_fail_explicitly(self):
        for robot in ('../escape', 'BadRobot', 'has space', ''):
            with self.subTest(robot=robot), self.assertRaises(ValueError):
                Write.write_ros2_package(self.destination, robot, self.joints, self.inertial,
                                         write_meshes=write_meshes)
        for invalid in ({'use_legacy_copy': 'false'}, {'ros_version': '1'},
                        {'effort': 'x'}, {'velocity': -1}, {'effort': float('nan')},
                        {'collision_method': 'mesh'}, {'mesh_refinement': 'invalid'}):
            with self.subTest(config=invalid), self.assertRaises(ValueError):
                self.export(config=invalid)
        (Path(self.tmp.name) / 'xacro_export.toml').write_text('effort = [broken')
        with self.assertRaises(ValueError):
            config.load_config(self.tmp.name)

    def test_writer_imports_without_fusion(self):
        result = subprocess.run([sys.executable, '-c',
            "import sys; from URDF_Exporter.core.Write import write_ros2_package; "
            "assert 'adsk' not in sys.modules; print(write_ros2_package.__name__)"],
            capture_output=True, text=True, cwd=Path(__file__).resolve().parents[1])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), 'write_ros2_package')

    def test_programmatic_config_without_tomllib(self):
        result = subprocess.run([sys.executable, '-c',
            "import sys; sys.modules['tomllib'] = None; "
            "from URDF_Exporter.core.Write import write_ros2_package; "
            "from URDF_Exporter.utils.config import load_config; "
            "print(load_config(overrides={'effort': 7.0})['effort'])"],
            capture_output=True, text=True, cwd=Path(__file__).resolve().parents[1])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '7.0')


if __name__ == '__main__':
    unittest.main()
