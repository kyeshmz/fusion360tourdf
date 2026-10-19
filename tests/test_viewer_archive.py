import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from zipfile import ZipFile

from URDF_Exporter.core.Write import write_ros2_package
from tests.test_package import model, write_meshes


class ViewerArchiveTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        joints, inertial = model()
        self.package = write_ros2_package(
            self.root / 'export' / 'bot_description', 'bot', joints, inertial,
            write_meshes=write_meshes)

    def test_viewer_only_zip_contains_only_launch_and_rviz(self):
        viewers = self.package / 'viewers'
        archive_path = self.root / 'viewers.zip'
        with ZipFile(archive_path, 'w') as archive:
            for path in viewers.rglob('*'):
                if path.is_file():
                    archive.write(path, path.relative_to(viewers))
        with ZipFile(archive_path) as archive:
            self.assertEqual(sorted(archive.namelist()), ['display.launch.py', 'display.rviz'])
            archive.extractall(self.root / 'viewer-extract')
        for name in ('display.launch.py', 'display.rviz'):
            self.assertEqual((self.root / 'viewer-extract' / name).read_bytes(),
                             (viewers / name).read_bytes())
        rviz = (self.root / 'viewer-extract/display.rviz').read_text()
        self.assertIn('Fixed Frame: base_link', rviz)
        self.assertIn('Class: rviz_default_plugins/RobotModel', rviz)
        self.assertIn('Description Source: Topic', rviz)
        self.assertIn('Value: /robot_description', rviz)
        self.assertIn('Durability Policy: Transient Local', rviz)
        self.assertIn('Class: rviz_default_plugins/TF', rviz)

    def test_full_package_zip_resolves_launch_assets_after_relocation(self):
        expected = {
            'CMakeLists.txt', 'package.xml', 'urdf/bot.xacro', 'urdf/bot.urdf',
            'urdf/components/base_link.urdf', 'urdf/components/arm_1.urdf', 'urdf/components/tool.urdf',
            'meshes/base_link.stl', 'meshes/arm_1.stl', 'meshes/tool.stl',
            'viewers/display.launch.py', 'viewers/display.rviz',
            'launch/control.launch.py', 'config/controllers.yaml', 'config/robot_data.yaml',
        }
        archive_path = self.root / 'bot_description.zip'
        with ZipFile(archive_path, 'w') as archive:
            for path in self.package.rglob('*'):
                if path.is_file():
                    archive.write(path, path.relative_to(self.package.parent))
        share = self.root / 'relocated-install' / 'share'
        with ZipFile(archive_path) as archive:
            self.assertEqual(sorted(archive.namelist()),
                             sorted('bot_description/' + name for name in expected))
            archive.extractall(share)
        installed = share / 'bot_description'
        for name in expected:
            self.assertEqual((installed / name).read_bytes(), (self.package / name).read_bytes())
        for name in ('display.launch.py', 'display.rviz'):
            self.assertEqual([p.relative_to(installed).as_posix() for p in installed.rglob(name)],
                             ['viewers/' + name])
        robot = ET.parse(installed / 'urdf/bot.xacro').getroot()
        self.assertEqual([link.get('name') for link in robot.findall('link')],
                         ['base_link', 'arm_1', 'tool'])
        for mesh in robot.findall('.//mesh'):
            uri = mesh.get('filename')
            self.assertTrue(uri.startswith('package://bot_description/meshes/'), uri)
            self.assertEqual((installed / uri.removeprefix('package://bot_description/')).read_bytes(),
                             (self.package / uri.removeprefix('package://bot_description/')).read_bytes())
        self.package.parent.rename(self.root / 'hidden-export')
        for relative, rviz_default in (('viewers/display.launch.py', 'true'),
                                       ('launch/control.launch.py', 'false')):
            with self.subTest(launch=relative):
                source = (installed / relative).read_text()
                self.assertNotIn(str(self.package.parent), source)
                self.assertNotIn("'config', 'display.rviz'", source)
                result = subprocess.run(
                    [sys.executable, '-c', r'''
import json
from pathlib import Path
import runpy
import sys
import types
from xml.dom.minidom import parse

package = Path(sys.argv[1])

def module(name, **attributes):
    sys.modules[name] = types.SimpleNamespace(**attributes)

def package_share(name):
    assert name == 'bot_description', name
    return str(package)

module('ament_index_python.packages', get_package_share_directory=package_share)
module('launch', LaunchDescription=list)
module('launch.actions', DeclareLaunchArgument=lambda name, **kw: dict(argument=name, **kw))
module('launch.conditions', IfCondition=lambda value: {'if': value},
       UnlessCondition=lambda value: {'unless': value})
module('launch.substitutions', LaunchConfiguration=lambda name: name)
module('launch_ros.actions', Node=lambda **kw: kw)
module('xacro', process_file=parse)
launch = runpy.run_path(str(package / sys.argv[2]))
print(json.dumps(launch['generate_launch_description']()))
''', str(installed), relative], capture_output=True, text=True,
                    cwd=self.root, timeout=30)
                self.assertEqual(result.returncode, 0, result.stderr)
                actions = json.loads(result.stdout)
                arguments = {a['argument']: a['default_value'] for a in actions if 'argument' in a}
                self.assertEqual(arguments['rviz'], rviz_default)
                nodes = {a['executable']: a for a in actions if 'executable' in a}
                self.assertEqual(nodes['rviz2']['arguments'],
                                 ['-d', str(installed / 'viewers/display.rviz')])
                self.assertEqual(nodes['rviz2']['condition'], {'if': 'rviz'})
                description = nodes['robot_state_publisher']['parameters'][0]['robot_description']
                self.assertEqual(ET.fromstring(description).get('name'), 'bot')
                if relative.startswith('launch/'):
                    self.assertEqual(nodes['ros2_control_node']['parameters'],
                                     [str(installed / 'config/controllers.yaml')])
                    controllers = json.loads((installed / 'config/controllers.yaml').read_text())
                    self.assertEqual(controllers['joint_trajectory_controller']['ros__parameters']['joints'],
                                     ['shoulder'])
                else:
                    self.assertEqual(arguments['gui'], 'true')
                    self.assertEqual(nodes['joint_state_publisher_gui']['condition'], {'if': 'gui'})
                    self.assertEqual(nodes['joint_state_publisher']['condition'], {'unless': 'gui'})


if __name__ == '__main__':
    unittest.main()
