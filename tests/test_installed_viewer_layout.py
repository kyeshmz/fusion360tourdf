import json
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from tests.test_export_workflow import offline_bundle
from tools.export_bundle import bundle_hashes, read_bundle
from tools.verify_ros import check_installed


class InstalledViewerLayoutTest(unittest.TestCase):
    def check(self, package, output, **kwargs):
        def lookup(*, package_name, file_name):
            self.assertEqual(package_name, package.name)
            matches = list(package.rglob(file_name))
            self.assertEqual(len(matches), 1)
            return str(matches[0])

        def expand_without_xacro(package, destination):
            shutil.copyfile(next((package / 'urdf').glob('*.xacro')), destination)
            return destination

        api = types.ModuleType('ros2launch.api')
        api.get_share_file_path_from_package = lookup
        with patch.dict(sys.modules, {'ros2launch': types.ModuleType('ros2launch'),
                                     'ros2launch.api': api}), \
                patch('tools.verify_ros.expand', side_effect=expand_without_xacro), \
                patch('tools.verify_ros.check_urdfdom'):
            return check_installed(package, 'fixture_articulated', output, **kwargs)

    def test_current_exports_keep_strict_viewer_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, _ = offline_bundle(root / 'bundle')
            package = root / 'install' / source.name
            shutil.copytree(source, package)
            expected = sorted(str(path.resolve()) for path in (package / 'meshes').glob('*.stl'))
            self.assertEqual(self.check(package, root), expected)
            self.assertEqual(self.check(package, root, source=source), expected)
            shutil.copyfile(package / 'viewers/display.launch.py', package / 'launch/display.launch.py')
            with self.assertRaisesRegex(AssertionError, 'viewer asset'):
                self.check(package, root)

    def test_receipted_legacy_bundle_keeps_original_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / 'bundle'
            source, report = offline_bundle(bundle)
            (source / 'viewers/display.launch.py').rename(source / 'launch/display.launch.py')
            (source / 'viewers/display.rviz').rename(source / 'config/display.rviz')
            (source / 'viewers').rmdir()
            for launch in (source / 'launch').glob('*.py'):
                launch.write_text(launch.read_text().replace("'viewers', 'display.rviz'",
                                                             "'config', 'display.rviz'"))
            cmake = source / 'CMakeLists.txt'
            cmake.write_text(cmake.read_text().replace('launch config viewers', 'launch config'))
            report['outputs_sha256'] = bundle_hashes(bundle, source.name)
            (bundle / 'native.json').write_text(json.dumps(report))
            verified, _ = read_bundle(bundle)
            package = root / 'install' / source.name
            shutil.copytree(source, package)
            with self.assertRaisesRegex(AssertionError, 'Missing installed asset viewers/'):
                self.check(package, root)
            before = bundle_hashes(bundle, source.name)
            expected = sorted(str(path.resolve()) for path in (package / 'meshes').glob('*.stl'))
            self.assertEqual(self.check(package, root, source=verified), expected)
            self.assertEqual(bundle_hashes(bundle, source.name), before)
            for path in source.rglob('*'):
                if path.is_file():
                    self.assertEqual((package / path.relative_to(source)).read_bytes(), path.read_bytes())
