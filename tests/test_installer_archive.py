from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from zipfile import ZipFile

from tools.build_installer import build_installer


ROOT = Path(__file__).resolve().parents[1]


class InstallerArchiveTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.archive = self.root / 'installer with spaces.pyz'
        self.checksum = build_installer(ROOT, self.archive)

    def run_installer(self, *arguments):
        return subprocess.run([sys.executable, str(self.archive), *arguments], cwd=self.root,
                              capture_output=True, text=True, timeout=30)

    def test_archive_runs_without_checkout_and_installs_notices_and_identical_payload(self):
        with ZipFile(self.archive) as archive:
            self.assertEqual(archive.read('tools/install_fusion.py'), (ROOT/'tools/install_fusion.py').read_bytes())
            self.assertNotIn('URDF_Exporter/xacro_export.toml', archive.namelist())
        result = self.run_installer('--scripts-dir', 'Scripts with spaces', '--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Would install:', result.stdout)
        self.assertFalse((self.root/'Scripts with spaces').exists())
        result = self.run_installer('--scripts-dir', 'Scripts with spaces')
        self.assertEqual(result.returncode, 0, result.stderr)
        installed = self.root/'Scripts with spaces/URDF_Exporter'
        for path in (ROOT/'URDF_Exporter').rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc', '.pyo') and path.name not in ('xacro_export.toml', '.DS_Store'):
                self.assertEqual((installed/path.relative_to(ROOT/'URDF_Exporter')).read_bytes(), path.read_bytes())
        again = self.run_installer('--scripts-dir', 'Scripts with spaces')
        self.assertEqual(again.returncode, 0, again.stderr)
        self.assertIn('Unchanged:', again.stdout)

    def test_shared_update_policy_retains_backup_and_local_config(self):
        arguments = ('--scripts-dir', 'Scripts')
        self.assertEqual(self.run_installer(*arguments).returncode, 0)
        installed = self.root/'Scripts/URDF_Exporter'
        config = b'effort = 2.0\r\n# local settings\r\n'
        (installed/'xacro_export.toml').write_bytes(config)
        (installed/'core/Write.py').write_text('old version', encoding='utf-8')
        refused = self.run_installer(*arguments)
        self.assertEqual(refused.returncode, 1)
        self.assertIn('--update', refused.stderr)
        updated = self.run_installer(*arguments, '--update')
        self.assertEqual(updated.returncode, 0, updated.stderr)
        self.assertEqual((installed/'xacro_export.toml').read_bytes(), config)
        backups = list((self.root/'.fusion360tourdf-backups').glob('*/URDF_Exporter'))
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0]/'core/Write.py').read_text(), 'old version')

    def test_help_and_bad_arguments_preserve_exit_codes(self):
        help_result = self.run_installer('--help')
        self.assertEqual(help_result.returncode, 0, help_result.stderr)
        self.assertIn('--scripts-dir', help_result.stdout)
        self.assertEqual(self.run_installer('--unknown-option').returncode, 2)

    def test_existing_archive_is_never_replaced(self):
        before = self.archive.read_bytes()
        with self.assertRaises(FileExistsError):
            build_installer(ROOT, self.archive)
        self.assertEqual(self.archive.read_bytes(), before)

    def test_build_excludes_private_config_and_caches(self):
        checkout = self.root/'checkout'
        shutil.copytree(ROOT/'URDF_Exporter', checkout/'URDF_Exporter', ignore=shutil.ignore_patterns('__pycache__'))
        (checkout/'tools').mkdir()
        shutil.copy2(ROOT/'tools/install_fusion.py', checkout/'tools/install_fusion.py')
        shutil.copy2(ROOT/'LICENSE', checkout/'LICENSE')
        (checkout/'URDF_Exporter/xacro_export.toml').write_text('private settings', encoding='utf-8')
        (checkout/'URDF_Exporter/__pycache__').mkdir()
        (checkout/'URDF_Exporter/__pycache__/local.pyc').write_bytes(b'cache')
        output = self.root/'clean.pyz'
        build_installer(checkout, output)
        with ZipFile(output) as archive:
            self.assertNotIn('URDF_Exporter/xacro_export.toml', archive.namelist())
            self.assertFalse(any('__pycache__' in path for path in archive.namelist()))

    @unittest.skipIf(sys.platform == 'win32', 'Creating symlinks requires separate Windows privileges')
    def test_build_refuses_symlinked_source(self):
        checkout = self.root/'symlink-checkout'
        checkout.mkdir()
        (checkout/'URDF_Exporter').symlink_to(ROOT/'URDF_Exporter', target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            build_installer(checkout, self.root/'unsafe.pyz')

    @unittest.skipIf(sys.platform == 'win32', 'POSIX executable shebang')
    def test_posix_archive_is_executable(self):
        result = subprocess.run([str(self.archive), '--help'], cwd=self.root, text=True,
                                capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
