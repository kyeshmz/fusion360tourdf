import contextlib
import importlib.util
import io
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
INSTALLER = ROOT / 'tools' / 'install_fusion.py'
CONFIG = 'xacro_export.toml'


def snapshot(root):
    return {str(path.relative_to(root)): path.read_bytes() if path.is_file() else None
            for path in root.rglob('*')}


class InstallFusionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='fusion-installer-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.checkout = self.root / 'checkout'
        self.source = self.checkout / 'URDF_Exporter'
        shutil.copytree(ROOT / 'URDF_Exporter', self.source,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.pyo'))
        self.script = self.checkout / 'tools' / 'install_fusion.py'
        self.script.parent.mkdir()
        if INSTALLER.exists():
            shutil.copy2(INSTALLER, self.script)
        self.scripts = self.root / 'Wine prefix 雪' / 'API' / 'Scripts'
        self.target = self.scripts / 'URDF_Exporter'
        self.cwd = self.root / 'unrelated working directory'
        self.cwd.mkdir()
        self.env = dict(os.environ, HOME=str(self.root / 'home'), USERPROFILE=str(self.root / 'home'),
                        APPDATA=str(self.root / 'roaming'), PYTHONDONTWRITEBYTECODE='1',
                        PYTHONIOENCODING='utf-8')

    def cli(self, *args, platform=None, automatic=False, env=None, encoding='utf-8'):
        command = [sys.executable]
        if platform:
            command += ['-c', "import runpy, sys, types; namespace = runpy.run_path(sys.argv[2]); "
                        "namespace['main'].__globals__['sys'] = types.SimpleNamespace("
                        "platform=sys.argv[1], stdout=sys.stdout, stderr=sys.stderr); "
                        "sys.exit(namespace['main'](sys.argv[3:]))", platform]
        command += [str(self.script)]
        if not automatic:
            command += ['--scripts-dir', str(self.scripts)]
        return subprocess.run(command + list(args), cwd=self.cwd,
                              env=self.env if env is None else env,
                              capture_output=True, encoding=encoding)

    def module(self):
        spec = importlib.util.spec_from_file_location('fusion_installer_under_test', self.script)
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {spec.name: module}):
            spec.loader.exec_module(module)
        return module

    def install_old(self):
        self.target.mkdir(parents=True)
        (self.target / 'URDF_Exporter.py').write_bytes(b'old production code\n')
        (self.target / 'personal notes.txt').write_bytes(b'user-owned extra\x00\xff')
        (self.target / CONFIG).write_bytes(b'# Preserve bytes, not TOML validity\r\ninvalid = [\xff\n')
        cache = self.target / '__pycache__'
        cache.mkdir()
        (cache / 'old.pyc').write_bytes(b'previous cached bytecode')
        return snapshot(self.target)

    def backups(self):
        root = self.scripts.parent / '.fusion360tourdf-backups'
        return list(root.glob('*/URDF_Exporter')) if root.exists() else []

    def symlink(self, link, destination):
        try:
            link.symlink_to(destination, target_is_directory=destination.is_dir())
        except (OSError, NotImplementedError) as error:
            self.skipTest('Symlinks unavailable: %s' % error)

    def test_fresh_install_from_any_cwd_copies_complete_exporter_only(self):
        (self.source / CONFIG).write_bytes(b'# checked-out config\r\neffort = 42.0\r\n')
        cache = self.source / '__pycache__'
        cache.mkdir()
        (cache / 'cached.pyc').write_bytes(b'cached')
        (self.source / 'core' / 'cached.pyo').write_bytes(b'cached')
        before = snapshot(self.source)
        result = self.cli()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Installed', result.stdout)
        self.assertIn(str(self.target), result.stdout)
        for relative in ('URDF_Exporter.py', 'URDF_Exporter.manifest', 'LICENSE',
                         'xacro_export.template.toml', 'core/Write.py', 'utils/config.py',
                         'utils/transforms.py', CONFIG):
            self.assertEqual((self.target / relative).read_bytes(), (self.source / relative).read_bytes())
        self.assertFalse((self.target / '__pycache__').exists())
        self.assertFalse((self.target / 'core' / 'cached.pyo').exists())
        self.assertEqual(sorted(path.name for path in self.scripts.iterdir()), ['URDF_Exporter'])
        self.assertEqual(snapshot(self.source), before)
        installed = subprocess.run(
            [sys.executable, '-c',
             'import pathlib, py_compile, sys; sys.path.insert(0, sys.argv[1]); '
             'from URDF_Exporter.utils.config import load_config; '
             'from URDF_Exporter.core.Write import write_ros2_package; '
             'assert load_config(pathlib.Path(sys.argv[1]) / "URDF_Exporter")["effort"] == 42.0; '
             '[py_compile.compile(str(p), doraise=True) for p in '
             '(pathlib.Path(sys.argv[1]) / "URDF_Exporter").rglob("*.py")]; print("installed import and compile passed")',
             str(self.scripts)], cwd=self.cwd, env=self.env, capture_output=True, encoding='utf-8')
        self.assertEqual(installed.returncode, 0, installed.stderr)
        self.assertEqual(installed.stdout.strip(), 'installed import and compile passed')
        self.assertEqual(self.cli().returncode, 0)

    def test_cli_through_parent_alias_reports_canonical_target_and_installs_exact_bytes(self):
        alias = self.root / 'parent alias'
        self.symlink(alias, self.root)
        self.script = alias / self.script.relative_to(self.root)
        scripts = alias / self.scripts.relative_to(self.root)
        (self.source / CONFIG).write_bytes(b'# checked-out config\r\neffort = 42.0\r\n')
        expected = snapshot(self.source)
        result = self.cli('--scripts-dir', str(scripts))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'Installed: %s\n' % self.target.resolve())
        self.assertEqual(snapshot(self.target), expected)

    def test_cp1252_fresh_success_escapes_report_without_changing_installed_bytes(self):
        (self.source / CONFIG).write_bytes(b'# checked-out config\r\ninvalid = [\xff\n')
        expected = snapshot(self.source)
        env = dict(self.env, PYTHONIOENCODING='cp1252')
        result = self.cli(env=env, encoding='cp1252')
        self.assertEqual(snapshot(self.target), expected)
        self.assertEqual(result.returncode, 0, result.stderr)
        target = str(self.target).encode('cp1252', 'backslashreplace').decode('cp1252')
        self.assertEqual(result.stdout, 'Installed: %s\n' % target)
        self.assertIn(r'Wine prefix \u96ea', result.stdout)
        self.assertEqual(result.stderr, '')
        self.assertEqual(self.backups(), [])

    def test_cp1252_update_reports_backup_and_preserves_exact_config_and_backup_bytes(self):
        before = self.install_old()
        (self.source / CONFIG).write_bytes(b'effort = 900.0\n')
        expected = snapshot(self.source)
        expected[CONFIG] = before[CONFIG]
        env = dict(self.env, PYTHONIOENCODING='cp1252')
        result = self.cli('--update', env=env, encoding='cp1252')
        self.assertEqual(snapshot(self.target), expected)
        backups = self.backups()
        self.assertEqual(len(backups), 1)
        self.assertEqual(snapshot(backups[0]), before)
        self.assertEqual(result.returncode, 0, result.stderr)
        target = str(self.target).encode('cp1252', 'backslashreplace').decode('cp1252')
        backup = str(backups[0]).encode('cp1252', 'backslashreplace').decode('cp1252')
        self.assertEqual(result.stdout, 'Updated: %s\nRetained backup: %s\n' % (target, backup))
        self.assertIn(r'Wine prefix \u96ea', result.stdout)
        self.assertEqual(result.stderr, '')

    def test_cp1252_refusal_reports_path_without_traceback_or_writes(self):
        self.install_old()
        before = snapshot(self.root)
        env = dict(self.env, PYTHONIOENCODING='cp1252')
        result = self.cli(env=env, encoding='cp1252')
        self.assertEqual(result.returncode, 1)
        target = str(self.target).encode('cp1252', 'backslashreplace').decode('cp1252')
        self.assertIn('Install failed: Existing install differs at %s.' % target, result.stderr)
        self.assertIn(r'Wine prefix \u96ea', result.stderr)
        self.assertIn('--update', result.stderr)
        self.assertNotIn('Traceback', result.stderr)
        self.assertEqual(result.stdout, '')
        self.assertEqual(snapshot(self.root), before)

    def test_windows_automatic_location_uses_scratch_appdata(self):
        result = self.cli(platform='win32', automatic=True)
        target = Path(self.env['APPDATA']) / 'Autodesk/Autodesk Fusion 360/API/Scripts/URDF_Exporter'
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((target / 'LICENSE').read_bytes(), (self.source / 'LICENSE').read_bytes())
        self.assertIn(str(target), result.stdout)

    def test_macos_automatic_location_uses_scratch_home(self):
        result = self.cli(platform='darwin', automatic=True)
        target = Path(self.env['HOME']) / 'Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/URDF_Exporter'
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((target / 'LICENSE').read_bytes(), (self.source / 'LICENSE').read_bytes())
        self.assertIn(str(target), result.stdout)

    def test_unsupported_platform_requires_explicit_scripts_dir(self):
        before = snapshot(self.root)
        result = self.cli(platform='linux', automatic=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--scripts-dir', result.stderr)
        self.assertEqual(snapshot(self.root), before)
        explicit = self.cli(platform='linux')
        self.assertEqual(explicit.returncode, 0, explicit.stderr)
        self.assertTrue((self.target / 'URDF_Exporter.manifest').is_file())

    def test_windows_missing_appdata_refuses_without_writes(self):
        env = self.env.copy()
        env.pop('APPDATA')
        before = snapshot(self.root)
        result = self.cli(platform='win32', automatic=True, env=env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('APPDATA', result.stderr)
        self.assertEqual(snapshot(self.root), before)

    def test_default_resolver_uses_only_supplied_platform_environment_and_home(self):
        module = self.module()
        roaming = self.root / 'chosen roaming'
        home = self.root / 'chosen home'
        self.assertEqual(module.default_scripts_dir('win32', {'APPDATA': str(roaming)}, home),
                         roaming / 'Autodesk/Autodesk Fusion 360/API/Scripts')
        self.assertEqual(module.default_scripts_dir('darwin', {}, home),
                         home / 'Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts')

    def test_repeat_is_unchanged_even_with_local_config_and_caches(self):
        self.assertEqual(self.cli().returncode, 0)
        (self.target / CONFIG).write_bytes(b'local config\xff')
        cache = self.target / '__pycache__'
        cache.mkdir()
        (cache / 'old.pyc').write_bytes(b'old cached bytecode')
        before = snapshot(self.target)
        for args in ((), ('--update',)):
            result = self.cli(*args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('Unchanged', result.stdout)
            self.assertEqual(snapshot(self.target), before)
        self.assertEqual(self.backups(), [])

    def test_different_install_refuses_without_update(self):
        before = self.install_old()
        for args in ((), ('--dry-run',)):
            result = self.cli(*args)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('--update', result.stderr)
            self.assertEqual(snapshot(self.target), before)
        self.assertEqual(self.backups(), [])

    def test_update_keeps_complete_backup_outside_scripts_and_preserves_config_bytes(self):
        before = self.install_old()
        (self.source / CONFIG).write_bytes(b'effort = 900.0\n')
        result = self.cli('--update')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Updated', result.stdout)
        self.assertEqual((self.target / CONFIG).read_bytes(), before[CONFIG])
        self.assertFalse((self.target / 'personal notes.txt').exists())
        self.assertEqual((self.target / 'URDF_Exporter.py').read_bytes(),
                         (self.source / 'URDF_Exporter.py').read_bytes())
        backups = self.backups()
        self.assertEqual(len(backups), 1)
        self.assertEqual(snapshot(backups[0]), before)
        self.assertNotIn(self.scripts, backups[0].parents)
        self.assertIn(str(backups[0]), result.stdout)
        self.assertEqual(sorted(path.name for path in self.scripts.iterdir()), ['URDF_Exporter'])
        again = self.cli('--update')
        self.assertEqual(again.returncode, 0, again.stderr)
        self.assertIn('Unchanged', again.stdout)
        self.assertEqual(self.backups(), backups)

    def test_dry_run_fresh_and_update_create_nothing(self):
        before = snapshot(self.root)
        result = self.cli('--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(str(self.target), result.stdout)
        self.assertIn('Would install', result.stdout)
        self.assertEqual(snapshot(self.root), before)
        self.install_old()
        before = snapshot(self.root)
        result = self.cli('--update', '--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Would update', result.stdout)
        self.assertEqual(snapshot(self.root), before)

    def test_file_target_is_not_replaced(self):
        self.scripts.mkdir(parents=True)
        self.target.write_bytes(b'not a directory')
        result = self.cli('--update')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('directory', result.stderr)
        self.assertEqual(self.target.read_bytes(), b'not a directory')

    def test_symlink_target_is_not_followed_or_replaced(self):
        outside = self.root / 'outside'
        outside.mkdir()
        (outside / 'sentinel').write_bytes(b'untouched')
        self.scripts.mkdir(parents=True)
        self.symlink(self.target, outside)
        result = self.cli('--update')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('symlink', result.stderr.lower())
        self.assertTrue(self.target.is_symlink())
        self.assertEqual((outside / 'sentinel').read_bytes(), b'untouched')

    def test_config_directory_and_symlink_are_refused_even_during_dry_run(self):
        for kind in ('directory', 'symlink'):
            with self.subTest(kind=kind):
                self.install_old()
                config = self.target / CONFIG
                config.unlink()
                if kind == 'directory':
                    config.mkdir()
                    (config / 'sentinel').write_bytes(b'untouched')
                else:
                    outside = self.root / 'outside-config'
                    outside.write_bytes(b'untouched')
                    self.symlink(config, outside)
                result = self.cli('--update', '--dry-run')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(CONFIG, result.stderr)
                self.assertEqual((self.target / 'URDF_Exporter.py').read_bytes(), b'old production code\n')
                if kind == 'directory':
                    self.assertEqual((config / 'sentinel').read_bytes(), b'untouched')
                    shutil.rmtree(config)
                else:
                    self.assertEqual(outside.read_bytes(), b'untouched')
                    config.unlink()
                shutil.rmtree(self.target)

    def test_source_symlink_and_missing_required_file_refuse_without_writes(self):
        outside = self.root / 'outside'
        outside.write_bytes(b'untouched')
        link = self.source / 'linked.py'
        self.symlink(link, outside)
        result = self.cli()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('symlink', result.stderr.lower())
        self.assertEqual(outside.read_bytes(), b'untouched')
        self.assertFalse(self.scripts.exists())
        link.unlink()
        for name in ('URDF_Exporter.manifest', 'utils/transforms.py', 'utils/stl.py',
                     'THIRD_PARTY_NOTICES.md', 'licenses/Adriaeik-MIT.txt'):
            with self.subTest(missing=name):
                path = self.source / name
                original = path.read_bytes()
                path.unlink()
                try:
                    result = self.cli()
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(name, result.stderr.replace('\\', '/'))
                    self.assertFalse(self.scripts.exists())
                finally:
                    path.write_bytes(original)

    @unittest.skipUnless(os.name == 'nt', 'Windows junctions require native Windows')
    def test_windows_junction_target_and_source_entry_are_refused(self):
        outside = self.root / 'junction-outside'
        outside.mkdir()
        (outside / 'sentinel').write_bytes(b'untouched')
        self.scripts.mkdir(parents=True)
        for link in (self.target, self.source / 'junction-entry'):
            result = subprocess.run(['cmd', '/c', 'mklink', '/J', str(link), str(outside)],
                                    capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            refused = self.cli('--update')
            self.assertNotEqual(refused.returncode, 0)
            self.assertIn('link', refused.stderr.lower())
            self.assertEqual((outside / 'sentinel').read_bytes(), b'untouched')
            link.rmdir()

    def test_source_root_symlink_is_refused(self):
        moved = self.checkout / 'actual-exporter'
        self.source.rename(moved)
        self.symlink(self.source, moved)
        result = self.cli()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('symlink', result.stderr.lower())
        self.assertTrue((moved / 'LICENSE').is_file())
        self.assertFalse(self.scripts.exists())

    def test_source_target_overlap_is_refused_before_writes(self):
        before = snapshot(self.checkout)
        for scripts in (self.checkout, self.source / 'nested Scripts'):
            with self.subTest(scripts=scripts):
                result = self.cli('--scripts-dir', str(scripts), '--update')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('overlap', result.stderr.lower())
                self.assertEqual(snapshot(self.checkout), before)

    def test_backup_symlink_is_refused_without_changing_old_install(self):
        before = self.install_old()
        outside = self.root / 'outside-backup'
        outside.mkdir()
        self.symlink(self.scripts.parent / '.fusion360tourdf-backups', outside)
        result = self.cli('--update')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('symlink', result.stderr.lower())
        self.assertEqual(snapshot(self.target), before)
        self.assertEqual(list(outside.iterdir()), [])

    def test_old_install_symlink_is_not_followed_or_deleted(self):
        self.install_old()
        outside = self.root / 'outside-extra'
        outside.mkdir()
        (outside / 'sentinel').write_bytes(b'untouched')
        self.symlink(self.target / 'user-link', outside)
        result = self.cli('--update')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('symlink', result.stderr.lower())
        self.assertEqual((outside / 'sentinel').read_bytes(), b'untouched')
        self.assertTrue((self.target / 'user-link').is_symlink())
        self.assertEqual((self.target / 'URDF_Exporter.py').read_bytes(), b'old production code\n')

    def test_caller_api_returns_paths_and_status_for_dry_run_install_repeat_and_update(self):
        module = self.module()
        dry = module.install(self.source, self.scripts, dry_run=True)
        self.assertEqual((dry.status, dry.target, dry.backup), ('would-install', self.target, None))
        self.assertFalse(self.scripts.exists())
        fresh = module.install(self.source, self.scripts)
        self.assertEqual((fresh.status, fresh.target, fresh.backup), ('installed', self.target, None))
        repeated = module.install(self.source, self.scripts)
        self.assertEqual((repeated.status, repeated.target, repeated.backup), ('unchanged', self.target, None))
        (self.target / 'extra.txt').write_bytes(b'keep in backup only')
        updated = module.install(self.source, self.scripts, update=True)
        self.assertEqual((updated.status, updated.target), ('updated', self.target))
        self.assertEqual((updated.backup / 'extra.txt').read_bytes(), b'keep in backup only')
        self.assertNotIn(self.scripts, updated.backup.parents)
        self.assertFalse((self.target / 'extra.txt').exists())

    def test_copy_failure_leaves_old_install_intact(self):
        before = self.install_old()
        module = self.module()

        def partial_copy(source, destination, **kwargs):
            destination.mkdir()
            (destination / 'partial.py').write_bytes(b'incomplete copy')
            raise OSError('copy denied')

        with mock.patch.object(module.shutil, 'copytree', side_effect=partial_copy):
            with self.assertRaisesRegex(OSError, 'copy denied'):
                module.install(self.source, self.scripts, update=True)
        self.assertEqual(snapshot(self.target), before)
        self.assertEqual(self.backups(), [])
        self.assertEqual(sorted(path.name for path in self.scripts.iterdir()), ['URDF_Exporter'])

    def test_fresh_publish_failure_does_not_leave_a_partial_install(self):
        module = self.module()
        with mock.patch.object(Path, 'rename', side_effect=OSError('publish denied')):
            with self.assertRaisesRegex(OSError, 'publish denied'):
                module.install(self.source, self.scripts)
        self.assertEqual(list(self.scripts.iterdir()), [])
        self.assertTrue((self.source / 'URDF_Exporter.py').is_file())

    def test_new_destination_appearing_during_copy_is_not_overwritten(self):
        module = self.module()
        copytree = shutil.copytree

        def copy_and_occupy(source, destination, *args, **kwargs):
            result = copytree(source, destination, *args, **kwargs)
            if Path(source) == self.source:
                self.target.mkdir()
                (self.target / 'concurrent.txt').write_bytes(b'new owner')
            return result

        with mock.patch.object(module.shutil, 'copytree', side_effect=copy_and_occupy):
            with self.assertRaisesRegex(FileExistsError, 'unexpected destination'):
                module.install(self.source, self.scripts)
        self.assertEqual(snapshot(self.target), {'concurrent.txt': b'new owner'})

    def test_target_changed_during_copy_is_not_replaced(self):
        before = self.install_old()
        module = self.module()
        copytree = shutil.copytree

        def copy_and_modify(source, destination, *args, **kwargs):
            result = copytree(source, destination, *args, **kwargs)
            if Path(source) == self.source:
                (self.target / 'personal notes.txt').write_bytes(b'edited during staging')
            return result

        with mock.patch.object(module.shutil, 'copytree', side_effect=copy_and_modify):
            with self.assertRaisesRegex(FileExistsError, 'changed while staging'):
                module.install(self.source, self.scripts, update=True)
        expected = dict(before, **{'personal notes.txt': b'edited during staging'})
        self.assertEqual(snapshot(self.target), expected)
        self.assertEqual(self.backups(), [])

    def test_target_rename_failure_leaves_old_install_intact(self):
        before = self.install_old()
        module = self.module()
        rename = Path.rename

        def fail_old_move(path, destination):
            if path == self.target:
                raise OSError('old target rename denied')
            return rename(path, destination)

        with mock.patch.object(Path, 'rename', fail_old_move):
            with self.assertRaisesRegex(OSError, 'old target rename denied'):
                module.install(self.source, self.scripts, update=True)
        self.assertEqual(snapshot(self.target), before)
        self.assertEqual(self.backups(), [])

    def test_publish_failure_restores_old_install(self):
        before = self.install_old()
        module = self.module()
        rename = Path.rename

        def fail_publish(path, destination):
            if Path(destination) == self.target and '.fusion360tourdf-backups' not in path.parts:
                raise OSError('publish denied')
            return rename(path, destination)

        with mock.patch.object(Path, 'rename', fail_publish):
            with self.assertRaisesRegex(OSError, 'publish denied'):
                module.install(self.source, self.scripts, update=True)
        self.assertEqual(snapshot(self.target), before)
        self.assertEqual(self.backups(), [])
        self.assertEqual(sorted(path.name for path in self.scripts.iterdir()), ['URDF_Exporter'])

    def test_failed_restore_reports_retained_recovery_backup(self):
        before = self.install_old()
        module = self.module()
        rename = Path.rename

        def fail_publish_and_restore(path, destination):
            if Path(destination) == self.target:
                raise OSError('destination unavailable')
            return rename(path, destination)

        with mock.patch.object(Path, 'rename', fail_publish_and_restore):
            with self.assertRaises(OSError) as raised:
                module.install(self.source, self.scripts, update=True)
        backups = self.backups()
        self.assertEqual(len(backups), 1)
        self.assertEqual(snapshot(backups[0]), before)
        self.assertIn(str(backups[0]), str(raised.exception))
        self.assertIn('recover', str(raised.exception).lower())
        self.assertFalse(self.target.exists())

    def test_concurrently_appeared_target_is_not_overwritten_during_rollback(self):
        before = self.install_old()
        module = self.module()
        rename = Path.rename

        def occupy_destination(path, destination):
            if Path(destination) == self.target and '.fusion360tourdf-backups' not in path.parts:
                self.target.mkdir()
                (self.target / 'concurrent.txt').write_bytes(b'new owner')
                raise FileExistsError('another destination appeared')
            return rename(path, destination)

        with mock.patch.object(Path, 'rename', occupy_destination):
            with self.assertRaises(OSError) as raised:
                module.install(self.source, self.scripts, update=True)
        self.assertEqual(snapshot(self.target), {'concurrent.txt': b'new owner'})
        backups = self.backups()
        self.assertEqual(len(backups), 1)
        self.assertEqual(snapshot(backups[0]), before)
        self.assertIn(str(backups[0]), str(raised.exception))

    def test_cli_failure_is_readable_and_does_not_dump_traceback(self):
        self.install_old()
        module = self.module()
        stderr = io.StringIO()
        with mock.patch.object(module.shutil, 'copytree', side_effect=OSError('copy denied')):
            with contextlib.redirect_stderr(stderr):
                status = module.main(['--scripts-dir', str(self.scripts), '--update'])
        self.assertEqual(status, 1)
        self.assertIn('copy denied', stderr.getvalue())
        self.assertNotIn('Traceback', stderr.getvalue())
        self.assertEqual((self.target / 'URDF_Exporter.py').read_bytes(), b'old production code\n')


if __name__ == '__main__':
    unittest.main()
