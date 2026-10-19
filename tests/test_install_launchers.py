import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASH = shutil.which('bash')
DOWNLOAD_VARS = ('PYTHON_MANAGER_AUTOMATIC_INSTALL', 'PYLAUNCHER_ALLOW_INSTALL',
                 'PYLAUNCHER_ALWAYS_INSTALL')


def snapshot(root):
    return {str(path.relative_to(root)): path.read_bytes() if path.is_file() else None
            for path in root.rglob('*')}


class LauncherFilesTests(unittest.TestCase):
    def test_entrypoints_are_in_the_checkout(self):
        for name in ('install.sh', 'install.cmd'):
            with self.subTest(name=name):
                self.assertTrue((ROOT / name).is_file(), 'Missing entrypoint %s' % name)


class LauncherContract:
    def setUp(self):
        temp = tempfile.TemporaryDirectory(prefix='fusion-launcher-test-')
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name).resolve()
        self.checkout = self.root / "checkout 雪 & ! (native) $ ; '"
        self.source = self.checkout / 'URDF_Exporter'
        shutil.copytree(ROOT / 'URDF_Exporter', self.source,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.pyo'))
        self.backend = self.checkout / 'tools' / 'install_fusion.py'
        self.backend.parent.mkdir()
        shutil.copy2(ROOT / 'tools' / 'install_fusion.py', self.backend)
        self.launcher = self.checkout / self.launcher_name
        if (ROOT / self.launcher_name).exists():
            shutil.copy2(ROOT / self.launcher_name, self.launcher)
        self.cwd = self.root / 'unrelated 雪 & ! (cwd)'
        self.cwd.mkdir()
        self.scripts = self.root / 'Wine prefix 雪 & ! (target) $ ;' / 'API' / 'Scripts'
        self.target = self.scripts / 'URDF_Exporter'
        self.env = dict(os.environ, HOME=str(self.root / 'home'),
                        USERPROFILE=str(self.root / 'home'), APPDATA=str(self.root / 'roaming'),
                        PYTHONDONTWRITEBYTECODE='1', PYTHONIOENCODING='utf-8')

    def cli(self, *args, env=None, direct=False):
        self.assertTrue(self.launcher.is_file(), 'Missing entrypoint %s' % self.launcher_name)
        if self.launcher_name == 'install.cmd':
            invocation = ' '.join('"%s"' % arg for arg in (str(self.launcher), *args))
            command = self.cmd_command(invocation)
        else:
            command = ([str(self.launcher)] if direct else [BASH, str(self.launcher)]) + list(args)
        return subprocess.run(command, cwd=self.cwd, env=self.env if env is None else env,
                              capture_output=True, encoding='utf-8', timeout=30)

    def test_dry_run_writes_nothing(self):
        before = snapshot(self.root)
        result = self.cli('--scripts-dir', str(self.scripts), '--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'Would install: %s\n' % self.target)
        self.assertEqual(result.stderr, '')
        self.assertEqual(snapshot(self.root), before)

    def test_terminal_command_from_checkout(self):
        if self.launcher_name == 'install.cmd':
            command = self.cmd_command('".\\install.cmd" "--scripts-dir" "%s" "--dry-run"' % self.scripts)
        else:
            command = [BASH, 'install.sh', '--scripts-dir', str(self.scripts), '--dry-run']
        before = snapshot(self.root)
        result = subprocess.run(command, cwd=self.checkout, env=self.env, capture_output=True,
                                encoding='utf-8', timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'Would install: %s\n' % self.target)
        self.assertEqual(snapshot(self.root), before)

    def test_fresh_install_copies_exact_bytes_from_unrelated_cwd(self):
        (self.source / 'xacro_export.toml').write_bytes(b'local = [\xff\r\n')
        expected = snapshot(self.source)
        result = self.cli('--scripts-dir', str(self.scripts))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'Installed: %s\n' % self.target)
        self.assertEqual(result.stderr, '')
        self.assertEqual(snapshot(self.target), expected)
        self.assertEqual(snapshot(self.source), expected)
        self.assertEqual(sorted(path.name for path in self.scripts.iterdir()), ['URDF_Exporter'])

    def test_relative_target_is_relative_to_caller_not_checkout(self):
        relative = 'relative Scripts 雪 & ! (path)'
        target = self.cwd / relative / 'URDF_Exporter'
        result = self.cli('--scripts-dir', relative)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'Installed: %s\n' % target)
        self.assertEqual(snapshot(target), snapshot(self.source))
        self.assertFalse((self.checkout / relative).exists())

    def test_repeat_and_update_preserve_config_and_complete_backup(self):
        installed = self.cli('--scripts-dir', str(self.scripts))
        self.assertEqual(installed.returncode, 0, installed.stderr)
        config = b'# User bytes\r\ninvalid = [\xff\n'
        (self.target / 'xacro_export.toml').write_bytes(config)
        before = snapshot(self.target)
        repeated = self.cli('--scripts-dir', str(self.scripts))
        self.assertEqual(repeated.returncode, 0, repeated.stderr)
        self.assertEqual(repeated.stdout, 'Unchanged: %s\n' % self.target)
        self.assertEqual(snapshot(self.target), before)
        (self.target / 'core' / 'Write.py').write_bytes(b'previous module\n')
        (self.target / 'personal notes.txt').write_bytes(b'user data\x00\xff')
        before = snapshot(self.root)
        refused = self.cli('--scripts-dir', str(self.scripts))
        self.assertEqual(refused.returncode, 1)
        self.assertIn('--update', refused.stderr)
        self.assertEqual(snapshot(self.root), before)
        planned = self.cli('--scripts-dir', str(self.scripts), '--update', '--dry-run')
        self.assertEqual(planned.returncode, 0, planned.stderr)
        self.assertEqual(planned.stdout, 'Would update: %s\n' % self.target)
        self.assertEqual(snapshot(self.root), before)
        previous = snapshot(self.target)
        updated = self.cli('--scripts-dir', str(self.scripts), '--update')
        self.assertEqual(updated.returncode, 0, updated.stderr)
        backups = list((self.scripts.parent / '.fusion360tourdf-backups').glob('*/URDF_Exporter'))
        self.assertEqual(len(backups), 1)
        self.assertEqual(updated.stdout, 'Updated: %s\nRetained backup: %s\n' % (self.target, backups[0]))
        self.assertEqual(snapshot(backups[0]), previous)
        expected = snapshot(self.source)
        expected['xacro_export.toml'] = config
        self.assertEqual(snapshot(self.target), expected)
        before = snapshot(self.root)
        repeated = self.cli('--scripts-dir', str(self.scripts), '--update')
        self.assertEqual(repeated.returncode, 0, repeated.stderr)
        self.assertEqual(repeated.stdout, 'Unchanged: %s\n' % self.target)
        self.assertEqual(snapshot(self.root), before)

    def test_backend_failure_exit_one_and_data_are_preserved(self):
        self.scripts.mkdir(parents=True)
        self.target.write_bytes(b'not a directory\x00\xff')
        before = snapshot(self.root)
        result = self.cli('--scripts-dir', str(self.scripts), '--update')
        self.assertEqual(result.returncode, 1)
        self.assertIn('Install failed:', result.stderr)
        self.assertIn('directory', result.stderr)
        self.assertEqual(result.stdout, '')
        self.assertEqual(snapshot(self.root), before)

    def test_unknown_flag_exit_two_is_forwarded_without_writes(self):
        before = snapshot(self.root)
        result = self.cli('--scripts-dir', str(self.scripts), '--unknown=two words')
        self.assertEqual(result.returncode, 2)
        self.assertIn('unrecognized arguments: --unknown=two words', result.stderr)
        self.assertEqual(result.stdout, '')
        self.assertEqual(snapshot(self.root), before)

    def test_missing_backend_has_actionable_ascii_error(self):
        self.backend.unlink()
        result = self.cli('--scripts-dir', str(self.scripts))
        self.assertEqual(result.returncode, 1)
        self.assertIn('complete checkout', result.stderr.lower())
        self.assertTrue(result.stderr.isascii())
        self.assertFalse(self.scripts.exists())

    def test_real_interpreter_receives_no_download_environment_and_single_dispatch(self):
        hook = self.root / 'python hook'
        hook.mkdir()
        capture = self.root / 'interpreter-capture.jsonl'
        (hook / 'sitecustomize.py').write_text(
            'import json, os, sys\n'
            'record = {"argv": sys.argv, "cwd": os.getcwd(), "env": '
            '{key: os.environ.get(key) for key in %r}}\n'
            'with open(os.environ["LAUNCHER_CAPTURE"], "a", encoding="utf-8") as stream:\n'
            '    stream.write(json.dumps(record) + "\\n")\n' % (DOWNLOAD_VARS,), encoding='utf-8')
        env = dict(self.env, PYTHONPATH=str(hook), LAUNCHER_CAPTURE=str(capture),
                   **{key: 'true' for key in DOWNLOAD_VARS})
        result = self.cli('--scripts-dir', str(self.scripts), '--dry-run', env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'Would install: %s\n' % self.target)
        records = [json.loads(line) for line in capture.read_text(encoding='utf-8').splitlines()]
        probes = [record for record in records if record['argv'] == ['-c']]
        dispatches = [record for record in records if record['argv'] != ['-c']]
        self.assertGreaterEqual(len(probes), 1)
        self.assertEqual(len(dispatches), 1)
        self.assertEqual(dispatches[0]['argv'],
                         [str(self.backend), '--scripts-dir', str(self.scripts), '--dry-run'])
        for record in records:
            self.assertEqual(record['env'], {'PYTHON_MANAGER_AUTOMATIC_INSTALL': 'false',
                                             'PYLAUNCHER_ALLOW_INSTALL': None,
                                             'PYLAUNCHER_ALWAYS_INSTALL': None})
            self.assertEqual(record['cwd'], str(self.cwd))
        self.assertFalse(self.scripts.exists())


@unittest.skipUnless(os.name == 'posix' and BASH, 'Bash launcher execution requires POSIX and Bash')
class BashLauncherTests(LauncherContract, unittest.TestCase):
    launcher_name = 'install.sh'

    def standin(self, name, probe_exit=0, dispatch_exit=37):
        self.bin.mkdir(exist_ok=True)
        script = self.bin / (name + '-capture.py')
        script.write_text(
            'import json, os, sys\n'
            'with open(os.environ["LAUNCHER_CAPTURE"], "a", encoding="utf-8") as stream:\n'
            '    stream.write(json.dumps({"name": %r, "argv": sys.argv[1:]}) + "\\n")\n'
            'sys.exit(%d if sys.argv[1] == "-c" else %d)\n' %
            (name, probe_exit, dispatch_exit), encoding='utf-8')
        executable = self.bin / name
        executable.write_text('#!/bin/sh\nexec %s %s "$@"\n' %
                              (shlex.quote(sys.executable), shlex.quote(str(script))), encoding='utf-8')
        executable.chmod(0o755)

    def isolated_env(self):
        self.bin = self.root / 'only these commands'
        self.capture = self.root / 'standin-capture.jsonl'
        self.bin.mkdir()
        return dict(self.env, PATH=str(self.bin), LAUNCHER_CAPTURE=str(self.capture))

    def records(self):
        return [json.loads(line) for line in self.capture.read_text(encoding='utf-8').splitlines()]

    def test_prefers_python3_and_forwards_exact_argv_and_arbitrary_exit(self):
        env = self.isolated_env()
        self.standin('python3')
        self.standin('python')
        args = ('--scripts-dir', str(self.scripts), '', 'two words', '雪 ! & ; $()',
                '$(touch injected-marker)', '--dry-run')
        result = self.cli(*args, env=env)
        self.assertEqual(result.returncode, 37, result.stderr)
        records = self.records()
        self.assertEqual([record['name'] for record in records], ['python3', 'python3'])
        self.assertEqual(records[0]['argv'][0], '-c')
        self.assertEqual(records[1]['argv'], [str(self.backend), *args])
        self.assertEqual(list(self.cwd.iterdir()), [])

    def test_old_python3_falls_back_to_usable_python(self):
        env = self.isolated_env()
        self.standin('python3', probe_exit=1)
        self.standin('python', dispatch_exit=39)
        result = self.cli('--dry-run', env=env)
        self.assertEqual(result.returncode, 39, result.stderr)
        records = self.records()
        self.assertEqual([record['name'] for record in records], ['python3', 'python', 'python'])
        self.assertEqual([record['argv'][0] for record in records], ['-c', '-c', str(self.backend)])

    def test_missing_python3_falls_back_to_python(self):
        env = self.isolated_env()
        self.standin('python')
        result = self.cli('--help', env=env)
        self.assertEqual(result.returncode, 37, result.stderr)
        self.assertEqual([record['name'] for record in self.records()], ['python', 'python'])

    def test_exported_function_cannot_bypass_the_executable_version_check(self):
        env = self.isolated_env()
        self.standin('python3', probe_exit=1)
        command = [BASH, '-c',
                   'python3() { return 0; }; export -f python3; '
                   'exec "$1" "$2" --dry-run',
                   'launcher-parent', BASH, str(self.launcher)]
        result = subprocess.run(command, cwd=self.cwd, env=env, capture_output=True,
                                encoding='utf-8', timeout=30)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn('Python 3.11', result.stderr)
        records = self.records()
        self.assertEqual([record['name'] for record in records], ['python3'])
        self.assertEqual(records[0]['argv'][0], '-c')
        self.assertFalse(self.scripts.exists())

    def test_all_old_interpreters_refuse_without_dispatch(self):
        env = self.isolated_env()
        self.standin('python3', probe_exit=1)
        self.standin('python', probe_exit=1)
        result = self.cli('--dry-run', env=env)
        self.assertEqual(result.returncode, 1)
        self.assertIn('Python 3.11', result.stderr)
        self.assertEqual([record['argv'][0] for record in self.records()], ['-c', '-c'])
        self.assertFalse(self.scripts.exists())

    def test_missing_interpreters_have_actionable_ascii_error(self):
        result = self.cli('--dry-run', env=self.isolated_env())
        self.assertEqual(result.returncode, 1)
        self.assertIn('Python 3.11', result.stderr)
        self.assertIn('install', result.stderr.lower())
        self.assertTrue(result.stderr.isascii())
        self.assertFalse(self.capture.exists())

    def test_missing_backend_does_not_probe_interpreters(self):
        env = self.isolated_env()
        self.standin('python3')
        self.backend.unlink()
        result = self.cli('--dry-run', env=env)
        self.assertEqual(result.returncode, 1)
        self.assertIn('complete checkout', result.stderr.lower())
        self.assertFalse(self.capture.exists())

    def test_executable_entrypoint_and_parent_environment(self):
        env = dict(self.env, **{key: 'caller-value' for key in DOWNLOAD_VARS})
        result = self.cli('--scripts-dir', str(self.scripts), '--dry-run', env=env, direct=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        command = ['bash', '-c',
                   '"$1" --help >/dev/null || exit $?; '
                   'printf "%s\\n" "$PYTHON_MANAGER_AUTOMATIC_INSTALL" '
                   '"$PYLAUNCHER_ALLOW_INSTALL" "$PYLAUNCHER_ALWAYS_INSTALL"',
                   'launcher-parent', str(self.launcher)]
        parent = subprocess.run(command, cwd=self.cwd, env=env, capture_output=True,
                                encoding='utf-8', timeout=30)
        self.assertEqual(parent.returncode, 0, parent.stderr)
        self.assertEqual(parent.stdout, 'caller-value\n' * 3)

    @unittest.skipUnless(sys.platform.startswith('linux'), 'Linux default refusal requires Linux')
    def test_linux_requires_explicit_target(self):
        before = snapshot(self.root)
        result = self.cli('--dry-run')
        self.assertEqual(result.returncode, 1)
        self.assertIn('--scripts-dir', result.stderr)
        self.assertEqual(snapshot(self.root), before)


@unittest.skipUnless(os.name == 'nt', 'Native CMD execution requires Windows; not verified on this host')
class CmdLauncherTests(LauncherContract, unittest.TestCase):
    launcher_name = 'install.cmd'

    def cmd_command(self, invocation):
        return '"%s" /d /s /v:off /c "%s"' % (os.environ['COMSPEC'], invocation)

    def test_missing_interpreters_with_isolated_path(self):
        empty = self.root / 'no commands'
        empty.mkdir()
        env = dict(self.env, PATH=str(empty), NoDefaultCurrentDirectoryInExePath='1')
        before = snapshot(self.root)
        result = self.cli('--scripts-dir', str(self.scripts), '--dry-run', env=env)
        self.assertEqual(snapshot(self.root), before)
        if result.returncode == 0 and result.stdout == 'Would install: %s\n' % self.target:
            self.skipTest('A native runtime remains discoverable outside the isolated PATH')
        self.assertEqual(result.returncode, 1)
        self.assertIn('Python 3.11', result.stderr)
        self.assertIn('install', result.stderr.lower())
        self.assertTrue(result.stderr.isascii())

    def test_caller_cmd_environment_is_restored(self):
        env = dict(self.env, **{key: 'caller-value' for key in DOWNLOAD_VARS})
        invocation = '"%s" --help >nul & %s' % (
            self.launcher, ' & '.join('set %s' % key for key in DOWNLOAD_VARS))
        result = subprocess.run(self.cmd_command(invocation), cwd=self.cwd, env=env,
                                capture_output=True, encoding='utf-8', timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.splitlines(), ['%s=caller-value' % key for key in DOWNLOAD_VARS])


if __name__ == '__main__':
    unittest.main()
