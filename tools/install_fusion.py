#!/usr/bin/env python3
"""Copy this checkout's exporter into Fusion's Scripts directory."""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile
from typing import Optional


CONFIG = 'xacro_export.toml'
EXPORTER = 'URDF_Exporter'


@dataclass(frozen=True)
class InstallResult:
    status: str
    target: Path
    backup: Optional[Path] = None


def default_scripts_dir(platform, environ, home):
    """Resolve supported native defaults using only the supplied inputs."""
    if platform == 'win32':
        appdata = environ.get('APPDATA', '')
        if not appdata or not Path(appdata).is_absolute():
            raise ValueError('APPDATA must be an absolute directory, or use --scripts-dir.')
        return Path(appdata) / 'Autodesk' / 'Autodesk Fusion 360' / 'API' / 'Scripts'
    if platform == 'darwin':
        if not Path(home).is_absolute():
            raise ValueError('Home must be an absolute directory, or use --scripts-dir.')
        return Path(home) / 'Library' / 'Application Support' / 'Autodesk' / 'Autodesk Fusion 360' / 'API' / 'Scripts'
    raise ValueError('No automatic Fusion Scripts location on %s. Use --scripts-dir, including on Linux/Wine.' % platform)


def _is_link(path):
    try:
        info = path.lstat()
    except FileNotFoundError:
        return False
    return stat.S_ISLNK(info.st_mode) or bool(
        getattr(info, 'st_file_attributes', 0) & getattr(stat, 'FILE_ATTRIBUTE_REPARSE_POINT', 0))


def _require_directory(path):
    if _is_link(path):
        raise ValueError('Refusing symlink or reparse-point directory: %s' % path)
    if not path.is_dir():
        raise ValueError('Expected a directory: %s' % path)


def _contents(root):
    config = root / CONFIG
    if _is_link(config) or (config.exists() and not config.is_file()):
        raise ValueError('Expected a regular config file, not a directory or symlink: %s' % config)
    contents = {}

    def visit(directory):
        for path in directory.iterdir():
            if _is_link(path):
                raise ValueError('Refusing symlink or reparse-point entry: %s' % path)
            if path.name == '__pycache__' or path.suffix in ('.pyc', '.pyo'):
                continue
            relative = path.relative_to(root)
            if path.is_dir():
                contents[relative] = None
                visit(path)
            elif path.is_file():
                contents[relative] = path.read_bytes()
            else:
                raise ValueError('Expected a regular file or directory: %s' % path)

    visit(root)
    return contents


def _overlap(left, right):
    return left == right or left in right.parents or right in left.parents


def _rename_if_absent(source, destination):
    if destination.exists() or _is_link(destination):
        raise FileExistsError('Refusing to overwrite an unexpected destination: %s' % destination)
    source.rename(destination)


def install(source, scripts_dir, *, update=False, dry_run=False):
    """Install a complete exporter, retaining updates outside Scripts for recovery.

    The caller supplies the source and Scripts parent. Existing local config bytes
    survive updates. This operation does not coordinate concurrent installers.
    """
    source = Path(source).absolute()
    _require_directory(source)
    source = source.resolve()
    scripts = Path(scripts_dir).expanduser().resolve()
    target = scripts / EXPORTER
    if _overlap(source, target):
        raise ValueError('Source and install target overlap: %s and %s' % (source, target))
    if scripts.exists() and not scripts.is_dir():
        raise ValueError('Expected a Scripts directory: %s' % scripts)
    source_contents = _contents(source)
    for name in ('URDF_Exporter.py', 'URDF_Exporter.manifest', 'LICENSE', 'xacro_export.template.toml',
                 'core/__init__.py', 'core/Write.py', 'core/Joint.py', 'core/Link.py', 'utils/__init__.py',
                 'utils/config.py', 'utils/utils.py', 'utils/stl.py', 'utils/transforms.py',
                 'utils/materials.py', 'utils/validate.py'):
        if source_contents.get(Path(name)) is None:
            raise ValueError('Source exporter is missing a required regular file: %s' % (source / name))

    previous = None
    target_stat = None
    backup_root = scripts.parent / '.fusion360tourdf-backups'
    if target.exists() or _is_link(target):
        _require_directory(target)
        target_stat = target.lstat()
        previous = _contents(target)
        if ({key: value for key, value in previous.items() if key != Path(CONFIG)} ==
                {key: value for key, value in source_contents.items() if key != Path(CONFIG)}):
            return InstallResult('unchanged', target)
        if not update:
            raise ValueError('Existing install differs at %s. Use --update to retain a backup and replace it.' % target)
        if _is_link(backup_root):
            raise ValueError('Refusing symlink or reparse-point backup directory: %s' % backup_root)
        if backup_root.exists():
            _require_directory(backup_root)
        if _overlap(backup_root, scripts) or _overlap(backup_root, source):
            raise ValueError('Backup directory must be outside Scripts and source, without overlap: %s' % backup_root)

    status = 'installed' if previous is None else 'updated'
    if dry_run:
        return InstallResult('would-install' if previous is None else 'would-update', target)

    scripts.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.fusion360tourdf-stage-', dir=scripts) as staging:
        payload = Path(staging) / EXPORTER
        shutil.copytree(source, payload, symlinks=True,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.pyo'))
        if _contents(payload) != source_contents:
            raise OSError('Staged exporter differs from the validated source. Nothing was installed.')
        expected = dict(source_contents)
        if previous is not None and Path(CONFIG) in previous:
            (payload / CONFIG).write_bytes(previous[Path(CONFIG)])
            expected[Path(CONFIG)] = previous[Path(CONFIG)]
        if _contents(payload) != expected:
            raise OSError('Staged exporter failed validation. Nothing was installed.')

        backup = None
        if previous is not None:
            _require_directory(target)
            if not os.path.samestat(target_stat, target.lstat()) or _contents(target) != previous:
                raise FileExistsError('Install target changed while staging. Refusing to replace %s.' % target)
            backup_root.mkdir(exist_ok=True)
            _require_directory(backup_root)
            backup_holder = Path(tempfile.mkdtemp(prefix=EXPORTER + '-', dir=backup_root))
            backup = backup_holder / EXPORTER
            try:
                _rename_if_absent(target, backup)
            except OSError:
                backup_holder.rmdir()
                raise
        try:
            _rename_if_absent(payload, target)
        except OSError as publish_error:
            if backup is not None:
                try:
                    _rename_if_absent(backup, target)
                except OSError as restore_error:
                    raise OSError('Publish failed (%s). Could not restore (%s). '
                                  'Previous install retained for recovery at %s. '
                                  'Do not overwrite the current destination.' %
                                  (publish_error, restore_error, backup)) from publish_error
                backup_holder.rmdir()
                raise OSError('Publish failed (%s). Previous install restored at %s.' %
                              (publish_error, target)) from publish_error
            raise
    return InstallResult(status, target, backup)


def _report(message, stream):
    encoding = getattr(stream, 'encoding', None)
    if encoding:
        message = message.encode(encoding, 'backslashreplace').decode(encoding)
    print(message, file=stream)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scripts-dir', type=Path, help='Fusion Scripts parent, required on Linux/Wine')
    parser.add_argument('--update', action='store_true', help='Replace a different install and retain its backup outside Scripts')
    parser.add_argument('--dry-run', action='store_true', help='Validate and report without writing any files or directories')
    args = parser.parse_args(argv)
    try:
        scripts = args.scripts_dir
        if scripts is None:
            scripts = default_scripts_dir(sys.platform, os.environ, Path.home())
        result = install(Path(__file__).resolve().parents[1] / EXPORTER, scripts,
                         update=args.update, dry_run=args.dry_run)
    except (OSError, ValueError, RuntimeError) as error:
        _report('Install failed: %s' % error, sys.stderr)
        return 1
    labels = {'installed': 'Installed', 'updated': 'Updated', 'unchanged': 'Unchanged',
              'would-install': 'Would install', 'would-update': 'Would update'}
    _report('%s: %s' % (labels[result.status], result.target), sys.stdout)
    if result.backup is not None:
        _report('Retained backup: %s' % result.backup, sys.stdout)
    return 0


if __name__ == '__main__':
    sys.exit(main())
