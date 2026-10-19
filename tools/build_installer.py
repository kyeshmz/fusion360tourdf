"""Build a single-file Python installer without bundling a Python runtime or Fusion."""

import argparse
import hashlib
from pathlib import Path
import shutil
import sys
import tempfile
import zipapp


BOOTSTRAP = '''import sys

if sys.version_info < (3, 11):
    print("fusion360tourdf installer requires Python 3.11 or newer.", file=sys.stderr)
    sys.exit(1)

import runpy
import tempfile
from pathlib import Path
from zipfile import ZipFile

with tempfile.TemporaryDirectory(prefix="fusion360tourdf-installer-") as temporary:
    with ZipFile(sys.argv[0]) as archive:
        archive.extractall(temporary)
    runpy.run_path(str(Path(temporary) / "tools" / "install_fusion.py"), run_name="__main__")
'''


def build_installer(checkout, output):
    checkout, output = Path(checkout), Path(output)
    if output.exists() or output.is_symlink():
        raise FileExistsError('Refusing to replace an existing output: %s' % output)
    with tempfile.TemporaryDirectory(prefix='fusion360tourdf-build-') as temporary:
        stage = Path(temporary) / 'payload'
        exporter = checkout / 'URDF_Exporter'
        for path in (exporter, *exporter.rglob('*')):
            if path.is_symlink():
                raise ValueError('Refusing a symlink in the exporter payload: %s' % path)
        shutil.copytree(exporter, stage / 'URDF_Exporter', ignore=shutil.ignore_patterns(
            '__pycache__', '*.pyc', '*.pyo', '.DS_Store', 'xacro_export.toml'))
        (stage / 'tools').mkdir()
        shutil.copy2(checkout / 'tools/install_fusion.py', stage / 'tools/install_fusion.py')
        shutil.copy2(checkout / 'LICENSE', stage / 'LICENSE')
        for relative in ('LICENSE', 'THIRD_PARTY_NOTICES.md', 'licenses'):
            if not (stage / 'URDF_Exporter' / relative).exists():
                raise ValueError('Missing distribution notice: %s' % relative)
        (stage / '__main__.py').write_text(BOOTSTRAP, encoding='utf-8')
        archive = Path(temporary) / 'installer.pyz'
        zipapp.create_archive(stage, archive, interpreter='/usr/bin/env python3', compressed=True)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('xb') as destination, archive.open('rb') as source:
            shutil.copyfileobj(source, destination)
        output.chmod(output.stat().st_mode | 0o111)
    return hashlib.sha256(output.read_bytes()).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/fusion360tourdf-install.pyz'),
                        help='New output path (existing files are never overwritten)')
    args = parser.parse_args(argv)
    try:
        digest = build_installer(Path(__file__).resolve().parents[1], args.output)
    except (OSError, ValueError) as error:
        print('Build failed: %s' % error, file=sys.stderr)
        return 1
    print('Built: %s\nSHA256: %s' % (args.output, digest))
    return 0


if __name__ == '__main__':
    sys.exit(main())
