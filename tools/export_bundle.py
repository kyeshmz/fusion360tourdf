"""Integrity boundary between a native Fusion export and downstream verifiers."""

import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def bundle_hashes(directory, package_name):
    directory = Path(directory).resolve()
    package = directory / package_name
    paths = [directory / 'expected.json', directory / 'assembly.f3d']
    for path in package.rglob('*'):
        if path.is_symlink():
            raise ValueError('Bundle files must not escape the evidence directory: ' + str(path))
        if path.is_file():
            paths.append(path)
    result = {}
    for path in sorted(paths):
        if path.is_symlink() or not path.resolve().is_relative_to(directory):
            raise ValueError('Bundle files must not escape the evidence directory: ' + str(path))
        result[path.relative_to(directory).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def read_bundle(directory):
    directory = Path(directory).resolve()
    report = json.loads((directory / 'native.json').read_text())
    if report.get('status') != 'pass' or report.get('evidence_kind') != 'native-fusion':
        raise ValueError('A passing native-Fusion export receipt is required')
    for key in ('source_preservation', 'entity_identity'):
        if report.get(key, {}).get('status') != 'pass':
            raise ValueError('Native receipt is missing ' + key)
    package_name = report.get('package_name', '')
    if not re.fullmatch(r'[a-z][a-z0-9_]*_description', package_name):
        raise ValueError('Invalid native package name')
    actual = bundle_hashes(directory, package_name)
    if not actual or actual != report.get('outputs_sha256'):
        raise ValueError('Native export bundle changed or is incomplete')
    package = directory / package_name
    if ET.parse(package / 'package.xml').findtext('name') != package_name:
        raise ValueError('Native package identity differs from its receipt')
    expected = json.loads((directory / 'expected.json').read_text())
    if not {'zero', 'moved'} <= expected.keys():
        raise ValueError('Native bundle needs independent zero and moved expectations')
    if set(expected['zero']['positions']) != set(expected['moved']['positions']):
        raise ValueError('Each expected pose must specify every actuated joint')
    return package, expected
