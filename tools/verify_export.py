"""Verify one actual Fusion export through loaders, Blender, and installed ROS 2 motion."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ET

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))

from tools.export_bundle import read_bundle
from tools.verify_fusion.verification import capture_provenance
from tools.verify_loaders import expand, load_and_check


def negative_controls(urdf, package, expected, output):
    output = Path(output)
    output.mkdir()
    tree = ET.parse(urdf)
    joint = next(joint for joint in tree.findall('joint') if joint.get('type') != 'fixed')
    axis = joint.find('axis')
    axis.set('xyz', '0 1 0' if axis.get('xyz') == '1 0 0' else '1 0 0')
    changed = output / 'wrong-axis.urdf'
    tree.write(changed)
    cases = [('wrong_axis', changed, package)]
    for kind in ('shifted_mesh', 'missing_mesh'):
        copied = output / kind / package.name
        shutil.copytree(package, copied)
        mesh = next((copied / 'meshes').glob('*.stl'))
        if kind == 'missing_mesh':
            mesh.unlink()
        else:
            data = bytearray(mesh.read_bytes())
            for triangle in range(struct.unpack_from('<I', data, 80)[0]):
                for vertex in range(3):
                    offset = 84 + triangle * 50 + 12 + vertex * 12
                    struct.pack_into('<f', data, offset, struct.unpack_from('<f', data, offset)[0] + 100)
            mesh.write_bytes(data)
        cases.append((kind, urdf, copied))
    result = {}
    for name, model, source in cases:
        try:
            load_and_check(model, source, expected)
        except (AssertionError, ValueError) as error:
            result[name] = {'status': 'pass', 'rejected_with': str(error)}
        else:
            raise AssertionError('Corrupted export was accepted: ' + name)
    return result


def copy_fusion_image(bundle, output, prefix):
    metadata = json.loads((bundle / 'native.json').read_text()).get('images', {}).get('fusion')
    if metadata is None:
        return None
    source = bundle / 'fusion.png'
    if hashlib.sha256(source.read_bytes()).hexdigest() != metadata['sha256']:
        raise ValueError('Native Fusion screenshot changed since capture')
    filename = prefix + '-fusion.png'
    shutil.copy2(source, output / filename)
    return dict(metadata, path=filename)


def verify_export(bundle, output_parent, distros=('humble', 'jazzy')):
    bundle = Path(bundle).resolve()
    package, expected = read_bundle(bundle)
    receipt_hash = hashlib.sha256((bundle / 'native.json').read_bytes()).hexdigest()
    output_parent = Path(output_parent).resolve()
    output_parent.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='export-e2e-', dir=output_parent))
    report = dict(status='running', evidence_kind='actual-native-fusion-export',
                  bundle=str(bundle), native_receipt_sha256=receipt_hash,
                  provenance=capture_provenance(REPOSITORY, None), commands=[], stages={})
    report_path = output / 'summary.json'

    def save():
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')

    def command(arguments, log):
        report['commands'].append(arguments)
        save()
        with (output / log).open('w') as stream:
            subprocess.run(arguments, cwd=REPOSITORY, stdout=stream, stderr=subprocess.STDOUT, check=True)

    save()
    print('End-to-end evidence: ' + str(output), flush=True)
    try:
        urdf = expand(package, output / 'robot.urdf')
        command([sys.executable, str(REPOSITORY / 'tools/validate_urdf.py'), str(urdf),
                 '--meshes', str(package / 'meshes'), '--strict'], 'lint.log')
        loaders = load_and_check(urdf, package, expected)
        (output / 'loaders.json').write_text(json.dumps(loaders, indent=2) + '\n')
        report['stages']['loaders'] = {'status': 'pass', 'poses': list(loaders)}
        report['stages']['negative_controls'] = negative_controls(urdf, package, expected, output / 'negative-controls')
        save()
        blender = os.environ.get('BLENDER', 'blender')
        if not shutil.which(blender):
            raise RuntimeError('Blender is required; set BLENDER to its executable path')
        command([blender, '--background', '--factory-startup', '--python-exit-code', '1',
                 '--python', str(REPOSITORY / 'tools/verify_blender.py'), '--',
                 '--urdf', str(urdf), '--package', str(package), '--expected', str(bundle / 'expected.json'),
                 '--output', str(output / 'blender')], 'blender.log')
        blender_result = json.loads((output / 'blender/blender.json').read_text())
        if set(blender_result) != set(expected) or not (output / 'blender/inspection.blend').is_file():
            raise AssertionError('Blender did not retain every expected pose and inspection scene')
        report['stages']['blender'] = {'status': 'pass', 'poses': list(blender_result)}
        save()
        prefix = package.name[:-len('_description')].replace('_', '-')
        command([blender, '--background', '--factory-startup', '--disable-autoexec', '--python-exit-code', '1',
                 '--python', str(REPOSITORY / 'tools/render_readme.py'), '--',
                 '--inspection', str(output / 'blender/inspection.blend'), '--expected', str(bundle / 'expected.json'),
                 '--output', str(output / 'images'), '--prefix', prefix], 'images.log')
        images = json.loads((output / 'images/images.json').read_text())
        native_image = copy_fusion_image(bundle, output / 'images', prefix)
        if native_image is not None:
            images['images']['fusion'] = native_image
        images['native_receipt_sha256'] = receipt_hash
        (output / 'images/images.json').write_text(json.dumps(images, indent=2) + '\n')
        report['stages']['images'] = {'status': 'pass', 'files': [item['path'] for item in images['images'].values()]}
        save()
        for distro in distros:
            runtime = output / distro
            command(['bash', str(REPOSITORY / 'tools/verify_docker.sh'), distro, str(runtime), str(bundle)],
                    distro + '.log')
            reports = list(runtime.glob('docker-*/ros-*/ros.json'))
            if len(reports) != 1:
                raise AssertionError('Expected one completed ROS report for ' + distro)
            ros = json.loads(reports[0].read_text())
            if ros.get('evidence_kind') != 'installed-native-fusion-export':
                raise AssertionError('ROS verified fixtures instead of the native export')
            control = ros['packages'][package.name[:-len('_description')]]['control']
            if control['action_error_code'] != 0:
                raise AssertionError('Native export trajectory failed')
            report['stages'][distro] = {'status': 'pass', 'ros_report': str(reports[0]),
                                       'controllers': control['controllers'], 'action_error_code': 0}
            save()
        read_bundle(bundle)
        if hashlib.sha256((bundle / 'native.json').read_bytes()).hexdigest() != receipt_hash:
            raise AssertionError('Native receipt changed during verification')
        report['status'] = 'pass'
    except Exception:
        report.update(status='fail', traceback=traceback.format_exc())
        raise
    finally:
        save()
        print('Report: ' + str(report_path), flush=True)
    return report_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path, help='Directory produced by the Fusion create_demo script')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--distros', nargs='+', choices=('humble', 'jazzy'), default=['humble', 'jazzy'])
    args = parser.parse_args()
    verify_export(args.bundle, args.output, tuple(dict.fromkeys(args.distros)))
    print('PASS actual Fusion export: loader, Blender, installed ROS, controllers, trajectory feedback, and TF')


if __name__ == '__main__':
    main()
