"""Expand and load exported packages with xacro, urdfdom, and yourdfpy."""

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def mesh_path(filename, package, urdf):
    package = Path(package).resolve()
    if filename.startswith('package://'):
        name, relative = filename[len('package://'):].split('/', 1)
        package_name = ET.parse(package / 'package.xml').findtext('name')
        if name != package_name:
            raise AssertionError('External mesh package is not self-contained: ' + filename)
        path = package / relative
    elif filename.startswith('file://'):
        path = Path(filename[len('file://'):])
    else:
        path = Path(urdf).parent / filename
    path = path.resolve()
    if not path.is_relative_to(package) or not path.is_file():
        raise AssertionError('Missing or external mesh: ' + str(path))
    return str(path)


def expand(package, output):
    try:
        import xacro
    except ImportError as exc:
        raise RuntimeError('Selected loader suite requires requirements-test.txt') from exc
    files = list((Path(package) / 'urdf').glob('*.xacro'))
    if len(files) != 1:
        raise AssertionError('Expected one self-contained xacro')
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xacro.process_file(str(files[0])).toxml())
    return output


def check_urdfdom(urdf, log):
    executable = os.environ.get('CHECK_URDF', 'check_urdf')
    if not shutil.which(executable):
        raise RuntimeError('Selected loader suite requires check_urdf from urdfdom')
    result = subprocess.run([executable, str(urdf)], text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=30)
    Path(log).write_text(result.stdout)
    if result.returncode != 0 or 'Successfully Parsed XML' not in result.stdout:
        raise AssertionError('urdfdom rejected the model. See ' + str(log))
    return result.stdout


def load_and_check(urdf, package, expected=None, collision_method='visual'):
    try:
        import numpy as np
        from yourdfpy import URDF
    except ImportError as exc:
        raise RuntimeError('Selected loader suite requires requirements-test.txt') from exc
    root = ET.parse(urdf).getroot()
    links = root.findall('link')
    assert links and not root.findall('collision'), 'Collisions must belong to links'
    for link in links:
        assert link.findall('visual'), 'Missing visual on ' + link.get('name')
        assert link.findall('collision'), 'Missing collision on ' + link.get('name')
        for mesh in link.findall('.//mesh'):
            mesh_path(mesh.get('filename'), package, urdf)
            scale = [float(v) for v in mesh.get('scale', '1 1 1').split()]
            assert len(scale) == 3 and all(math.isfinite(v) and v > 0 for v in scale)
            if expected:
                np.testing.assert_allclose(scale, [.001, .001, .001], atol=1e-12, rtol=0)
        if expected:
            kind = 'box' if collision_method == 'primitive_box' else 'mesh'
            assert link.find('collision/geometry/' + kind) is not None
    robot = URDF.load(str(urdf), filename_handler=lambda fname: mesh_path(fname, package, urdf),
                      build_scene_graph=True, build_collision_scene_graph=True,
                      load_meshes=True, load_collision_meshes=True, force_mesh=True)
    assert robot.validate_filenames(), 'Mesh resolver could not load every resource'
    cases = expected or {'zero': {'positions': {name: 0.0 for name in robot.actuated_joint_names}}}
    report = {}
    for case, values in cases.items():
        robot.update_cfg(values['positions'])
        if expected:
            assert set(robot.link_map) == set(values['link_poses']), 'Unexpected link membership'
            for link, pose in values['link_poses'].items():
                np.testing.assert_allclose(robot.get_transform(link), pose, atol=2e-6, rtol=0,
                                           err_msg=case + ' FK ' + link)
                np.testing.assert_allclose(robot.get_transform(link, collision_geometry=True),
                                           pose, atol=2e-6, rtol=0)
        report[case] = {}
        for kind, scene in (('visual', robot.scene), ('collision', robot.collision_scene)):
            assert len(scene.geometry) > 0, 'Loader produced an empty ' + kind + ' scene'
            loaded = set()
            bounds = {}
            for node in scene.graph.nodes_geometry:
                transform, geometry_name = scene.graph.get(node)
                mesh = scene.geometry[geometry_name]
                vertices = np.asarray(mesh.vertices)
                assert len(vertices) >= 4 and len(mesh.faces) >= 4, 'Empty or degenerate mesh'
                assert np.isfinite(vertices).all() and np.isfinite(transform).all()
                assert np.all(np.asarray(mesh.area_faces) > 0), 'Degenerate triangle'
                points = vertices @ transform[:3, :3].T + transform[:3, 3]
                actual = np.array([points.min(axis=0), points.max(axis=0)])
                assert np.all(actual[1] > actual[0]), 'Geometry has no volume bounds'
                owner = scene.graph.transforms.parents[node]
                assert owner in robot.link_map, 'Geometry is not attached to a link'
                loaded.add(owner)
                bounds[owner] = actual.tolist()
                if expected:
                    np.testing.assert_allclose(actual, values['bounds'][owner], atol=2e-6, rtol=0,
                                               err_msg=case + ' ' + kind + ' bounds ' + owner)
            assert loaded == set(robot.link_map), 'Not all link geometry was loaded'
            report[case][kind] = bounds
    return report


def verify_fixtures(output):
    from tests.fixtures import VARIANTS, generate_fixture, oracle

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = {}
    for variant in VARIANTS:
        package = generate_fixture(output, variant)
        urdf = expand(package, output / ('fixture_' + variant + '.urdf'))
        check_urdfdom(urdf, output / (variant + '-urdfdom.log'))
        report[variant] = load_and_check(
            urdf, package, oracle(variant == 'fixed'),
            'primitive_box' if variant == 'boxes' else 'visual')
    (output / 'loaders.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--package', type=Path)
    args = parser.parse_args()
    if bool(args.urdf) != bool(args.package):
        parser.error('--urdf and --package must be supplied together')
    if args.urdf:
        args.output.mkdir(parents=True, exist_ok=True)
        check_urdfdom(args.urdf, args.output / 'urdfdom.log')
        report = load_and_check(args.urdf, args.package)
        (args.output / 'loaders.json').write_text(json.dumps(report, indent=2) + '\n')
    else:
        report = verify_fixtures(args.output)
    if args.urdf:
        print('PASS urdfdom and zero-pose visual/collision mesh loading')
    else:
        print('PASS xacro, urdfdom, loaded visual/collision meshes, and numeric FK')
    print('Report at ' + str(args.output / 'loaders.json'))


if __name__ == '__main__':
    main()
