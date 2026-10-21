"""Expand and load exported packages with xacro, urdfdom, and yourdfpy."""

import argparse
import hashlib
import json
import math
from fractions import Fraction
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from URDF_Exporter.utils.visual_assets import visual_descriptors
from URDF_Exporter.utils.stl import read_binary_stl


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


def expected_link_kinds(root, package, expected=None):
    names = {link.get('name') for link in root.findall('link')}
    if expected and 'link_kinds' in expected:
        kinds = expected['link_kinds']
    else:
        sidecar = Path(package) / 'config/robot_data.yaml'
        receipt = json.loads(sidecar.read_text()) if sidecar.is_file() else {}
        sources = receipt.get('link_sources')
        kinds = {name: entry['kind'] for name, entry in sources.items()} if sources is not None else dict.fromkeys(names, 'physical')
        if sources is not None and root.get('name') != receipt.get('robot_name'):
            assert len(names) == 1 and not root.findall('joint'), 'Unexpected component URDF shape'
            name = next(iter(names))
            assert root.get('name') == receipt['robot_name'] + '_' + name, 'Unknown component URDF identity'
            kinds = {name: kinds[name]}
    assert set(kinds) == names, 'Link-kind declaration does not match link membership'
    assert set(kinds.values()) <= {'physical', 'frame'}, 'Unknown link kind'
    for link in root.findall('link'):
        name = link.get('name')
        if kinds[name] == 'frame':
            assert not any(link.findall(tag) for tag in ('inertial', 'visual', 'collision')), 'Frame contains physical data: ' + name
            assert not link.findall('.//mesh'), 'Frame contains mesh data: ' + name
            assert not (Path(package) / 'meshes' / (name + '.stl')).exists(), 'Frame has an unexpected mesh target: ' + name
        else:
            assert link.find('inertial/mass') is not None, 'Missing physical inertia on ' + name
            assert link.findall('visual') and link.findall('collision'), 'Missing physical geometry on ' + name
    return kinds


def asset_receipts(package):
    from URDF_Exporter.utils.visual_assets import validate_asset_bundles
    package = Path(package).resolve()
    sidecar_path = package / 'config/robot_data.yaml'
    sidecar = json.loads(sidecar_path.read_text()) if sidecar_path.is_file() else {}
    descriptors = sidecar.get('visual_assets', {})
    if not descriptors:
        return {}
    bundles = {}
    for name, value in descriptors.items():
        parts = [_read_asset_bundle(package, descriptor) for descriptor in visual_descriptors(value)]
        bundles[name] = parts if type(value) is list else parts[0]
    policies = {surface['appearance']['shader_policy'] for value in descriptors.values()
                for d in visual_descriptors(value) for surface in d['surfaces']}
    assert len(policies) == 1, 'Mixed visual policies are not supported in one package'
    kinds = {name: dict(kind=entry['kind'], source_path=entry.get('path'))
             for name, entry in sidecar['link_sources'].items()}
    validate_asset_bundles(bundles, kinds, policies.pop(), sidecar.get('visual_meshes', 'combined'))
    return descriptors


def _read_asset_bundle(package, descriptor):
    files = {}
    for entry in descriptor['files']:
        relative = Path(entry['path'])
        path = package / relative
        assert not relative.is_absolute() and '..' not in relative.parts
        assert path.resolve().is_relative_to(package) and path.is_file() and not path.is_symlink(), 'Missing or external visual asset'
        assert not any(parent.is_symlink() for parent in path.parents if parent.is_relative_to(package)), 'Symlink visual asset directory'
        data = path.read_bytes()
        assert len(data) == entry['size'] and hashlib.sha256(data).hexdigest() == entry['sha256'], 'Visual asset manifest mismatch'
        files[entry['path']] = data
    return {'descriptor': descriptor, 'files': files}


def descriptor_for_mesh(value, filename):
    matches = [d for d in visual_descriptors(value) if filename.endswith('/' + d['mesh_path'])]
    assert len(matches) == 1, 'Visual URI has no unique asset descriptor'
    return matches[0]


def geometry_contract(values, physical):
    dimensions = values.get('geometry_dimensions', {})
    triangles = values.get('geometry_triangles', {})
    assert set(dimensions) <= physical and set(triangles) <= physical, 'Unknown geometry declaration owner'
    for name, dimension in dimensions.items():
        assert type(dimension) is int and dimension in (2, 3), 'Geometry dimension must be 2 or 3'
        if dimension == 2:
            counts = triangles.get(name, {})
            assert set(counts) == {'visual', 'collision'}, 'Surface requires independent triangle counts for both geometry kinds'
            assert all(type(n) is int and n > 0 for n in counts.values()), 'Invalid surface triangle count'
    return {name: dimensions.get(name, 3) for name in physical}, triangles


def check_geometry(vertices, faces, *, dimension=3, expected_faces=None, part=False):
    vertices = [tuple(float(v) for v in point) for point in vertices]
    faces = [tuple(int(i) for i in face) for face in faces]
    minimum = (3, 1) if dimension == 2 or part else (4, 4)
    assert len(vertices) >= minimum[0] and len(faces) >= minimum[1], 'Empty or degenerate mesh'
    assert all(len(p) == 3 and all(math.isfinite(v) for v in p) for p in vertices), 'Nonfinite geometry'
    normals = []
    for face in faces:
        assert len(face) == 3 and all(0 <= i < len(vertices) for i in face), 'Invalid triangle indices'
        a, b, c = [vertices[i] for i in face]
        u, v = [[p[i]-a[i] for i in range(3)] for p in (b, c)]
        normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        length = math.hypot(*normal)
        assert math.isfinite(length) and length > 0, 'Degenerate triangle'
        normals.append(tuple(v/length for v in normal))
    if expected_faces is not None:
        assert len(faces) == expected_faces, 'Geometry triangle count changed'
    if dimension == 2:
        anchor = vertices[faces[0][0]]
        scale = max(math.dist(anchor, p) for p in vertices)
        assert all(abs(sum((p[i]-anchor[i])*normals[0][i] for i in range(3))) <= scale*1e-10
                   for p in vertices), 'Declared surface is not planar'
    elif not part:
        assert all(max(p[i] for p in vertices) > min(p[i] for p in vertices) for i in range(3)), 'Geometry has no volume bounds'
        a,b,c = [tuple(Fraction(v) for v in vertices[i]) for i in faces[0]]
        u,v = [tuple(p[i]-a[i] for i in range(3)) for p in (b,c)]
        normal = (u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
        assert any(sum((Fraction(p[i])-a[i])*normal[i] for i in range(3)) != 0
                   for p in set(vertices)), 'Undeclared planar geometry has no solid dimension'


def inspect_material_assets(package, descriptors, expected=None):
    import numpy as np
    import trimesh
    from PIL import Image
    from tests.test_visual_assets import decode_assets
    report = {}
    for name, descriptor in descriptors.items():
        if type(descriptor) is list:
            wanted = expected[name]['visuals'] if expected else [None] * len(descriptor)
            assert len(wanted) == len(descriptor), 'Per-body visual oracle membership changed'
            report[name] = [inspect_material_assets(package, {name: d}, {name: e} if e else None)[name]
                            for d, e in zip(descriptor, wanted)]
            continue
        path = Path(package) / descriptor['mesh_path']
        scene = trimesh.load(str(path), force='scene', process=False)
        files = {entry['path']: (Path(package) / entry['path']).read_bytes() for entry in descriptor['files']}
        obj, mtl, image_directory = decode_assets({'descriptor': descriptor, 'files': files})
        assert sum(len(g.faces) for g in scene.geometry.values()) == descriptor['triangle_count'], 'OBJ subgeometry was dropped'
        wanted = {}
        for surface in descriptor['surfaces']:
            wanted[surface['material']] = wanted.get(surface['material'], 0) + surface['triangle_count']
        found = {}
        for geometry in scene.geometry.values():
            material = geometry.visual.material
            assert material.name in wanted, 'Unknown loaded OBJ material'
            found[material.name] = found.get(material.name, 0) + len(geometry.faces)
            assert np.all(np.isfinite(geometry.vertices)) and np.all(geometry.area_faces > 0)
            source_material = mtl[material.name]
            np.testing.assert_allclose(material.diffuse[:3], np.array(source_material['Kd'], dtype=float)*255, atol=1, rtol=0)
            assert list(material.kwargs['d']) == source_material['d'], 'Loaded MTL opacity changed'
            wanted_triangles = []
            for selected, corners in obj['triangles']:
                if selected == material.name:
                    wanted_triangles.append(sorted(tuple(xyz)+tuple(uv or ()) for xyz, uv, _ in corners))
            loaded_triangles = []
            for triangle in geometry.faces:
                loaded_triangles.append(sorted(tuple(geometry.vertices[i]) +
                    (tuple(geometry.visual.uv[i]) if 'map_Kd' in source_material else ()) for i in triangle))
            np.testing.assert_allclose(sorted(loaded_triangles), sorted(wanted_triangles), atol=1e-10, rtol=0,
                                       err_msg='OBJ triangle/material/UV association changed')
            if material.image is not None:
                material.image.load()
                assert geometry.visual.uv is not None and np.isfinite(geometry.visual.uv).all(), 'Missing loaded UVs'
                source_path = Path(package) / image_directory / source_material['map_Kd'][0]
                with Image.open(source_path) as source_image:
                    source_image.load()
                    assert material.image.size == source_image.size
                    assert material.image.convert('RGBA').tobytes() == source_image.convert('RGBA').tobytes(), 'Loaded material image changed'
            else:
                assert 'map_Kd' not in source_material, 'Loader silently dropped a texture'
        assert found == wanted, 'OBJ face/material membership changed'
        images = []
        for image in descriptor['images']:
            with Image.open(Path(package) / image['path']) as decoded:
                decoded.load()
                mode = 'RGBA' if image['format'] == 'png' else 'L' if decoded.mode == 'L' else 'RGB'
                pixels = np.asarray(decoded.convert(mode)).reshape((-1, len(mode) if mode != 'L' else 1)).tolist()
                if expected:
                    key = 'png_rgba8' if image['format'] == 'png' else 'progressive_gray8' if mode == 'L' else 'jpeg_rgb8'
                    wanted_pixels = expected[name][key]
                    assert pixels == ([[v] for v in wanted_pixels] if mode == 'L' else wanted_pixels), 'Decoded material pixels changed'
                images.append(dict(path=image['path'], size=list(decoded.size), mode=mode, pixels=pixels))
        if expected:
            assert descriptor['triangle_count'] == expected[name]['triangles'] and len(found) == expected[name]['materials']
        report[name] = dict(material_triangles=found, images=images,
                            opacity_note='MTL d remains separately verified; trimesh does not promise rendering fidelity')
    return report


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
    kinds = expected_link_kinds(root, package, next(iter(expected.values())) if expected else None)
    descriptors = asset_receipts(package)
    physical = {name for name, kind in kinds.items() if kind == 'physical'}
    checked_stls = set()
    for link in links:
        if kinds[link.get('name')] == 'frame':
            continue
        for mesh in link.findall('.//mesh'):
            path = mesh_path(mesh.get('filename'), package, urdf)
            if Path(path).suffix.lower() == '.stl' and path not in checked_stls:
                read_binary_stl(path)
                checked_stls.add(path)
            scale = [float(v) for v in mesh.get('scale', '1 1 1').split()]
            assert len(scale) == 3 and all(math.isfinite(v) and v > 0 for v in scale)
            if expected:
                np.testing.assert_allclose(scale, [.001, .001, .001], atol=1e-12, rtol=0)
        if expected:
            kind = 'box' if collision_method == 'primitive_box' else 'mesh'
            assert link.find('collision/geometry/' + kind) is not None
    robot = URDF.load(str(urdf), filename_handler=lambda fname: mesh_path(fname, package, urdf),
                      build_scene_graph=True, build_collision_scene_graph=True,
                      load_meshes=True, load_collision_meshes=True,
                       force_mesh=False, force_collision_mesh=False)
    assert robot.validate_filenames(), 'Mesh resolver could not load every resource'
    cases = expected or {'zero': {'positions': {name: 0.0 for name in robot.actuated_joint_names}}}
    report = {}
    for case, values in cases.items():
        dimensions, triangles = geometry_contract(values, physical)
        robot.update_cfg(values['positions'])
        if expected:
            assert set(robot.actuated_joint_names) == set(values['positions']), 'Independent DOF membership changed'
        if expected:
            assert set(robot.link_map) == set(values['link_poses']), 'Unexpected link membership'
            for link, pose in values['link_poses'].items():
                np.testing.assert_allclose(robot.get_transform(link), pose, atol=2e-6, rtol=0,
                                           err_msg=case + ' FK ' + link)
                np.testing.assert_allclose(robot.get_transform(link, collision_geometry=True),
                                            pose, atol=2e-6, rtol=0)
            for name, mass in values.get('masses', {}).items():
                assert float(root.find("link[@name='%s']/inertial/mass" % name).get('value')) == mass, 'Physical mass changed'
            for name, physics in values.get('physics', {}).items():
                entry = root.find("link[@name='%s']/inertial" % name)
                np.testing.assert_allclose([float(v) for v in entry.find('origin').get('xyz').split()],
                    physics['com'], atol=1e-10, rtol=0, err_msg='Aggregate CAD COM changed ' + name)
                np.testing.assert_allclose([float(entry.find('inertia').get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'iyz', 'ixz')],
                    physics['inertia'], atol=1e-10, rtol=0, err_msg='Aggregate CAD tensor changed ' + name)
        report[case] = {}
        if any(kind == 'frame' for kind in kinds.values()):
            report[case]['frames'] = {name: robot.get_transform(name).tolist()
                                      for name, kind in kinds.items() if kind == 'frame'}
        for kind, scene in (('visual', robot.scene), ('collision', robot.collision_scene)):
            assert len(scene.geometry) > 0 or not physical, 'Loader produced an empty physical ' + kind + ' scene'
            loaded = set()
            counts = {}
            bounds = {}
            face_counts = {}
            asset_triangles = {}
            for node in scene.graph.nodes_geometry:
                transform, geometry_name = scene.graph.get(node)
                mesh = scene.geometry[geometry_name]
                owner = scene.graph.transforms.parents[node]
                vertices = np.asarray(mesh.vertices)
                obj_part = kind == 'visual' and owner in descriptors
                check_geometry(vertices, mesh.faces, dimension=dimensions[owner], part=obj_part)
                assert np.isfinite(vertices).all() and np.isfinite(transform).all()
                assert np.all(np.asarray(mesh.area_faces) > 0), 'Degenerate triangle'
                points = vertices @ transform[:3, :3].T + transform[:3, 3]
                if obj_part:
                    material = mesh.visual.material.name
                    for face in mesh.faces:
                        corners = sorted(tuple(round(float(v), 9) for v in points[i]) for i in face)
                        asset_triangles.setdefault(owner, []).append((material, corners))
                actual = np.array([points.min(axis=0), points.max(axis=0)])
                if not obj_part and dimensions[owner] == 3:
                    assert np.all(actual[1] > actual[0]), 'Geometry has no volume bounds'
                assert owner in robot.link_map, 'Geometry is not attached to a link'
                loaded.add(owner)
                counts[owner] = counts.get(owner, 0) + 1
                face_counts[owner] = face_counts.get(owner, 0) + len(mesh.faces)
                previous = bounds.get(owner)
                bounds[owner] = (np.array([np.minimum(previous[0], actual[0]), np.maximum(previous[1], actual[1])])
                                 if previous is not None else actual).tolist()
            assert loaded == physical, 'Not all physical link geometry was loaded'
            expected_counts = {}
            for link in links:
                owner = link.get('name')
                if owner not in physical:
                    continue
                count = 0
                for element in link.findall(kind):
                    mesh_element = element.find('geometry/mesh')
                    if kind == 'visual' and owner in descriptors and mesh_element is not None:
                        descriptor = descriptor_for_mesh(descriptors[owner], mesh_element.get('filename'))
                        count += len({surface['material'] for surface in descriptor['surfaces']})
                    else:
                        count += 1
                expected_counts[owner] = count
                if kind == 'visual' and owner in descriptors:
                    declared = [d['mesh_path'] for d in visual_descriptors(descriptors[owner])]
                    actual_paths = [descriptor_for_mesh(descriptors[owner], e.find('geometry/mesh').get('filename'))['mesh_path']
                                    for e in link.findall(kind)]
                    assert sorted(actual_paths) == sorted(declared), 'Visual descriptor/resource membership changed'
            assert counts == expected_counts, 'Loader dropped or duplicated geometry elements or OBJ material parts'
            for owner, actual in bounds.items():
                if dimensions[owner] == 3:
                    assert np.all(np.asarray(actual)[1] > np.asarray(actual)[0]), 'Aggregate physical geometry has no volume bounds'
                if owner in triangles:
                    assert face_counts[owner] == triangles[owner][kind], 'Geometry triangle count changed'
            if expected:
                for owner, actual in bounds.items():
                    np.testing.assert_allclose(actual, values.get(kind + '_bounds', values['bounds'])[owner],
                                               atol=2e-6, rtol=0, err_msg=case + ' ' + kind + ' bounds ' + owner)
            if kind == 'visual':
                from tests.test_visual_assets import decode_assets
                from URDF_Exporter.utils import transforms as tf
                for link in links:
                    owner = link.get('name')
                    if owner not in descriptors:
                        continue
                    wanted_triangles = []
                    for index, element in enumerate(link.findall('visual')):
                        descriptor = descriptor_for_mesh(descriptors[owner], element.find('geometry/mesh').get('filename'))
                        files = {e['path']: (Path(package) / e['path']).read_bytes() for e in descriptor['files']}
                        obj, _, _ = decode_assets({'descriptor': descriptor, 'files': files})
                        origin = element.find('origin')
                        local = np.eye(4)
                        local[:3, :3] = np.asarray(tf.mat3_from_rpy([float(v) for v in origin.get('rpy').split()])).reshape(3, 3)
                        local[:3, 3] = [float(v) for v in origin.get('xyz').split()]
                        transform = robot.get_transform(owner) @ local
                        points = []
                        for material, corners in obj['triangles']:
                            world = [(np.asarray(xyz)*.001) @ transform[:3, :3].T + transform[:3, 3]
                                     for xyz, _, _ in corners]
                            points.extend(world)
                            wanted_triangles.append((material, sorted(tuple(round(float(v), 9) for v in p) for p in world)))
                        per_visual = values.get('visual_descriptor_bounds', {}).get(owner)
                        if per_visual is not None:
                            np.testing.assert_allclose([np.min(points, axis=0), np.max(points, axis=0)],
                                per_visual[index], atol=2e-6, rtol=0, err_msg=case + ' body visual bounds ' + owner)
                    actual_triangles = sorted(asset_triangles.get(owner, []))
                    wanted_triangles.sort()
                    assert [m for m, _ in actual_triangles] == [m for m, _ in wanted_triangles], 'Loaded visual body/material triangle membership changed'
                    np.testing.assert_allclose([t for _, t in actual_triangles], [t for _, t in wanted_triangles],
                        atol=2e-6, rtol=0, err_msg=case + ' visual body/material world triangles ' + owner)
            report[case][kind] = bounds
        if descriptors:
            report[case]['materials'] = inspect_material_assets(package, descriptors, values.get('materials'))
    return report


def verify_fixtures(output, fixture=None):
    from tests.fixtures import fixture_variants, generate_fixture, fixture_oracle, fixture_collision_method

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = {}
    for variant in fixture_variants(fixture):
        package = generate_fixture(output, variant)
        urdf = expand(package, output / ('fixture_' + variant + '.urdf'))
        check_urdfdom(urdf, output / (variant + '-urdfdom.log'))
        report[variant] = load_and_check(
            urdf, package, fixture_oracle(variant),
            fixture_collision_method(variant))
    (output / 'loaders.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    from tests.fixtures import OPTIONAL_VARIANTS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--package', type=Path)
    parser.add_argument('--fixture', choices=OPTIONAL_VARIANTS)
    args = parser.parse_args()
    if bool(args.urdf) != bool(args.package):
        parser.error('--urdf and --package must be supplied together')
    if args.urdf:
        args.output.mkdir(parents=True, exist_ok=True)
        check_urdfdom(args.urdf, args.output / 'urdfdom.log')
        report = load_and_check(args.urdf, args.package)
        (args.output / 'loaders.json').write_text(json.dumps(report, indent=2) + '\n')
    else:
        report = verify_fixtures(args.output, args.fixture)
    if args.urdf:
        print('PASS urdfdom and zero-pose visual/collision mesh loading')
    else:
        print('PASS xacro, urdfdom, loaded visual/collision meshes, and numeric FK')
    print('Report at ' + str(args.output / 'loaders.json'))


if __name__ == '__main__':
    main()
