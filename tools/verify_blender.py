"""Blender geometry check, not a native URDF importer or physics simulation."""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.verify_loaders import mesh_path


def vector(element, attribute, default):
    values = [float(value) for value in element.get(attribute, default).split()]
    assert len(values) == 3 and all(math.isfinite(value) for value in values)
    return Vector(values)


def origin(element):
    if element is None:
        return Matrix.Identity(4)
    rotation = Euler(vector(element, 'rpy', '0 0 0'), 'XYZ').to_matrix().to_4x4()
    rotation.translation = vector(element, 'xyz', '0 0 0')
    return rotation


def forward_kinematics(root, positions):
    links = {link.get('name') for link in root.findall('link')}
    joints = root.findall('joint')
    children = {joint.find('child').get('link') for joint in joints}
    roots = links - children
    assert len(roots) == 1, 'URDF must be one rooted tree'
    poses = {roots.pop(): Matrix.Identity(4)}
    pending = list(joints)
    while pending:
        before = len(pending)
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in poses:
                continue
            child = joint.find('child').get('link')
            motion = Matrix.Identity(4)
            kind = joint.get('type')
            if kind in ('revolute', 'continuous', 'prismatic'):
                axis_element = joint.find('axis')
                axis = vector(axis_element if axis_element is not None else ET.Element('axis'),
                              'xyz', '1 0 0').normalized()
                value = positions.get(joint.get('name'), 0.0)
                if kind == 'prismatic':
                    motion.translation = axis * value
                else:
                    motion = Matrix.Rotation(value, 4, axis)
            else:
                assert kind == 'fixed', 'Unsupported Blender verification joint ' + str(kind)
            assert child not in poses, 'Link has multiple parents'
            poses[child] = poses[parent] @ origin(joint.find('origin')) @ motion
            pending.remove(joint)
        assert len(pending) < before, 'Disconnected joint tree'
    assert set(poses) == links
    return poses


def close(actual, expected, label, tolerance=3e-6):
    actual = [float(value) for row in actual for value in row]
    expected = [float(value) for row in expected for value in row]
    assert len(actual) == len(expected)
    assert all(math.isfinite(value) for value in actual)
    difference = max(abs(a - b) for a, b in zip(actual, expected))
    assert difference <= tolerance, f'{label} differs by {difference}, expected {expected}, got {actual}'


def import_geometry(element, package, urdf):
    geometry = element.find('geometry')
    mesh = geometry.find('mesh')
    if mesh is not None:
        path = mesh_path(mesh.get('filename'), package, urdf)
        assert Path(path).suffix.lower() == '.stl', 'This geometry check supports STL exports'
        before = set(bpy.data.objects)
        if hasattr(bpy.ops.wm, 'stl_import'):
            bpy.ops.wm.stl_import(filepath=path, global_scale=1.0, use_scene_unit=False)
        else:
            bpy.ops.import_mesh.stl(filepath=path, global_scale=1.0, use_scene_unit=False)
        imported = set(bpy.data.objects) - before
        assert len(imported) == 1, 'Expected exactly one mesh object per STL'
        obj = imported.pop()
        scale = vector(mesh, 'scale', '1 1 1')
        return obj, Matrix.Diagonal((*scale, 1.0))
    box = geometry.find('box')
    assert box is not None, 'This geometry check supports mesh or box geometry'
    bpy.ops.mesh.primitive_cube_add(size=1)
    return bpy.context.object, Matrix.Diagonal((*vector(box, 'size', '0 0 0'), 1.0))


def verify(urdf, package, output, expected=None, positions=None):
    root = ET.parse(urdf).getroot()
    assert not root.findall('collision'), 'Collisions must be nested in links'
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.context.scene.unit_settings.system = 'METRIC'
    bpy.context.scene.unit_settings.scale_length = 1.0
    cases = expected or {'pose': {'positions': positions or {}}}
    report = {}
    for case, values in cases.items():
        collection = bpy.data.collections.new(case)
        bpy.context.scene.collection.children.link(collection)
        poses = forward_kinematics(root, values['positions'])
        report[case] = {}
        for link in root.findall('link'):
            name = link.get('name')
            if expected:
                close(poses[name], values['link_poses'][name], case + ' link ' + name)
            frame = bpy.data.objects.new(case + '/' + name, None)
            collection.objects.link(frame)
            frame.matrix_world = poses[name]
            frame.empty_display_size = .03
            assert link.findall('visual') and link.findall('collision'), 'Missing link geometry'
            for kind in ('visual', 'collision'):
                for index, element in enumerate(link.findall(kind)):
                    obj, scale = import_geometry(element, package, urdf)
                    for existing_collection in list(obj.users_collection):
                        existing_collection.objects.unlink(obj)
                    collection.objects.link(obj)
                    obj.name = f'{case}/{name}/{kind}/{index}'
                    obj.parent = frame
                    obj.matrix_local = origin(element.find('origin')) @ scale
                    bpy.context.view_layer.update()
                    assert len(obj.data.vertices) >= 4 and len(obj.data.polygons) >= 4
                    assert all(polygon.area > 0 for polygon in obj.data.polygons)
                    points = [obj.matrix_world @ vertex.co for vertex in obj.data.vertices]
                    assert all(math.isfinite(value) for point in points for value in point)
                    bounds = [[min(point[i] for point in points) for i in range(3)],
                              [max(point[i] for point in points) for i in range(3)]]
                    assert all(hi > lo for lo, hi in zip(*bounds)), 'Degenerate geometry bounds'
                    if expected:
                        close(bounds, values['bounds'][name], obj.name + ' bounds')
                        if kind == 'visual':
                            expected_matrix = Matrix(values['visual_poses'][name])
                            expected_matrix = expected_matrix @ Matrix.Diagonal((.001, .001, .001, 1))
                            close(obj.matrix_world, expected_matrix, obj.name + ' matrix')
                    obj.display_type = 'WIRE' if kind == 'collision' else 'SOLID'
                    obj.hide_render = kind == 'collision'
                    report[case][obj.name] = {
                        'bounds': bounds,
                        'matrix': [list(row) for row in obj.matrix_world],
                        'vertices': len(obj.data.vertices),
                    }
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / 'blender.json').write_text(json.dumps(report, indent=2) + '\n')
    bpy.ops.wm.save_as_mainfile(filepath=str(output / 'inspection.blend'))
    print('PASS Blender imported mesh geometry, URDF placement, and articulated bounds')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected', type=Path)
    parser.add_argument('--positions', default='{}', help='Joint position JSON for a real export')
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    verify(args.urdf, args.package, args.output,
           json.loads(args.expected.read_text()) if args.expected else None,
           json.loads(args.positions))
