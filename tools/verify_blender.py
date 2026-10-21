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
from tools.verify_loaders import mesh_path, expected_link_kinds, asset_receipts, geometry_contract, check_geometry, descriptor_for_mesh
from URDF_Exporter.utils.visual_assets import visual_descriptors


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
    positions = dict(positions)
    dependencies = {j.get('name'): j.find('mimic') for j in joints if j.find('mimic') is not None}
    while dependencies:
        before = len(dependencies)
        for name, mimic in list(dependencies.items()):
            if mimic.get('joint') in positions:
                positions[name] = positions[mimic.get('joint')]*float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
                del dependencies[name]
        assert len(dependencies) < before, 'Unresolvable mimic dependency'
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
    def flatten(value):
        if isinstance(value, (int, float)):
            return [float(value)]
        return [v for item in value for v in flatten(item)]
    actual = flatten(actual)
    expected = flatten(expected)
    assert len(actual) == len(expected)
    assert all(math.isfinite(value) for value in actual)
    difference = max(abs(a - b) for a, b in zip(actual, expected))
    assert difference <= tolerance, f'{label} differs by {difference}, expected {expected}, got {actual}'


def import_geometry(element, package, urdf):
    geometry = element.find('geometry')
    mesh = geometry.find('mesh')
    if mesh is not None:
        path = mesh_path(mesh.get('filename'), package, urdf)
        before = set(bpy.data.objects)
        suffix = Path(path).suffix.lower()
        if suffix == '.obj':
            if hasattr(bpy.ops.wm, 'obj_import'):
                bpy.ops.wm.obj_import(filepath=path, forward_axis='Y', up_axis='Z')
            else:
                bpy.ops.import_scene.obj(filepath=path, axis_forward='Y', axis_up='Z')
        elif suffix == '.stl' and hasattr(bpy.ops.wm, 'stl_import'):
            bpy.ops.wm.stl_import(filepath=path, global_scale=1.0, use_scene_unit=False)
        elif suffix == '.stl':
            bpy.ops.import_mesh.stl(filepath=path, global_scale=1.0, use_scene_unit=False)
        else:
            raise AssertionError('Unsupported geometry format ' + suffix)
        imported = set(bpy.data.objects) - before
        assert imported and all(obj.type == 'MESH' for obj in imported), 'Importer did not produce only mesh geometry'
        if suffix == '.stl':
            assert len(imported) == 1, 'Expected exactly one mesh object per STL'
        scale = vector(mesh, 'scale', '1 1 1')
        return [(obj, obj.matrix_world.copy()) for obj in imported], Matrix.Diagonal((*scale, 1.0))
    box = geometry.find('box')
    assert box is not None, 'This geometry check supports mesh or box geometry'
    bpy.ops.mesh.primitive_cube_add(size=1)
    return [(bpy.context.object, Matrix.Identity(4))], Matrix.Diagonal((*vector(box, 'size', '0 0 0'), 1.0))


def verify_obj_materials(imported, package, descriptor, expected=None):
    from tests.test_visual_assets import decode_assets
    files = {item['path']: (Path(package) / item['path']).read_bytes() for item in descriptor['files']}
    obj, mtl, directory = decode_assets({'descriptor': descriptor, 'files': files})
    wanted = {}
    for name, corners in obj['triangles']:
        wanted.setdefault(name, []).append(sorted(tuple(xyz)+tuple(uv or ()) for xyz, uv, _ in corners))
    found, images = {}, {}
    for part, intrinsic in imported:
        for polygon in part.data.polygons:
            material = part.data.materials[polygon.material_index]
            name = material.name
            if name not in wanted:
                stem, separator, suffix = name.rpartition('.')
                assert separator and suffix.isdecimal() and len(suffix) >= 3 and stem in wanted, 'Unknown imported face material'
                name = stem
            assert name in wanted, 'Unknown imported face material'
            bsdf = next(n for n in material.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
            assert max(abs(a-float(b)) for a, b in zip(bsdf.inputs['Base Color'].default_value[:3], mtl[name]['Kd'])) < 1e-5
            assert abs(bsdf.inputs['Alpha'].default_value-float(mtl[name]['d'][0])) < 1e-5, 'Imported opacity changed'
            textured = 'map_Kd' in mtl[name]
            corners = []
            for loop_index in polygon.loop_indices:
                loop = part.data.loops[loop_index]
                point = intrinsic @ part.data.vertices[loop.vertex_index].co
                uv = tuple(part.data.uv_layers.active.data[loop_index].uv) if textured else ()
                corners.append(tuple(point)+uv)
            found.setdefault(name, []).append(sorted(corners))
            if textured:
                image_nodes = [n for n in material.node_tree.nodes if n.type == 'TEX_IMAGE']
                assert len(image_nodes) == 1 and image_nodes[0].image is not None
                image = image_nodes[0].image
                relative = str(directory / mtl[name]['map_Kd'][0])
                assert Path(bpy.path.abspath(image.filepath)).resolve() == (Path(package) / relative).resolve()
                assert image.size[0] > 0 and image.size[1] > 0 and len(image.pixels) == image.size[0]*image.size[1]*4
                pixels = list(image.pixels)
                if expected:
                    kind = next(i['format'] for i in descriptor['images'] if i['path'] == relative)
                    key = 'png_rgba8' if kind == 'png' else 'progressive_gray8' if list(image.size) == [1, 1] else 'jpeg_rgb8'
                    raw = expected[key]
                    rgba = ([[v, v, v, 255] for v in raw] if key == 'progressive_gray8' else
                            [v+[255] for v in raw] if key == 'jpeg_rgb8' else raw)
                    width, height = image.size
                    bottom_up = [rgba[y*width+x] for y in reversed(range(height)) for x in range(width)]
                    assert max(abs(a-b/255) for a, b in zip(pixels, [v for p in bottom_up for v in p])) < 2/255, 'Blender decoded pixels changed'
                images[relative] = dict(size=list(image.size), pixels_rgba=pixels)
    assert set(found) == set(wanted), 'Blender dropped material subgeometry'
    for name in wanted:
        close(sorted(found[name]), sorted(wanted[name]), 'Blender face/material/UV association', tolerance=1e-4)
    return dict(material_triangles={name: len(rows) for name, rows in found.items()}, images=images,
                limitation='Imported diffuse channels and decoded image data, not native shader/render equivalence')


def verify(urdf, package, output, expected=None, positions=None):
    root = ET.parse(urdf).getroot()
    assert not root.findall('collision'), 'Collisions must be nested in links'
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.context.scene.unit_settings.system = 'METRIC'
    bpy.context.scene.unit_settings.scale_length = 1.0
    cases = expected or {'pose': {'positions': positions or {}}}
    kinds = expected_link_kinds(root, package, next(iter(expected.values())) if expected else None)
    descriptors = asset_receipts(package)
    report = {}
    for case, values in cases.items():
        dimensions, triangles = geometry_contract(values, {name for name, kind in kinds.items() if kind == 'physical'})
        collection = bpy.data.collections.new(case)
        bpy.context.scene.collection.children.link(collection)
        poses = forward_kinematics(root, values['positions'])
        report[case] = {}
        if any(kind == 'frame' for kind in kinds.values()):
            report[case]['frame_poses'] = {name: [list(row) for row in poses[name]]
                                         for name, kind in kinds.items() if kind == 'frame'}
        for link in root.findall('link'):
            name = link.get('name')
            if expected:
                close(poses[name], values['link_poses'][name], case + ' link ' + name)
            frame = bpy.data.objects.new(case + '/' + name, None)
            collection.objects.link(frame)
            frame.matrix_world = poses[name]
            frame.empty_display_size = .03
            if kinds[name] == 'frame':
                continue
            for kind in ('visual', 'collision'):
                aggregate_points, seen_assets, aggregate_polygons = [], [], 0
                for index, element in enumerate(link.findall(kind)):
                    imported, scale = import_geometry(element, package, urdf)
                    descriptor, material_report = None, None
                    if kind == 'visual' and name in descriptors:
                        descriptor = descriptor_for_mesh(descriptors[name], element.find('geometry/mesh').get('filename'))
                        seen_assets.append(descriptor['mesh_path'])
                        wanted = values.get('materials', {}).get(name)
                        if wanted and type(descriptors[name]) is list:
                            wanted = wanted['visuals'][index]
                        material_report = verify_obj_materials(imported, package, descriptor, wanted)
                    points, vertex_count, polygon_count = [], 0, 0
                    for part_index, (obj, intrinsic) in enumerate(imported):
                        for existing_collection in list(obj.users_collection):
                            existing_collection.objects.unlink(obj)
                        collection.objects.link(obj)
                        obj.name = f'{case}/{name}/{kind}/{index}/{part_index}'
                        obj.parent = frame
                        obj.matrix_local = origin(element.find('origin')) @ scale @ intrinsic
                        bpy.context.view_layer.update()
                        obj.data.calc_loop_triangles()
                        check_geometry([tuple(v.co) for v in obj.data.vertices],
                                       [tuple(t.vertices) for t in obj.data.loop_triangles],
                                       dimension=dimensions[name], part=material_report is not None)
                        assert all(polygon.area > 0 for polygon in obj.data.polygons)
                        points.extend(obj.matrix_world @ vertex.co for vertex in obj.data.vertices)
                        vertex_count += len(obj.data.vertices)
                        polygon_count += len(obj.data.polygons)
                        obj.display_type = 'WIRE' if kind == 'collision' else 'SOLID'
                        obj.hide_render = kind == 'collision'
                    minimum = (3, 1) if dimensions[name] == 2 else (4, 4)
                    assert vertex_count >= minimum[0] and polygon_count >= minimum[1]
                    if descriptor is not None:
                        assert polygon_count == descriptor['triangle_count'], 'Blender dropped visual triangles'
                    assert all(math.isfinite(value) for point in points for value in point)
                    aggregate_points.extend(points)
                    aggregate_polygons += polygon_count
                    bounds = [[min(point[i] for point in points) for i in range(3)],
                              [max(point[i] for point in points) for i in range(3)]]
                    if dimensions[name] == 3:
                        assert all(hi > lo for lo, hi in zip(*bounds)), 'Degenerate geometry bounds'
                    if expected:
                        per_visual = values.get(kind + '_descriptor_bounds', {}).get(name)
                        if per_visual is not None:
                            close(bounds, per_visual[index], obj.name + ' descriptor bounds')
                        expected_pose = (values['visual_poses'][name] if kind == 'visual'
                                         else values.get('collision_poses', {}).get(name))
                        if expected_pose is not None:
                            assert element.find('geometry/mesh') is not None, 'Source-coordinate pose requires mesh geometry'
                            expected_matrix = Matrix(expected_pose)
                            expected_matrix = expected_matrix @ Matrix.Diagonal((.001, .001, .001, 1))
                            close(poses[name] @ origin(element.find('origin')) @ scale, expected_matrix, obj.name + ' matrix')
                    report[case][f'{name}/{kind}/{index}'] = {
                        'bounds': bounds,
                        'matrix': [list(row) for row in obj.matrix_world],
                        'vertices': vertex_count, 'polygons': polygon_count,
                        'materials': material_report,
                    }
                if kind == 'visual' and name in descriptors:
                    assert sorted(seen_assets) == sorted(d['mesh_path'] for d in visual_descriptors(descriptors[name])), 'Blender visual descriptor membership changed'
                assert len(aggregate_points) >= (3 if dimensions[name] == 2 else 4), 'Missing aggregate physical geometry'
                if name in triangles:
                    assert aggregate_polygons == triangles[name][kind], 'Geometry triangle count changed'
                aggregate_bounds = [[min(p[i] for p in aggregate_points) for i in range(3)],
                                    [max(p[i] for p in aggregate_points) for i in range(3)]]
                if dimensions[name] == 3:
                    assert all(hi > lo for lo, hi in zip(*aggregate_bounds)), 'Degenerate aggregate physical geometry bounds'
                if expected:
                    close(aggregate_bounds, values.get(kind+'_bounds', values['bounds'])[name], name + '/' + kind + ' aggregate bounds')
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
