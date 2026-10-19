"""Render checked URDF inspection poses as README images inside headless Blender."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.verify_blender import close


def render_images(inspection, expected_path, output, prefix):
    if not re.fullmatch(r'[a-z][a-z0-9_-]*', prefix):
        raise ValueError('Image prefix must be a lowercase filename stem')
    inspection, expected_path, output = map(lambda path: Path(path).resolve(), (inspection, expected_path, output))
    expected = json.loads(expected_path.read_text())
    bpy.ops.wm.open_mainfile(filepath=str(inspection), use_scripts=False)
    scene = bpy.context.scene
    poses, points = {}, []
    for name in ('zero', 'moved'):
        collection = bpy.data.collections.get(name)
        if collection is None or name not in expected:
            raise ValueError('Inspection needs checked zero and moved pose collections')
        visuals = [obj for obj in collection.objects if obj.type == 'MESH' and '/visual/' in obj.name]
        if {obj.name.split('/')[1] for obj in visuals} != set(expected[name]['bounds']):
            raise ValueError('Inspection visual links differ from the native expectations')
        for obj in visuals:
            vertices = [obj.matrix_world @ vertex.co for vertex in obj.data.vertices]
            actual = [[min(point[i] for point in vertices) for i in range(3)],
                      [max(point[i] for point in vertices) for i in range(3)]]
            close(actual, expected[name]['bounds'][obj.name.split('/')[1]], obj.name)
            points.extend(vertices)
        poses[name] = visuals
    low = Vector(tuple(min(point[i] for point in points) for i in range(3)))
    high = Vector(tuple(max(point[i] for point in points) for i in range(3)))
    center, span = (low + high) / 2, (high - low).length

    for obj in scene.objects:
        obj.hide_render = obj.type == 'MESH'
    camera_data = bpy.data.cameras.new('README camera')
    camera = bpy.data.objects.new('README camera', camera_data)
    scene.collection.objects.link(camera)
    direction = Vector((.44, -.56, .31)).normalized()
    camera.location = center + direction * span * 3
    camera.rotation_euler = (-direction).to_track_quat('-Z', 'Y').to_euler()
    camera_data.type = 'ORTHO'
    camera_data.clip_start, camera_data.clip_end = span / 100, span * 20
    view = camera.rotation_euler.to_matrix().transposed()
    projected = [view @ (point - center) for point in points]
    width = max(point.x for point in projected) - min(point.x for point in projected)
    height = max(point.y for point in projected) - min(point.y for point in projected)
    camera_data.ortho_scale = max(width, height * 4 / 3) * 1.25
    scene.camera = camera
    bpy.ops.mesh.primitive_plane_add(size=span * 20, location=(center.x, center.y, low.z - span * .002))
    floor = bpy.context.object
    floor.name = 'README backdrop'

    def material(name, color, metallic=0):
        result = bpy.data.materials.new(name)
        result.use_nodes = True
        shader = result.node_tree.nodes.get('Principled BSDF')
        shader.inputs['Base Color'].default_value = (*color, 1)
        shader.inputs['Roughness'].default_value = .65
        shader.inputs['Metallic'].default_value = metallic
        return result

    steel = material('Neutral preview', (.16, .18, .21), .15)
    floor.data.materials.append(material('White backdrop', (.94, .95, .97)))
    for objects in poses.values():
        for obj in objects:
            obj.data.materials.clear()
            obj.data.materials.append(steel)
    for name, offset, strength, size in (
            ('Key', (2, -3, 4), 180, 2),
            ('Fill', (-3, -1, 2), 35, 3),
            ('Rim', (1, 3, 3), 90, 2)):
        light_data = bpy.data.lights.new(name, 'AREA')
        light = bpy.data.objects.new(name, light_data)
        scene.collection.objects.link(light)
        light.location = center + Vector(offset) * span
        light.rotation_euler = (center - light.location).to_track_quat('-Z', 'Y').to_euler()
        light_data.energy, light_data.shape, light_data.size = strength * span**2, 'DISK', size * span
    scene.world.use_nodes = True
    scene.world.node_tree.nodes.get('Background').inputs['Color'].default_value = (.9, .93, .97, 1)
    scene.world.node_tree.nodes.get('Background').inputs['Strength'].default_value = .8
    scene.render.engine = 'BLENDER_EEVEE'
    scene.render.resolution_x, scene.render.resolution_y = 1280, 960
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGB'
    scene.render.film_transparent = False
    scene.view_settings.view_transform = 'AgX'

    output.mkdir(parents=True, exist_ok=True)
    report = dict(inspection_sha256=hashlib.sha256(inspection.read_bytes()).hexdigest(),
                  expected_sha256=hashlib.sha256(expected_path.read_bytes()).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  renderer=bpy.app.version_string, shading='neutral-preview', width=1280, height=960, images={})
    for name, visuals in poses.items():
        for objects in poses.values():
            for obj in objects:
                obj.hide_render = True
        for obj in visuals:
            obj.hide_render = False
        scene.render.filepath = str(output / (prefix + '-' + name + '.png'))
        bpy.ops.render.render(write_still=True)
        image = Path(scene.render.filepath)
        if not image.is_file() or not image.stat().st_size:
            raise RuntimeError('Blender did not save ' + str(image))
        rendered = bpy.data.images.load(str(image), check_existing=False)
        pixels = list(rendered.pixels)
        minimum = min(min(pixels[channel::rendered.channels]) for channel in range(3))
        maximum = max(max(pixels[channel::rendered.channels]) for channel in range(3))
        if maximum - minimum < .1:
            raise RuntimeError('Blender produced an effectively blank image: ' + str(image))
        bpy.data.images.remove(rendered)
        report['images'][name] = dict(path=image.name, positions=expected[name]['positions'],
                                     sha256=hashlib.sha256(image.read_bytes()).hexdigest())
    (output / 'images.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS rendered checked zero/moved URDF geometry: ' + str(output))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inspection', type=Path, required=True)
    parser.add_argument('--expected', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prefix', default='rotary-slide')
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    render_images(args.inspection, args.expected, args.output, args.prefix)
