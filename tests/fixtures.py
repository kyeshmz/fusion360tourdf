"""Root-frame millimeter meshes and independently calculated meter-space oracles."""

import argparse
import copy
import itertools
import json
import math
from pathlib import Path
import struct


MESH_BOUNDS_MM = {
    'base_link': ((10, -40, 20), (90, 10, 50)),
    'mount': ((180, -120, 300), (220, -70, 360)),
    'arm': ((190, 110, 310), (240, 140, 390)),
    'carriage': ((310, 120, 320), (370, 160, 350)),
    'wheel': ((400, 120, 330), (430, 190, 370)),
}

ZERO_POSES = {
    'base_link': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    'mount': [[0, -1, 0, .2], [1, 0, 0, -.1], [0, 0, 1, .3], [0, 0, 0, 1]],
    'arm': [[0, 0, 1, .2], [1, 0, 0, .1], [0, 1, 0, .3], [0, 0, 0, 1]],
    'carriage': [[0, 0, 1, .3], [1, 0, 0, .1], [0, 1, 0, .3], [0, 0, 0, 1]],
    'wheel': [[0, 0, 1, .4], [1, 0, 0, .1], [0, 1, 0, .3], [0, 0, 0, 1]],
}

MOVED_POSES = {
    'base_link': ZERO_POSES['base_link'],
    'mount': ZERO_POSES['mount'],
    'arm': [[0, 0, 1, .2], [0, -1, 0, .1], [1, 0, 0, .3], [0, 0, 0, 1]],
    'carriage': [[0, 0, 1, .3], [0, -1, 0, .06], [1, 0, 0, .3], [0, 0, 0, 1]],
    'wheel': [[0, -1, 0, .4], [0, 0, -1, .06], [1, 0, 0, .3], [0, 0, 0, 1]],
}

ZERO_BOUNDS = {
    'base_link': [[.01, -.04, .02], [.09, .01, .05]],
    'mount': [[.18, -.12, .30], [.22, -.07, .36]],
    'arm': [[.19, .11, .31], [.24, .14, .39]],
    'carriage': [[.31, .12, .32], [.37, .16, .35]],
    'wheel': [[.40, .12, .33], [.43, .19, .37]],
}

MOVED_BOUNDS = {
    'base_link': ZERO_BOUNDS['base_link'],
    'mount': ZERO_BOUNDS['mount'],
    'arm': [[.19, .01, .31], [.24, .09, .34]],
    'carriage': [[.31, .01, .32], [.37, .04, .36]],
    'wheel': [[.33, .03, .32], [.37, .06, .39]],
}

MOVED_VISUAL_POSES = {
    'base_link': ZERO_POSES['base_link'],
    'mount': ZERO_POSES['base_link'],
    'arm': [[1, 0, 0, 0], [0, 0, -1, .4], [0, 1, 0, .2], [0, 0, 0, 1]],
    'carriage': [[1, 0, 0, 0], [0, 0, -1, .36], [0, 1, 0, .2], [0, 0, 0, 1]],
    'wheel': [[0, 0, -1, .7], [-1, 0, 0, .46], [0, 1, 0, .2], [0, 0, 0, 1]],
}

TARGET = {'shoulder': math.pi / 2, 'slide': .04, 'spin': -math.pi / 2}
VARIANTS = ('articulated', 'fixed', 'boxes')
OPTIONAL_VARIANTS = ('frames', 'generalized', 'mimic', 'materials', 'policy_rotation', 'policy_boxes_dh',
                     'policy_floor', 'mesh', 'surface', 'hull', 'materials_hull', 'materials_cells',
                     'per_body', 'per_body_hull')


def fixture_collision_method(variant):
    if variant in ('boxes', 'policy_boxes_dh'):
        return 'primitive_box'
    if variant == 'materials_cells':
        return 'convex_decomposition'
    if variant in ('hull', 'materials_hull', 'per_body_hull'):
        return 'convex_hull'
    return 'visual'


def fixture_variants(selected=None):
    if selected is None:
        return VARIANTS
    if selected not in OPTIONAL_VARIANTS:
        raise ValueError('Unknown optional fixture ' + selected)
    return (selected,)


def model(fixed_only=False):
    joints = {
        'mount_fixed': dict(parent='base_link', child='mount', type='fixed',
                            xyz=[.2, -.1, .3], rpy=[0, 0, math.pi / 2], axis=[0, 0, 1]),
        'shoulder': dict(parent='mount', child='arm', type='revolute',
                         xyz=[.2, .1, .3], rpy=[math.pi / 2, 0, 0], axis=[0, 0, 1]),
        'slide': dict(parent='arm', child='carriage', type='prismatic',
                      xyz=[.3, .1, .3], rpy=[0, 0, 0], axis=[0, 1, 0]),
        'spin': dict(parent='carriage', child='wheel', type='continuous',
                     xyz=[.4, .1, .3], rpy=[0, 0, 0], axis=[1, 0, 0]),
    }
    if fixed_only:
        joints = {'mount_fixed': joints['mount_fixed']}
    for joint in joints.values():
        joint.update(lower_limit=-2.0, upper_limit=2.0, effort=20.0, velocity=3.0)
    links = ['base_link'] + [joint['child'] for joint in joints.values()]
    inertials = {}
    for link in links:
        lo, hi = MESH_BOUNDS_MM[link]
        inertials[link] = {
            'mass': 1.0,
            'center_of_mass': [(3 * a + b) / 4000 for a, b in zip(lo, hi)],
            'inertia': [.02, .03, .04, .001, .002, .003],
            'bbox_center_m': [(a + b) / 2000 for a, b in zip(lo, hi)],
            'bbox_size_m': [(b - a) / 1000 for a, b in zip(lo, hi)],
            'color_rgba': [.2, .5, .8, 1.0],
        }
    return joints, inertials


def write_tetrahedron(path, bounds):
    lo, hi = bounds
    vertices = (lo, (hi[0], lo[1], lo[2]), (lo[0], hi[1], lo[2]),
                (lo[0], lo[1], hi[2]))
    triangles = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    data = bytearray(b'fusion360tourdf asymmetric fixture'.ljust(80, b'\0'))
    data.extend(struct.pack('<I', len(triangles)))
    for indices in triangles:
        a, b, c = [vertices[i] for i in indices]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        normal = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                  u[0] * v[1] - u[1] * v[0]]
        length = math.sqrt(sum(n * n for n in normal))
        data.extend(struct.pack('<12fH', *[n / length for n in normal], *a, *b, *c, 0))
    Path(path).write_bytes(data)


def oracle(fixed_only=False):
    links = ('base_link', 'mount') if fixed_only else tuple(ZERO_POSES)
    cases = {
        'zero': {
            'positions': {} if fixed_only else dict.fromkeys(TARGET, 0.0),
            'link_poses': {link: ZERO_POSES[link] for link in links},
            'visual_poses': {link: ZERO_POSES['base_link'] for link in links},
            'bounds': {link: ZERO_BOUNDS[link] for link in links},
        },
    }
    if not fixed_only:
        cases['moved'] = {'positions': TARGET, 'link_poses': MOVED_POSES,
                          'visual_poses': MOVED_VISUAL_POSES, 'bounds': MOVED_BOUNDS}
    return copy.deepcopy(cases)


def fixture_oracle(variant):
    if variant == 'per_body':
        from tests.test_per_body_visuals import oracle as per_body_oracle
        return per_body_oracle()
    if variant == 'per_body_hull':
        from tests.test_per_body_hull import oracle as per_body_hull_oracle
        return per_body_hull_oracle()
    if variant in OPTIONAL_VARIANTS[4:]:
        from tests.test_integrated_policies import integrated_oracle
        return integrated_oracle(variant)
    if variant in ('generalized', 'mimic'):
        return motion_oracle(variant)
    cases = oracle(variant == 'fixed')
    if variant == 'materials':
        cases = oracle(True)
        for values in cases.values():
            values['materials'] = material_oracle()
    if variant == 'frames':
        kinds = {name: 'frame' if name in ('base_link', 'arm') else 'physical' for name in ZERO_POSES}
        for values in cases.values():
            values['link_kinds'] = kinds
            for key in ('bounds', 'visual_poses'):
                values[key] = {name: value for name, value in values[key].items() if kinds[name] == 'physical'}
    return cases


def generate_fixture(output, variant='articulated'):
    if variant not in VARIANTS + OPTIONAL_VARIANTS:
        raise ValueError('Unknown fixture ' + variant)
    from URDF_Exporter.core.Write import write_ros2_package

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if variant in ('per_body', 'per_body_hull'):
        if variant == 'per_body':
            from tests.test_per_body_visuals import export_fixture
        else:
            from tests.test_per_body_hull import export_fixture
        robot_name = 'fixture_' + variant
        package = export_fixture(output, robot_name)
        (output / (robot_name + '.expected.json')).write_text(json.dumps(fixture_oracle(variant), indent=2) + '\n')
        return package
    if variant in OPTIONAL_VARIANTS[4:]:
        from tests.test_integrated_policies import generate_integrated
        return generate_integrated(output, variant)
    robot_name = 'fixture_' + variant
    if variant in ('generalized', 'mimic'):
        from tests.test_public_motion import assembly, native_link, ENTRY
        from tests.test_fusion_inputs import design
        component, joint, _ = assembly('cylindrical', reverse=True)
        if variant == 'mimic':
            component.motionLinks = [native_link(joint, reversed=True)]
        fixture, _ = design(component)
        package = ENTRY.export_design(fixture, output / (robot_name + '_description'),
                                     config={'generalized_joints': 'decompose',
                                             'motion_links': 'mimic' if variant == 'mimic' else 'reject'})
        (output / (robot_name + '.expected.json')).write_text(json.dumps(fixture_oracle(variant), indent=2) + '\n')
        return package
    joints, inertials = model(variant in ('fixed', 'materials'))
    if variant == 'frames':
        for name in ('base_link', 'arm'):
            inertials[name] = {'kind': 'frame'}
    physical = {name for name, entry in inertials.items() if entry.get('kind', 'physical') == 'physical'}
    calls = []

    def write_meshes(paths):
        calls.append(set(paths))
        if set(paths) != physical:
            raise AssertionError('Mesh callback link membership differs from the fixture')
        for link, path in paths.items():
            write_tetrahedron(path, MESH_BOUNDS_MM[link])

    package = write_ros2_package(
        output / (robot_name + '_description'), robot_name, joints, inertials,
        write_meshes=write_meshes,
        config={'collision_method': 'primitive_box' if variant == 'boxes' else 'visual',
                **({'visual_materials': 'diffuse-subset'} if variant == 'materials' else {})},
        visual_assets=material_assets() if variant == 'materials' else None)
    if calls != [physical]:
        raise AssertionError('Mesh callback must be called exactly once')
    (output / (robot_name + '.expected.json')).write_text(
        json.dumps(fixture_oracle(variant), indent=2) + '\n')
    return Path(package)


def motion_oracle(variant):
    from tests.test_generalized_motion import I, inverse, mm, shift, turn
    driver, follower = 'fusion_helper_compound__rotate', 'fusion_helper_compound__slide'
    pose = mm(shift((.37, -.12, .41)), turn((1, 2, 3), .61))
    pivot = shift((.2, -.1, .3))
    kinds = {'base_link': 'physical', 'body_1': 'physical',
             'fusion_helper_compound__frame_1': 'frame', 'fusion_helper_compound__frame_2': 'frame'}
    cases = {}
    for case, r, s in (('zero', 0., 0.), ('moved', .8, -.02 if variant == 'mimic' else .04)):
        first = mm(pivot, shift((0, 0, -s)))
        second = mm(first, turn((0, 0, 1), -r))
        action = mm(second, inverse(pivot))
        states = {driver: r, follower: s}
        poses = {'base_link': I, 'body_1': mm(action, pose),
                 'fusion_helper_compound__frame_1': first, 'fusion_helper_compound__frame_2': second}
        bounds = {}
        for name, lo, hi, transform in (('base_link', (-.01,)*3, (.01,)*3, I),
                                         ('body_1', (.09, -.01, -.01), (.11, .01, .01), action)):
            points = [[sum(transform[i][k]*v[k] for k in range(3))+transform[i][3] for i in range(3)]
                      for v in itertools.product(*zip(lo, hi))]
            bounds[name] = [[min(v[i] for v in points) for i in range(3)],
                            [max(v[i] for v in points) for i in range(3)]]
        cases[case] = dict(positions={driver: r} if variant == 'mimic' else states,
                           states=states, link_poses=poses, link_kinds=kinds, bounds=bounds,
                           visual_poses={'base_link': I, 'body_1': action},
                           masses={'base_link': 1., 'body_1': 3.},
                           source_current_positions_si={'rotate': .43, 'slide': .025},
                           source_endpoint_reversed=True)
    return cases


def material_oracle():
    return {name: {'triangles': 4, 'materials': 4, 'constant_rgba': [1., 0., 0., 128/255*.5],
                   'png_rgba8': [[255, 0, 0, 255], [0, 255, 0, 64]],
                   'jpeg_rgb8': [[245, 2, 4], [3, 255, 2], [2, 0, 247], [255, 249, 250]],
                   'progressive_gray8': [128], 'loss_count': 4}
            for name in ('base_link', 'mount')}


def material_assets():
    import hashlib
    from tests.test_visual_assets import png_rgba, KNOWN_JPEG, progressive_gray_jpeg
    from URDF_Exporter.utils.visual_assets import build_obj_assets
    images = {hashlib.sha256(data).hexdigest(): {'format': fmt, 'data': data}
              for data, fmt in ((png_rgba(), 'png'), (KNOWN_JPEG, 'jpeg'), (progressive_gray_jpeg(), 'jpeg'))}
    results = {}
    for name in ('base_link', 'mount'):
        lo, hi = MESH_BOUNDS_MM[name]
        vertices = [lo, (hi[0], lo[1], lo[2]), (lo[0], hi[1], lo[2]), (lo[0], lo[1], hi[2])]
        surfaces = []
        for index, indices in enumerate(((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))):
            points = [vertices[i] for i in indices]
            a, b, c = points
            u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
            normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
            image = list(images)[index-1] if index else None
            surfaces.append(dict(context_id=name+'_body_0', face_index=index, vertices_mm=points,
                normals=[normal]*3, uvs=[[0, 0], [1, 0], [0, 1]] if image else [], triangles=[[0, 1, 2]],
                material={'rgba': [1., 1., 1., .5] if image else [1., 0., 0., 128/255*.5], 'image': image},
                appearance={'shader_policy': 'diffuse-subset', 'source': 'face',
                    'selected_property': {'id': 'generic_diffuse', 'type': 'adsk::core::ColorProperty'},
                    'losses': [{'id': 'roughness', 'type': 'adsk::core::FloatProperty', 'reason': 'omitted-shader-property'}],
                    'warnings': ['lossy-diffuse-subset-not-shader-equivalent'] +
                                (['provided-uvs-not-native-render-proof'] if image else [])}))
        results[name] = build_obj_assets({'descriptor': {'schema': 'fusion-visual-snapshot/1', 'frame': 'root',
                                                        'units': 'mm', 'surfaces': surfaces}, 'images': images})
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--fixture', choices=OPTIONAL_VARIANTS)
    args = parser.parse_args()
    for variant in fixture_variants(args.fixture):
        print(generate_fixture(args.output, variant))
