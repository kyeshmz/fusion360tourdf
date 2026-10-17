"""Root-frame millimeter meshes and independently calculated meter-space oracles."""

import argparse
import copy
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


def generate_fixture(output, variant='articulated'):
    if variant not in VARIANTS:
        raise ValueError('Unknown fixture ' + variant)
    from URDF_Exporter.core.Write import write_ros2_package

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    robot_name = 'fixture_' + variant
    joints, inertials = model(variant == 'fixed')
    calls = []

    def write_meshes(paths):
        calls.append(set(paths))
        if set(paths) != set(inertials):
            raise AssertionError('Mesh callback link membership differs from the fixture')
        for link, path in paths.items():
            write_tetrahedron(path, MESH_BOUNDS_MM[link])

    package = write_ros2_package(
        output / (robot_name + '_description'), robot_name, joints, inertials,
        write_meshes=write_meshes,
        config={'collision_method': 'primitive_box' if variant == 'boxes' else 'visual'})
    if calls != [set(inertials)]:
        raise AssertionError('Mesh callback must be called exactly once')
    (output / (robot_name + '.expected.json')).write_text(
        json.dumps(oracle(variant == 'fixed'), indent=2) + '\n')
    return Path(package)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    for variant in VARIANTS:
        print(generate_fixture(args.output, variant))
