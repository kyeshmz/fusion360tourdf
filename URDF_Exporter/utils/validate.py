"""Fail-closed preflight validation and closed-loop detection (pure Python).

Ideas follow Adriaeik/fusion2URDF (topology warnings before mesh export,
loop-closing joints stashed in a sidecar) and ACDC4Robot (fail-closed
preflight with actionable errors), reduced to this repo's flat
joints_dict model: {joint_name: {parent, child, type, ...}}.
"""

import math
import re
from . import transforms as tf


def _inertia_error(tensor):
    scale = max(abs(value) for value in tensor)
    if scale == 0:
        return None
    xx, yy, zz, xy, yz, xz = [value / scale for value in tensor]
    matrix = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
    # Symmetric Jacobi rotations avoid determinant underflow and repeated-root cancellation.
    for _ in range(32):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(matrix[pair[0]][pair[1]]))
        off = matrix[p][q]
        if abs(off) <= math.ulp(1.0):
            break
        angle = 0.5 * math.atan2(2 * off, matrix[q][q] - matrix[p][p])
        c, s = math.cos(angle), math.sin(angle)
        a, b = matrix[p][p], matrix[q][q]
        matrix[p][p] = c*c*a - 2*s*c*off + s*s*b
        matrix[q][q] = s*s*a + 2*s*c*off + c*c*b
        matrix[p][q] = matrix[q][p] = 0.0
        for k in range(3):
            if k not in (p, q):
                a, b = matrix[k][p], matrix[k][q]
                matrix[k][p] = matrix[p][k] = c*a - s*b
                matrix[k][q] = matrix[q][k] = s*a + c*b
    low, middle, high = sorted(matrix[index][index] for index in range(3))
    tolerance = 64 * math.ulp(1.0)
    if low < -tolerance:
        return 'must be positive semidefinite (negative principal moment)'
    if high - middle - low > tolerance:
        return 'violates principal-moment triangle inequalities'
    return None


def detect_closing_joints(joints_dict):
    """Keep the first root-reachable parent for each child and retain loop data."""
    tree = {}
    closing = []
    warnings = []
    reached = {'base_link'}
    pending = list(joints_dict.items())
    while pending:
        remaining = []
        for name, joint in pending:
            parent, child = joint.get('parent'), joint.get('child')
            if not parent or not child:
                tree[name] = joint
            elif parent not in reached:
                remaining.append((name, joint))
            elif child in reached:
                closing.append(dict(joint, joint=name, source='auto_detected'))
                warnings.append('Joint %s (%s -> %s) closes a loop; retained only in robot_data.yaml.'
                                % (name, parent, child))
            else:
                tree[name] = joint
                reached.add(child)
        if len(remaining) == len(pending):
            reached.add(remaining[0][1]['parent'])
        pending = remaining
    return tree, closing, warnings


def validate_export(joints_dict, inertial_dict, config=None):
    """Return preflight errors and warnings. Single-link designs are valid."""
    errors = []
    warnings = []
    if not isinstance(joints_dict, dict) or not isinstance(inertial_dict, dict):
        return ['Joint and inertial data must be dictionaries.'], warnings
    if not inertial_dict:
        errors.append('No link components found. Add one component per link.')
        return errors, warnings
    if 'base_link' not in inertial_dict:
        errors.append("There is no base_link. Name the root component 'base_link' "
                       'or let the exporter synthesize it from design-root bodies.')

    def number(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    def vector(value, size):
        return isinstance(value, (list, tuple)) and len(value) == size and all(number(v) for v in value)

    for name, entry in inertial_dict.items():
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name):
            errors.append('Unsafe link name: %r.' % name)
        if not isinstance(entry, dict):
            errors.append('Link %s has unreadable inertial data.' % name)
            continue
        kind = entry.get('kind', 'physical')
        if kind == 'frame':
            physical_fields = {'mass', 'center_of_mass', 'inertia', 'color_rgba', 'bbox_center_m', 'bbox_size_m',
                               'visual', 'collision', 'mesh', 'mesh_physics'}
            if physical_fields.intersection(entry):
                errors.append('Frame link %s must not contain physical or geometry data.' % name)
            continue
        if kind != 'physical':
            errors.append('Link %s has unsupported kind %r.' % (name, kind))
        if not number(entry.get('mass')) or entry['mass'] <= 0:
            errors.append('Link %s needs finite positive mass.' % name)
        if not vector(entry.get('center_of_mass'), 3):
            errors.append('Link %s needs a finite center_of_mass vector.' % name)
        if not vector(entry.get('inertia'), 6) or any(v < 0 for v in entry['inertia'][:3]):
            errors.append('Link %s needs a finite inertia tensor with nonnegative diagonal.' % name)
        else:
            inertia_error = _inertia_error(entry['inertia'])
            if inertia_error:
                errors.append('Link %s inertia %s.' % (name, inertia_error))
        color = entry.get('color_rgba')
        if color is not None and (not vector(color, 4) or any(v < 0 or v > 1 for v in color)):
            errors.append('Link %s has invalid color_rgba.' % name)
        for key in ('bbox_center_m', 'bbox_size_m'):
            value = entry.get(key)
            if value is not None and (not vector(value, 3) or (key == 'bbox_size_m' and any(v <= 0 for v in value))):
                errors.append('Link %s has invalid %s.' % (name, key))
        if entry.get('mesh_physics'):
            warnings.append('Link %s uses explicit caller mesh physics, not native Fusion mass. '
                            'See mesh_physics provenance.' % name)
            if any(item['policy'] == 'caller-supplied-central-SI-physics' for item in entry['mesh_physics']):
                warnings.append('Link %s supplied properties deliberately override geometry-derived physics. '
                                'Surface thickness and density are not inferred.' % name)
            if (config or {}).get('collision_method') == 'primitive_box' and entry.get('bbox_size_m') is None:
                errors.append('Link %s has no positive-volume mesh bounds for primitive_box; use visual collision mode for surfaces.' % name)
    for joint_name, joint in joints_dict.items():
        if not isinstance(joint_name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', joint_name):
            errors.append('Unsafe joint name: %r.' % joint_name)
        if not isinstance(joint, dict):
            errors.append('Joint %s data must be a dictionary.' % joint_name)
            continue
        parent = joint.get('parent')
        child = joint.get('child')
        if not isinstance(parent, str) or not isinstance(child, str) or not parent or not child:
            errors.append('Joint %s is missing parent/child.' % joint_name)
            continue
        if child not in inertial_dict:
            errors.append('Joint %s child link %s has no inertial data. '
                          'Check Component1/Component2 order and visibility.'
                          % (joint_name, child))
        if parent not in inertial_dict:
            errors.append('Joint %s parent link %s has no inertial data.'
                          % (joint_name, parent))
        jtype = joint.get('type')
        if jtype not in ('fixed', 'revolute', 'continuous', 'prismatic'):
            errors.append('Joint %s has unsupported type %r.' % (joint_name, jtype))
        if not vector(joint.get('xyz'), 3):
            errors.append('Joint %s needs a finite root-frame xyz vector.' % joint_name)
        if not vector(joint.get('rpy', [0, 0, 0]), 3):
            errors.append('Joint %s needs a finite rpy vector.' % joint_name)
        if jtype in ('revolute', 'continuous', 'prismatic'):
            axis = joint.get('axis')
            if not vector(axis, 3) or not any(tf.normalize(axis)):
                errors.append('Joint %s (%s) has a zero axis or nonfinite axis vector.' % (joint_name, jtype))
            for key in ('effort', 'velocity'):
                value = joint.get(key, (config or {}).get(key, 100.0))
                if not number(value) or value <= 0:
                    errors.append('Joint %s needs finite positive %s.' % (joint_name, key))
        if jtype in ('revolute', 'prismatic'):
            lo, hi = joint.get('lower_limit'), joint.get('upper_limit')
            if number(lo) and number(hi):
                if hi < lo:
                    errors.append('Joint %s limits inverted: lower %s > upper %s.'
                                  % (joint_name, lo, hi))
            else:
                errors.append('Joint %s needs finite lower_limit and upper_limit.' % joint_name)
    reachable = {'base_link'}
    for _ in range(len(inertial_dict)):
        for joint in joints_dict.values():
            if (isinstance(joint, dict) and isinstance(joint.get('parent'), str)
                    and isinstance(joint.get('child'), str) and joint['parent'] in reachable):
                reachable.add(joint['child'])
    for link in inertial_dict:
        if link not in reachable:
            errors.append('Link %s is disconnected from base_link.' % link)
    return errors, warnings
