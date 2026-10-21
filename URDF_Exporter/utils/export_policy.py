"""Pure, opt-in export policies. No Fusion objects or serialization live here.

See docs/export-policies.md for the source schema and mandatory emitter contract.
Matrices are row-major active rotations. Inertia order is xx, yy, zz, xy, yz, xz.
"""

from copy import deepcopy
import math

from . import transforms as tf

__all__ = ['apply_export_policies', 'regularize_inertia', 'Y_UP_TO_Z_UP']

Y_UP_TO_Z_UP = (1., 0., 0., 0., 0., -1., 0., 1., 0.)
_IDENTITY = tf.IDENTITY_3X3
_ROUND_OFF = 64 * math.ulp(1.)
_PRIMITIVES = ('revolute', 'continuous', 'prismatic')


def _number(value, name):
    try:
        valid = (isinstance(value, (int, float)) and not isinstance(value, bool)
                 and math.isfinite(value))
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(name + ' must be a finite number')
    return value


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(name + ' must have %d components' % size)
    for item in value:
        _number(item, name)
    return list(value)


def _rotation(value, name):
    value = _vector(value, 9, name)
    gram = tf.mat3_mul(tf.mat3_transpose(value), value)
    determinant = (value[0]*(value[4]*value[8] - value[5]*value[7])
                   - value[1]*(value[3]*value[8] - value[5]*value[6])
                   + value[2]*(value[3]*value[7] - value[4]*value[6]))
    if (max(abs(a-b) for a, b in zip(gram, _IDENTITY)) > 1e-12
            or abs(determinant - 1) > 1e-12):
        raise ValueError(name + ' must be a proper orthonormal rotation')
    return value


def _add(a, b):
    return [x+y for x, y in zip(a, b)]


def _sub(a, b):
    return [x-y for x, y in zip(a, b)]


def _scale(a, factor):
    return [x*factor for x in a]


def _dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def _cross(a, b):
    return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]


def _unit(a):
    length = math.hypot(*a)
    if not math.isfinite(length) or length == 0:
        raise ValueError('joint axis must have finite nonzero length')
    return [x/length for x in a]


def _tensor(six):
    xx, yy, zz, xy, yz, xz = six
    return (xx, xy, xz, xy, yy, yz, xz, yz, zz)


def _six(matrix):
    return [matrix[i] for i in (0, 4, 8, 1, 5, 2)]


def _rotate_tensor(six, rotation):
    return _six(tf.mat3_mul(tf.mat3_mul(rotation, _tensor(six)),
                           tf.mat3_transpose(rotation)))


def _principal(six):
    scale = max(abs(v) for v in six)
    if scale == 0:
        return [0., 0., 0.], list(_IDENTITY)
    flat = _tensor([v/scale for v in six])
    matrix = [list(flat[i:i+3]) for i in (0, 3, 6)]
    vectors = [[float(i == j) for j in range(3)] for i in range(3)]
    for _ in range(64):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(matrix[pair[0]][pair[1]]))
        off = matrix[p][q]
        if abs(off) <= math.ulp(1.):
            break
        angle = .5 * math.atan2(2*off, matrix[q][q] - matrix[p][p])
        c, s = math.cos(angle), math.sin(angle)
        a, b = matrix[p][p], matrix[q][q]
        matrix[p][p] = c*c*a - 2*s*c*off + s*s*b
        matrix[q][q] = s*s*a + 2*s*c*off + c*c*b
        matrix[p][q] = matrix[q][p] = 0.
        for k in range(3):
            if k not in (p, q):
                a, b = matrix[k][p], matrix[k][q]
                matrix[k][p] = matrix[p][k] = c*a - s*b
                matrix[k][q] = matrix[q][k] = s*a + c*b
            a, b = vectors[k][p], vectors[k][q]
            vectors[k][p], vectors[k][q] = c*a - s*b, s*a + c*b
    values = [matrix[i][i] for i in range(3)]
    low, middle, high = sorted(values)
    if low < -_ROUND_OFF:
        raise ValueError('inertia must be positive semidefinite')
    if high - middle - low > _ROUND_OFF:
        raise ValueError('inertia violates principal-moment triangle inequalities')
    moments = _vector([v*scale for v in values], 3, 'principal moments')
    return moments, [v for row in vectors for v in row]


def _threshold(value):
    if value is not None:
        _number(value, 'minimum_principal_inertia')
        if value < 0:
            raise ValueError('minimum_principal_inertia must be nonnegative')


def regularize_inertia(inertia, minimum_principal_inertia=None):
    """Validate then optionally floor principal moments, in kg*m^2.

    None disables the policy, including reconstruction of the original tensor.
    The returned original/final tensors and receipt own their storage.
    """
    six = _vector(inertia, 6, 'inertia')
    if any(v < 0 for v in six[:3]):
        raise ValueError('inertia diagonal must be nonnegative')
    _threshold(minimum_principal_inertia)
    values, vectors = _principal(six)
    final = deepcopy(inertia)
    changed = False
    if minimum_principal_inertia is not None:
        changes = [max(v, minimum_principal_inertia) - v for v in values]
        changed = any(v != 0 for v in changes)
        if changed:
            matrix = _tensor(six)
            final = _six([matrix[3*i+j] + sum(vectors[3*i+k]*delta*vectors[3*j+k]
                           for k, delta in enumerate(changes))
                          for i in range(3) for j in range(3)])
            _vector(final, 6, 'regularized inertia')
            _principal(final)
    return {'original': deepcopy(inertia), 'final': final,
            'threshold_kg_m2': minimum_principal_inertia,
            'enabled': minimum_principal_inertia is not None, 'changed': changed,
            'principal_moments_before': values,
            'principal_moments_after': [max(v, minimum_principal_inertia)
                                      if minimum_principal_inertia is not None else v for v in values],
            'warning': ('Physical inertia properties changed by an explicit spectral floor. '
                        'Mass is unchanged; controller stability is not established.') if changed else None}


def _source(joints, inertial, root_frame, minimum_principal_inertia):
    if not isinstance(joints, dict) or not isinstance(inertial, dict) or 'base_link' not in inertial:
        raise ValueError('joints and inertial must be dictionaries containing base_link')
    root = {'t': [0., 0., 0.], 'R': list(_IDENTITY)}
    if root_frame is not None:
        if not isinstance(root_frame, dict) or set(root_frame) != {'t', 'R'}:
            raise ValueError('source_root_frame must contain exactly t and R')
        root = {'t': _vector(root_frame['t'], 3, 'root t'),
                'R': _rotation(root_frame['R'], 'root R')}
    inertia_receipts = {}
    for name, entry in inertial.items():
        if not isinstance(name, str) or not name or not isinstance(entry, dict):
            raise ValueError('links must be named dictionaries')
        if entry.get('kind') == 'frame':
            if any(key in entry for key in ('mass', 'center_of_mass', 'inertia', 'bbox_center_m', 'bbox_size_m')):
                raise ValueError('frame-only link must not carry physical properties or geometry')
            continue
        if entry.get('kind', 'physical') != 'physical':
            raise ValueError('unsupported link kind')
        if _number(entry.get('mass'), name + ' mass') <= 0:
            raise ValueError('physical mass must be positive')
        _vector(entry.get('center_of_mass'), 3, name + ' center_of_mass')
        inertia_receipts[name] = regularize_inertia(entry.get('inertia'), minimum_principal_inertia)
        if ('bbox_center_m' in entry) != ('bbox_size_m' in entry):
            raise ValueError('box center and size must be supplied together')
        if (entry.get('bbox_center_m') is None) != (entry.get('bbox_size_m') is None):
            raise ValueError('box center and size must both be available or both be None')
        if entry.get('bbox_center_m') is not None:
            _vector(entry['bbox_center_m'], 3, name + ' bbox_center_m')
            if any(v <= 0 for v in _vector(entry['bbox_size_m'], 3, name + ' bbox_size_m')):
                raise ValueError('box size must be positive')
    incoming = set()
    for name, joint in joints.items():
        if not isinstance(name, str) or not name or not isinstance(joint, dict):
            raise ValueError('joints must be named dictionaries')
        parent, child = joint.get('parent'), joint.get('child')
        if not isinstance(parent, str) or not isinstance(child, str) or parent not in inertial or child not in inertial:
            raise ValueError('joint parent and child must name links')
        if child == 'base_link' or child == parent or child in incoming:
            raise ValueError('topology must be a rooted tree with one parent per non-root link')
        incoming.add(child)
        if joint.get('type') not in (*_PRIMITIVES, 'fixed'):
            raise ValueError('unsupported joint type')
        _vector(joint.get('xyz'), 3, name + ' xyz')
        _vector(joint.get('rpy', [0., 0., 0.]), 3, name + ' rpy')
        if joint['type'] in _PRIMITIVES:
            _unit(_vector(joint.get('axis'), 3, name + ' axis'))
        for key in ('lower_limit', 'upper_limit', 'effort', 'velocity'):
            if key in joint:
                if key in ('lower_limit', 'upper_limit') and joint[key] is None and joint['type'] in ('fixed', 'continuous'):
                    continue
                _number(joint[key], name + ' ' + key)
        if ('lower_limit' in joint and 'upper_limit' in joint
                and joint['lower_limit'] is not None and joint['upper_limit'] is not None
                and joint['lower_limit'] > joint['upper_limit']):
            raise ValueError('joint limits are inverted')
    frames = {'base_link': root}
    order = []
    pending = list(joints)
    while pending:
        remaining = []
        for name in pending:
            joint = joints[name]
            if joint['parent'] not in frames:
                remaining.append(name)
                continue
            frames[joint['child']] = {'t': list(joint['xyz']),
                'R': list(tf.mat3_mul(frames[joint['parent']]['R'],
                                     tf.mat3_from_rpy(joint.get('rpy', [0., 0., 0.]))))}
            order.append(name)
        if len(remaining) == len(pending):
            raise ValueError('topology is cyclic or disconnected from base_link')
        pending = remaining
    if set(frames) != set(inertial):
        raise ValueError('every link must be connected to base_link')
    return frames, order, inertia_receipts


def _reference_x(z, root_rotation):
    for column in (0, 1, 2):
        reference = [root_rotation[column+3*i] for i in range(3)]
        projected = _sub(reference, _scale(z, _dot(reference, z)))
        if math.hypot(*projected) > 1e-8:
            return _unit(projected)
    raise ValueError('could not choose a transverse reference')


def _dh_frames(joints, frames, order):
    children = {}
    for name in order:
        joint = joints[name]
        if joint['type'] not in _PRIMITIVES:
            raise ValueError('dh_common_normal requires only serial revolute/continuous/prismatic joints')
        if joint['parent'] in children:
            raise ValueError('dh_common_normal does not support branching')
        children[joint['parent']] = name
    axes = {name: _unit(tf.rotate_vector_by_mat3(joints[name]['axis'], frames[joints[name]['child']]['R']))
            for name in order}
    root = frames['base_link']
    result = {'base_link': deepcopy(root)}
    receipt = {}
    for name in order:
        joint = joints[name]
        p, z = frames[joint['child']]['t'], axes[name]
        next_name = children.get(joint['child'])
        other_foot = None
        if next_name is None:
            origin, x, case = list(p), _reference_x(z, root['R']), 'terminal_source_pivot'
        else:
            q, w = frames[joints[next_name]['child']]['t'], axes[next_name]
            cross = _cross(z, w)
            sine = math.hypot(*cross)
            if _ROUND_OFF < sine < 1e-8:
                raise ValueError('near-parallel axes are ill-conditioned for dh_common_normal')
            if sine <= _ROUND_OFF:
                origin = _add(p, _scale(z, _dot(_sub(root['t'], p), z)))
                other_foot = _add(q, _scale(w, _dot(_sub(origin, q), w)))
                normal = _sub(other_foot, origin)
                if math.hypot(*normal) <= 1e-12 * max(1., math.hypot(*p), math.hypot(*q)):
                    x, case = _reference_x(z, root['R']), 'coincident'
                else:
                    x, case = _unit(normal), 'parallel'
            else:
                difference = _sub(q, p)
                # Cross-product formulas avoid cancellation in 1 - dot(z,w)^2.
                along = _dot(_cross(difference, w), cross) / (sine*sine)
                other_along = _dot(_cross(difference, z), cross) / (sine*sine)
                origin = _add(p, _scale(z, along))
                other_foot = _add(q, _scale(w, other_along))
                normal = _sub(other_foot, origin)
                if math.hypot(*normal) <= 1e-12 * max(1., math.hypot(*p), math.hypot(*q)):
                    x, case = _unit(cross), 'intersecting'
                else:
                    x, case = _unit(normal), 'skew'
        x = _unit(_sub(x, _scale(z, _dot(x, z))))
        y = _cross(z, x)
        rotation = [axis[i] for i in range(3) for axis in (x, y, z)]
        result[joint['child']] = {'t': origin, 'R': rotation}
        receipt[name] = {'rule': case, 'next_joint': next_name,
                         'origin_on_current_axis': list(origin),
                         'foot_on_next_axis': other_foot}
    return result, receipt


def _relative(parent, child):
    inverse = tf.mat3_transpose(parent['R'])
    return {'t': tf.rotate_vector_by_mat3(_sub(child['t'], parent['t']), inverse),
            'R': list(tf.mat3_mul(inverse, child['R']))}


def apply_export_policies(joints, inertial, *, output_rotation=None,
                          frame_convention=None, minimum_principal_inertia=None,
                          source_root_frame=None):
    """Return owned dictionaries, receipts, and local emitter poses.

    All three policy defaults are disabled. Input xyz/COM/tensors are in source
    design-frame SI, rpy is parent-relative, axes are joint-local. The source
    root frame defaults to identity. No mesh bytes or source objects are edited.
    """
    if frame_convention not in (None, 'dh_common_normal'):
        raise ValueError('frame_convention must be None or dh_common_normal')
    _threshold(minimum_principal_inertia)
    rotation = list(_IDENTITY) if output_rotation is None else _rotation(output_rotation, 'output_rotation')
    source_frames, order, inertia_receipts = _source(joints, inertial, source_root_frame,
                                                    minimum_principal_inertia)
    result_joints, result_inertial = deepcopy(joints), deepcopy(inertial)
    frames = {name: {'t': tf.rotate_vector_by_mat3(frame['t'], rotation),
                     'R': list(tf.mat3_mul(rotation, frame['R']))}
              for name, frame in source_frames.items()}
    warnings = []
    for name, entry in result_inertial.items():
        if entry.get('kind') == 'frame':
            continue
        receipt = inertia_receipts[name]
        if receipt['warning']:
            warnings.append(name + ': ' + receipt['warning'])
        if receipt['changed']:
            entry['inertia'] = deepcopy(receipt['final'])
        if output_rotation is not None:
            entry['center_of_mass'] = tf.rotate_vector_by_mat3(entry['center_of_mass'], rotation)
            entry['inertia'] = _rotate_tensor(entry['inertia'], rotation)
            if entry.get('bbox_center_m') is not None:
                entry['bbox_center_m'] = tf.rotate_vector_by_mat3(entry['bbox_center_m'], rotation)
    if output_rotation is not None:
        for joint in result_joints.values():
            joint['xyz'] = tf.rotate_vector_by_mat3(joint['xyz'], rotation)
    dh_receipt = {}
    if frame_convention is not None:
        old_frames = frames
        frames, dh_receipt = _dh_frames(result_joints, frames, order)
        for name in order:
            joint = result_joints[name]
            frame = frames[joint['child']]
            joint['xyz'] = list(frame['t'])
            joint['rpy'] = list(tf.rpy_from_mat3(_relative(frames[joint['parent']], frame)['R']))
            world_axis = tf.rotate_vector_by_mat3(_unit(joint['axis']), old_frames[joint['child']]['R'])
            joint['axis'] = tf.rotate_vector_by_mat3(world_axis, tf.mat3_transpose(frame['R']))
    source_to_output = {'t': [0., 0., 0.], 'R': rotation}
    emission = {'root_pose': deepcopy(frames['base_link']), 'joint_origins': {},
                'mesh_origins': {}, 'mesh_scale': [.001, .001, .001],
                'boxes': {}, 'inertials': {}}
    for name, joint in result_joints.items():
        emission['joint_origins'][name] = _relative(frames[joint['parent']], frames[joint['child']])
    for name, entry in result_inertial.items():
        if entry.get('kind') == 'frame':
            continue
        frame = frames[name]
        inverse = tf.mat3_transpose(frame['R'])
        emission['mesh_origins'][name] = _relative(frame, source_to_output)
        emission['inertials'][name] = {
            'mass': entry['mass'],
            'center_of_mass': tf.rotate_vector_by_mat3(_sub(entry['center_of_mass'], frame['t']), inverse),
            'inertia': _rotate_tensor(entry['inertia'], inverse)}
        if entry.get('bbox_center_m') is not None:
            emission['boxes'][name] = dict(_relative(frame, {'t': entry['bbox_center_m'], 'R': rotation}),
                                            size=list(entry['bbox_size_m']))
    result = {'joints': result_joints, 'inertial': result_inertial,
              'link_frames': frames, 'mesh_source_to_output': source_to_output,
              'emission': emission, 'warnings': warnings,
              'receipts': {'coordinate_rotation': {'enabled': output_rotation is not None,
                                                   'source_to_output_rotation': list(rotation)},
                           'frame_convention': {'selected': frame_convention, 'joints': dh_receipt},
                           'inertia': inertia_receipts}}
    def finite(value):
        if isinstance(value, dict):
            for item in value.values():
                finite(item)
        elif isinstance(value, (list, tuple)):
            for item in value:
                finite(item)
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            _number(value, 'derived result')
    finite({'frames': frames, 'emission': emission, 'receipts': result['receipts']})
    return result
