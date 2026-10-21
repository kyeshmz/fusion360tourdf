"""Pure current-pose generalized motion IR, without CAD edits."""

from dataclasses import dataclass
import math
import re

from ..utils import transforms as tf

__all__ = ['ExpansionError', 'SourceIdentity', 'Pose', 'Primitive', 'Frame',
           'MotionBinding', 'Expansion', 'expand_motion']


class ExpansionError(ValueError):
    """An input or native-coordinate mapping cannot be represented faithfully."""


@dataclass(frozen=True, eq=False)
class SourceIdentity:
    native: object
    context: object = None

    def __post_init__(self):
        for value in (self.native, self.context):
            if isinstance(value, (str, bytes, int, float, bool)):
                raise ExpansionError('SOURCE_IDENTITY: use resolved native/context objects, not tokens')
        if self.native is None:
            raise ExpansionError('SOURCE_IDENTITY: native object is required')

    def matches(self, native, context):
        return self.native is native and self.context is context


@dataclass(frozen=True)
class Pose:
    """Root or parent-relative rigid pose. Meters and a row-major 3x3 rotation."""

    xyz: tuple
    rotation: tuple


@dataclass(frozen=True)
class Primitive:
    name: str
    parent: str
    child: str
    type: str
    coordinate: str
    axis: tuple
    origin: Pose
    lower_limit: float | None
    upper_limit: float | None
    effort: float
    velocity: float


@dataclass(frozen=True)
class Frame:
    name: str
    kind: str
    pose: Pose
    source_path: str


@dataclass(frozen=True)
class MotionBinding:
    source: SourceIdentity
    motion_type: int
    primitive: str
    current_position_si: float
    unit: str
    coordinate_scale: float = 1.


@dataclass(frozen=True)
class Expansion:
    source: SourceIdentity
    primitives: tuple
    frames: tuple
    bindings: tuple
    parameterization: str
    blocked_native_motion_ids: tuple = ()

    def binding_for(self, native, context, motion_type):
        if not self.source.matches(native, context):
            raise ExpansionError('SOURCE_IDENTITY: native/context objects do not match')
        if motion_type in self.blocked_native_motion_ids:
            raise ExpansionError('BALL_NATIVE_COORDINATES_UNPROVEN: export-local angles are not native pitch/yaw/roll')
        for binding in self.bindings:
            if binding.motion_type == motion_type:
                return binding
        raise ExpansionError('MOTION_COORDINATE_UNAVAILABLE: %r' % (motion_type,))


# Coordinate, documented axis/value/limits fields, SI scale, JointMotionTypes ID.
_FIELDS = {
    'cylindrical': (
        ('rotate', 'rotationAxisVector', 'rotationValue', 'rotationLimits', 1., 3),
        ('slide', 'rotationAxisVector', 'slideValue', 'slideLimits', .01, 4)),
    'pin_slot': (
        ('slide', 'slideDirectionVector', 'slideValue', 'slideLimits', .01, 6),
        ('rotate', 'rotationAxisVector', 'rotationValue', 'rotationLimits', 1., 5)),
    'planar': (
        ('primary', 'primarySlideDirectionVector', 'primarySlideValue', 'primarySlideLimits', .01, 8),
        ('secondary', 'secondarySlideDirectionVector', 'secondarySlideValue', 'secondarySlideLimits', .01, 9),
        ('rotate', 'normalDirectionVector', 'rotationValue', 'rotationLimits', 1., 7)),
}
_RESERVED = ('fusion_helper_', 'rigid_attachment_')


def _number(value, field):
    try:
        if isinstance(value, bool):
            raise ValueError()
        value = float(value)
        if not math.isfinite(value):
            raise ValueError()
        return value
    except (ValueError, TypeError, OverflowError) as error:
        raise ExpansionError('INVALID_DATA: %s must be finite' % field) from error


def _get(owner, field):
    try:
        return getattr(owner, field)
    except Exception as error:
        raise ExpansionError('INVALID_DATA: unreadable %s' % field) from error


def _rotation(values, field):
    try:
        values = tuple(_number(v, field) for v in values)
        if len(values) != 9:
            raise ValueError()
        gram = tf.mat3_mul(tf.mat3_transpose(values), values)
        det = (values[0]*(values[4]*values[8]-values[5]*values[7])
               - values[1]*(values[3]*values[8]-values[5]*values[6])
               + values[2]*(values[3]*values[7]-values[4]*values[6]))
        if max(abs(a-b) for a, b in zip(gram, tf.IDENTITY_3X3)) > 1e-8 or abs(det-1) > 1e-8:
            raise ValueError()
        return values
    except (ValueError, TypeError) as error:
        raise ExpansionError('INVALID_ROTATION: %s must be a proper orthonormal 3x3' % field) from error


def _pose_cm(values, field):
    try:
        values = tuple(_number(v, field) for v in values)
        if len(values) != 16 or any(abs(a-b) > 1e-10 for a, b in zip(values[12:], (0, 0, 0, 1))):
            raise ValueError()
        return Pose(tuple(values[i]*.01 for i in (3, 7, 11)),
                    _rotation(tf.rotation3_from_array16(values), field))
    except (ValueError, TypeError) as error:
        raise ExpansionError('INVALID_POSE: %s must be a rigid row-major 4x4 in cm' % field) from error


def _relative(parent, child):
    rt = tf.mat3_transpose(parent.rotation)
    delta = tuple(c-p for c, p in zip(child.xyz, parent.xyz))
    return Pose(tuple(tf.rotate_vector_by_mat3(delta, rt)), tf.mat3_mul(rt, child.rotation))


def _axis(motion, field, vector_rotation):
    try:
        values = tuple(_number(v, field) for v in _get(motion, field).asArray())
        norm = math.hypot(*values)
        if len(values) != 3 or norm < 1e-12 or not math.isfinite(norm):
            raise ValueError()
        return tuple(tf.rotate_vector_by_mat3(tuple(v/norm for v in values), vector_rotation))
    except Exception as error:
        raise ExpansionError('INVALID_AXIS: %s must be a readable nonzero finite Vector3D' % field) from error


def _bounds(limits, field, scale, current):
    enabled = [_get(limits, f) for f in ('isMinimumValueEnabled', 'isMaximumValueEnabled')]
    if any(type(v) is not bool for v in enabled):
        raise ExpansionError('INVALID_DATA: %s enabled flags must be bool' % field)
    lo = (_number(_number(_get(limits, 'minimumValue'), field+'.minimumValue')*scale-current,
                  field+'.rebasedMinimum') if enabled[0] else None)
    hi = (_number(_number(_get(limits, 'maximumValue'), field+'.maximumValue')*scale-current,
                  field+'.rebasedMaximum') if enabled[1] else None)
    if lo is not None and hi is not None and lo > hi:
        raise ExpansionError('LIMIT_ORDER: %s lower exceeds upper' % field)
    return lo, hi


def _name(value, field):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', value):
        raise ExpansionError('INVALID_NAME: %s requires an allocated ASCII export alias' % field)
    if value.startswith(_RESERVED):
        raise ExpansionError('RESERVED_NAME: %s is in a generated namespace' % field)
    return value


def _positive(value, field):
    value = _number(value, field)
    if value <= 0:
        raise ExpansionError('INVALID_DATA: %s must be positive' % field)
    return value


def expand_motion(kind, motion, *, source, source_name, source_path,
                  parent, child, parent_pose_cm, child_pose_cm, joint_pose_cm,
                  vector_rotation=tf.IDENTITY_3X3, reverse=False, occupied_names=(),
                  effort=1., velocity=1., dof_settings=None):
    """Expand a macro edge at current-pose zero. See docs/generalized-joints.md.

    Read-only motion fields use SDK cm/radians. Poses and SDK-vector-to-root
    rotation must already be resolved in one actual assembly context by caller.
    The joint pose is the current translated pivot with a caller-chosen basis.
    Returned axes are in each child/joint frame. No endpoint gets duplicated.
    """
    if kind not in (*_FIELDS, 'ball'):
        raise ExpansionError('UNSUPPORTED_KIND: %s' % kind)
    if not isinstance(source, SourceIdentity):
        raise ExpansionError('SOURCE_IDENTITY: SourceIdentity is required')
    for field, value in (('source_name', source_name), ('parent', parent), ('child', child)):
        _name(value, field)
    if parent == child:
        raise ExpansionError('DISTINCT_ENDPOINTS: parent and child must differ')
    if not isinstance(source_path, str) or not source_path:
        raise ExpansionError('INVALID_DATA: source_path must be a nonempty diagnostic path')
    if type(reverse) is not bool:
        raise ExpansionError('INVALID_DATA: reverse must be bool')
    p = _pose_cm(parent_pose_cm, 'parent_pose_cm')
    c = _pose_cm(child_pose_cm, 'child_pose_cm')
    h = _pose_cm(joint_pose_cm, 'joint_pose_cm')
    vector_rotation = _rotation(vector_rotation, 'vector_rotation')
    effort, velocity = _positive(effort, 'effort'), _positive(velocity, 'velocity')
    factors = []
    if kind == 'ball':
        for field in ('pitchLimits', 'yawLimits', 'rollLimits'):
            if _bounds(_get(motion, field), field, 1., 0.) != (None, None):
                raise ExpansionError('BALL_NATIVE_BOUNDS_UNPROVEN: %s cannot map to export-local angles' % field)
        for index, coordinate in enumerate(('local_x', 'local_y', 'local_z')):
            axis = tuple(h.rotation[row*3+index] for row in range(3))
            factors.append((coordinate, axis, 'continuous', None, None, None, 0., 'rad'))
    else:
        for coordinate, axis_field, value_field, limit_field, scale, motion_id in _FIELDS[kind]:
            axis = _axis(motion, axis_field, vector_rotation)
            current = _number(_get(motion, value_field), value_field)*scale
            lo, hi = _bounds(_get(motion, limit_field), limit_field, scale, current)
            jtype = 'prismatic' if scale == .01 else 'continuous' if lo is hi is None else 'revolute'
            factors.append((coordinate, axis, jtype, lo, hi, motion_id, current, 'm' if scale == .01 else 'rad'))
        if kind != 'cylindrical':
            for i, first in enumerate(factors):
                for second in factors[i+1:]:
                    if abs(sum(a*b for a, b in zip(first[1], second[1]))) > 1e-8:
                        raise ExpansionError('AXES_NOT_ORTHOGONAL: %s and %s' % (first[0], second[0]))
    if reverse:
        factors.reverse()
        parent, child, p, c = child, parent, c, p
    joint_names = ['fusion_helper_%s__%s' % (source_name, f[0]) for f in factors]
    frame_names = ['fusion_helper_%s__frame_%d' % (source_name, i+1) for i in range(len(factors))]
    attachment_name = 'fusion_helper_%s__attachment' % source_name
    generated = joint_names + frame_names + [attachment_name]
    if len(set(generated)) != len(generated):
        raise ExpansionError('NAME_COLLISION: generated names are not distinct')
    collisions = set(generated).intersection(set(occupied_names) | {parent, child, source_name})
    if collisions:
        raise ExpansionError('NAME_COLLISION: ' + ', '.join(sorted(collisions)))
    settings = {} if dof_settings is None else dict(dof_settings)
    unknown = set(settings) - set(joint_names)
    if unknown:
        raise ExpansionError('DOF_SELECTOR: use exact generated DOF names: ' + ', '.join(sorted(unknown)))
    frames = tuple(Frame(name, 'frame', h, source_path) for name in frame_names)
    names, poses = [parent]+frame_names, [p]+[h]*len(frame_names)
    primitives, bindings = [], []
    for i, factor in enumerate(factors):
        coordinate, axis, jtype, lo, hi, motion_id, current, unit = factor
        overrides = dict(settings.get(joint_names[i], {}))
        if set(overrides) - {'effort', 'velocity'}:
            raise ExpansionError('DOF_SETTING: only effort and velocity can be selected per DOF')
        signed_axis = tuple((-1 if reverse else 1)*v for v in axis)
        local_axis = tuple(tf.rotate_vector_by_mat3(signed_axis, tf.mat3_transpose(poses[i+1].rotation)))
        primitives.append(Primitive(joint_names[i], names[i], names[i+1], jtype, coordinate,
                                    local_axis, _relative(poses[i], poses[i+1]), lo, hi,
                                    _positive(overrides.get('effort', effort), 'effort'),
                                    _positive(overrides.get('velocity', velocity), 'velocity')))
        if motion_id is not None:
            bindings.append(MotionBinding(source, motion_id, joint_names[i], current, unit))
    primitives.append(Primitive(attachment_name, frame_names[-1], child, 'fixed', 'attachment',
                                (0., 0., 0.), _relative(h, c), None, None, 0., 0.))
    return Expansion(source, tuple(primitives), frames, tuple(bindings),
                     'export_local_rx_ry_rz' if kind == 'ball' else 'native_scalar_deltas',
                     (0, 1, 2) if kind == 'ball' else ())
