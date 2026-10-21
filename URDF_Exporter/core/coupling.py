"""Read-only MotionLink adapter and pure affine planning. No writer or CAD edits."""

from fractions import Fraction
import math
import re

from .generalized import MotionBinding, SourceIdentity

__all__ = ['CouplingError', 'ExcludedMotion', 'read_motion_link', 'affine_relation', 'apply_couplings',
           'plain_data', 'evaluate_affine']

_LINEAR = frozenset((4, 6, 8, 9, 11))
_MOVING = frozenset(('continuous', 'revolute', 'prismatic'))


class CouplingError(ValueError):
    """A native read, affine dependency, or representation cannot be proved safe."""


class ExcludedMotion(CouplingError):
    """The identity inventory proves this source is entirely outside selection."""


def _get(owner, field, path=None):
    try:
        return getattr(owner, field)
    except Exception as error:
        raise CouplingError('NATIVE_READ: unreadable ' + (path or field)) from error


def _number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CouplingError('NUMERIC: %s must be a finite number' % field)
    try:
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('nonfinite')
    except (ValueError, OverflowError) as error:
        raise CouplingError('NUMERIC: %s must be finite' % field) from error
    return result


def _fraction(value, field):
    return Fraction(_number(value, field))


def _float(value, field, direction=0):
    try:
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('overflow')
        if direction and (Fraction(result)-value)*direction < 0:
            result = math.nextafter(result, math.inf if direction > 0 else -math.inf)
        if not math.isfinite(result) or (value and result == 0.):
            raise ValueError('overflow or nonzero underflow after bound rounding')
        return result
    except (ValueError, OverflowError) as error:
        raise CouplingError('NUMERIC_RANGE: unrepresentable ' + field) from error


def _name(value, field):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', value):
        raise CouplingError('JOINT_NAME: %s needs an allocated ASCII alias' % field)
    return value


def evaluate_affine(multiplier, position, offset, *, field='affine state'):
    """Evaluate a*q+b without overflowing a representable final result."""
    value = _fraction(multiplier, 'multiplier') * _fraction(position, 'position') + _fraction(offset, 'offset')
    return _float(value, field)


def _flag(owner, field):
    value = _get(owner, field)
    if type(value) is not bool:
        raise CouplingError('NATIVE_METADATA: %s must be bool' % field)
    return value


def _motion_id(value):
    if isinstance(value, bool) or not isinstance(value, int):
        raise CouplingError('MOTION_ID: expected a documented JointMotionTypes integer')
    if value in (0, 1, 2):
        raise CouplingError('BALL_NATIVE_COORDINATES_UNPROVEN: no native Euler binding')
    if value not in range(3, 12):
        raise CouplingError('MOTION_ID: unsupported native motion %r' % value)
    return value


def _reference(value, field, nullable=False):
    if value is None and nullable:
        return
    if value is None or isinstance(value, (str, bytes, int, float, bool, list, tuple, dict, set)):
        raise CouplingError('NATIVE_METADATA: %s requires an actual SDK object reference' % field)


def _binding(binding, motion_id=None):
    if not isinstance(binding, MotionBinding) or not isinstance(binding.source, SourceIdentity):
        raise CouplingError('BINDING: resolver must return a canonical MotionBinding')
    _reference(binding.source.native, 'binding.source.native')
    _reference(binding.source.context, 'binding.source.context', nullable=True)
    native_id = _motion_id(binding.motion_type)
    if motion_id is not None and native_id != motion_id:
        raise CouplingError('BINDING: resolver returned the wrong motion ID')
    expected_unit = 'm' if native_id in _LINEAR else 'rad'
    if binding.unit != expected_unit:
        raise CouplingError('BINDING: %s requires SI unit %s' % (native_id, expected_unit))
    _name(binding.primitive, 'binding.primitive')
    _number(binding.current_position_si, 'binding.current_position_si')
    if _number(binding.coordinate_scale, 'binding.coordinate_scale') == 0.:
        raise CouplingError('BINDING: coordinate_scale cannot be zero')
    return binding


def affine_relation(driver, follower, value_one, value_two, *, is_reversed=False,
                    link_name=None):
    """Convert database step values into q_f = multiplier*q_d at current-pose zero.

    Bindings define q_export = coordinate_scale * (q_native_SI - current_SI).
    Native absolute phase is derived from the supplied current positions, not
    guessed from expressions. Endpoint reversal does not change source signs.
    """
    driver, follower = _binding(driver), _binding(follower)
    if type(is_reversed) is not bool:
        raise CouplingError('NATIVE_METADATA: is_reversed must be bool')
    if link_name is not None and (not isinstance(link_name, str) or not link_name):
        raise CouplingError('NATIVE_METADATA: name must be a nonempty string')
    values = [_number(v, f) for v, f in ((value_one, 'valueOne.value'), (value_two, 'valueTwo.value'))]
    steps = [Fraction(v) / (100 if b.motion_type in _LINEAR else 1)
             for v, b in zip(values, (driver, follower))]
    si_steps = [_float(v, 'step_values_si') for v in steps]
    if not steps[0]:
        raise CouplingError('ZERO_DENOMINATOR: valueOne.value cannot be zero')
    native_ratio = steps[1] / steps[0] * (-1 if is_reversed else 1)
    scales = [Fraction(_number(b.coordinate_scale, 'coordinate_scale')) for b in (driver, follower)]
    current = [_number(b.current_position_si, 'current_position_si') for b in (driver, follower)]
    phase = Fraction(current[1]) - native_ratio*Fraction(current[0])
    return dict(status='active', driver=driver.primitive, follower=follower.primitive,
                multiplier=_float(native_ratio*scales[1]/scales[0], 'export multiplier'), offset=0.,
                provenance=dict(link_name=link_name, motion_ids=[driver.motion_type, follower.motion_type],
                    units=[driver.unit, follower.unit], step_values_native=values, step_values_si=si_steps,
                    current_positions_si=current, coordinate_scales=[float(s) for s in scales],
                    is_reversed=is_reversed, native_ratio_si=_float(native_ratio, 'native ratio'),
                    native_phase_si=_float(phase, 'native phase'),
                    phase_basis='current_positions', exported_coordinates='scaled_current_pose_deltas'))


def read_motion_link(link, resolver, *, enclosing_context=None):
    """Read documented fields only. resolver(joint, link_context, motion_id) owns identity.

    The resolver must canonicalize SDK wrappers in the actual selected context.
    No token, source-name, collection fallback, or nativeObject heuristic is used.
    Suppressed links read only validity, name, and suppression metadata.
    """
    if not _flag(link, 'isValid'):
        raise CouplingError('NATIVE_INVALID: MotionLink is not valid')
    name = _get(link, 'name')
    if not isinstance(name, str) or not name:
        raise CouplingError('NATIVE_METADATA: name must be a nonempty string')
    if _flag(link, 'isSuppressed'):
        return dict(status='suppressed', provenance=dict(link_name=name))
    context = _get(link, 'assemblyContext')
    if context is None:
        context = enclosing_context
    one, two = _get(link, 'jointOne'), _get(link, 'jointTwo')
    _reference(context, 'assemblyContext', nullable=True)
    _reference(one, 'jointOne')
    if two is None:
        two = one
    _reference(two, 'jointTwo')
    ids = [_motion_id(_get(link, field)) for field in ('motionOne', 'motionTwo')]
    bindings, excluded = [], []
    for joint, motion_id in zip((one, two), ids):
        try:
            binding = resolver(joint, context, motion_id)
        except ExcludedMotion as error:
            bindings.append(None)
            excluded.append(str(error))
            continue
        except Exception as error:
            raise CouplingError('BINDING: resolver failed for motion %d in %s' % (motion_id, name)) from error
        bindings.append(_binding(binding, motion_id))
    if excluded:
        if len(excluded) != 2:
            raise CouplingError('MOTION_LINK_SELECTED_EXCLUDED: ' + name)
        return dict(status='excluded', provenance=dict(link_name=name, reason='All source joint endpoints are excluded'))
    values = [_get(_get(link, field), 'value', field+'.value') for field in ('valueOne', 'valueTwo')]
    return affine_relation(*bindings, *values, is_reversed=_flag(link, 'isReversed'), link_name=name)


def plain_data(value, path):
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float:
        return _number(value, path)
    if type(value) in (list, tuple):
        return [plain_data(v, path+'[]') for v in value]
    if type(value) is dict and all(type(k) is str for k in value):
        return {k: plain_data(v, path+'.'+k) for k, v in value.items()}
    raise CouplingError('PLAIN_DATA: %s contains non-serializable state; SDK references are transient' % path)


def _limits(joint, name):
    kind = joint.get('type')
    if kind not in (*_MOVING, 'fixed'):
        raise CouplingError('JOINT_TYPE: unsupported %s for %s' % (kind, name))
    if 'mimic' in joint:
        raise CouplingError('EXISTING_MIMIC: provide all relations to one planner')
    result = dict(type=kind)
    for key in ('lower_limit', 'upper_limit', 'velocity'):
        value = joint.get(key)
        result[key] = None if value is None else _number(value, name+'.'+key)
    lo, hi, velocity = (result[k] for k in ('lower_limit', 'upper_limit', 'velocity'))
    if lo is not None and hi is not None and lo > hi:
        raise CouplingError('LIMIT_ORDER: lower exceeds upper for ' + name)
    if velocity is not None and velocity < 0:
        raise CouplingError('VELOCITY: negative cap for ' + name)
    if kind == 'continuous' and (lo is not None or hi is not None):
        raise CouplingError('CONTINUOUS_BOUNDS: ambiguous source bounds for ' + name)
    return result


def apply_couplings(joints, relations, *, removed_joint_names=()):
    """Return owned plain joint records, independent names, and constraint receipts.

    Follower limits and effort remain unchanged. Independent travel and velocity
    are intersections of every dependent constraint. No controller or writer is
    created. Callers must reject representation_blockers before URDF publication.
    """
    owned, records = plain_data(joints, 'joints'), plain_data(relations, 'relations')
    if type(owned) is not dict or type(records) is not list:
        raise CouplingError('PLAIN_DATA: expected a joint mapping and relation sequence')
    source = {}
    for name, data in owned.items():
        _name(name, 'joint')
        if type(data) is not dict:
            raise CouplingError('PLAIN_DATA: joint records must be mappings')
        source[name] = _limits(data, name)
    if not isinstance(removed_joint_names, (list, tuple, set, frozenset)):
        raise CouplingError('REMOVED_CLOSURE_DOF: expected a collection of removed joint names')
    removed = {_name(n, 'removed_joint_names') for n in removed_joint_names}
    parents = {}
    for record in records:
        if type(record) is not dict:
            raise CouplingError('RELATION: expected mapping')
        if set(record) - {'status', 'driver', 'follower', 'multiplier', 'offset', 'provenance'}:
            raise CouplingError('RELATION_SCHEMA: only explicit affine dependencies are supported')
        if 'provenance' in record and type(record['provenance']) is not dict:
            raise CouplingError('RELATION_SCHEMA: provenance must be a plain mapping')
        status = record.get('status', 'active')
        if status in ('suppressed', 'excluded'):
            continue
        if status != 'active':
            raise CouplingError('RELATION: unknown status ' + str(status))
        driver, follower = record.get('driver'), record.get('follower')
        for name in (driver, follower):
            _name(name, 'relation endpoint')
            if name in removed:
                raise CouplingError('REMOVED_CLOSURE_DOF: dependency references removed joint ' + name)
            if name not in source:
                raise CouplingError('MISSING_JOINT: ' + name)
            if source[name]['type'] not in _MOVING:
                raise CouplingError('FIXED_DEPENDENCY: relation endpoint is not moving: ' + name)
        if driver == follower:
            raise CouplingError('SELF_DEPENDENCY: ' + driver)
        if follower in parents:
            raise CouplingError('DUPLICATE_FOLLOWER: ' + follower)
        a = _fraction(record.get('multiplier'), 'multiplier')
        b = _fraction(record.get('offset'), 'offset')
        parents[follower] = (driver, a, b)
        owned[follower]['mimic'] = dict(joint=driver, multiplier=float(a), offset=float(b))
    resolved = {}
    for name, limits in source.items():
        if limits['type'] not in _MOVING:
            continue
        path, seen, cursor = [], set(), name
        while cursor not in resolved:
            if cursor in seen:
                raise CouplingError('DEPENDENCY_CYCLE: ' + ' -> '.join(path+[cursor]))
            seen.add(cursor)
            path.append(cursor)
            if cursor not in parents:
                resolved[cursor] = (cursor, Fraction(1), Fraction(0))
                break
            cursor = parents[cursor][0]
        for item in reversed(path):
            if item in resolved:
                continue
            driver, a, b = parents[item]
            root, pa, pb = resolved[driver]
            result = (root, a*pa, a*pb+b)
            _float(result[1], 'composite multiplier for '+item)
            _float(result[2], 'composite offset for '+item)
            resolved[item] = result
    independent = [n for n in owned if n in resolved and n not in parents]
    domains = {n: [None, None, None] for n in independent}
    receipts, dependencies = [], {}
    for name, (root, a, b) in resolved.items():
        limits = source[name]
        lo, hi, cap = (None if limits[k] is None else Fraction(limits[k])
                       for k in ('lower_limit', 'upper_limit', 'velocity'))
        dependencies[name] = dict(independent_joint=root, multiplier=_float(a, 'multiplier'), offset=_float(b, 'offset'))
        projected_lo = projected_hi = projected_cap = None
        if not a:
            if (lo is not None and b < lo) or (hi is not None and b > hi):
                raise CouplingError('CONSTANT_OUTSIDE_LIMITS: ' + name)
        else:
            projected_lo = None if lo is None else (lo-b)/a
            projected_hi = None if hi is None else (hi-b)/a
            if a < 0:
                projected_lo, projected_hi = projected_hi, projected_lo
            projected_cap = None if cap is None else cap/abs(a)
            domain = domains[root]
            if projected_lo is not None:
                domain[0] = projected_lo if domain[0] is None else max(domain[0], projected_lo)
            if projected_hi is not None:
                domain[1] = projected_hi if domain[1] is None else min(domain[1], projected_hi)
            if projected_cap is not None:
                domain[2] = projected_cap if domain[2] is None else min(domain[2], projected_cap)
        receipts.append(dict(source_joint=name, independent_joint=root, constant=not bool(a),
            source_limits=dict(limits), multiplier=_float(a, 'receipt multiplier'), offset=_float(b, 'receipt offset'),
            projected_limits=dict(lower_limit=None if projected_lo is None else _float(projected_lo, 'projected lower', 1),
                                  upper_limit=None if projected_hi is None else _float(projected_hi, 'projected upper', -1),
                                  velocity=None if projected_cap is None else _float(projected_cap, 'projected velocity', -1))))
    effective, changes = {}, []
    for name, (lo, hi, cap) in domains.items():
        if lo is not None and hi is not None and lo > hi:
            raise CouplingError('EMPTY_FEASIBLE_DOMAIN: ' + name)
        limits = dict(lower_limit=None if lo is None else _float(lo, 'effective lower', 1),
                      upper_limit=None if hi is None else _float(hi, 'effective upper', -1),
                      velocity=None if cap is None else _float(cap, 'effective velocity', -1))
        if limits['lower_limit'] is not None and limits['upper_limit'] is not None and limits['lower_limit'] > limits['upper_limit']:
            raise CouplingError('NUMERIC_RANGE: no representable feasible coordinate for ' + name)
        effective[name] = limits
        owned[name].update(limits)
        if source[name]['type'] == 'continuous' and (lo is not None or hi is not None):
            owned[name]['type'] = 'revolute'
            changes.append(dict(joint=name, before='continuous', after='revolute',
                                reason='follower travel bounds constrain the independent angle'))
    blockers = []
    for name, data in owned.items():
        if data['type'] in ('revolute', 'prismatic') and (data.get('lower_limit') is None or data.get('upper_limit') is None):
            blockers.append(dict(joint=name, code='URDF_FINITE_TRAVEL_REQUIRED', reason='standard URDF requires both finite travel bounds'))
        if data['type'] in _MOVING and data.get('velocity') is None:
            blockers.append(dict(joint=name, code='URDF_VELOCITY_REQUIRED', reason='no finite velocity cap was supplied or derived'))
    return dict(joints=owned, relations=records, independent_joints=independent,
                dependencies=dependencies, source_limits=source, effective_limits=effective,
                limit_receipts=receipts, representation_changes=changes, representation_blockers=blockers)
