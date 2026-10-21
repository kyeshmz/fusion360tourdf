"""Independent spatial FK checks for the pure expansion boundary, not Fusion proof."""

import math
import random
import unittest
from types import SimpleNamespace as NS

from URDF_Exporter.core.generalized import (
    ExpansionError, SourceIdentity, expand_motion,
)


I = ((1., 0., 0., 0.), (0., 1., 0., 0.),
     (0., 0., 1., 0.), (0., 0., 0., 1.))


def mm(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(4))
                       for j in range(4)) for i in range(4))


def shift(v):
    return tuple(tuple(v[i] if j == 3 and i < 3 else I[i][j]
                       for j in range(4)) for i in range(4))


def turn(axis, angle):
    n = math.sqrt(sum(x*x for x in axis))
    x, y, z = (v/n for v in axis)
    c, s, d = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return ((c+x*x*d, x*y*d-z*s, x*z*d+y*s, 0.),
            (y*x*d+z*s, c+y*y*d, y*z*d-x*s, 0.),
            (z*x*d-y*s, z*y*d+x*s, c+z*z*d, 0.), I[3])


def inverse(a):
    return tuple(tuple(a[j][i] if j < 3 else
                       -sum(a[k][i]*a[k][3] for k in range(3))
                       for j in range(4)) for i in range(3)) + (I[3],)


def cm(a):
    return tuple(a[i][j] * (100 if i < 3 and j == 3 else 1)
                 for i in range(4) for j in range(4))


def pose_matrix(p):
    return tuple(tuple(p.rotation[i*3+j] if j < 3 else p.xyz[i]
                       for j in range(4)) for i in range(3)) + (I[3],)


def limits(lo=None, hi=None):
    return NS(isMinimumValueEnabled=lo is not None,
              isMaximumValueEnabled=hi is not None,
              minimumValue=lo, maximumValue=hi)


class Vector:
    def __init__(self, values):
        self.values = values

    def asArray(self):
        return self.values


def motion(kind, q0=(.43, 2.5, -1.7), bounded=True):
    r, s, t = q0
    fields = dict(rotationValue=r, rotationLimits=limits(-2, 3),
                  slideValue=s, slideLimits=limits(-20, 30))
    if not bounded:
        fields.update(rotationLimits=limits(), slideLimits=limits())
    if kind == 'cylindrical':
        fields['rotationAxisVector'] = Vector((0, 0, 2))
    elif kind == 'pin_slot':
        fields.update(rotationAxisVector=Vector((0, 0, 3)),
                      slideDirectionVector=Vector((2, 0, 0)))
    elif kind == 'planar':
        fields.update(normalDirectionVector=Vector((0, 0, 3)),
                      primarySlideDirectionVector=Vector((2, 0, 0)),
                      secondarySlideDirectionVector=Vector((0, -4, 0)),
                      primarySlideValue=s, secondarySlideValue=t,
                      primarySlideLimits=limits(-20, 30),
                      secondarySlideLimits=limits(-10, 40))
        if not bounded:
            fields.update(primarySlideLimits=limits(), secondarySlideLimits=limits())
    elif kind == 'ball':
        fields = {p+'Limits': limits() for p in ('pitch', 'yaw', 'roll')}
    return NS(**fields)


def native_factors(kind, q):
    r, s, t = q
    rot = turn((0, 0, 1), r)
    if kind == 'cylindrical':
        return mm(rot, shift((0, 0, s*.01)))
    if kind == 'pin_slot':
        return mm(shift((s*.01, 0, 0)), rot)
    return mm(shift((s*.01, -t*.01, 0)), rot)


class GeneralizedMotionTests(unittest.TestCase):
    def setUp(self):
        self.native, self.context = object(), object()
        self.source = SourceIdentity(self.native, self.context)
        self.a = mm(shift((.27, -.13, .42)), turn((1, 2, -3), .71))
        self.b = mm(shift((.04, .07, -.03)), turn((-2, 4, 1), -.62))
        self.p = mm(shift((-.34, .11, .08)), turn((3, -1, 2), -.37))

    def scene(self, kind, q0, reverse=False, **kwargs):
        f0 = native_factors(kind, q0)
        child = mm(mm(self.a, f0), self.b)
        anchor = mm(self.a, shift((0, 0, q0[1]*.01) if kind == 'cylindrical'
                                  else (q0[1]*.01, -q0[2]*.01 if kind == 'planar' else 0, 0)))
        helper_rotation = kwargs.pop('helper_rotation', None)
        if helper_rotation is not None:
            anchor = tuple(tuple(helper_rotation[i][j] if j < 3 else anchor[i][3]
                                 for j in range(4)) for i in range(3)) + (I[3],)
        result = expand_motion(
            kind, kwargs.pop('motion', motion(kind, q0)), source=self.source,
            source_name='joint_A', source_path='assembly+joint A',
            parent='base_link', child='body', parent_pose_cm=cm(self.p),
            child_pose_cm=cm(child), joint_pose_cm=cm(anchor),
            vector_rotation=tuple(self.a[i][j] for i in range(3) for j in range(3)),
            reverse=reverse, **kwargs)
        return result, child

    def assertPose(self, actual, expected):
        for a, e in zip(sum(actual, ()), sum(expected, ())):
            self.assertAlmostEqual(a, e, places=11)

    def fk(self, result, deltas):
        out = I
        for joint in result.primitives:
            out = mm(out, pose_matrix(joint.origin))
            if joint.type == 'fixed':
                continue
            q = deltas[joint.coordinate]
            action = (shift(tuple(q*x for x in joint.axis)) if joint.type == 'prismatic'
                      else turn(joint.axis, q))
            out = mm(out, action)
        return out

    def test_independent_native_fk_zero_mixed_current_and_reversed(self):
        rng = random.Random(20261020)
        for kind in ('cylindrical', 'pin_slot', 'planar'):
            for q0 in ((0, 0, 0), (.43, 2.5, -1.7)):
                for reverse in (False, True):
                    result, child = self.scene(kind, q0, reverse)
                    samples = [q0, (0, 0, 0), (.81, -3.7, 2.9)]
                    samples += [(rng.uniform(-1.5, 1.5), rng.uniform(-8, 8),
                                 rng.uniform(-6, 6)) for _ in range(24)]
                    for q in samples:
                        delta = dict(rotate=q[0]-q0[0], slide=(q[1]-q0[1])*.01,
                                     primary=(q[1]-q0[1])*.01,
                                     secondary=(q[2]-q0[2])*.01)
                        desired = mm(mm(self.a, native_factors(kind, q)), self.b)
                        if reverse:
                            expected = mm(inverse(desired), self.p)
                        else:
                            expected = mm(inverse(self.p), desired)
                        with self.subTest(kind=kind, q0=q0, reverse=reverse, q=q):
                            self.assertPose(self.fk(result, delta), expected)

    def test_primitive_counts_frame_ownership_and_current_poses(self):
        for kind, count in (('cylindrical', 2), ('pin_slot', 2), ('planar', 3)):
            for reverse in (False, True):
                result, child = self.scene(kind, (.43, 2.5, -1.7), reverse)
                self.assertEqual(len(result.primitives), count+1)
                self.assertEqual(sum(j.type != 'fixed' for j in result.primitives), count)
                self.assertEqual(len(result.frames), count)
                for physical in ('base_link', 'body'):
                    self.assertEqual(sum((j.parent == physical)+(j.child == physical)
                                         for j in result.primitives), 1)
                self.assertEqual(result.primitives[0].parent, 'body' if reverse else 'base_link')
                self.assertEqual(result.primitives[-1].child, 'base_link' if reverse else 'body')
                current, terminal = (child, self.p) if reverse else (self.p, child)
                frames = {f.name: f for f in result.frames}
                for joint in result.primitives:
                    current = mm(current, pose_matrix(joint.origin))
                    expected = pose_matrix(frames[joint.child].pose) if joint.child in frames else terminal
                    self.assertPose(current, expected)
                for frame in result.frames:
                    self.assertEqual(frame.kind, 'frame')
                    self.assertEqual(frame.source_path, 'assembly+joint A')
                    for forbidden in ('mass', 'inertia', 'mesh', 'body', 'owner'):
                        self.assertFalse(hasattr(frame, forbidden))

    def test_arbitrary_vector_and_helper_bases_do_not_change_fk(self):
        rng = random.Random(914)
        q0, q = (.37, 3.2, -2.1), (-.82, -2.8, 4.1)
        for _ in range(10):
            self.a = mm(shift(tuple(rng.uniform(-.4, .4) for _ in range(3))),
                        turn(tuple(rng.uniform(-1, 1) for _ in range(3)), rng.uniform(-3, 3)))
            helper = turn(tuple(rng.uniform(-1, 1) for _ in range(3)), rng.uniform(-3, 3))
            for kind in ('cylindrical', 'pin_slot', 'planar'):
                for reverse in (False, True):
                    result, _ = self.scene(kind, q0, reverse, helper_rotation=helper)
                    desired = mm(mm(self.a, native_factors(kind, q)), self.b)
                    expected = mm(inverse(desired), self.p) if reverse else mm(inverse(self.p), desired)
                    delta = dict(rotate=q[0]-q0[0], slide=(q[1]-q0[1])*.01,
                                 primary=(q[1]-q0[1])*.01, secondary=(q[2]-q0[2])*.01)
                    self.assertPose(self.fk(result, delta), expected)

    def test_native_binding_ids_si_limits_and_exact_identity(self):
        for kind, ids in (('cylindrical', {3, 4}), ('pin_slot', {5, 6}), ('planar', {7, 8, 9})):
            result, _ = self.scene(kind, (.43, 2.5, -1.7), True)
            self.assertEqual({b.motion_type for b in result.bindings}, ids)
            for binding in result.bindings:
                self.assertIs(binding.source.native, self.native)
                self.assertIs(binding.source.context, self.context)
                self.assertIs(result.binding_for(self.native, self.context, binding.motion_type), binding)
                joint = next(j for j in result.primitives if j.name == binding.primitive)
                if binding.unit == 'rad':
                    self.assertAlmostEqual(binding.current_position_si, .43)
                    self.assertAlmostEqual(joint.lower_limit, -2.43)
                    self.assertAlmostEqual(joint.upper_limit, 2.57)
                elif binding.motion_type == 9:
                    self.assertAlmostEqual(binding.current_position_si, -.017)
                    self.assertAlmostEqual(joint.lower_limit, -.083)
                    self.assertAlmostEqual(joint.upper_limit, .417)
                else:
                    self.assertAlmostEqual(binding.current_position_si, .025)
                    self.assertAlmostEqual(joint.lower_limit, -.225)
                    self.assertAlmostEqual(joint.upper_limit, .275)
                self.assertEqual(binding.coordinate_scale, 1.)
            with self.assertRaisesRegex(ExpansionError, 'SOURCE_IDENTITY'):
                result.binding_for(object(), self.context, min(ids))
            with self.assertRaisesRegex(ExpansionError, 'SOURCE_IDENTITY'):
                result.binding_for(self.native, object(), min(ids))

    def test_reversal_reorders_factors_not_source_coordinate_sign(self):
        for kind in ('cylindrical', 'pin_slot', 'planar'):
            forward, _ = self.scene(kind, (.43, 2.5, -1.7))
            backward, _ = self.scene(kind, (.43, 2.5, -1.7), True)
            self.assertEqual([j.coordinate for j in backward.primitives if j.type != 'fixed'],
                             [j.coordinate for j in forward.primitives if j.type != 'fixed'][::-1])
            self.assertTrue(all(b.coordinate_scale == 1 for b in backward.bindings))

    def test_unbounded_translation_has_no_fabricated_bounds(self):
        for kind in ('cylindrical', 'pin_slot', 'planar'):
            result, _ = self.scene(kind, (0, 0, 0), motion=motion(kind, bounded=False))
            for joint in result.primitives:
                if joint.type == 'fixed':
                    continue
                self.assertIsNone(joint.lower_limit)
                self.assertIsNone(joint.upper_limit)
                self.assertEqual(joint.type, 'continuous' if joint.coordinate == 'rotate' else 'prismatic')

    def test_one_sided_limits_preserved_and_malformed_data_rejected(self):
        m = motion('pin_slot')
        m.slideLimits = limits(-7, None)
        result, _ = self.scene('pin_slot', (.43, 2.5, -1.7), motion=m)
        j = next(j for j in result.primitives if j.coordinate == 'slide')
        self.assertAlmostEqual(j.lower_limit, -.095)
        self.assertIsNone(j.upper_limit)
        for mutate, diagnostic in (
            (lambda m: setattr(m, 'slideValue', float('nan')), 'slideValue'),
            (lambda m: setattr(m, 'rotationAxisVector', Vector((0, 0, 0))), 'rotationAxisVector'),
            (lambda m: setattr(m, 'slideDirectionVector', Vector((1, 0, 1))), 'ORTHOGONAL'),
            (lambda m: setattr(m, 'slideLimits', limits(5, -5)), 'LIMIT_ORDER'),
            (lambda m: delattr(m, 'slideValue'), 'slideValue'),
        ):
            m = motion('pin_slot')
            mutate(m)
            with self.subTest(diagnostic=diagnostic), self.assertRaisesRegex(ExpansionError, diagnostic):
                self.scene('pin_slot', (.43, 2.5, -1.7), motion=m)

    def test_reserved_names_collisions_and_exact_dof_selectors(self):
        result, _ = self.scene('planar', (0, 0, 0))
        name = result.primitives[0].name
        changed, _ = self.scene('planar', (0, 0, 0), dof_settings={name: {'effort': 7, 'velocity': .2}})
        self.assertEqual(changed.primitives[0].effort, 7)
        self.assertEqual(changed.primitives[0].velocity, .2)
        for kwargs, diagnostic in (({'occupied_names': [name]}, 'NAME_COLLISION'),
                                   ({'occupied_names': [result.frames[0].name]}, 'NAME_COLLISION'),
                                   ({'occupied_names': [result.primitives[-1].name]}, 'NAME_COLLISION'),
                                   ({'dof_settings': {'joint_A': {'effort': 3}}}, 'DOF_SELECTOR'),
                                   ({'dof_settings': {name: {'lower': -2}}}, 'DOF_SETTING'),
                                   ({'effort': -1}, 'effort')):
            with self.assertRaisesRegex(ExpansionError, diagnostic):
                self.scene('planar', (0, 0, 0), **kwargs)
        with self.assertRaisesRegex(ExpansionError, 'SOURCE_IDENTITY'):
            SourceIdentity('entity-token', self.context)

    def test_planar_secondary_is_read_and_validated(self):
        m = motion('planar')
        m.secondarySlideDirectionVector = Vector((1, 0, 0))
        with self.assertRaisesRegex(ExpansionError, 'ORTHOGONAL'):
            self.scene('planar', (0, 0, 0), motion=m)

    def test_read_only_inputs_and_only_documented_getters(self):
        class ReadOnly:
            def __init__(self, value):
                object.__setattr__(self, 'value', value)

            def __getattr__(self, name):
                return getattr(self.value, name)

            def __setattr__(self, name, value):
                raise AssertionError('Native mutation attempted: ' + name)

        for kind in ('cylindrical', 'pin_slot', 'planar', 'ball'):
            m = motion(kind)
            for key, value in list(vars(m).items()):
                if key.endswith('Limits'):
                    setattr(m, key, ReadOnly(value))
            original = dict(vars(m))
            if kind == 'ball':
                self.ball(motion=ReadOnly(m))
            else:
                self.scene(kind, (.43, 2.5, -1.7), motion=ReadOnly(m))
            self.assertEqual(vars(m), original)

    def ball(self, reverse=False, **kwargs):
        c = mm(mm(self.a, turn((2, -1, 3), 1.27)), self.b)
        return expand_motion('ball', kwargs.pop('motion', motion('ball')),
                             source=self.source, source_name='ball_A', source_path='ball',
                             parent='base_link', child='body', parent_pose_cm=cm(self.p),
                             child_pose_cm=cm(c), joint_pose_cm=cm(self.a),
                             reverse=reverse, **kwargs), c

    def test_unbounded_ball_export_local_so3_and_singular_chart(self):
        rng = random.Random(731)
        samples = [(0, 0, 0), (.31, -.74, 1.1), (.4, math.pi/2, -.8),
                   (-.7, -math.pi/2, .2)]
        samples += [tuple(rng.uniform(-math.pi, math.pi) for _ in range(3)) for _ in range(96)]
        for reverse in (False, True):
            result, c = self.ball(reverse)
            self.assertEqual(result.parameterization, 'export_local_rx_ry_rz')
            self.assertEqual(len(result.primitives), 4)
            self.assertEqual(len(result.frames), 3)
            self.assertEqual(result.bindings, ())
            self.assertEqual(result.blocked_native_motion_ids, (0, 1, 2))
            for angles in samples:
                r = mm(mm(turn((1, 0, 0), angles[0]), turn((0, 1, 0), angles[1])),
                       turn((0, 0, 1), angles[2]))
                desired = mm(mm(mm(self.a, r), inverse(self.a)), c)
                expected = mm(inverse(desired), self.p) if reverse else mm(inverse(self.p), desired)
                self.assertPose(self.fk(result, dict(zip(('local_x', 'local_y', 'local_z'), angles))), expected)
            for motion_id in (0, 1, 2):
                with self.assertRaisesRegex(ExpansionError, 'BALL_NATIVE_COORDINATES_UNPROVEN'):
                    result.binding_for(self.native, self.context, motion_id)

    def test_bounded_ball_never_discards_any_enabled_bound(self):
        for field in ('pitch', 'yaw', 'roll'):
            for bounds in (limits(-1, 1), limits(None, 2), limits(-2, None)):
                m = motion('ball')
                setattr(m, field+'Limits', bounds)
                with self.assertRaisesRegex(ExpansionError, 'BALL_NATIVE_BOUNDS_UNPROVEN'):
                    self.ball(motion=m)

    def test_ball_chart_reaches_independent_random_so3_targets(self):
        rng = random.Random(187)
        for reverse in (False, True):
            result, c0 = self.ball(reverse)
            for _ in range(25):
                raw = [rng.gauss(0, 1) for _ in range(4)]
                norm = math.sqrt(sum(v*v for v in raw))
                w, x, y, z = [v/norm for v in raw]
                r = ((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y), 0.),
                     (2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x), 0.),
                     (2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y), 0.), I[3])
                angles = (math.atan2(-r[1][2], r[2][2]), math.asin(r[0][2]),
                          math.atan2(-r[0][1], r[0][0]))
                desired = mm(mm(mm(self.a, r), inverse(self.a)), c0)
                expected = mm(inverse(desired), self.p) if reverse else mm(inverse(self.p), desired)
                self.assertPose(self.fk(result, dict(zip(('local_x', 'local_y', 'local_z'), angles))), expected)

    def test_pose_and_kind_validation(self):
        args = dict(source=self.source, source_name='x', source_path='x', parent='a', child='b',
                    parent_pose_cm=cm(I), child_pose_cm=cm(I), joint_pose_cm=cm(I))
        for field, value, diagnostic in (
            ('parent_pose_cm', (0,)*16, 'POSE'), ('source_name', 'fusion_helper_bad', 'RESERVED'),
            ('parent', 'rigid_attachment_bad', 'RESERVED'),
            ('child', 'a', 'DISTINCT'), ('vector_rotation', (1,)*9, 'ROTATION')):
            with self.subTest(field=field), self.assertRaisesRegex(ExpansionError, diagnostic):
                expand_motion('cylindrical', motion('cylindrical'), **dict(args, **{field: value}))
        with self.assertRaisesRegex(ExpansionError, 'UNSUPPORTED_KIND'):
            expand_motion('inferred', NS(), **args)


if __name__ == '__main__':
    unittest.main()
