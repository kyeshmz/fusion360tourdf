"""Affine backend oracles. SDK doubles and spatial equations are not native proof."""

from copy import deepcopy
from dataclasses import replace
from fractions import Fraction
import json
import math
import random
from types import SimpleNamespace as NS
import unittest

from URDF_Exporter.core import coupling as C
from URDF_Exporter.core.generalized import MotionBinding, SourceIdentity, expand_motion
from tests.test_generalized_motion import (
    I, cm, inverse, mm, motion, native_factors, pose_matrix, shift, turn,
)


def joint(kind='revolute', lo=-10., hi=10., velocity=20.):
    return dict(type=kind, lower_limit=lo, upper_limit=hi, velocity=velocity,
                effort=7., parent='base_link', child='body', axis=[0., 0., 1.])


def relation(driver='a', follower='b', a=2., b=0.):
    return dict(driver=driver, follower=follower, multiplier=a, offset=b,
                provenance={'reason': 'explicit affine input'})


class Broken:
    def __init__(self, values, bad):
        self.values, self.bad = values, bad

    def __getattr__(self, field):
        if field == self.bad:
            raise RuntimeError('SDK getter failed for ' + field)
        return getattr(self.values, field)


class MotionCouplingTests(unittest.TestCase):
    def setUp(self):
        self.n1, self.n2, self.ctx = object(), object(), object()
        self.b1 = MotionBinding(SourceIdentity(self.n1, self.ctx), 10, 'a', .7, 'rad')
        self.b2 = MotionBinding(SourceIdentity(self.n2, self.ctx), 11, 'b', .13, 'm')
        self.link = NS(name='Motion link 1', isValid=True, isSuppressed=False,
                       jointOne=self.n1, jointTwo=self.n2, assemblyContext=self.ctx,
                       motionOne=10, motionTwo=11, valueOne=NS(value=2.),
                       valueTwo=NS(value=5.), isReversed=False)
        self.calls = []

    def resolve(self, native, context, motion_id):
        self.calls.append((native, context, motion_id))
        for binding in (self.b1, self.b2):
            if binding.source.matches(native, context) and binding.motion_type == motion_id:
                return binding
        raise LookupError('not selected or unavailable native DOF')

    def read(self, **changes):
        return C.read_motion_link(NS(**(vars(self.link) | changes)), self.resolve)

    def test_native_mixed_units_current_phase_and_no_native_refs(self):
        r = self.read()
        self.assertEqual((r['driver'], r['follower']), ('a', 'b'))
        self.assertAlmostEqual(r['multiplier'], .025)
        self.assertEqual(r['offset'], 0.)
        p = r['provenance']
        self.assertEqual(p['current_positions_si'], [.7, .13])
        self.assertAlmostEqual(p['native_phase_si'], .13-.025*.7)
        self.assertEqual(p['step_values_si'], [2., .05])
        self.assertEqual(p['motion_ids'], [10, 11])
        self.assertEqual(self.calls, [(self.n1, self.ctx, 10), (self.n2, self.ctx, 11)])
        self.assertEqual(json.loads(json.dumps(r)), r)

    def test_inverse_units_reversal_and_coordinate_scaling(self):
        self.b1 = replace(self.b1, motion_type=11, unit='m', coordinate_scale=-2.)
        self.b2 = replace(self.b2, motion_type=10, unit='rad', coordinate_scale=3.)
        r = self.read(motionOne=11, motionTwo=10, isReversed=True)
        self.assertAlmostEqual(r['multiplier'], 375.)
        self.assertEqual(r['offset'], 0.)

    def test_all_documented_scalar_motion_ids(self):
        for motion_id in range(3, 12):
            with self.subTest(motion_id=motion_id):
                unit = 'm' if motion_id in (4, 6, 8, 9, 11) else 'rad'
                self.b2 = replace(self.b2, motion_type=motion_id, unit=unit)
                r = self.read(motionTwo=motion_id)
                self.assertEqual(r['provenance']['motion_ids'][1], motion_id)

    def test_same_joint_null_joint_two_uses_resolver_not_names(self):
        self.b1 = replace(self.b1, motion_type=3)
        self.b2 = replace(self.b2, source=self.b1.source, motion_type=4)
        r = self.read(jointTwo=None, motionOne=3, motionTwo=4)
        self.assertEqual(self.calls, [(self.n1, self.ctx, 3), (self.n1, self.ctx, 4)])
        self.assertEqual((r['driver'], r['follower']), ('a', 'b'))

    def test_suppressed_does_not_read_active_fields(self):
        r = C.read_motion_link(NS(name='Suppressed', isValid=True, isSuppressed=True),
                               lambda *args: self.fail('suppressed resolver called'))
        self.assertEqual(r['status'], 'suppressed')
        result = C.apply_couplings({'a': joint()}, [r])
        self.assertEqual(result['independent_joints'], ['a'])
        self.assertEqual(result['relations'], [r])

    def test_each_native_getter_and_parameter_value_fails_closed_with_cause(self):
        fields = ('name', 'isValid', 'isSuppressed', 'assemblyContext', 'jointOne',
                  'jointTwo', 'motionOne', 'motionTwo', 'valueOne', 'valueTwo', 'isReversed')
        for field in fields:
            with self.subTest(field=field):
                with self.assertRaisesRegex(C.CouplingError, field) as caught:
                    C.read_motion_link(Broken(self.link, field), self.resolve)
                self.assertIsInstance(caught.exception.__cause__, RuntimeError)
        for parameter in ('valueOne', 'valueTwo'):
            with self.subTest(parameter=parameter):
                with self.assertRaisesRegex(C.CouplingError, parameter+'.value') as caught:
                    self.read(**{parameter: Broken(NS(value=1.), 'value')})
                self.assertIsInstance(caught.exception.__cause__, RuntimeError)

    def test_invalid_native_metadata(self):
        cases = [dict(isValid=False), dict(isValid=1), dict(isSuppressed=0),
                 dict(isReversed=1), dict(name=''), dict(jointOne=None),
                 dict(jointOne='joint token'), dict(jointTwo=False), dict(assemblyContext='context token'),
                 dict(motionOne=99), dict(motionOne=True), dict(motionTwo=1.5),
                 dict(valueOne=None), dict(valueTwo=NS(value='5 cm'))]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(C.CouplingError):
                self.read(**case)

    def test_ball_ids_are_explicitly_blocked(self):
        for motion_id in (0, 1, 2):
            with self.subTest(motion_id=motion_id), self.assertRaisesRegex(C.CouplingError, 'BALL_NATIVE'):
                self.read(motionTwo=motion_id)

    def test_resolver_missing_wrong_binding_and_exception(self):
        with self.assertRaisesRegex(C.CouplingError, 'BINDING') as caught:
            self.read(jointTwo=object())
        self.assertIsInstance(caught.exception.__cause__, LookupError)
        for value in (None, 'b', replace(self.b2, motion_type=10),
                      replace(self.b2, unit='cm'), replace(self.b2, coordinate_scale=0.),
                      replace(self.b2, current_position_si=math.nan)):
            with self.subTest(value=value), self.assertRaises(C.CouplingError):
                C.read_motion_link(self.link, lambda n, c, m: self.b1 if m == 10 else value)

    def test_affine_numbers_denominator_overflow_underflow(self):
        cases = [(0., 1.), (math.nan, 1.), (1., math.inf),
                 (1e-300, 1e300), (1e300, 1e-300), (1., 5e-324)]
        for first, second in cases:
            with self.subTest(values=(first, second)), self.assertRaises(C.CouplingError):
                self.read(valueOne=NS(value=first), valueTwo=NS(value=second))
        r = self.read(valueTwo=NS(value=0.))
        self.assertEqual(r['multiplier'], 0.)
        self.assertEqual(r['offset'], 0.)
        self.assertEqual(r['provenance']['native_phase_si'], .13)

    def test_independent_scalar_equation_matrix(self):
        rng = random.Random(16002026)
        for du, fu in (('rad', 'rad'), ('m', 'rad'), ('rad', 'm'), ('m', 'm')):
            self.b1 = replace(self.b1, unit=du, motion_type=10 if du == 'rad' else 11)
            self.b2 = replace(self.b2, unit=fu, motion_type=10 if fu == 'rad' else 11)
            for rev in (False, True):
                for scale_d, scale_f in ((1., 1.), (-1., 1.), (2., -3.)):
                    self.b1 = replace(self.b1, coordinate_scale=scale_d)
                    self.b2 = replace(self.b2, coordinate_scale=scale_f)
                    r = C.affine_relation(self.b1, self.b2, 2., 5., is_reversed=rev)
                    native_ratio = (5.*(.01 if fu == 'm' else 1.))/(2.*(.01 if du == 'm' else 1.))
                    native_ratio *= -1 if rev else 1
                    phase = .13-native_ratio*.7
                    for delta in [0.] + [rng.uniform(-2., 2.) for _ in range(10)]:
                        native_driver = .7+delta
                        native_follower = native_ratio*native_driver+phase
                        expected = scale_f*(native_follower-.13)
                        actual = r['multiplier']*(scale_d*delta)+r['offset']
                        self.assertAlmostEqual(actual, expected, places=10)

    def test_spatial_fk_current_pose_same_joint_and_endpoint_reversal(self):
        a = mm(shift((.2, -.1, .4)), turn((1, -2, 3), .63))
        b = mm(shift((-.1, .07, .2)), turn((2, 3, -1), -.32))
        p = mm(shift((.3, .2, -.1)), turn((-1, 2, 2), .4))
        q0 = (.43, 2.5, -1.7)
        ids = {'cylindrical': (3, 4), 'pin_slot': (5, 6), 'planar': (7, 8)}
        rng = random.Random(161616)
        for kind in ids:
            child = mm(mm(a, native_factors(kind, q0)), b)
            pivot = mm(a, shift((0, 0, .025) if kind == 'cylindrical'
                                else (.025, .017 if kind == 'planar' else 0, 0)))
            for endpoint_reversed in (False, True):
                expansion = expand_motion(kind, motion(kind, q0), source=self.b1.source,
                    source_name='native_joint', source_path='selected/path', parent='base_link', child='body',
                    parent_pose_cm=cm(p), child_pose_cm=cm(child), joint_pose_cm=cm(pivot),
                    vector_rotation=tuple(a[i][j] for i in range(3) for j in range(3)), reverse=endpoint_reversed)
                driver, follower = [expansion.binding_for(self.n1, self.ctx, i) for i in ids[kind]]
                for rev in (False, True):
                    r = C.affine_relation(driver, follower, 2., 5., is_reversed=rev)
                    self.assertAlmostEqual(r['multiplier'], -.025 if rev else .025)
                    for d in [0.] + [rng.uniform(-1., 1.) for _ in range(10)]:
                        values = {v.name: 0. for v in expansion.primitives}
                        values[driver.primitive] = d
                        values[follower.primitive] = r['multiplier']*d
                        actual = I
                        for j in expansion.primitives:
                            actual = mm(actual, pose_matrix(j.origin))
                            if j.type != 'fixed':
                                actual = mm(actual, shift(tuple(values[j.name]*v for v in j.axis))
                                            if j.type == 'prismatic' else turn(j.axis, values[j.name]))
                        native_q = (q0[0]+d, q0[1]+(-2.5 if rev else 2.5)*d, q0[2])
                        expected_child = mm(mm(a, native_factors(kind, native_q)), b)
                        expected = mm(inverse(expected_child), p) if endpoint_reversed else mm(inverse(p), expected_child)
                        for actual_value, expected_value in zip(sum(actual, ()), sum(expected, ())):
                            self.assertAlmostEqual(actual_value, expected_value, places=11)

    def test_owned_application_preserves_follower_limits_effort_and_provenance(self):
        joints = {'a': joint(), 'b': joint(lo=-1., hi=3., velocity=4.)}
        relations = [relation(a=-2., b=1.)]
        before = deepcopy((joints, relations))
        result = C.apply_couplings(joints, relations)
        self.assertEqual(result['joints']['b']['mimic'], {'joint': 'a', 'multiplier': -2., 'offset': 1.})
        self.assertEqual(result['independent_joints'], ['a'])
        self.assertEqual(result['effective_limits']['a']['lower_limit'], -1.)
        self.assertEqual(result['effective_limits']['a']['upper_limit'], 1.)
        self.assertEqual(result['effective_limits']['a']['velocity'], 2.)
        for key in ('lower_limit', 'upper_limit', 'velocity', 'effort'):
            self.assertEqual(result['joints']['b'][key], joints['b'][key])
        self.assertEqual(result['joints']['a']['effort'], 7.)
        result['joints']['b']['axis'][0] = 88.
        self.assertEqual((joints, relations), before)
        self.assertEqual(json.loads(json.dumps(result)), result)

    def test_chains_one_to_many_and_independent_commands(self):
        joints = {'a': joint(lo=-100, hi=100), 'b': joint(lo=-8, hi=8, velocity=4),
                  'c': joint(lo=-7, hi=5, velocity=3), 'd': joint(lo=-3, hi=7, velocity=6),
                  'free': joint(), 'attachment': joint('fixed')}
        result = C.apply_couplings(joints, [relation('b', 'c', -3, 2), relation('a', 'b', -2, 1), relation('a', 'd', 2, 1)])
        self.assertEqual(result['independent_joints'], ['a', 'free'])
        self.assertEqual(result['effective_limits']['a'], dict(lower_limit=-1., upper_limit=1., velocity=.5))
        self.assertEqual(result['dependencies']['c'], dict(independent_joint='a', multiplier=6., offset=-1.))
        self.assertEqual(len(result['limit_receipts']), 5)

    def test_continuous_acquires_real_bounds_with_auditable_reason(self):
        result = C.apply_couplings({'a': joint('continuous', None, None), 'b': joint(lo=-2., hi=6.)}, [relation()])
        self.assertEqual(result['joints']['a']['type'], 'revolute')
        self.assertEqual(result['effective_limits']['a']['lower_limit'], -1.)
        self.assertEqual(result['effective_limits']['a']['upper_limit'], 3.)
        self.assertEqual(result['source_limits']['a']['type'], 'continuous')
        self.assertIn('follower', result['representation_changes'][0]['reason'])
        self.assertEqual(result['representation_blockers'], [])

    def test_one_sided_and_unbounded_limits_are_explicit_representation_gates(self):
        for lo, hi in ((None, 3.), (-2., None), (None, None)):
            with self.subTest(bounds=(lo, hi)):
                result = C.apply_couplings({'a': joint('continuous', None, None),
                                           'b': joint('prismatic', lo, hi)}, [relation()])
                self.assertTrue(result['representation_blockers'])
                self.assertEqual(result['joints']['b']['lower_limit'], lo)
                self.assertEqual(result['joints']['b']['upper_limit'], hi)

    def test_constants_inside_bounds_do_not_constrain_driver(self):
        result = C.apply_couplings({'a': joint(), 'b': joint(lo=2., hi=4., velocity=0.),
                                   'c': joint(lo=4., hi=8.)},
                                  [relation('a', 'b', 0., 3.), relation('b', 'c', 2., 0.)])
        self.assertEqual(result['effective_limits']['a'], dict(lower_limit=-10., upper_limit=10., velocity=20.))
        self.assertEqual(result['dependencies']['c']['multiplier'], 0.)
        self.assertEqual(result['dependencies']['c']['offset'], 6.)

    def test_empty_domains_and_out_of_bounds_constants_fail_without_mutation(self):
        for r in (relation(a=0., b=4.), relation(a=1., b=100.)):
            joints = {'a': joint(lo=-1., hi=1.), 'b': joint(lo=-1., hi=1.)}
            before = deepcopy(joints)
            with self.assertRaisesRegex(C.CouplingError, 'EMPTY|CONSTANT'):
                C.apply_couplings(joints, [r])
            self.assertEqual(joints, before)

    def test_missing_fixed_self_duplicate_cycles_existing_mimic_and_removed_closure(self):
        cases = [([relation('missing', 'b')], {}), ([relation('a', 'missing')], {}),
                 ([relation('a', 'a')], {}), ([relation(), relation()], {}),
                 ([relation('a', 'b'), relation('b', 'a')], {}),
                 ([relation('a', 'b'), relation('b', 'c'), relation('c', 'a')], {}),
                 ([relation('fixed', 'b')], {}), ([relation('a', 'fixed')], {}),
                 ([relation('closure', 'b')], {'removed_joint_names': ['closure']}),
                 ([relation('a', 'closure')], {'removed_joint_names': ['closure']})]
        for rs, options in cases:
            with self.subTest(relations=rs), self.assertRaises(C.CouplingError):
                C.apply_couplings({'a': joint(), 'b': joint(), 'c': joint(), 'fixed': joint('fixed')}, rs, **options)
        with self.assertRaisesRegex(C.CouplingError, 'EXISTING_MIMIC'):
            C.apply_couplings({'a': joint() | {'mimic': {}}}, [])

    def test_nonfinite_coefficients_limits_and_invalid_types_reject(self):
        for value in (math.nan, math.inf, -math.inf, True, '2'):
            for field in ('multiplier', 'offset'):
                with self.subTest(field=field, value=value), self.assertRaises(C.CouplingError):
                    C.apply_couplings({'a': joint(), 'b': joint()}, [relation() | {field: value}])
            for field in ('lower_limit', 'upper_limit', 'velocity'):
                with self.subTest(field=field, value=value), self.assertRaises(C.CouplingError):
                    C.apply_couplings({'a': joint() | {field: value}}, [])
        for bad in (joint(lo=3., hi=2.), joint(velocity=-1.), joint('floating'),
                    joint('continuous', -1., 1.)):
            with self.subTest(joint=bad), self.assertRaises(C.CouplingError):
                C.apply_couplings({'a': bad}, [])

    def test_composite_overflow_and_underflow_are_not_silent_locks(self):
        for multiplier in (1e300, 1e-300):
            joints = {n: joint('continuous', None, None, None) for n in ('a', 'b', 'c')}
            with self.subTest(multiplier=multiplier), self.assertRaisesRegex(C.CouplingError, 'NUMERIC'):
                C.apply_couplings(joints, [relation('a', 'b', multiplier), relation('b', 'c', multiplier)])

    def test_limit_rounding_is_conservative_and_cannot_silently_underflow(self):
        result = C.apply_couplings({'a': joint(), 'b': joint(lo=1., hi=2., velocity=1.)}, [relation(a=3.)])
        limits = result['effective_limits']['a']
        self.assertGreaterEqual(Fraction(limits['lower_limit'])*3, 1)
        self.assertLessEqual(Fraction(limits['upper_limit'])*3, 2)
        self.assertLessEqual(Fraction(limits['velocity'])*3, 1)
        for field in ('upper_limit', 'velocity'):
            joints = {'a': joint(), 'b': joint() | {field: 5e-324}}
            with self.subTest(field=field), self.assertRaisesRegex(C.CouplingError, 'NUMERIC_RANGE'):
                C.apply_couplings(joints, [relation(a=1.5)])
        with self.assertRaisesRegex(C.CouplingError, 'NUMERIC_RANGE'):
            C.apply_couplings({'a': joint(), 'b': joint(lo=1., hi=1.)}, [relation(a=3.)])

    def test_sdk_objects_never_copied_or_serialized_into_ir(self):
        class Native:
            def __deepcopy__(self, memo):
                raise AssertionError('SDK object must never be deep copied')
        with self.assertRaisesRegex(C.CouplingError, 'PLAIN_DATA'):
            C.apply_couplings({'a': joint() | {'_native': Native()}}, [])

    def test_canonical_sdk_wrappers_are_resolved_only_by_identity_owner(self):
        class SDK:
            def __getattr__(self, field):
                raise AssertionError('unexpected native read: ' + field)

            def __setattr__(self, field, value):
                raise AssertionError('native mutation: ' + field)

        wrapper_one, wrapper_two, context_wrapper = SDK(), SDK(), SDK()
        fields = vars(self.link) | dict(jointOne=wrapper_one, jointTwo=wrapper_two,
                                        assemblyContext=context_wrapper)

        class ReadOnlyLink(SDK):
            def __getattr__(self, field):
                if field in fields:
                    return fields[field]
                return super().__getattr__(field)

        canonical = {wrapper_one: self.b1, wrapper_two: self.b2}

        def resolver(j, context, motion_id):
            self.assertIs(context, context_wrapper)
            self.assertEqual(canonical[j].motion_type, motion_id)
            return canonical[j]

        result = C.read_motion_link(ReadOnlyLink(), resolver)
        self.assertEqual((result['driver'], result['follower']), ('a', 'b'))
        json.dumps(result)

    def test_unknown_nonlinear_schema_and_removed_dof_have_named_errors(self):
        joints = {'a': joint(), 'b': joint()}
        for extra in ({'expression': 'sin(q)'}, {'nonlinear': True}, {'provenance': []}, {'status': 'unknown'}):
            with self.subTest(extra=extra), self.assertRaisesRegex(C.CouplingError, 'RELATION'):
                C.apply_couplings(joints, [relation() | extra])
        for rs in ([relation('closure', 'b')], [relation('a', 'closure')]):
            with self.subTest(relations=rs), self.assertRaisesRegex(C.CouplingError, 'REMOVED_CLOSURE_DOF'):
                C.apply_couplings(joints, rs, removed_joint_names=['closure'])

    def test_effective_constraints_independent_bruteforce_oracle(self):
        joints = {'a': joint(lo=-5., hi=5., velocity=9.), 'b': joint(lo=-2., hi=6., velocity=4.),
                  'c': joint(lo=-5., hi=7., velocity=3.), 'd': joint(lo=-1., hi=4., velocity=6.)}
        result = C.apply_couplings(joints, [relation('a', 'b', -2., 1.), relation('b', 'c', .5, -1.), relation('a', 'd', 3., 1.)])
        effective = result['effective_limits']['a']
        for i in range(-500, 501):
            q = i/100.
            values = dict(a=q, b=-2*q+1, c=-q-.5, d=3*q+1)
            source_ok = all(joints[n]['lower_limit'] <= v <= joints[n]['upper_limit'] for n, v in values.items())
            planned_ok = effective['lower_limit'] <= q <= effective['upper_limit']
            self.assertEqual(source_ok, planned_ok)
        self.assertEqual(effective['velocity'], 2.)
        for velocity in (0., .5, 1.5, 2., 2.5, 9.):
            source_ok = all(abs(v) <= joints[n]['velocity'] for n, v in dict(a=velocity, b=-2*velocity, c=-velocity, d=3*velocity).items())
            self.assertEqual(source_ok, velocity <= effective['velocity'])


if __name__ == '__main__':
    unittest.main()
