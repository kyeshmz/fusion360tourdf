"""Public-export frame-link evidence from analytic doubles, not native CAD."""

import copy
import importlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from tests.test_combined_frames import (
    add, axis_rotate, origin_apply, rpy_apply, source_snapshot, subtract, times, vector,
)
from tests.test_exporter_logic import FakeMatrix, FakePoint, identity_at
from tests.test_fusion_inputs import Body, design, fixed, group, hinge, occurrence, root
from URDF_Exporter.core import Write
from URDF_Exporter.utils import utils, validate


ROOT_AXIS = times(1 / math.sqrt(14), (1, 2, 3))
SLIDE_AXIS = times(1 / math.sqrt(2), (1, 0, 1))
PIVOTS = ((12, -8, 20), (35, 17, -6), (70, -12, 31))
SAMPLES = ({'root_drive': 0, 'frame_drive': 0, 'tool_slide': 0},
           {'root_drive': .31, 'frame_drive': -.27, 'tool_slide': .043})


def pose(xyz, rpy):
    axes = [rpy_apply(axis, rpy) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return [value for i in range(3) for value in
            (axes[0][i], axes[1][i], axes[2][i], xyz[i])] + [0, 0, 0, 1]


def analytic_scene():
    arm = occurrence('arm:1', [Body('arm solid', 40, mass=2)],
                     matrix=pose((24, 13, -9), (.27, -.41, .19)))
    frame = occurrence('arm:1+reference:1', matrix=pose((35, 17, -6), (-.33, .22, .47)))
    tool = occurrence('arm:1+reference:1+tool:1', [Body('tool solid', 80, mass=3)],
                      matrix=pose((70, -12, 31), (.18, .37, -.29)))
    unused = occurrence('unused:1')
    root_drive = hinge('root_drive', arm, None)
    root_drive.geometryTwoTransform = FakeMatrix(identity_at(PIVOTS[0]))
    root_drive.jointMotion.rotationAxisVector = FakePoint(ROOT_AXIS)
    frame_drive = hinge('frame_drive', frame, arm)
    frame_drive.geometryTwoTransform = FakeMatrix(identity_at(PIVOTS[1]))
    frame_drive.jointMotion.rotationAxisVector = FakePoint((0, 1, 0))
    slide = fixed('tool_slide', tool, frame, PIVOTS[2])
    slide.jointMotion = types.SimpleNamespace(jointType=2, slideValue=0,
        slideDirectionVector=FakePoint(SLIDE_AXIS),
        slideLimits=types.SimpleNamespace(isMinimumValueEnabled=True,
            isMaximumValueEnabled=True, minimumValue=-10, maximumValue=20))
    fixture, exported = design(root([unused, tool, frame, arm], joints=[slide, frame_drive, root_drive]))
    fixture.unitsManager = types.SimpleNamespace(defaultLengthUnits='cm')
    fixture.timeline = types.SimpleNamespace(markerPosition=4, entityTokens=('sketch', 'solid', 'reference', 'joint'))
    return fixture, exported


def expected_point(link, point, sample):
    if link.endswith('tool_1'):
        point = add(point, times(100 * sample['tool_slide'], SLIDE_AXIS))
        point = add(PIVOTS[1], axis_rotate(subtract(point, PIVOTS[1]), (0, 1, 0), sample['frame_drive']))
    return times(.01, add(PIVOTS[0], axis_rotate(subtract(point, PIVOTS[0]), ROOT_AXIS, sample['root_drive'])))


class BodylessFrameTest(unittest.TestCase):
    def test_unicode_frames_share_alias_policy_receipts_and_one_joint_snapshot(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        first, second = occurrence('腕:1'), occurrence('脚:1')
        tool = occurrence('tool:1', [Body('tool', 20)])
        component = root([second, tool, first, base], joints=[hinge('回転', first, base),
            hinge('移動', second, first), fixed('mount', tool, second)])
        fixture, exported = design(component)
        destination = self.output / 'unicode_frames_description'
        with patch.object(utils, 'root_joints', wraps=utils.root_joints) as discovery:
            self.entry.export_design(fixture, destination)
        self.assertEqual(discovery.call_count, 1)
        robot = ET.parse(destination / 'urdf/unicode_frames.urdf').getroot()
        receipt = json.loads((destination / 'config/robot_data.yaml').read_text())
        for name, path in (('link_1__627b68572913', '腕:1'), ('link_1__759563cb037b', '脚:1')):
            self.assert_frame(robot, name)
            self.assertEqual(receipt['source_identifiers']['links'][name], path)
            self.assertEqual(receipt['link_sources'][name], {'kind': 'frame', 'path': path})
        self.assertEqual(receipt['source_identifiers']['joints']['link__be22550c9368'], '回転')
        self.assertEqual(receipt['source_identifiers']['joints']['link__de2596ff8552'], '移動')
        self.assertEqual(sorted(exported), ['base', 'tool'])
        self.assertEqual(sorted(path.name for path in (destination / 'meshes').iterdir()), ['base_link.stl', 'tool_1.stl'])

    def setUp(self):
        artifacts = os.environ.get('BODYLESS_ARTIFACT_DIR')
        if artifacts:
            self.output = Path(tempfile.mkdtemp(prefix=self._testMethodName + '-', dir=artifacts))
        else:
            temporary = tempfile.TemporaryDirectory(prefix='bodyless-')
            self.addCleanup(temporary.cleanup)
            self.output = Path(temporary.name)
        self.entry = importlib.import_module('URDF_Exporter.URDF_Exporter')

    def export(self, component, config=None):
        fixture, exported = design(component)
        destination = self.output / 'frames_description'
        self.entry.export_design(fixture, destination, config=config)
        return ET.parse(destination / 'urdf/frames.urdf').getroot(), destination, exported

    def assert_frame(self, robot, name):
        link = robot.find("link[@name='%s']" % name)
        self.assertIsNotNone(link)
        self.assertEqual(list(link), [])

    def assert_vector(self, actual, expected, tolerance=2e-9):
        for got, want in zip(actual, expected):
            self.assertAlmostEqual(got, want, delta=tolerance)

    def world(self, robot, link, point, sample):
        joint = next((j for j in robot.findall('joint') if j.find('child').get('link') == link), None)
        if joint is None:
            return point
        if joint.get('type') != 'fixed':
            axis, q = vector(joint.find('axis'), 'xyz'), sample.get(joint.get('name'), 0)
            point = add(point, times(q, axis)) if joint.get('type') == 'prismatic' else axis_rotate(point, axis, q)
        point = origin_apply(point, joint.find('origin'))
        return self.world(robot, joint.find('parent').get('link'), point, sample)

    def test_design_root_internal_frame_3d_fk_and_once_only_physics_meshes(self):
        fixture, exported = analytic_scene()
        before = source_snapshot(fixture)
        destination = self.output / 'frames_description'
        self.entry.export_design(fixture, destination)
        self.assertEqual(source_snapshot(fixture), before)
        self.assertEqual(sorted(exported), ['arm solid', 'tool solid'])
        robot = ET.parse(destination / 'urdf/frames.urdf').getroot()
        self.assertEqual({link.get('name') for link in robot.findall('link')},
                         {'base_link', 'arm_1', 'arm_1_reference_1', 'arm_1_reference_1_tool_1'})
        for frame in ('base_link', 'arm_1_reference_1'):
            self.assert_frame(robot, frame)
            self.assertFalse((destination / 'meshes' / (frame + '.stl')).exists())
        self.assertEqual(len(robot.findall('ros2_control/joint')), 3)
        self.assertEqual(sum(float(m.get('value')) for m in robot.findall('link/inertial/mass')), 5)
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        self.assertEqual(sidecar['link_sources']['base_link'], {'kind': 'frame', 'path': 'design_root'})
        self.assertEqual(sidecar['link_sources']['arm_1_reference_1'],
                         {'kind': 'frame', 'path': 'arm:1+reference:1'})
        for name in sidecar['links']:
            component = ET.parse(destination / sidecar['component_urdfs'][name]).getroot()
            self.assertEqual(ET.tostring(component.find('link')).strip(),
                             ET.tostring(robot.find("link[@name='%s']" % name)).strip())
        self.assertIn('install(DIRECTORY urdf meshes launch config viewers',
                      (destination / 'CMakeLists.txt').read_text())
        checked_points = 0
        for name, x in (('arm_1', 40), ('arm_1_reference_1_tool_1', 80)):
            data = (destination / 'meshes' / (name + '.stl')).read_bytes()
            self.assertEqual(struct.unpack_from('<I', data, 80), (12,))
            link = robot.find("link[@name='%s']" % name)
            for sample in SAMPLES:
                for kind in ('visual', 'collision'):
                    element = link.find(kind)
                    self.assertEqual(element.find('geometry/mesh').get('filename'),
                                     'package://frames_description/meshes/' + name + '.stl')
                    for index in range(12):
                        values = struct.unpack_from('<12fH', data, 84 + index*50)
                        for offset in (3, 6, 9):
                            point_cm = times(.1, values[offset:offset+3])
                            actual = self.world(robot, name, origin_apply(times(.01, point_cm), element.find('origin')), sample)
                            self.assert_vector(actual, expected_point(name, point_cm, sample))
                            checked_points += 1
                physics = link.find('inertial')
                self.assert_vector(self.world(robot, name, vector(physics.find('origin'), 'xyz'), sample),
                                   expected_point(name, (x, 0, 0), sample))
        self.assertEqual(checked_points, 288)
        frame = 'arm_1_reference_1'
        for sample in SAMPLES:
            expected_origin = expected_point('arm_1', PIVOTS[1], sample)
            actual_origin = self.world(robot, frame, (0, 0, 0), sample)
            self.assert_vector(actual_origin, expected_origin)
            for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                expected_axis = axis_rotate(axis_rotate(rpy_apply(axis, (-.33, .22, .47)),
                                           (0, 1, 0), sample['frame_drive']), ROOT_AXIS, sample['root_drive'])
                self.assert_vector(subtract(self.world(robot, frame, axis, sample), actual_origin), expected_axis)
        (self.output / 'analytic-evidence.json').write_text(json.dumps({
            'evidence_kind': 'analytic_api_double', 'samples': SAMPLES, 'vertex_comparisons': checked_points,
            'frame_pose_vectors': 8, 'center_of_mass_vectors': 4,
            'physical_links': 2, 'frame_links': 2, 'body_exports': len(exported),
            'triangles': 24, 'mass_kg': 5, 'source_preserved': True,
            'link_sources': sidecar['link_sources']}, indent=2) + '\n')

    def test_none_endpoint_is_design_root_even_when_physical_base_is_explicit(self):
        base = occurrence('anchor:1', [Body('solid', 20)], component='base_link')
        robot, destination, exported = self.export(root([base], joints=[fixed('world_mount', None, base, (4, 5, 6))]))
        self.assert_frame(robot, 'design_root')
        self.assertEqual(robot.find('joint/child').get('link'), 'design_root')
        self.assertEqual(robot.find('joint/origin').get('xyz'), '0.04 0.05 0.06')
        self.assertEqual(exported, ['solid'])
        self.assertEqual(sorted(p.name for p in (destination / 'meshes').iterdir()), ['base_link.stl'])

    def test_as_built_frame_chain_preserves_root_context_and_motion(self):
        fixture, exported = analytic_scene()
        component = fixture.rootComponent
        for joint in component.allJoints:
            joint.transform = joint.geometryTwoTransform
        component.allAsBuiltJoints, component.allJoints = component.allJoints, []
        before = source_snapshot(fixture)
        destination = self.output / 'frames_description'
        self.entry.export_design(fixture, destination)
        self.assertEqual(source_snapshot(fixture), before)
        self.assertEqual(len(exported), 2)
        robot = ET.parse(destination / 'urdf/frames.urdf').getroot()
        self.assert_frame(robot, 'base_link')
        self.assert_frame(robot, 'arm_1_reference_1')
        point = (.8, 0, 0)
        tool = robot.find("link[@name='arm_1_reference_1_tool_1']/visual/origin")
        for sample in SAMPLES:
            self.assert_vector(self.world(robot, 'arm_1_reference_1_tool_1', origin_apply(point, tool), sample),
                               expected_point('arm_1_reference_1_tool_1', (80, 0, 0), sample))

    def test_relocated_resources_and_component_urdfs_need_no_frame_meshes(self):
        fixture, _ = analytic_scene()
        source = self.output / 'source/frames_description'
        self.entry.export_design(fixture, source)
        installed = self.output / 'install/share/frames_description'
        shutil.copytree(source, installed)
        sidecar = json.loads((installed / 'config/robot_data.yaml').read_text())
        for name, relative in sidecar['component_urdfs'].items():
            component = ET.parse(installed / relative).getroot()
            link = component.find('link')
            self.assertEqual(link.get('name'), name)
            meshes = link.findall('.//mesh')
            if sidecar['link_sources'][name]['kind'] == 'frame':
                self.assertEqual(list(link), [])
                self.assertEqual(meshes, [])
            else:
                for mesh in meshes:
                    uri = mesh.get('filename')
                    self.assertTrue(uri.startswith('package://frames_description/'))
                    resource = installed / uri.removeprefix('package://frames_description/')
                    self.assertTrue(resource.is_file())
                    self.assertEqual(resource.read_bytes(), (source / resource.relative_to(installed)).read_bytes())
        self.assertEqual(sorted(p.name for p in (installed / 'meshes').iterdir()),
                         ['arm_1.stl', 'arm_1_reference_1_tool_1.stl'])

    def test_explicit_empty_occurrence_and_empty_design_root_selection(self):
        for selected in ('reference:1', 'design_root'):
            with self.subTest(selected=selected):
                frame = occurrence('reference:1', matrix=pose((3, 4, 5), (.2, .3, .4)))
                tool = occurrence('tool:1', [Body('solid', 20)])
                parent = frame if selected == 'reference:1' else None
                robot, _, exported = self.export(root([frame, tool, occurrence('unused:1')],
                    joints=[fixed('mount', tool, parent, (20, 10, 5))]), {'root_occurrence': selected})
                self.assert_frame(robot, 'base_link')
                self.assertEqual({link.get('name') for link in robot.findall('link')}, {'base_link', 'tool_1'})
                self.assertEqual(exported, ['solid'])
                self.output = self.output / 'next'

    def test_ground_to_empty_parent_uses_actual_moving_frame_not_containment_weld(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        frame = occurrence('reference:1', matrix=pose((10, 20, 30), (.2, -.3, .4)))
        tool = occurrence('reference:1+tool:1', [Body('tool', 40)])
        tool.isGroundToParent = True
        robot, _, exported = self.export(root([base, frame, tool], joints=[hinge('drive', frame, base)]))
        self.assert_frame(robot, 'reference_1')
        self.assertEqual(len(robot.findall('joint')), 2)
        self.assertEqual(len(robot.findall('ros2_control/joint')), 1)
        attachment = next(j for j in robot.findall('joint') if j.get('type') == 'fixed')
        self.assertEqual(attachment.find('parent').get('link'), 'reference_1')
        self.assertEqual(attachment.find('child').get('link'), 'reference_1_tool_1')
        self.assertEqual(sorted(exported), ['base', 'tool'])
        tool.isGroundToParent = False
        fixture, exported = design(root([base, frame, tool], joints=[hinge('drive', frame, base)]))
        with self.assertRaisesRegex(ValueError, 'DISCONNECTED_LINKS.*reference_1_tool_1'):
            self.entry.export_design(fixture, self.output / 'unwelded_description')
        self.assertEqual(exported, [])

    def test_selected_grounded_empty_ancestry_and_empty_rigid_member(self):
        frame = occurrence('reference:1')
        frame.isGrounded = True
        tool = occurrence('reference:1+tool:1', [Body('tool', 20)])
        tool.isGroundToParent = True
        robot, _, exported = self.export(root([frame, tool]), {'root_occurrence': 'reference:1'})
        self.assert_frame(robot, 'base_link')
        self.assertEqual(robot.find('joint').get('type'), 'fixed')
        self.assertEqual(exported, ['tool'])
        self.output = self.output / 'rigid'
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        empty = occurrence('empty:1', matrix=pose((8, 9, 10), (.1, .2, .3)))
        robot, _, exported = self.export(root([base, empty], groups=[group('clamp', [base, empty])]))
        self.assert_frame(robot, 'empty_1')
        self.assertEqual(robot.find('joint').get('type'), 'fixed')
        self.assertEqual(exported, ['base'])

    def test_unused_suppressed_frames_and_standalone_configured_empty_root(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        unused = occurrence('unused:1')
        suppressed = fixed('disabled_mount', unused, base)
        suppressed.isSuppressed = True
        robot, _, _ = self.export(root([base, unused], joints=[suppressed]))
        self.assertEqual([link.get('name') for link in robot.findall('link')], ['base_link'])
        self.output = self.output / 'empty'
        robot, destination, exported = self.export(root(), {'root_occurrence': 'design_root'})
        self.assert_frame(robot, 'base_link')
        self.assertEqual(robot.findall('joint'), [])
        self.assertEqual(exported, [])
        self.assertEqual(list((destination / 'meshes').iterdir()), [])

    def test_rigid_frame_membership_does_not_infer_child_membership_from_containment(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        frame = occurrence('reference:1')
        tool = occurrence('reference:1+tool:1', [Body('tool', 20)])
        robot, _, _ = self.export(root([base, frame, tool], joints=[hinge('drive', tool, base)],
            groups=[group('reference_clamp', [base, frame])]))
        self.assert_frame(robot, 'reference_1')
        attachment = next(j for j in robot.findall('joint') if j.get('type') == 'fixed')
        self.assertEqual(attachment.find('parent').get('link'), 'base_link')
        self.assertEqual(attachment.find('child').get('link'), 'reference_1')
        drive = robot.find("joint[@name='drive']")
        self.assertEqual(drive.find('parent').get('link'), 'base_link')
        self.assertEqual(drive.find('child').get('link'), 'reference_1_tool_1')
        self.assert_vector(self.world(robot, 'reference_1_tool_1', (.2, 0, 0), {'drive': math.pi/2}), (0, .2, 0))

    def test_required_invalid_missing_unreadable_and_dangling_endpoints_fail_before_output(self):
        for case in ('invalid_joint', 'invalid_endpoint', 'missing_validity', 'unreadable_validity',
                     'both_none', 'dangling', 'both_dangling', 'hidden_frame'):
            with self.subTest(case=case):
                frame = occurrence('reference:1')
                tool = occurrence('tool:1', [Body('tool', 20)], component='base_link')
                joint = fixed('required_mount', tool, frame)
                owners = [tool, frame]
                if case == 'invalid_joint':
                    joint.isValid = False
                elif case == 'invalid_endpoint':
                    frame.isValid = False
                elif case == 'missing_validity':
                    del frame.isValid
                elif case == 'unreadable_validity':
                    class Unreadable(types.SimpleNamespace):
                        @property
                        def isValid(self):
                            raise RuntimeError('unreadable reference validity')
                    values = dict(vars(frame))
                    values.pop('isValid')
                    frame = Unreadable(**values)
                    owners[1] = joint.occurrenceTwo = frame
                elif case == 'both_none':
                    joint.occurrenceOne = joint.occurrenceTwo = None
                elif case == 'dangling':
                    owners.remove(frame)
                elif case == 'both_dangling':
                    joint.occurrenceOne = occurrence('missing:1')
                    owners.remove(frame)
                elif case == 'hidden_frame':
                    frame.isVisible = False
                fixture, exported = design(root(owners, joints=[joint]))
                fixture.unitsManager = types.SimpleNamespace(defaultLengthUnits='cm')
                fixture.timeline = types.SimpleNamespace(markerPosition=2)
                before = source_snapshot(fixture)
                target = self.output / (case + '_description')
                with self.assertRaises(ValueError):
                    self.entry.export_design(fixture, target)
                self.assertEqual(source_snapshot(fixture), before)
                self.assertEqual(exported, [])
                self.assertFalse(target.exists())

    def test_mesh_failure_preserves_source_and_publishes_nothing(self):
        fixture, exported = analytic_scene()
        execute = fixture.exportManager.execute
        fixture.exportManager.execute = lambda option: False if exported else execute(option)
        before = source_snapshot(fixture)
        destination = self.output / 'failure_description'
        with self.assertRaisesRegex(RuntimeError, 'Fusion mesh export failed'):
            self.entry.export_design(fixture, destination)
        self.assertEqual(len(exported), 1)
        self.assertEqual(source_snapshot(fixture), before)
        self.assertEqual(list(self.output.iterdir()), [])

    def test_invalid_configured_and_grounding_frame_validity_fail_closed(self):
        for route in ('configured', 'grounding'):
            for case in ('invalid', 'missing', 'unreadable'):
                with self.subTest(route=route, case=case):
                    frame = occurrence('reference:1')
                    frame.isGrounded = True
                    if case == 'invalid':
                        frame.isValid = False
                    elif case == 'missing':
                        del frame.isValid
                    else:
                        class Unreadable(types.SimpleNamespace):
                            @property
                            def isValid(self):
                                raise RuntimeError('unreadable required frame')
                        values = dict(vars(frame))
                        values.pop('isValid')
                        frame = Unreadable(**values)
                    owners = [frame]
                    config = {'root_occurrence': 'reference:1'}
                    if route == 'grounding':
                        tool = occurrence('reference:1+tool:1', [Body('tool', 20)])
                        tool.isGroundToParent = True
                        owners.append(tool)
                        config = None
                    fixture, exported = design(root(owners))
                    target = self.output / (route + '_' + case + '_description')
                    with self.assertRaises(ValueError):
                        self.entry.export_design(fixture, target, config=config)
                    self.assertFalse(target.exists())
                    self.assertEqual(exported, [])

    def test_frame_contract_cannot_bypass_physical_validation(self):
        valid = {'base_link': {'kind': 'frame', 'source_path': 'design_root'}}
        self.assertEqual(validate.validate_export({}, valid)[0], [])
        for field, value in (('mass', 0), ('inertia', [0]*6), ('bbox_size_m', [1]*3),
                             ('color_rgba', [1]*4), ('center_of_mass', [0]*3)):
            bad = copy.deepcopy(valid)
            bad['base_link'][field] = value
            self.assertTrue(validate.validate_export({}, bad)[0])
        for kind in ('physical', 'unknown', None):
            self.assertTrue(validate.validate_export({}, {'base_link': {'kind': kind}})[0])
        fixture, exported = design(root(bodies=[Body('massless', 0, mass=0)]))
        with self.assertRaisesRegex(ValueError, 'positive mass'):
            self.entry.export_design(fixture, self.output / 'massless_description')
        self.assertEqual(exported, [])
        callbacks = []
        Write.write_ros2_package(self.output / 'frame_description', 'frame', {}, valid,
                                write_meshes=lambda targets: callbacks.append(targets))
        self.assertEqual(callbacks, [{}])
        for bodies, kind in (((), 'physical'), ((object(),), 'frame'), ((), 'unknown')):
            with self.assertRaises(ValueError):
                utils.LinkSource(object(), bodies, kind=kind)


if __name__ == '__main__':
    unittest.main()
