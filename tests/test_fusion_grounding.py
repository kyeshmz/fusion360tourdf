import importlib
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakeMatrix, identity_at
from tests.test_fusion_inputs import Body, design, group, hinge, occurrence, root


def grounded(path, x=0, *, parent=False, world=False, physical=True, component=None):
    owner = occurrence(path, [Body(path, x)] if physical else [],
                       matrix=identity_at([x, 0, 0]), component=component)
    owner.isGroundToParent = parent
    owner.isGrounded = world
    return owner


class GroundingExportTest(unittest.TestCase):
    def export(self, component, config=None):
        temporary = tempfile.TemporaryDirectory(prefix='fusion-grounding-')
        self.addCleanup(temporary.cleanup)
        destination = Path(temporary.name) / 'grounding_description'
        fixture, _ = design(component)
        before = [(getattr(owner, 'isGrounded', None), getattr(owner, 'isGroundToParent', None),
                   owner.transform2.asArray()) for owner in component.allOccurrences]
        importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(
            fixture, destination, config=config)
        self.assertEqual([(getattr(owner, 'isGrounded', None), getattr(owner, 'isGroundToParent', None),
                           owner.transform2.asArray()) for owner in component.allOccurrences], before)
        robot = ET.parse(destination / 'urdf/grounding.xacro').getroot()
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        self.destination = destination
        return robot, sidecar

    def reject(self, component, diagnostic, config=None):
        temporary = tempfile.TemporaryDirectory(prefix='fusion-grounding-reject-')
        self.addCleanup(temporary.cleanup)
        destination = Path(temporary.name) / 'rejected_description'
        fixture, exported = design(component)
        before = [(getattr(owner, 'isGrounded', None), getattr(owner, 'isGroundToParent', None))
                  for owner in component.allOccurrences]
        with self.assertRaisesRegex(ValueError, diagnostic):
            importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(
                fixture, destination, config=config)
        self.assertFalse(destination.exists())
        self.assertEqual(exported, [])
        self.assertEqual([(getattr(owner, 'isGrounded', None), getattr(owner, 'isGroundToParent', None))
                          for owner in component.allOccurrences], before)

    def assert_dofs(self, robot, count):
        self.assertEqual(sum(joint.get('type') != 'fixed' for joint in robot.findall('joint')), count)
        self.assertEqual(len(robot.findall('ros2_control/joint')), count)

    def provenance(self, sidecar):
        return [entry for joint in sidecar['joints'].values()
                for entry in joint.get('rigid_constraints', [])
                if entry.get('source') == 'Fusion grounding']

    def vertex(self, robot, link, positions):
        point = [value / 1000 for value in struct.unpack_from('<3f',
                 (self.destination / 'meshes' / (link + '.stl')).read_bytes(), 96)]

        def transform(origin, point, angle=0):
            x, y, z = map(float, origin.get('xyz').split())
            yaw = float(origin.get('rpy').split()[2]) + angle
            return [x + math.cos(yaw)*point[0] - math.sin(yaw)*point[1],
                    y + math.sin(yaw)*point[0] + math.cos(yaw)*point[1], z + point[2]]

        point = transform(robot.find("link[@name='%s']/visual/origin" % link), point)
        while link != 'base_link':
            joint = next(joint for joint in robot.findall('joint') if joint.find('child').get('link') == link)
            point = transform(joint.find('origin'), point, positions.get(joint.get('name'), 0))
            link = joint.find('parent').get('link')
        return point

    def test_nested_physical_parent_retains_motion_outside_fixed_set_and_root_pose(self):
        base = grounded('base:1', component='base_link')
        parent = grounded('module:1', 10)
        child = grounded('module:1+tool:1', 30, parent=True)
        child.transform2 = FakeMatrix([0, -1, 0, 30, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
        robot, sidecar = self.export(root([child, parent, base], joints=[hinge('drive', parent, base)]))
        self.assert_dofs(robot, 1)
        attachment = next(j for j in robot.findall('joint')
                          if j.find('child').get('link') == 'module_1_tool_1')
        self.assertEqual(attachment.find('parent').get('link'), 'module_1')
        self.assertEqual(attachment.find('origin').get('xyz'), '0.3 0 0')
        self.assertAlmostEqual(float(attachment.find('origin').get('rpy').split()[2]), math.pi / 2)
        visual = robot.find("link[@name='module_1_tool_1']/visual/origin")
        for actual, expected in zip(map(float, visual.get('xyz').split()), [0, .3, 0]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.provenance(sidecar)[0]['relationships'],
                         [{'path': 'module:1+tool:1', 'property': 'isGroundToParent'}])
        for q, expected in ((0, [.29, -.01, -.01]), (math.pi / 2, [.01, .29, -.01])):
            for actual, want in zip(self.vertex(robot, 'module_1_tool_1', {'drive': q}), expected):
                self.assertAlmostEqual(actual, want)

    def test_ground_to_physical_parent_infers_root_and_reverses_from_configured_child(self):
        for configured, expected_child, xyz in ((None, 'frame_1_tool_1', '0.3 0 0'),
                ({'root_occurrence': 'frame:1+tool:1'}, 'frame_1', '0.1 0 0')):
            with self.subTest(configured=configured):
                parent = grounded('frame:1', 10)
                child = grounded('frame:1+tool:1', 30, parent=True)
                robot, sidecar = self.export(root([child, parent]), configured)
                self.assert_dofs(robot, 0)
                self.assertEqual(len(robot.findall('joint')), 1)
                self.assertEqual(robot.find('joint/parent').get('link'), 'base_link')
                self.assertEqual(robot.find('joint/child').get('link'), expected_child)
                self.assertEqual(robot.find('joint/origin').get('xyz'), xyz)
                self.assertEqual(self.provenance(sidecar)[0]['relationships'],
                                 [{'path': 'frame:1+tool:1', 'property': 'isGroundToParent'}])

    def test_design_root_bodies_anchor_top_level_and_absolute_grounding(self):
        for flag in ('parent', 'world'):
            with self.subTest(flag=flag):
                child = grounded('fixture:1', 20, **{flag: True})
                robot, sidecar = self.export(root([child], bodies=[Body('world solid', 0)]))
                self.assert_dofs(robot, 0)
                self.assertEqual(robot.find('joint/parent').get('link'), 'base_link')
                self.assertEqual(robot.find('joint/child').get('link'), 'fixture_1')
                self.assertEqual(robot.find('joint/origin').get('xyz'), '0.2 0 0')
                self.assertEqual(self.provenance(sidecar)[0]['relationships'],
                    [{'path': 'fixture:1', 'property': 'isGroundToParent' if flag == 'parent' else 'isGrounded'}])

    def test_selected_world_anchor_has_no_self_edge_and_retains_grounding_receipt(self):
        for flag in ('parent', 'world'):
            with self.subTest(flag=flag):
                anchor = grounded('anchor:1', 10, **{flag: True})
                robot, sidecar = self.export(root([anchor]))
                self.assert_dofs(robot, 0)
                self.assertEqual([link.get('name') for link in robot.findall('link')], ['base_link'])
                self.assertEqual(robot.findall('joint'), [])
                self.assertEqual(sidecar['source_notes'][0]['source'], 'Fusion grounding')
                self.assertEqual(sidecar['source_notes'][0]['relationships'],
                    [{'path': 'anchor:1', 'property': 'isGroundToParent' if flag == 'parent' else 'isGrounded'}])

    def test_world_fixed_selected_root_anchors_other_absolute_and_parent_grounding(self):
        for anchor_flag in ('parent', 'world'):
            for other_flag in ('parent', 'world'):
                with self.subTest(anchor=anchor_flag, other=other_flag):
                    anchor = grounded('anchor:1', 10, **{anchor_flag: True})
                    other = grounded('fixture:1', 30, **{other_flag: True})
                    robot, sidecar = self.export(root([other, anchor]), {'root_occurrence': 'anchor:1'})
                    self.assert_dofs(robot, 0)
                    self.assertEqual(robot.find('joint/parent').get('link'), 'base_link')
                    self.assertEqual(robot.find('joint/child').get('link'), 'fixture_1')
                    self.assertEqual(robot.find('joint/origin').get('xyz'), '0.3 0 0')
                    self.assertEqual(self.provenance(sidecar)[0]['relationships'][0]['path'], 'fixture:1')

    def test_free_selected_root_is_not_invented_as_world_anchor(self):
        base = grounded('base:1', component='base_link')
        other = grounded('grounded:1', 20, world=True)
        self.reject(root([base, other], joints=[hinge('drive', other, base)]),
                    'GROUNDING_WORLD_ANCHOR_UNAVAILABLE.*grounded:1.*root_occurrence')

    def test_multiple_world_candidates_require_exact_root(self):
        a, b = grounded('a:1', parent=True), grounded('b:1', 10, parent=True)
        self.reject(root([a, b]), 'ROOT_AMBIGUOUS.*root_occurrence')

    def test_explicit_bodyless_fixed_chain_reaches_physical_ancestor(self):
        base = grounded('frame:1', 10)
        container = grounded('frame:1+assembly:1', parent=True, physical=False)
        child = grounded('frame:1+assembly:1+tool:1', 30, parent=True)
        robot, sidecar = self.export(root([child, container, base]))
        self.assert_dofs(robot, 0)
        self.assertEqual(robot.find('joint/child').get('link'), 'frame_1_assembly_1_tool_1')
        self.assertEqual(robot.find('joint/origin').get('xyz'), '0.3 0 0')
        self.assertEqual(self.provenance(sidecar)[0]['relationships'], [
            {'path': 'frame:1+assembly:1+tool:1', 'property': 'isGroundToParent'},
            {'path': 'frame:1+assembly:1', 'property': 'isGroundToParent'}])

    def test_bodyless_fixed_chain_proves_selected_root_world_anchor(self):
        for flag in ('parent', 'world'):
            with self.subTest(flag=flag):
                container = grounded('assembly:1', physical=False, **{flag: True})
                anchor = grounded('assembly:1+anchor:1', 10, parent=True)
                other = grounded('fixture:1', 30, world=True)
                robot, sidecar = self.export(root([container, anchor, other]),
                    {'root_occurrence': 'assembly:1+anchor:1'})
                self.assert_dofs(robot, 0)
                self.assertEqual(robot.find('joint/child').get('link'), 'fixture_1')
                notes = [note for note in sidecar['source_notes'] if note.get('source') == 'Fusion grounding']
                self.assertEqual(notes[0]['relationships'], [
                    {'path': 'assembly:1+anchor:1', 'property': 'isGroundToParent'},
                    {'path': 'assembly:1', 'property': 'isGroundToParent' if flag == 'parent' else 'isGrounded'}])

    def test_nonfixed_bodyless_parent_and_missing_inventory_are_named_gaps(self):
        for include_parent in (False, True):
            with self.subTest(include_parent=include_parent):
                base = grounded('base:1', component='base_link')
                container = grounded('assembly:1', physical=False)
                child = grounded('assembly:1+tool:1', 20, parent=True)
                owners = [base, child] + ([container] if include_parent else [])
                self.reject(root(owners),
                    r'BODYLESS_FRAME_GROUNDING_NOT_IMPLEMENTED.*assembly:1\+tool:1.*assembly:1')

    def test_dual_bodyless_flags_preserve_both_physical_and_world_relationships(self):
        base = grounded('frame:1', 10, world=True, component='base_link')
        container = grounded('frame:1+assembly:1', parent=True, world=True, physical=False)
        child = grounded('frame:1+assembly:1+tool:1', 30, parent=True)
        robot, sidecar = self.export(root([base, container, child]))
        self.assert_dofs(robot, 0)
        chains = [entry['relationships'] for entry in self.provenance(sidecar)]
        self.assertIn([{'path': 'frame:1+assembly:1+tool:1', 'property': 'isGroundToParent'},
                       {'path': 'frame:1+assembly:1', 'property': 'isGrounded'}], chains)
        self.assertIn([{'path': 'frame:1+assembly:1+tool:1', 'property': 'isGroundToParent'},
                       {'path': 'frame:1+assembly:1', 'property': 'isGroundToParent'}], chains)
        base.isGrounded = False
        robot, sidecar = self.export(root([base, container, child]),
            {'root_occurrence': 'frame:1+assembly:1+tool:1'})
        self.assert_dofs(robot, 0)
        self.assertEqual(robot.find('joint/child').get('link'), 'frame_1')
        self.assertEqual(robot.find('joint/origin').get('xyz'), '0.1 0 0')

    def test_reversed_internal_motion_reports_conflict_after_physical_parent_root_inference(self):
        for nested in (False, True):
            with self.subTest(nested=nested):
                parent = grounded('frame:1', 10)
                child = grounded('frame:1+tool:1', 30, parent=True)
                endpoint = grounded('frame:1+tool:1+tip:1', 50, parent=True) if nested else child
                owners = [child, parent, endpoint] if nested else [child, parent]
                self.reject(root(owners, joints=[hinge('reverse_drive', parent, endpoint)]),
                            r'GROUNDING_INTERNAL_MOTION.*reverse_drive.*frame:1\+tool:1')

    def test_grounding_flags_with_no_setters_export_without_source_mutation(self):
        class ReadOnlyFlags:
            def __init__(self, owner):
                self.__dict__.update(vars(owner))

            @property
            def isGrounded(self):
                return self.__dict__['isGrounded']

            @property
            def isGroundToParent(self):
                return self.__dict__['isGroundToParent']

        base = ReadOnlyFlags(grounded('base:1', world=True, component='base_link'))
        child = ReadOnlyFlags(grounded('base:1+tool:1', 20, parent=True))
        robot, sidecar = self.export(root([base, child]))
        self.assert_dofs(robot, 0)
        self.assertEqual(robot.find('joint/origin').get('xyz'), '0.2 0 0')
        self.assertEqual(self.provenance(sidecar)[0]['relationships'],
                         [{'path': 'base:1+tool:1', 'property': 'isGroundToParent'}])

    def test_physical_parent_chain_can_prove_selected_root_world_anchor(self):
        parent = grounded('frame:1', 10, parent=True)
        child = grounded('frame:1+tool:1', 30, parent=True)
        other = grounded('other:1', 50, world=True)
        robot, sidecar = self.export(root([parent, child, other]),
            {'root_occurrence': 'frame:1+tool:1'})
        self.assert_dofs(robot, 0)
        self.assertEqual(sorted(j.find('child').get('link') for j in robot.findall('joint')), ['frame_1', 'other_1'])
        self.assertEqual(sorted(j.find('origin').get('xyz') for j in robot.findall('joint')), ['0.1 0 0', '0.5 0 0'])
        self.assertEqual(len(self.provenance(sidecar)), 6)

    def test_bodyless_grounding_never_welds_free_descendant(self):
        base = grounded('base:1', world=True, component='base_link')
        container = grounded('assembly:1', world=True, physical=False)
        child = grounded('assembly:1+tool:1', 20)
        robot, _ = self.export(root([base, container, child], joints=[hinge('free_drive', child, base)]))
        self.assert_dofs(robot, 1)
        self.assertEqual(robot.find('joint').get('name'), 'free_drive')

    def test_grounding_active_joint_conflict_is_named_but_locked_motion_is_valid(self):
        base = grounded('frame:1', component='base_link')
        child = grounded('frame:1+tool:1', 20, parent=True)
        drive = hinge('forbidden_drive', child, base)
        component = root([base, child], joints=[drive])
        self.reject(component, r'GROUNDING_INTERNAL_MOTION.*forbidden_drive.*frame:1\+tool:1')
        drive.isLocked = True
        robot, sidecar = self.export(component)
        self.assert_dofs(robot, 0)
        self.assertEqual(len(robot.findall('joint')), 1)
        constraints = [entry for joint in sidecar['joints'].values() for entry in joint['rigid_constraints']]
        self.assertEqual(next(entry for entry in constraints if entry['name'] == 'forbidden_drive')
                         ['original_motion']['type'], 'continuous')

    def test_absolute_grounding_conflicts_with_active_motion(self):
        base = grounded('base:1', world=True, component='base_link')
        child = grounded('tool:1', 20, world=True)
        self.reject(root([base, child], joints=[hinge('absolute_drive', child, base)]),
                    'GROUNDING_INTERNAL_MOTION.*absolute_drive.*tool:1')

    def test_repeated_context_uses_root_owner_flags_parent_and_pose(self):
        native = grounded('tool:1', 99, parent=True, world=True)
        left = grounded('left:1', 10, parent=True)
        right = grounded('right:1', 30, parent=True)
        left_child = grounded('left:1+tool:1', 20, parent=True)
        right_child = grounded('right:1+tool:1', 40)
        left_child.nativeObject = right_child.nativeObject = native
        robot, sidecar = self.export(root([right_child, left, left_child, right],
            bodies=[Body('world solid', 0)], joints=[hinge('right_drive', right_child, right)]))
        self.assert_dofs(robot, 1)
        self.assertEqual(robot.find('ros2_control/joint').get('name'), 'right_drive')
        self.assertEqual(sidecar['joints']['rigid_attachment_left_1_tool_1']['xyz'], [.2, 0, 0])
        paths = [step['path'] for entry in self.provenance(sidecar) for step in entry['relationships']]
        self.assertIn('left:1+tool:1', paths)
        self.assertNotIn('right:1+tool:1', paths)
        self.assertEqual((native.isGrounded, native.isGroundToParent), (True, True))

    def test_excluded_grounding_is_not_rebound_and_crossing_selection_fails(self):
        base = grounded('base:1', component='base_link')
        parent = grounded('hidden:1', 10)
        child = grounded('hidden:1+tool:1', 20, parent=True)
        parent.isVisible = False
        component = root([base, parent, child])
        robot, sidecar = self.export(component)
        self.assert_dofs(robot, 0)
        self.assertEqual(len(robot.findall('link')), 1)
        self.assertEqual(sidecar['source_notes'][0]['relationships'],
            [{'path': 'hidden:1+tool:1', 'property': 'isGroundToParent'}])
        parent.isVisible = True
        child.isVisible = False
        self.reject(component, r'CONSTRAINT_TARGET_EXCLUDED.*grounding.*hidden:1\+tool:1')

    def test_body_visibility_cannot_drop_required_grounded_parent(self):
        base = grounded('base:1', component='base_link')
        parent = grounded('frame:1', 10)
        parent.bRepBodies[0].isVisible = False
        child = grounded('frame:1+tool:1', 20, parent=True)
        self.reject(root([base, parent, child]), r'CONSTRAINT_TARGET_EXCLUDED.*grounding.*frame:1')

    def test_grounding_joins_rigid_groups_without_reducing_unrelated_loop(self):
        base = grounded('base:1', component='base_link')
        accessory = grounded('base:1+accessory:1', 10, parent=True)
        a, b, c = grounded('a:1', 20), grounded('b:1', 30), grounded('c:1', 40)
        robot, sidecar = self.export(root([base, accessory, a, b, c],
            joints=[hinge('first', a, base), hinge('second', b, a), hinge('closing', b, base)],
            groups=[group('accessory_clamp', [accessory, c])]))
        self.assert_dofs(robot, 2)
        self.assertEqual(len(robot.findall('joint')), 4)
        self.assertEqual(len(sidecar['closing_joints']), 1)
        self.assertIn('base:1+accessory:1', [step['path'] for entry in self.provenance(sidecar)
                      for step in entry['relationships']])


if __name__ == '__main__':
    unittest.main()
