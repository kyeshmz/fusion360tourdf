import importlib
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import identity_at
from tests.test_fusion_contexts import context_case
from tests.test_fusion_inputs import Body, design, first_vertex_fk, fixed, group, hinge, occurrence, root


FAMILIES = {
    'accented': (('café:1', 'cafè:1'), ('réglage', 'règlage'),
                 ('caf_1__724db6062814', 'caf_1__5f2dc61a6848'),
                 ('r_glage__98266ee8fdeb', 'r_glage__bb27c16e586b')),
    'unicode': (('腕:1', '脚:1'), ('回転', '移動'),
                ('link_1__627b68572913', 'link_1__759563cb037b'),
                ('link__be22550c9368', 'link__de2596ff8552')),
    'punctuation': (('arm-pin:1', 'arm pin:1'), ('drive-pin', 'drive pin'),
                    ('arm_pin_1__8c4963c33bb2', 'arm_pin_1__7574e42ac5bd'),
                    ('drive_pin__6297b6b05121', 'drive_pin__884432114f66')),
    'nested': (('左:1+arm:1', '右:1+arm:1'), ('左:1+drive', '右:1+drive'),
               ('link_1_arm_1__342b04f24152', 'link_1_arm_1__5334f31d9ae3'),
               ('link_1_drive__2759b42d5191', 'link_1_drive__a0ee73455b89')),
}


def scene(family, mode='ordinary', reverse=False):
    paths, joint_paths, links, joints = FAMILIES[family]
    base = occurrence('base_link v7:1', [Body('base', 0)], component='base_link v7')
    children = [occurrence(path, [Body('solid_' + str(i), x, mass=i + 2)],
                           matrix=identity_at([x, 0, 0]))
                for i, (path, x) in enumerate(zip(paths, (10, 20)))]
    inventory = [base] + children
    constraints = []
    native = occurrence('arm:1')
    for path, child in zip(joint_paths, children):
        constraint = hinge(path.split('+')[-1], child, base)
        if family == 'nested':
            context = occurrence(path.rpartition('+')[0])
            inventory.append(context)
            child.nativeObject = native
            child.assemblyContext = context
            constraint.assemblyContext = context
            constraint.occurrenceOne = native
        constraints.append(constraint)
    if reverse:
        inventory.reverse()
        constraints.reverse()
    component = root(inventory, joints=constraints if mode == 'ordinary' else ())
    component.allAsBuiltJoints = constraints if mode == 'as_built' else []
    return component, {'root_occurrence': base.fullPathName}, links, joints


def snapshot(component):
    return (component.name, tuple((id(o), o.name, o.fullPathName, o.component.name,
            o.isVisible, o.isGrounded, tuple(o.transform2.asArray()),
            tuple((id(b), b.name, b.mesh, b.properties.mass) for b in o.bRepBodies))
            for o in component.allOccurrences),
            tuple((id(j), j.name, id(j.occurrenceOne), id(j.occurrenceTwo),
                   getattr(j, 'isSuppressed', False))
                  for j in component.allJoints + component.allAsBuiltJoints))


class IdentifierPolicyTest(unittest.TestCase):
    def export(self, component, config, *, corrupt=False):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        target = Path(temporary.name) / 'aliases_description'
        fixture, exported = design(component, corrupt=corrupt)
        before = snapshot(component)
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        entry.export_design(fixture, target, config=config)
        self.assertEqual(snapshot(component), before)
        return ET.parse(target / 'urdf/aliases.urdf').getroot(), target, exported

    def test_public_collision_matrix_preserves_names_mesh_physics_and_fk(self):
        for family in FAMILIES:
            for mode in ('ordinary', 'as_built'):
                mappings = []
                for reverse in (False, True):
                    with self.subTest(family=family, mode=mode, reverse=reverse):
                        component, config, links, joints = scene(family, mode, reverse)
                        robot, target, exported = self.export(component, config)
                        sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
                        mapping = sidecar['source_identifiers']
                        self.assertEqual(mapping['links'], dict(zip(('base_link',) + links,
                            ('base_link v7:1',) + FAMILIES[family][0])))
                        self.assertEqual(mapping['joints'], dict(zip(joints, FAMILIES[family][1])))
                        mappings.append(mapping)
                        self.assertEqual(set(exported), {'base', 'solid_0', 'solid_1'})
                        names = [element.get('name') for element in robot.findall('link')]
                        self.assertEqual(set(names), {'base_link', *links})
                        self.assertEqual(len(names), len(set(names)))
                        self.assertEqual({j.get('name') for j in robot.findall('joint')}, set(joints))
                        for name in names + list(joints):
                            self.assertRegex(name, r'^[A-Za-z_][A-Za-z0-9_]*$')
                        for i, (link_name, joint_name, x) in enumerate(zip(links, joints, (10, 20))):
                            link = robot.find("link[@name='%s']" % link_name)
                            joint = robot.find("joint[@name='%s']" % joint_name)
                            self.assertEqual(joint.find('parent').get('link'), 'base_link')
                            self.assertEqual(joint.find('child').get('link'), link_name)
                            self.assertEqual(float(link.find('inertial/mass').get('value')), i + 2)
                            self.assertEqual(list(map(float, link.find('inertial/origin').get('xyz').split())),
                                             [x / 100, 0, 0])
                            tensor = link.find('inertial/inertia')
                            for axis in ('ixx', 'iyy', 'izz'):
                                self.assertAlmostEqual(float(tensor.get(axis)), (i + 2) * 2 / 3 / 10000)
                            for axis in ('ixy', 'ixz', 'iyz'):
                                self.assertEqual(float(tensor.get(axis)), 0)
                            self.assertEqual((target / ('meshes/' + link_name + '.stl')).read_bytes()[84:],
                                             Body('oracle', x).mesh[84:])
                            self.assertEqual(link.find('visual/geometry/mesh').get('filename'),
                                             'package://aliases_description/meshes/' + link_name + '.stl')
                            self.assertTrue((target / ('urdf/components/' + link_name + '.urdf')).is_file())
                            for q, expected in ((0, [(x - 1) / 100, -.01, -.01]),
                                                (math.pi / 2, [.01, (x - 1) / 100, -.01])):
                                for actual, want in zip(first_vertex_fk(robot, target, joint_name, q), expected):
                                    self.assertAlmostEqual(actual, want, places=11)
                if len(mappings) == 2:
                    self.assertEqual(mappings[0], mappings[1])

    def test_selected_root_and_generated_attachment_namespaces_are_reserved(self):
        observed = []
        for reverse in (False, True):
            base = occurrence('anchor v7:1', [Body('base', 0)])
            literal = occurrence('base_link', [Body('literal', 10)])
            tool = occurrence('tool:1', [Body('tool', 20)])
            constraints = [hinge('rigid_attachment_tool_1', literal, base)]
            inventory = [base, literal, tool]
            if reverse:
                inventory.reverse()
            component = root(inventory, joints=constraints, groups=[group('clamp', [base, tool])])
            robot, target, _ = self.export(component, {'root_occurrence': 'anchor v7:1'})
            self.assertEqual({e.get('name') for e in robot.findall('link')},
                             {'base_link', 'base_link__715a4d03a58e', 'tool_1'})
            self.assertEqual({e.get('name') for e in robot.findall('joint')},
                             {'rigid_attachment_tool_1', 'source_rigid_attachment_tool_1__89585d59363f'})
            self.assertEqual(robot.find("joint[@name='rigid_attachment_tool_1']").get('type'), 'fixed')
            observed.append(json.loads((target / 'config/robot_data.yaml').read_text())['source_identifiers'])
        self.assertEqual(*observed)

    def test_unambiguous_ordinary_aliases_remain_unchanged(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        arm = occurrence('arm:1', [Body('arm', 10)])
        robot, _, _ = self.export(root([base, arm], joints=[hinge('drive pin', arm, base)]), {})
        self.assertEqual([e.get('name') for e in robot.findall('link')], ['base_link', 'arm_1'])
        self.assertEqual(robot.find('joint').get('name'), 'drive_pin')

    def test_single_link_receipt_keeps_exact_root_identity(self):
        owner = occurrence('stand v7:1', [Body('stand', 0)])
        _, target, _ = self.export(root([owner]), {'root_occurrence': 'stand v7:1'})
        receipt = json.loads((target / 'config/robot_data.yaml').read_text())['source_identifiers']
        self.assertEqual(receipt, {'links': {'base_link': 'stand v7:1'}, 'joints': {}})

    def test_joint_limits_bind_only_exact_exported_alias(self):
        component, config, links, joints = scene('punctuation')
        config['joint_limits'] = {joints[1]: {'effort': 4, 'velocity': .25}}
        robot, _, _ = self.export(component, config)
        self.assertEqual(robot.find("joint[@name='%s']/limit" % joints[1]).get('effort'), '4')
        self.assertEqual(robot.find("joint[@name='%s']/limit" % joints[1]).get('velocity'), '0.25')
        config['joint_limits'] = {'drive_pin': {'effort': 4}}
        self.reject(component, config, 'does not match an exported joint')

    def reject(self, component, config, pattern, *, corrupt=False):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'aliases_description'
            fixture, exported = design(component, corrupt=corrupt)
            before = snapshot(component)
            with self.assertRaisesRegex(ValueError, pattern):
                importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(fixture, target, config=config)
            self.assertEqual(snapshot(component), before)
            self.assertEqual(list(Path(directory).iterdir()), [])
            if not corrupt:
                self.assertEqual(exported, [])

    def test_duplicate_raw_identities_still_reject(self):
        component, config, _, _ = scene('punctuation')
        component.allOccurrences.append(occurrence('arm-pin:1', [Body('duplicate', 30)]))
        self.reject(component, config, 'Duplicate occurrence context identity')
        component, config, _, _ = scene('punctuation')
        component.allJoints[1].name = component.allJoints[0].name
        self.reject(component, config, 'Duplicate joint identity')

    def test_ambiguous_and_hidden_native_endpoints_are_not_resolved_by_alias(self):
        for case, error in (('native_hidden_alias', 'AMBIGUOUS_CONSTRAINT_TARGET'),
                            ('native_two_in_context', 'AMBIGUOUS_CONSTRAINT_TARGET'),
                            ('native_hidden_collision', 'CONSTRAINT_TARGET_EXCLUDED')):
            with self.subTest(case=case):
                component, _, _, _, _ = context_case(case, 'ordinary')
                self.reject(component, {}, error)

    def test_collision_exports_leave_source_unchanged_on_mesh_failure(self):
        component, config, _, _ = scene('unicode')
        self.reject(component, config, 'STL', corrupt=True)

    def test_ordinary_alias_cannot_steal_a_disambiguated_alias(self):
        component, config, links, _ = scene('punctuation')
        extra = occurrence(links[0], [Body('extra', 30)])
        component.allOccurrences.append(extra)
        component.allJoints.append(fixed('extra_mount', extra, component.allOccurrences[0]))
        robot, _, _ = self.export(component, config)
        self.assertEqual({e.get('name') for e in robot.findall('link')},
                         {'base_link', links[0], links[0] + '_', links[1]})


if __name__ == '__main__':
    unittest.main()
