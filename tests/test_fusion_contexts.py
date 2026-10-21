import importlib
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.test_fusion_inputs import Body, design, fixed, group, hinge, occurrence, root
from tests.test_exporter_logic import identity_at
from URDF_Exporter.utils import utils


class ApiOccurrence(types.SimpleNamespace):
    def __eq__(self, other):
        return isinstance(other, ApiOccurrence) and self.api_identity == other.api_identity


def api_occurrence(path, identity, bodies=(), x=0):
    return ApiOccurrence(**vars(occurrence(path, bodies, matrix=identity_at([x, 0, 0]))),
                         api_identity=identity, nativeObject=None, assemblyContext=None)


def context_case(case, mode):
    base = api_occurrence('base:1', 'base', [Body('base', 0)])
    base.component.name = 'base_link'
    native = api_occurrence('arm:1', 'native-arm', [Body('native', 99)], 99)
    left = api_occurrence('left:1', 'left')
    module = api_occurrence('left:1+module:1', 'left-module')
    child = api_occurrence('left:1+module:1+arm:1', 'left-arm', [Body('left', 10)], 10)
    child.nativeObject = ApiOccurrence(**vars(native))
    native.entityToken = 'native-token-before'
    child.nativeObject.entityToken = 'native-token-after'
    child.assemblyContext = module
    inventory = [base, left, module, child]
    endpoint, caller = native, left
    expected = 'left_1_module_1_arm_1'
    rejection = None

    if case.startswith('relative_'):
        endpoint = api_occurrence('module:1+arm:1', 'relative-arm', [Body('relative', 99)], 99)
        endpoint.nativeObject = ApiOccurrence(**vars(native))
        endpoint.assemblyContext = api_occurrence('module:1', 'native-module')
        module.nativeObject = endpoint.assemblyContext
    if 'collision' in case:
        other_module = api_occurrence('module:1', 'root-module')
        other = api_occurrence('module:1+arm:1', 'root-arm', [Body('root-arm', 20)], 20)
        other.nativeObject = ApiOccurrence(**vars(native))
        other.assemblyContext = other_module
        inventory.extend([other_module, other])
    if case in ('relative_hidden_collision', 'native_hidden_collision'):
        child.isVisible = False
        rejection = 'CONSTRAINT_TARGET_EXCLUDED'
    if case in ('native_hidden_alias', 'native_hidden_ancestor', 'native_two_in_context'):
        right = api_occurrence('right:1', 'right')
        other = api_occurrence('right:1+arm:1', 'right-arm', [Body('right', 20)], 20)
        other.nativeObject = ApiOccurrence(**vars(native))
        other.assemblyContext = right
        inventory.extend([right, other])
        if case == 'native_hidden_alias':
            child.isVisible = False
        if case == 'native_hidden_ancestor':
            left.isVisible = False
        caller = None
        rejection = 'AMBIGUOUS_CONSTRAINT_TARGET'
        if case == 'native_two_in_context':
            other.fullPathName = 'left:1+other:1+arm:1'
            caller = left
    if case == 'native_simple':
        child.fullPathName = 'left:1+arm:1'
        child.assemblyContext = left
        inventory.remove(module)
        expected = 'left_1_arm_1'
    if case == 'root_proxy':
        endpoint = ApiOccurrence(**vars(child))
        caller = api_occurrence('elsewhere:1', 'elsewhere')
    if case == 'root_proxy_context_path':
        endpoint = api_occurrence(child.fullPathName, 'alternate-proxy')
        endpoint.nativeObject = native
        endpoint.assemblyContext = types.SimpleNamespace(fullPathName=module.fullPathName)
        caller = endpoint.assemblyContext
    if case == 'relative_context_chain':
        endpoint = api_occurrence('arm:1', 'relative-arm', [Body('relative', 99)], 99)
        endpoint.nativeObject = ApiOccurrence(**vars(native))
        relative_module = api_occurrence('module:1', 'native-module')
        relative_module.assemblyContext = left
        module.nativeObject = api_occurrence('module:1', 'native-module')
        endpoint.assemblyContext = relative_module
        caller = None
    if case == 'unknown_context':
        caller = api_occurrence('missing:1', 'missing')
        rejection = 'CONSTRAINT_TARGET_EXCLUDED'
    if case == 'foreign_native_same_path':
        endpoint = api_occurrence('left:1+module:1+arm:1', 'foreign-proxy')
        endpoint.nativeObject = api_occurrence('arm:1', 'foreign-native')
        endpoint.assemblyContext = module
        rejection = 'CONSTRAINT_TARGET_EXCLUDED'

    joints = [fixed('other_mount_' + str(index), owner, base)
              for index, owner in enumerate(inventory)
              if owner is not child and owner is not base and owner.bRepBodies.count]
    parent = ApiOccurrence(**vars(base)) if case == 'root_alias_wrapper' else base
    constraint = group('drive', [parent, endpoint]) if mode == 'group' else hinge('drive', endpoint, parent)
    constraint.assemblyContext = caller
    if mode == 'as_built':
        constraint = fixed('drive', endpoint, parent)
        constraint.assemblyContext = caller
        constraint.transform = None
    component = root(inventory, joints=joints)
    if mode == 'ordinary':
        component.allJoints.append(constraint)
    elif mode == 'as_built':
        component.allAsBuiltJoints.append(constraint)
    else:
        component.allRigidGroups.append(constraint)
    return component, endpoint, caller, expected, rejection


CASES = ('relative_visible', 'relative_visible_collision', 'relative_hidden_collision',
         'native_deep', 'native_hidden_collision', 'native_hidden_alias',
         'native_hidden_ancestor', 'native_two_in_context', 'native_simple',
         'root_proxy', 'root_proxy_context_path', 'root_alias_wrapper', 'relative_context_chain',
         'unknown_context', 'foreign_native_same_path')
MODES = ('ordinary', 'as_built', 'group')


def census():
    rows = []
    for case in CASES:
        component, endpoint, caller, expected, rejection = context_case(case, 'ordinary')
        native = getattr(endpoint, 'nativeObject', None) or endpoint
        aliases = [owner.fullPathName for owner in component.allOccurrences
                   if (getattr(owner, 'nativeObject', None) or owner) == native]
        selected = {source.owner.fullPathName for source in utils.link_sources(component).values()}
        rows.append({'case': case, 'endpoint': endpoint.fullPathName,
            'native_identity': native.api_identity,
            'own_context': getattr(getattr(endpoint, 'assemblyContext', None), 'fullPathName', None),
            'caller_context': getattr(caller, 'fullPathName', None),
            'native_aliases': aliases, 'expected': rejection or expected,
            'inventory': [{'path': owner.fullPathName, 'visible': owner.isVisible,
                           'bodies': owner.bRepBodies.count, 'selected': owner.fullPathName in selected,
                           'native_identity': (getattr(owner, 'nativeObject', None) or owner).api_identity,
                           'own_context': getattr(getattr(owner, 'assemblyContext', None), 'fullPathName', None)}
                          for owner in component.allOccurrences]})
    return rows


def probe():
    entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
    rows = []
    for case in CASES:
        for mode in MODES:
            with tempfile.TemporaryDirectory() as directory:
                component, endpoint, caller, expected, rejection = context_case(case, mode)
                fixture, exported = design(component)
                destination = Path(directory) / 'contexts_description'
                row = {'case': case, 'mode': mode, 'expected': rejection or expected}
                try:
                    entry.export_design(fixture, destination)
                    robot = ET.parse(destination / 'urdf/contexts.xacro').getroot()
                    row['joints'] = [{'name': j.get('name'), 'parent': j.find('parent').get('link'),
                                      'child': j.find('child').get('link')} for j in robot.findall('joint')]
                except ValueError as error:
                    row['error'] = str(error)
                row.update(published=destination.exists(), exported=exported)
                rows.append(row)
    return rows


class ContextRegistryTest(unittest.TestCase):
    def test_export_endpoint_census(self):
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        for case in CASES:
            for mode in MODES:
                with self.subTest(case=case, mode=mode), tempfile.TemporaryDirectory() as directory:
                    component, endpoint, caller, expected, rejection = context_case(case, mode)
                    fixture, exported = design(component)
                    destination = Path(directory) / 'contexts_description'
                    if rejection:
                        with self.assertRaisesRegex(ValueError, rejection) as caught:
                            entry.export_design(fixture, destination)
                        self.assertIn('drive', str(caught.exception))
                        if case in ('relative_hidden_collision', 'native_hidden_collision'):
                            self.assertIn('left:1+module:1+arm:1', str(caught.exception))
                        self.assertEqual(list(Path(directory).iterdir()), [])
                        self.assertEqual(exported, [])
                    else:
                        self.assertEqual(entry.export_design(fixture, destination), destination)
                        robot = ET.parse(destination / 'urdf/contexts.xacro').getroot()
                        name = 'rigid_attachment_' + expected if mode == 'group' else (
                            caller.fullPathName.replace(':', '_').replace('+', '_') + '_drive' if caller else 'drive')
                        joint = robot.find("joint[@name='%s']" % name)
                        self.assertIsNotNone(joint)
                        self.assertEqual(joint.find('parent').get('link'), 'base_link')
                        self.assertEqual(joint.find('child').get('link'), expected)
                        if mode != 'ordinary':
                            self.assertEqual(joint.find('origin').get('xyz'), '0.1 0 0')
                        self.assertEqual((destination / ('meshes/' + expected + '.stl')).read_bytes()[84:],
                                         Body('expected', 10).mesh[84:])

    def test_repeated_articulated_instances_remain_distinct(self):
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        for mode in ('ordinary', 'as_built'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                base = api_occurrence('base:1', 'base', [Body('base', 0)])
                base.component.name = 'base_link'
                native = api_occurrence('arm:1', 'native-arm', [Body('native', 99)], 99)
                inventory, joints = [base], []
                for side, x in (('left', 10), ('right', 20)):
                    context = api_occurrence(side + ':1', side)
                    child = api_occurrence(side + ':1+arm:1', side + '-arm', [Body(side, x)], x)
                    child.nativeObject = ApiOccurrence(**vars(native))
                    child.assemblyContext = context
                    inventory.extend([context, child])
                    joint = hinge('drive', native, ApiOccurrence(**vars(base)))
                    joint.assemblyContext = context
                    joints.append(joint)
                component = root(inventory, joints=joints if mode == 'ordinary' else ())
                component.allAsBuiltJoints = joints if mode == 'as_built' else []
                fixture, exported = design(component)
                destination = Path(directory) / 'repeated_description'
                self.assertEqual(entry.export_design(fixture, destination), destination)
                robot = ET.parse(destination / 'urdf/repeated.xacro').getroot()
                self.assertEqual([(j.get('name'), j.get('type'), j.find('parent').get('link'),
                                   j.find('child').get('link')) for j in robot.findall('joint')],
                                 [('left_1_drive', 'continuous', 'base_link', 'left_1_arm_1'),
                                  ('right_1_drive', 'continuous', 'base_link', 'right_1_arm_1')])
                self.assertEqual(exported, ['base', 'left', 'right'])

    def test_relative_bodyless_container_uses_same_context_inventory(self):
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        for hidden in (False, True):
            with self.subTest(hidden=hidden), tempfile.TemporaryDirectory() as directory:
                base = api_occurrence('base:1', 'base', [Body('base', 0)])
                base.component.name = 'base_link'
                left = api_occurrence('left:1', 'left')
                native = api_occurrence('assembly:1', 'native-assembly')
                container = api_occurrence('left:1+module:1+assembly:1', 'left-assembly')
                container.nativeObject = native
                a = api_occurrence(container.fullPathName + '+a:1', 'a', [Body('a', 10)], 10)
                b = api_occurrence(container.fullPathName + '+b:1', 'b', [Body('b', 20)], 20)
                b.isVisible = not hidden
                module = api_occurrence('left:1+module:1', 'left-module')
                relative_module = api_occurrence('module:1', 'native-module')
                module.nativeObject = relative_module
                relative = api_occurrence('module:1+assembly:1', 'relative-assembly')
                relative.nativeObject = native
                relative.assemblyContext = relative_module
                clamp = group('container_clamp', [base, relative, a, b])
                clamp.assemblyContext = left
                fixture, exported = design(root([base, left, module, container, a, b], groups=[clamp]))
                destination = Path(directory) / 'containers_description'
                if hidden:
                    with self.assertRaisesRegex(ValueError, 'CONSTRAINT_TARGET_EXCLUDED.*container_clamp') as caught:
                        entry.export_design(fixture, destination)
                    self.assertIn('left:1+module:1+assembly:1+b:1', str(caught.exception))
                    self.assertEqual(list(Path(directory).iterdir()), [])
                    self.assertEqual(exported, [])
                else:
                    self.assertEqual(entry.export_design(fixture, destination), destination)
                    robot = ET.parse(destination / 'urdf/containers.xacro').getroot()
                    self.assertEqual([(j.find('parent').get('link'), j.find('child').get('link'))
                                      for j in robot.findall('joint')],
                                     [('base_link', 'left_1_module_1_assembly_1'),
                                      ('base_link', 'left_1_module_1_assembly_1_a_1'),
                                      ('base_link', 'left_1_module_1_assembly_1_b_1')])


if __name__ == '__main__':
    if '--census' in sys.argv:
        print(json.dumps(census(), indent=2))
    elif '--probe' in sys.argv:
        print(json.dumps(probe(), indent=2))
    else:
        unittest.main()
