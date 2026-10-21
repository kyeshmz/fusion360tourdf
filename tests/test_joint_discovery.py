import importlib
from pathlib import Path
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.test_fusion_inputs import Body, design, fixed, occurrence, root


class RaisingAttribute:
    def __init__(self, component, attribute, error):
        self._attribute = attribute
        self._error = error
        self.__dict__.update(vars(component))

    def __getattribute__(self, name):
        if name == object.__getattribute__(self, '_attribute'):
            raise object.__getattribute__(self, '_error')
        return object.__getattribute__(self, name)


class IndexedCollection:
    def __init__(self, items, error=None):
        self.items = items
        self.count = len(items)
        self.error = error

    def item(self, index):
        if index == 1 and self.error is not None:
            raise self.error
        return self.items[index]


class BrokenIterator(IndexedCollection):
    def __iter__(self):
        yield self.items[0]
        raise self.error

    def item(self, index):
        return self.items[index]


class ShortIterator(IndexedCollection):
    def __iter__(self):
        return iter(self.items[:1])


class UnreadableCount:
    def __init__(self, items, error):
        self.items = items
        self.error = error

    @property
    def count(self):
        raise self.error

    def __iter__(self):
        return iter(self.items)


class JointDiscoveryTest(unittest.TestCase):
    def assembly(self, *, inferred=False):
        base = occurrence('anchor:1', [Body('base', 0)],
                          component='anchor' if inferred else 'base_link')
        arm = occurrence('module:1+arm:1', [Body('arm', 10)])
        tool = occurrence('module:1+tool:1', [Body('tool', 20)])
        joints = [fixed('mount', arm, base), fixed('tool_mount', tool, arm)]
        for joint in joints:
            joint.assemblyContext = types.SimpleNamespace(fullPathName='module:1')
        return root([tool, arm, base], joints=joints), joints

    def assert_rejected(self, component, pattern, *, cause=None, config=None):
        fixture, exported = design(component)
        before = {key: value for key, value in vars(component).items()}
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            marker = parent / 'existing.txt'
            marker.write_text('untouched')
            target = parent / 'joints_description'
            with self.assertRaisesRegex(ValueError, pattern) as caught:
                importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(
                    fixture, target, config=config)
            self.assertFalse(target.exists())
            self.assertEqual(sorted(path.name for path in parent.iterdir()), ['existing.txt'])
            self.assertEqual(marker.read_text(), 'untouched')
            self.assertEqual(exported, [])
            self.assertEqual(vars(component), before)
            if cause is not None:
                self.assertIs(caught.exception.__cause__, cause)
            return caught.exception

    def test_raising_collection_getters_fail_with_context_and_original_cause(self):
        for inferred in (False, True):
            for attribute in ('allJoints', 'allAsBuiltJoints'):
                for error_type in (RuntimeError, AttributeError):
                    with self.subTest(inferred=inferred, attribute=attribute, error=error_type):
                        component, _ = self.assembly(inferred=inferred)
                        error = error_type('InternalValidationError from constraints')
                        component = RaisingAttribute(component, attribute, error)
                        phase = 'root inference' if inferred else 'joint extraction'
                        self.assert_rejected(component,
                            'JOINT_COLLECTION_UNAVAILABLE.*fixture.*' + attribute + '.*' + phase,
                            cause=error)

    def test_iteration_errors_cannot_be_retried_as_successful_indexed_discovery(self):
        for inferred in (False, True):
            for attribute in ('allJoints', 'allAsBuiltJoints'):
                for error_type in (RuntimeError, TypeError):
                    with self.subTest(inferred=inferred, attribute=attribute, error=error_type):
                        component, joints = self.assembly(inferred=inferred)
                        component.allJoints = []
                        error = error_type('context became invalid after first joint')
                        setattr(component, attribute, BrokenIterator(joints, error))
                        phase = 'root inference' if inferred else 'joint extraction'
                        self.assert_rejected(component,
                            'JOINT_COLLECTION_UNAVAILABLE.*' + attribute + '.*' + phase,
                            cause=error)

    def test_indexed_collection_item_failure_never_exports_a_partial_prefix(self):
        for inferred in (False, True):
            for attribute in ('allJoints', 'allAsBuiltJoints'):
                for error_type in (RuntimeError, TypeError):
                    with self.subTest(inferred=inferred, attribute=attribute, error=error_type):
                        component, joints = self.assembly(inferred=inferred)
                        component.allJoints = []
                        error = error_type('item(1) is unavailable')
                        setattr(component, attribute, IndexedCollection(joints, error))
                        self.assert_rejected(component,
                            'JOINT_COLLECTION_UNAVAILABLE.*' + attribute,
                            cause=error)

    def test_truncated_iteration_and_null_items_cannot_omit_constraints(self):
        for inferred in (False, True):
            for attribute in ('allJoints', 'allAsBuiltJoints'):
                for kind in ('truncated', 'null'):
                    with self.subTest(inferred=inferred, attribute=attribute, kind=kind):
                        component, joints = self.assembly(inferred=inferred)
                        component.allJoints = []
                        collection = (ShortIterator(joints) if kind == 'truncated'
                                      else IndexedCollection([joints[0], None]))
                        setattr(component, attribute, collection)
                        self.assert_rejected(component, 'JOINT_COLLECTION_UNAVAILABLE.*' + attribute)

    def test_unreadable_count_cannot_be_mistaken_for_absent_metadata(self):
        for inferred in (False, True):
            for attribute in ('allJoints', 'allAsBuiltJoints'):
                with self.subTest(inferred=inferred, attribute=attribute):
                    component, joints = self.assembly(inferred=inferred)
                    component.allJoints = []
                    error = AttributeError('constraint count is unavailable')
                    setattr(component, attribute, UnreadableCount(joints, error))
                    self.assert_rejected(component,
                        'JOINT_COLLECTION_UNAVAILABLE.*' + attribute, cause=error)

    def test_ordinary_as_built_and_indexed_collections_preserve_complete_root_inference(self):
        for indexed in (False, True):
            component, joints = self.assembly(inferred=True)
            component.allJoints = IndexedCollection(joints[:1]) if indexed else joints[:1]
            component.allAsBuiltJoints = IndexedCollection(joints[1:]) if indexed else joints[1:]
            fixture, exported = design(component)
            with self.subTest(indexed=indexed), tempfile.TemporaryDirectory() as directory:
                package = importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(
                    fixture, Path(directory) / 'joints_description')
                robot = ET.parse(package / 'urdf/joints.xacro').getroot()
                self.assertEqual({link.get('name') for link in robot.findall('link')},
                                 {'base_link', 'module_1_arm_1', 'module_1_tool_1'})
                self.assertEqual({(joint.get('name'), joint.find('parent').get('link'),
                                  joint.find('child').get('link')) for joint in robot.findall('joint')},
                    {('module_1_mount', 'base_link', 'module_1_arm_1'),
                     ('module_1_tool_mount', 'module_1_arm_1', 'module_1_tool_1')})
                self.assertEqual(sorted(exported), ['arm', 'base', 'tool'])

    def test_unreadable_endpoints_name_context_property_and_preserve_cause(self):
        for inferred in (False, True):
            for attribute in ('occurrenceOne', 'occurrenceTwo'):
                with self.subTest(inferred=inferred, attribute=attribute):
                    component, joints = self.assembly(inferred=inferred)
                    error = RuntimeError('historical occurrence is unavailable')
                    component.allJoints[0] = RaisingAttribute(joints[0], attribute, error)
                    self.assert_rejected(component,
                        'JOINT_ENDPOINT_UNAVAILABLE.*module:1\\+mount.*' + attribute,
                        cause=error)

    def test_invalid_joint_or_occurrence_is_not_rebound_to_design_root(self):
        for invalid in ('joint', 'occurrence'):
            with self.subTest(invalid=invalid):
                arm = occurrence('arm:1', [Body('arm', 10)])
                joint = fixed('dangling', arm, None)
                component = root([arm], bodies=[Body('root', 0)], joints=[joint])
                (joint if invalid == 'joint' else arm).isValid = False
                self.assert_rejected(component, 'INVALID_JOINT_ENDPOINT.*dangling')

    def test_unreadable_validity_metadata_preserves_errors_instead_of_assuming_valid(self):
        class UnreadableValidity:
            def __init__(self, value, error):
                self.__dict__.update(vars(value))
                self.error = error

            @property
            def isValid(self):
                raise self.error

        for inferred in (False, True):
            for target in ('joint', 'occurrenceOne', 'occurrenceTwo'):
                for error_type in (AttributeError, RuntimeError):
                    for wrapper in (UnreadableValidity, lambda value, error: RaisingAttribute(value, 'isValid', error)):
                        with self.subTest(inferred=inferred, target=target, error=error_type, wrapper=wrapper):
                            component, joints = self.assembly(inferred=inferred)
                            error = error_type('validity metadata is unreadable')
                            if target == 'joint':
                                component.allJoints[0] = wrapper(joints[0], error)
                            else:
                                original = getattr(joints[0], target)
                                wrapped = wrapper(original, error)
                                component.allOccurrences = [wrapped if item is original else item
                                                            for item in component.allOccurrences]
                                for joint in joints:
                                    for attribute in ('occurrenceOne', 'occurrenceTwo'):
                                        if getattr(joint, attribute) is original:
                                            setattr(joint, attribute, wrapped)
                            self.assert_rejected(component, 'JOINT_ENDPOINT_UNAVAILABLE', cause=error)

    def test_missing_required_validity_metadata_does_not_publish(self):
        for inferred in (False, True):
            for target in ('joint', 'occurrenceOne', 'occurrenceTwo'):
                with self.subTest(inferred=inferred, target=target):
                    component, joints = self.assembly(inferred=inferred)
                    value = joints[0] if target == 'joint' else getattr(joints[0], target)
                    del value.isValid
                    error = self.assert_rejected(component, 'JOINT_ENDPOINT_UNAVAILABLE')
                    self.assertIsInstance(error.__cause__, AttributeError)

    def test_bodyless_reference_and_design_root_remain_unsupported_not_invalid(self):
        for endpoint in (None, occurrence('frame:1')):
            with self.subTest(endpoint=endpoint):
                arm = occurrence('arm:1', [Body('arm', 10)], component='base_link')
                joint = fixed('frame_mount', arm, endpoint)
                joint.isValid = True
                component = root([arm] + ([] if endpoint is None else [endpoint]), joints=[joint])
                with tempfile.TemporaryDirectory() as directory:
                    fixture, exported = design(component)
                    destination = Path(directory) / 'frames_description'
                    importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(fixture, destination)
                    robot = ET.parse(destination / 'urdf/frames.urdf').getroot()
                    name = 'design_root' if endpoint is None else 'frame_1'
                    self.assertEqual(list(robot.find("link[@name='%s']" % name)), [])
                    self.assertEqual(robot.find('joint/parent').get('link'), 'base_link')
                    self.assertEqual(robot.find('joint/child').get('link'), name)
                    self.assertEqual(exported, ['arm'])
                    self.assertFalse((destination / 'meshes' / (name + '.stl')).exists())


if __name__ == '__main__':
    unittest.main()
