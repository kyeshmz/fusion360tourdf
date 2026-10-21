"""Offline tests for fusion360tourdf exporter logic (no Fusion required)."""
import os
import struct
import sys
import tempfile
import types
import unittest
from unittest import mock
import importlib
from pathlib import Path
from tests.test_package import mesh_bytes, write_meshes

# Stub the Fusion `adsk` modules before importing exporter code.
adsk = types.ModuleType('adsk')
adsk_core = types.ModuleType('adsk.core')
adsk_fusion = types.ModuleType('adsk.fusion')


class _FakeCalcAccuracy:
    VeryHighCalculationAccuracy = 1


adsk_fusion.CalculationAccuracy = _FakeCalcAccuracy
adsk_fusion.MeshRefinementSettings = types.SimpleNamespace(MeshRefinementLow=0)
adsk_fusion.JointOrigin = type('JointOrigin', (), {})
adsk.core = adsk_core
adsk.fusion = adsk_fusion
sys.modules['adsk'] = adsk
sys.modules['adsk.core'] = adsk_core
sys.modules['adsk.fusion'] = adsk_fusion

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from URDF_Exporter.utils import utils  # noqa: E402
from URDF_Exporter.core import Joint, Link, Write  # noqa: E402


class FakePoint:
    def __init__(self, xyz):
        self._xyz = list(xyz)

    def asArray(self):
        return list(self._xyz)


class FakeMatrix:
    def __init__(self, array16):
        self._array = list(array16)
        self.translation = FakePoint([array16[3], array16[7], array16[11]])

    def asArray(self):
        return list(self._array)


def identity_at(xyz):
    return [
        1, 0, 0, xyz[0],
        0, 1, 0, xyz[1],
        0, 0, 1, xyz[2],
        0, 0, 0, 1,
    ]


class FakeOccurrence:
    def __init__(self, name, component_name, xyz):
        self.isValid = True
        self.name = name
        self.component = types.SimpleNamespace(name=component_name)
        self.transform2 = FakeMatrix(identity_at(xyz))
        self.bRepBodies = [object()]


class FakeJointMotion:
    def __init__(self, joint_type):
        self.jointType = joint_type


class FakeJoint:
    def __init__(self, name, occ_one, occ_two, one_xyz, two_xyz):
        self.isValid = True
        self.name = name
        self.occurrenceOne = occ_one
        self.occurrenceTwo = occ_two
        self.geometryOrOriginOne = types.SimpleNamespace(origin=FakePoint(one_xyz))
        self.geometryOrOriginTwo = types.SimpleNamespace(origin=FakePoint(two_xyz))
        self.geometryTwoTransform = FakeMatrix(identity_at(two_xyz))
        self.transform = self.geometryTwoTransform
        self.jointMotion = FakeJointMotion(0)


class FakeRoot:
    def __init__(self, joints, as_built=()):
        self.joints = joints
        self.asBuiltJoints = list(as_built)
        self.allJoints = joints
        self.allAsBuiltJoints = list(as_built)
        occurrences = {}
        self.bRepBodies = []
        for joint in list(joints) + list(as_built):
            for occ in (joint.occurrenceOne, joint.occurrenceTwo):
                if occ is None:
                    self.bRepBodies = [object()]
                else:
                    occurrences.setdefault(getattr(occ, 'fullPathName', None) or occ.name, occ)
        self.allOccurrences = list(occurrences.values())


class ExporterLogicTest(unittest.TestCase):
    def test_sanitize_name(self):
        self.assertEqual(utils.sanitize_name('base_link (1):1'), 'base_link_1_1')
        self.assertEqual(utils.sanitize_name('base-link/foo'), 'base_link_foo')
        self.assertEqual(utils.sanitize_name('base_link'), 'base_link')

    def test_transform_point_identity(self):
        out = utils.transform_point(identity_at([10, 20, 30]), [1, 2, 3])
        self.assertEqual(out, [11, 22, 33])

    def test_occurrence_helpers_prefer_transform2(self):
        occ = FakeOccurrence('a:1', 'a', [1, 2, 3])
        # Legacy `transform` returns the wrong answer; transform2 must win.
        occ.transform = FakeMatrix(identity_at([99, 99, 99]))
        self.assertEqual(utils.occurrence_translation(occ), [1, 2, 3])
        self.assertEqual(utils.occurrence_matrix_array(occ)[:4], [1, 0, 0, 1])

    def test_make_joints_dict_uses_current_root_context_transform(self):
        occ_one = FakeOccurrence('child:1', 'child', [0, 0, 0])
        occ_two = FakeOccurrence('parent:1', 'parent', [100, 0, 0])
        joint = FakeJoint('Rev1', occ_one, occ_two, [10, 0, 0], [5, 0, 0])
        joint.geometryTwoTransform = FakeMatrix(identity_at([105, 0, 0]))
        occ_two.transform = FakeMatrix(identity_at([0, 0, 0]))
        joints_dict, msg = Joint.make_joints_dict(FakeRoot([joint]), 'ok')
        self.assertEqual(msg, 'ok')
        self.assertIn('Rev1', joints_dict)
        self.assertEqual(joints_dict['Rev1']['xyz'], [1.05, 0.0, 0.0])
        self.assertEqual(joints_dict['Rev1']['type'], 'fixed')

    def test_make_joints_dict_rejects_ball(self):
        occ_one = FakeOccurrence('child:1', 'child', [0, 0, 0])
        occ_two = FakeOccurrence('parent:1', 'parent', [0, 0, 0])
        joint = FakeJoint('Ball1', occ_one, occ_two, [0, 0, 0], [0, 0, 0])
        joint.jointMotion = FakeJointMotion(6)
        _, msg = Joint.make_joints_dict(FakeRoot([joint]), 'ok')
        self.assertIn('Ball', msg)

    def test_nested_regular_and_as_built_joints_use_root_context_collections(self):
        base = FakeOccurrence('base:1', 'base_link', [0, 0, 0])
        arm = FakeOccurrence('module:1+arm:1', 'arm', [0, 0, 10])
        tool = FakeOccurrence('module:1+tool:1', 'tool', [0, 0, 20])
        regular = FakeJoint('hinge', arm, base, [0, 0, 10], [0, 0, 10])
        built = FakeJoint('mount', tool, arm, [0, 0, 20], [0, 0, 20])
        built.geometry = types.SimpleNamespace(origin=FakePoint([0, 0, 20]))
        for joint in (regular, built):
            joint.assemblyContext = types.SimpleNamespace(fullPathName='module:1')
        root = FakeRoot([])
        root.allJoints, root.allAsBuiltJoints = [regular], [built]
        root.allOccurrences = [base, arm, tool]
        joints, message = Joint.make_joints_dict(root, 'ok')
        self.assertEqual(message, 'ok')
        self.assertEqual(set(joints), {'module_1_hinge', 'module_1_mount'})
        self.assertEqual(joints['module_1_hinge']['child'], 'module_1_arm_1')
        self.assertEqual(joints['module_1_mount']['xyz'], [0, 0, .2])

    def test_repeated_subassembly_joints_keep_distinct_identities(self):
        base = FakeOccurrence('base:1', 'base_link', [0, 0, 0])
        proxies = []
        for side in ('left:1', 'right:1'):
            child = FakeOccurrence(side + '+wheel:1', 'wheel', [0, 0, 0])
            joint = FakeJoint('pivot', child, base, [0, 0, 0], [0, 0, 0])
            joint.assemblyContext = types.SimpleNamespace(fullPathName=side)
            proxies.append(joint)
        joints, message = Joint.make_joints_dict(FakeRoot(proxies), 'ok')
        self.assertEqual(message, 'ok')
        self.assertEqual(set(joints), {'left_1_pivot', 'right_1_pivot'})
        self.assertEqual(joints['right_1_pivot']['child'], 'right_1_wheel_1')

    def test_origin2center_of_mass(self):
        out = utils.origin2center_of_mass([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0)
        self.assertEqual(out, [1, 0, 0, 0, 0, 0])

    def test_write_urdf_roundtrip(self):
        joints_dict = {
            'Rev1': {
                'type': 'revolute', 'axis': [0, 0, 1],
                'upper_limit': 1.0, 'lower_limit': -1.0,
                'parent': 'base_link', 'child': 'link1',
                'xyz': [0.0, 0.0, 1.0],
            }
        }
        inertial_dict = {
            'base_link': {'center_of_mass': [0, 0, 0.5], 'mass': 1.0,
                          'inertia': [0.1, 0.1, 0.1, 0, 0, 0]},
            'link1': {'center_of_mass': [0, 0, 1.5], 'mass': 2.0,
                      'inertia': [0.2, 0.2, 0.2, 0, 0, 0]},
        }
        with tempfile.TemporaryDirectory() as tmp:
            save_dir = Path(tmp) / 'bot_description'
            Write.write_ros2_package(save_dir, 'bot', joints_dict, inertial_dict,
                                     write_meshes=write_meshes)
            xacro = os.path.join(save_dir, 'urdf', 'bot.xacro')
            self.assertTrue(os.path.exists(xacro))
            with open(xacro) as fh:
                text = fh.read()
            self.assertIn('joint name="Rev1"', text)
            self.assertIn('link name="link1"', text)


class FakeCollection(list):
    @property
    def count(self):
        return len(self)

    def item(self, index):
        return self[index]


class MeshExtractionTest(unittest.TestCase):
    def test_native_body_mesh_is_transformed_once_into_each_occurrence_context(self):
        native = object()
        one, two = self.occurrence('wheel:1'), self.occurrence('wheel:2')
        for occurrence in (one, two):
            occurrence.bRepBodies = FakeCollection([types.SimpleNamespace(nativeObject=native)])
        one.transform2 = FakeMatrix([0, -1, 0, 10, 1, 0, 0, 20, 0, 0, 1, 30, 0, 0, 0, 1])
        two.transform2 = FakeMatrix(identity_at([40, 50, 60]))
        design, exports = self.design([one, two])
        with tempfile.TemporaryDirectory() as tmp:
            targets = {name: Path(tmp) / (name + '.stl') for name in ('base_link', 'wheel_2')}
            utils.export_stl(design, targets, config={'root_occurrence': 'wheel:1'})
            self.assertEqual([option.geometry for option in exports], [native, native])
            triangle = struct.unpack_from('<12fH', targets['base_link'].read_bytes(), 84)
            original = struct.unpack_from('<12fH', mesh_bytes(), 84)
            for offset in (3, 6, 9):
                x, y, z = original[offset:offset+3]
                self.assertEqual(triangle[offset:offset+3], (100-y, 200+x, 300+z))
            x, y, z = original[:3]
            self.assertEqual(triangle[:3], (-y, x, z))
            repeated = struct.unpack_from('<12fH', targets['wheel_2'].read_bytes(), 84)
            for offset in (3, 6, 9):
                x, y, z = original[offset:offset+3]
                self.assertEqual(repeated[offset:offset+3], (400+x, 500+y, 600+z))

    def occurrence(self, name, component='wheel'):
        occ = FakeOccurrence(name, component, [0, 0, 0])
        occ.bRepBodies = FakeCollection([object()])
        occ.childOccurrences = FakeCollection()
        return occ

    def design(self, occurrences, bodies=(), fail=False):
        root = types.SimpleNamespace(allOccurrences=FakeCollection(occurrences),
                                     bRepBodies=FakeCollection(bodies))
        exports = []

        def options(geometry, filename):
            return types.SimpleNamespace(geometry=geometry, filename=filename, unitType=None)

        def execute(opt):
            exports.append(opt)
            if fail:
                return False
            Path(opt.filename).write_bytes(mesh_bytes())
            return True

        adsk.fusion.DistanceUnits = types.SimpleNamespace(MillimeterDistanceUnits=1)
        design = types.SimpleNamespace(rootComponent=root, exportManager=types.SimpleNamespace(
            createSTLExportOptions=options, execute=execute))
        return design, exports

    def test_repeated_components_use_occurrence_names_and_exact_targets(self):
        occurrences = [self.occurrence('wheel:1'), self.occurrence('wheel:2')]
        design, exports = self.design(occurrences)
        with tempfile.TemporaryDirectory() as tmp:
            targets = {name: Path(tmp) / (name + '.stl') for name in ('base_link', 'wheel_2')}
            utils.export_stl(design, targets, config={'root_occurrence': 'wheel:1'})
            self.assertEqual([o.geometry for o in exports], [o.bRepBodies[0] for o in occurrences])
            self.assertEqual([o.unitType for o in exports], [1, 1])
            self.assertEqual(targets['wheel_2'].read_bytes()[84:], mesh_bytes()[84:])

    def test_alias_allocation_does_not_hide_ambiguous_root_identity(self):
        occurrences = [self.occurrence('wheel:1'), self.occurrence('wheel_1')]
        design, _ = self.design(occurrences)
        sources = utils.link_sources(design.rootComponent, config={'root_occurrence': 'wheel:1'})
        self.assertEqual(set(sources), {'base_link', 'wheel_1'})
        self.assertIs(sources['base_link'].owner, occurrences[0])
        self.assertIs(sources['wheel_1'].owner, occurrences[1])
        design, _ = self.design([self.occurrence('a', 'base_link'), self.occurrence('b', 'base_link')])
        with self.assertRaisesRegex(ValueError, 'ROOT_AMBIGUOUS'):
            utils.link_sources(design.rootComponent)

    def test_nested_full_paths_are_shared_by_extraction_and_export(self):
        one, two = self.occurrence('wheel:1'), self.occurrence('wheel:1')
        one.fullPathName, two.fullPathName = 'left:1+wheel:1', 'right:1+wheel:1'
        design, _ = self.design([one, two])
        sources = utils.link_sources(design.rootComponent, config={'root_occurrence': 'right:1+wheel:1'})
        self.assertEqual(list(sources), ['left_1_wheel_1', 'base_link'])
        self.assertEqual(utils.link_name(one, sources), 'left_1_wheel_1')

    def test_synthetic_root_exports_only_own_bodies(self):
        root_bodies = [object(), object()]
        wheel = self.occurrence('wheel:1')
        design, exports = self.design([wheel], bodies=root_bodies)
        with tempfile.TemporaryDirectory() as tmp:
            targets = {name: Path(tmp) / (name + '.stl') for name in ('wheel_1', 'base_link')}
            utils.export_stl(design, targets)
            self.assertEqual([o.geometry for o in exports], list(wheel.bRepBodies) + root_bodies)
            self.assertNotIn(design.rootComponent, [o.geometry for o in exports])
            data = targets['base_link'].read_bytes()
            self.assertEqual(len(data), 184)
            self.assertEqual(int.from_bytes(data[80:84], 'little'), 2)

    def test_export_failure_propagates_for_mixed_geometry(self):
        wheel = self.occurrence('wheel:1')
        design, _ = self.design([wheel], fail=True)
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(RuntimeError, 'base_link'):
                utils.export_stl(design, {'base_link': Path(tmp) / 'base_link.stl'})
            wheel.childOccurrences = FakeCollection([object()])
            with self.assertRaisesRegex(RuntimeError, 'base_link'):
                utils.export_stl(design, {'base_link': Path(tmp) / 'base_link.stl'})

    def test_synthetic_root_inertia_does_not_include_assembly_children(self):
        def body_at(x):
            prop = types.SimpleNamespace(mass=1, centerOfMass=FakePoint([x, 0, 0]),
                getXYZMomentsOfInertia=lambda: (True, 10000, 20000, 30000, 0, 0, 0))
            return types.SimpleNamespace(isValid=True, isSolid=True, getPhysicalProperties=lambda _: prop)

        design, _ = self.design([], [body_at(-100), body_at(100)])
        design.rootComponent.getPhysicalProperties = lambda _: self.fail('whole assembly mass was requested')
        inertial, status = Link.make_inertial_dict(utils.link_sources(design.rootComponent), 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(inertial['base_link']['mass'], 2)
        self.assertEqual(inertial['base_link']['center_of_mass'], [0, 0, 0])
        self.assertEqual(inertial['base_link']['inertia'], [2, 4, 6, 0, 0, 0])

    def test_fusion_ui_reports_mesh_failure_without_publishing(self):
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        design, _ = self.design([], bodies=[object()], fail=True)
        design.rootComponent.name = 'bot'
        design.rootComponent.allJoints = []
        design.rootComponent.allAsBuiltJoints = []
        messages = []
        ui = types.SimpleNamespace(messageBox=lambda text, title: messages.append(text))
        app = types.SimpleNamespace(userInterface=ui, activeProduct=design)
        inertia = {'base_link': {'mass': 1, 'center_of_mass': [0, 0, 0], 'inertia': [1, 1, 1, 0, 0, 0]}}
        with tempfile.TemporaryDirectory() as tmp, \
             mock.patch.object(entry.adsk.core, 'Application', types.SimpleNamespace(get=lambda: app), create=True), \
             mock.patch.object(entry.adsk.fusion, 'Design', types.SimpleNamespace(cast=lambda product: product), create=True), \
             mock.patch.object(utils, 'file_dialog', return_value=tmp), \
             mock.patch.object(Joint, 'make_joints_dict', return_value=({}, 'ok')), \
             mock.patch.object(Link, 'make_inertial_dict', return_value=(inertia, 'ok')):
            entry.run(None)
            self.assertTrue(messages[-1].startswith('Export failed:'))
            self.assertIn('Fusion mesh export failed for base_link', messages[-1])
            self.assertEqual(list(Path(tmp).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
