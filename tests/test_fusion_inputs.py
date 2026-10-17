import importlib
import json
import math
from pathlib import Path
import struct
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakeCollection, FakeJoint, FakeMatrix, FakePoint, identity_at
from URDF_Exporter.core import Joint, Link
from URDF_Exporter.utils import utils


class Body:
    def __init__(self, name, x, mass=1, visible=True):
        self.name = name
        self.isVisible = visible
        self.boundingBox = types.SimpleNamespace(
            minPoint=FakePoint([x - 1, -1, -1]), maxPoint=FakePoint([x + 1, 1, 1]))
        self.properties = types.SimpleNamespace(mass=mass, centerOfMass=FakePoint([x, 0, 0]),
            getXYZMomentsOfInertia=lambda: (True, (2 / 3) * mass, (2 / 3 + x*x) * mass,
                                          (2 / 3 + x*x) * mass, 0, 0, 0))
        vertices = [((x + dx) * 10, dy * 10, dz * 10) for dx, dy, dz in (
            (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))]
        triangles = ((0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
                     (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
                     (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7))
        data = bytearray(b'body'.ljust(80, b'\0') + struct.pack('<I', len(triangles)))
        for indices in triangles:
            a, b, c = [vertices[index] for index in indices]
            u, v = ([b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)])
            normal = [u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0]]
            length = math.sqrt(sum(value * value for value in normal))
            data.extend(struct.pack('<12fH', *[value / length for value in normal], *a, *b, *c, 0))
        self.mesh = bytes(data)

    def getPhysicalProperties(self, accuracy):
        return self.properties


def occurrence(path, bodies=(), *, component=None, matrix=None):
    return types.SimpleNamespace(name=path.split('+')[-1], fullPathName=path,
        component=types.SimpleNamespace(name=component or path.split(':')[0], motionLinks=[]),
        bRepBodies=FakeCollection(bodies), meshBodies=[], isVisible=True,
        transform2=FakeMatrix(matrix or identity_at([0, 0, 0])), isGrounded=False)


def root(occurrences=(), bodies=(), joints=(), groups=()):
    return types.SimpleNamespace(name='fixture', allOccurrences=FakeCollection(occurrences),
        bRepBodies=FakeCollection(bodies), meshBodies=[], allJoints=list(joints),
        allAsBuiltJoints=[], allRigidGroups=list(groups), motionLinks=[])


def design(component, *, corrupt=False):
    exported = []

    def options(geometry, filename):
        return types.SimpleNamespace(geometry=geometry, filename=filename, unitType=None)

    def execute(option):
        exported.append(option.geometry.name)
        Path(option.filename).write_bytes(b'bad' if corrupt else option.geometry.mesh)
        return True

    utils.adsk.fusion.DistanceUnits = types.SimpleNamespace(MillimeterDistanceUnits=1)
    return types.SimpleNamespace(rootComponent=component, exportManager=types.SimpleNamespace(
        createSTLExportOptions=options, execute=execute)), exported


def fixed(name, child, parent, xyz=(0, 0, 0)):
    joint = FakeJoint(name, child, parent, xyz, xyz)
    joint.geometryTwoTransform = FakeMatrix(identity_at(xyz))
    return joint


def hinge(name, child, parent):
    joint = fixed(name, child, parent)
    joint.jointMotion = types.SimpleNamespace(jointType=1, rotationAxisVector=FakePoint([0, 0, 1]),
        rotationValue=0, rotationLimits=types.SimpleNamespace(isMaximumValueEnabled=False,
                                                            isMinimumValueEnabled=False))
    return joint


def group(name, members, suppressed=False):
    return types.SimpleNamespace(name=name, occurrences=list(members), isSuppressed=suppressed)


def first_vertex_fk(robot, destination, joint_name, q):
    joint = robot.find("joint[@name='%s']" % joint_name)
    child = joint.find('child').get('link')
    visual = robot.find("link[@name='%s']/visual" % child)
    mesh = visual.find('geometry/mesh')
    point = struct.unpack_from('<3f', (destination / 'meshes' / (child + '.stl')).read_bytes(), 96)
    point = [value * scale for value, scale in zip(point, map(float, mesh.get('scale').split()))]

    def transform(point, origin):
        x, y, z = map(float, origin.get('xyz').split())
        yaw = float(origin.get('rpy').split()[2])
        return [x + math.cos(yaw)*point[0] - math.sin(yaw)*point[1],
                y + math.sin(yaw)*point[0] + math.cos(yaw)*point[1], z + point[2]]

    point = transform(point, visual.find('origin'))
    axis = [float(v) for v in joint.find('axis').get('xyz').split()]
    if joint.get('type') == 'prismatic':
        point = [value + q*direction for value, direction in zip(point, axis)]
    else:
        angle = axis[2]*q
        point = [math.cos(angle)*point[0] - math.sin(angle)*point[1],
                 math.sin(angle)*point[0] + math.cos(angle)*point[1], point[2]]
    return transform(point, joint.find('origin'))


class DirectBodyTest(unittest.TestCase):
    def test_mixed_container_uses_same_visible_direct_bodies_for_mass_and_stl(self):
        first, second = Body('left', -10), Body('right', 10, mass=3)
        hidden = Body('hidden', 100, mass=99, visible=False)
        parent = occurrence('frame:1', [first, hidden, second], component='base_link')
        child = occurrence('frame:1+tool:1', [Body('tool', 30)])
        parent.childOccurrences = FakeCollection([child])
        parent.getPhysicalProperties = lambda _: self.fail('Descendant-inclusive properties read')
        component = root([parent, child])
        sources = utils.link_sources(component)
        self.assertEqual(sources['base_link'].bodies, (first, second))
        inertial, status = Link.make_inertial_dict(sources, 'ok')
        self.assertEqual(status, 'ok')
        entry = inertial['base_link']
        self.assertEqual(entry['mass'], 4)
        for actual, expected in zip(entry['center_of_mass'], [.05, 0, 0]):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual, expected in zip(entry['inertia'],
                [.0002666666666666667, .030266666666666667, .030266666666666667, 0, 0, 0]):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(entry['bbox_center_m'], [0, 0, 0])
        self.assertEqual(entry['bbox_size_m'], [.22, .02, .02])
        fixture, exported = design(component)
        second.isVisible = False
        with tempfile.TemporaryDirectory() as directory:
            targets = {name: Path(directory) / (name + '.stl') for name in sources}
            utils.export_stl(fixture, targets, sources=sources)
            data = targets['base_link'].read_bytes()
            self.assertEqual(struct.unpack_from('<I', data, 80), (24,))
            self.assertEqual(data[84:], first.mesh[84:] + second.mesh[84:])
            self.assertEqual(exported, ['left', 'right', 'tool'])

    def test_world_origin_products_are_summed_then_shifted_once(self):
        one, two = Body('offset', 10, mass=2), Body('counterweight', -10)
        one.properties.centerOfMass = FakePoint([10, 20, 0])
        one.properties.getXYZMomentsOfInertia = lambda: (True, 804, 205, 1006, -400, 0, 0)
        two.properties.getXYZMomentsOfInertia = lambda: (True, 1, 102, 103, 0, 0, 0)
        inertial, status = Link.make_inertial_dict(utils.link_sources(root(bodies=[one, two])), 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(inertial['base_link']['mass'], 3)
        for actual, expected in zip(inertial['base_link']['center_of_mass'], [.0333333333333333, .1333333333333333, 0]):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual, expected in zip(inertial['base_link']['inertia'],
                [.0271666666666667, .0273666666666667, .0542333333333333, -.0266666666666667, 0, 0]):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_shared_export_publishes_actual_body_mesh_and_rejects_corruption(self):
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        component = root(bodies=[Body('root solid', 10)])
        fixture, exported = design(component)
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'body_description'
            result = entry.export_design(fixture, target)
            self.assertEqual(result, target)
            robot = ET.parse(target / 'urdf/body.xacro').getroot()
            self.assertEqual(robot.find('link').get('name'), 'base_link')
            self.assertEqual(robot.find('link/inertial/origin').get('xyz'), '0.1 0 0')
            self.assertEqual((target / 'meshes/base_link.stl').read_bytes()[84:], component.bRepBodies[0].mesh[84:])
            self.assertEqual(exported, ['root solid'])
            with self.assertRaises(FileExistsError):
                entry.export_design(fixture, target)
            self.assertEqual(robot.find('link/inertial/origin').get('xyz'), '0.1 0 0')
            for invalid in ('BadRobot_description', 'missing_suffix'):
                with self.assertRaisesRegex(ValueError, 'Destination'):
                    entry.export_design(fixture, Path(directory) / invalid)
            bad, _ = design(component, corrupt=True)
            with self.assertRaises(ValueError):
                entry.export_design(bad, Path(directory) / 'bad_description')
            self.assertEqual(sorted(p.name for p in Path(directory).iterdir()), ['body_description'])

    def test_visible_mesh_bodies_are_named_unimplemented_not_silently_dropped(self):
        component = root([occurrence('hybrid:1', [Body('solid', 0)])])
        component.allOccurrences[0].meshBodies = [types.SimpleNamespace(name='scan', isVisible=True)]
        with self.assertRaisesRegex(ValueError, 'MESH_BODY_NOT_IMPLEMENTED.*hybrid:1.*scan'):
            utils.link_sources(component)
        component.allOccurrences[0].bRepBodies = []
        with self.assertRaisesRegex(ValueError, 'MESH_BODY_NOT_IMPLEMENTED.*hybrid:1.*scan'):
            utils.link_sources(component)


class ExportFixture:
    def export(self, component, config=None):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        destination = Path(temporary.name) / 'roots_description'
        fixture, _ = design(component)
        importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(
            fixture, destination, config=config)
        return ET.parse(destination / 'urdf/roots.xacro').getroot(), destination


class RootAndDirectionTest(ExportFixture, unittest.TestCase):
    def test_excluded_contextual_endpoint_is_not_rebound_to_visible_native_sibling(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        native = occurrence('arm:1', [Body('native arm', 10)])
        left = occurrence('left:1+arm:1', [Body('left arm', 10)])
        right = occurrence('right:1+arm:1', [Body('right arm', 20)])
        left.nativeObject = right.nativeObject = native
        left.assemblyContext = types.SimpleNamespace(fullPathName='left:1')
        right.assemblyContext = types.SimpleNamespace(fullPathName='right:1')
        left.isVisible = False
        component = root([base, left, right], joints=[hinge('hidden_arm_drive', left, base)])
        fixture, exported = design(component)
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'hidden_description'
            with self.assertRaisesRegex(ValueError,
                    r'hidden_arm_drive.*CONSTRAINT_TARGET_EXCLUDED.*left:1\+arm:1'):
                entry.export_design(fixture, target)
            self.assertFalse(target.exists())
            self.assertEqual(exported, [])

    def test_public_joint_extraction_uses_the_same_selected_root_alias(self):
        base = occurrence('anchor:1', [Body('anchor', 0)])
        base.isGrounded = True
        arm = occurrence('arm:1', [Body('arm', 10)])
        component = root([base, arm], joints=[fixed('mount', arm, base)])
        joints, status = Joint.make_joints_dict(component, 'ok')
        self.assertEqual(status, 'ok')
        self.assertEqual(joints['mount']['parent'], 'base_link')
        self.assertEqual(joints['mount']['child'], 'arm_1')

    def test_explicit_base_and_root_bodies_stay_separate(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        component = root([base], bodies=[Body('imported root', 10)])
        component.allJoints = [fixed('root_mount', None, base, (10, 0, 0))]
        robot, _ = self.export(component)
        self.assertEqual([link.get('name') for link in robot.findall('link')], ['base_link', 'design_root'])
        self.assertEqual([float(link.find('inertial/mass').get('value')) for link in robot.findall('link')], [1, 1])
        self.assertEqual(robot.find('joint/parent').get('link'), 'base_link')
        self.assertEqual(robot.find('joint/child').get('link'), 'design_root')

    def test_root_precedence_exact_configuration_and_named_ambiguity(self):
        one = occurrence('module:1+base:1', [Body('one', 0)], component='base_link')
        two = occurrence('module:2+base:1', [Body('two', 10)], component='base_link')
        component = root([one, two])
        with self.assertRaisesRegex(ValueError, 'ROOT_AMBIGUOUS.*root_occurrence'):
            utils.link_sources(component)
        sources = utils.link_sources(component, config={'root_occurrence': 'module:2+base:1'})
        self.assertIs(sources['base_link'].owner, two)
        self.assertEqual(set(sources), {'module_1_base_1', 'base_link'})
        with self.assertRaisesRegex(ValueError, 'ROOT_NOT_FOUND.*base:1'):
            utils.link_sources(component, config={'root_occurrence': 'base:1'})
        two.isVisible = False
        with self.assertRaisesRegex(ValueError, 'ROOT_NOT_FOUND.*module:2'):
            utils.link_sources(component, config={'root_occurrence': 'module:2+base:1'})
        one.isVisible = False
        with self.assertRaisesRegex(ValueError, 'ROOT_NOT_FOUND.*module:2'):
            utils.link_sources(component, config={'root_occurrence': 'module:2+base:1'})
        one.isVisible = True
        one.component.name = 'plain'
        one.isGrounded = True
        self.assertIs(utils.link_sources(component)['base_link'].owner, one)
        one.isGrounded = False
        self.assertIs(utils.link_sources(component)['base_link'].owner, one)
        two.isVisible = True
        two.component.name = 'plain'
        component.allJoints = [fixed('outward', two, one)]
        self.assertIs(utils.link_sources(component)['base_link'].owner, one)
        one.isGrounded = two.isGrounded = True
        with self.assertRaisesRegex(ValueError, 'ROOT_AMBIGUOUS'):
            utils.link_sources(component)

    def test_reversed_revolute_preserves_native_coordinate_sign_and_pose_limits(self):
        anchor = occurrence('nested:1+anchor:1', [Body('anchor', 0)],
            matrix=[0, -1, 0, 20, 1, 0, 0, 30, 0, 0, 1, 0, 0, 0, 0, 1])
        anchor.isGrounded = True
        arm = occurrence('arm:1', [Body('arm', 10)],
            matrix=[0, 1, 0, 40, -1, 0, 0, 30, 0, 0, 1, 0, 0, 0, 0, 1])
        joint = fixed('hinge', anchor, arm, (20, 30, 0))
        joint.jointMotion = types.SimpleNamespace(jointType=1, rotationAxisVector=FakePoint([0, 0, 1]),
            rotationValue=.25, rotationLimits=types.SimpleNamespace(isMaximumValueEnabled=True,
                isMinimumValueEnabled=True, minimumValue=-.5, maximumValue=1))
        robot, destination = self.export(root([arm, anchor], joints=[joint]))
        hinge = robot.find('joint')
        self.assertEqual(hinge.find('parent').get('link'), 'base_link')
        self.assertEqual(hinge.find('child').get('link'), 'arm_1')
        self.assertEqual(hinge.find('axis').get('xyz'), '0 0 -1')
        self.assertEqual(float(hinge.find('limit').get('lower')), -.75)
        self.assertEqual(float(hinge.find('limit').get('upper')), .75)
        self.assertEqual(hinge.find('origin').get('xyz'), '0.2 0.3 0')
        yaw = float(hinge.find('origin').get('rpy').split()[2])
        self.assertAlmostEqual(yaw, -math.pi/2)
        for q, expected in ((0, [.09, -.01, -.01]), (math.pi/2, [-.11, .41, -.01])):
            for actual, want in zip(first_vertex_fk(robot, destination, 'hinge', q), expected):
                self.assertAlmostEqual(actual, want, places=12)

    def test_reversed_slider_rotates_axis_and_moves_geometry_in_negative_world_x(self):
        anchor = occurrence('anchor:1', [Body('anchor', 0)])
        carriage = occurrence('module:1+carriage:1', [Body('carriage', 10)],
            matrix=[0, -1, 0, 10, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
        joint = fixed('slide', anchor, carriage, (10, 0, 0))
        joint.jointMotion = types.SimpleNamespace(jointType=2, slideDirectionVector=FakePoint([1, 0, 0]),
            slideValue=5, slideLimits=types.SimpleNamespace(isMaximumValueEnabled=True,
                isMinimumValueEnabled=True, minimumValue=0, maximumValue=20))
        robot, destination = self.export(root([anchor, carriage], joints=[joint]), {'root_occurrence': 'anchor:1'})
        slide = robot.find('joint')
        self.assertEqual(slide.find('axis').get('xyz'), '0 1 0')
        self.assertEqual(float(slide.find('limit').get('lower')), -.05)
        self.assertAlmostEqual(float(slide.find('limit').get('upper')), .15)
        self.assertAlmostEqual(float(slide.find('origin').get('rpy').split()[2]), math.pi/2)
        for q, expected in ((0, [.09, -.01, -.01]), (.03, [.06, -.01, -.01])):
            for actual, want in zip(first_vertex_fk(robot, destination, 'slide', q), expected):
                self.assertAlmostEqual(actual, want, places=12)


class RigidConstraintTest(ExportFixture, unittest.TestCase):
    def assembly(self):
        return [occurrence(name + ':1', [Body(name, index*10)],
                component='base_link' if index == 0 else name,
                matrix=identity_at([index*10, 0, 0]))
                for index, name in enumerate(('base', 'a', 'b', 'c'))]

    def test_overlapping_groups_and_fixed_connections_export_zero_dofs(self):
        base, a, b, c = self.assembly()
        component = root([base, a, b, c], joints=[fixed('mount', c, b)],
            groups=[group('base_to_a', [base, a]), group('overlap', [a, b])])
        robot, destination = self.export(component)
        self.assertEqual(len(robot.findall('link')), 4)
        self.assertEqual([joint.get('type') for joint in robot.findall('joint')], ['fixed']*3)
        self.assertIsNone(robot.find('ros2_control'))
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        provenance = [entry for joint in sidecar['joints'].values() for entry in joint.get('rigid_constraints', [])]
        self.assertIn('mount', [entry['name'] for entry in provenance])
        self.assertIn('overlap', [entry['name'] for entry in provenance])

    def test_group_on_movable_attachment_remains_connected_and_order_independent(self):
        observed = []
        for reverse in (False, True):
            base, a, b, c = self.assembly()
            joints = [hinge('drive', b, base), fixed('existing_mount', c, a)]
            groups = [group('pair', [a, b]), group('pair_overlap', [b, c])]
            if reverse:
                joints.reverse()
                groups.reverse()
            robot, _ = self.export(root([c, b, a, base] if reverse else [base, a, b, c],
                                        joints=joints, groups=groups))
            observed.append(sorted((j.get('name'), j.get('type'), j.find('parent').get('link'),
                                    j.find('child').get('link')) for j in robot.findall('joint')))
            self.assertEqual([j.get('name') for j in robot.findall('ros2_control/joint')], ['drive'])
            self.assertEqual(len(robot.findall('joint')), 3)
            for joint in robot.findall('joint'):
                if joint.get('type') == 'fixed':
                    self.assertEqual(joint.find('parent').get('link'), 'b_1')
        self.assertEqual(observed[0], observed[1])

    def test_active_internal_motion_and_coupled_rigid_loop_are_named_failures(self):
        base, a, b, c = self.assembly()
        internal = root([base, a], joints=[hinge('forbidden_motion', a, base)],
                        groups=[group('clamp', [base, a])])
        with self.assertRaisesRegex(ValueError, 'RIGID_GROUP_INTERNAL_MOTION.*forbidden_motion.*clamp'):
            self.export(internal)
        coupled = root([base, a, b], joints=[hinge('left_drive', a, base), hinge('right_drive', b, base)],
                       groups=[group('coupler', [a, b])])
        with self.assertRaisesRegex(ValueError, 'RIGID_GROUP_KINEMATIC_LOOP.*left_drive.*right_drive'):
            self.export(coupled)

    def test_bodyless_group_container_expands_to_physical_descendants(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        container = occurrence('assembly:1')
        a = occurrence('assembly:1+a:1', [Body('a', 10)])
        b = occurrence('assembly:1+b:1', [Body('b', 20)])
        robot, _ = self.export(root([base, container, a, b], groups=[group('assembly_clamp', [base, container])]))
        self.assertEqual([j.get('type') for j in robot.findall('joint')], ['fixed', 'fixed'])
        self.assertEqual(len(robot.findall('link')), 3)

    def test_group_container_cannot_silently_drop_hidden_physical_descendants(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        container = occurrence('assembly:1')
        a = occurrence('assembly:1+a:1', [Body('a', 10)])
        b = occurrence('assembly:1+b:1', [Body('b', 20)])
        b.isVisible = False
        component = root([base, container, a, b], groups=[group('partly_hidden', [base, container])])
        with self.assertRaisesRegex(ValueError, 'CONSTRAINT_TARGET_EXCLUDED.*partly_hidden.*assembly:1.b:1'):
            self.export(component)

    def test_native_bodyless_group_container_requires_unambiguous_context(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        native = occurrence('assembly:1')
        left, right = occurrence('left:1+assembly:1'), occurrence('right:1+assembly:1')
        left.nativeObject = right.nativeObject = native
        a = occurrence('left:1+assembly:1+a:1', [Body('a', 10)])
        b = occurrence('right:1+assembly:1+b:1', [Body('b', 20)])
        clamp = group('context_clamp', [base, native])
        component = root([base, left, right, a, b], joints=[fixed('other_mount', b, base)], groups=[clamp])
        with self.assertRaisesRegex(ValueError, 'context_clamp.*AMBIGUOUS_CONSTRAINT_TARGET.*assembly:1'):
            self.export(component)
        clamp.assemblyContext = types.SimpleNamespace(fullPathName='left:1')
        robot, _ = self.export(component)
        self.assertEqual(len(robot.findall('link')), 3)
        self.assertEqual([j.get('type') for j in robot.findall('joint')], ['fixed', 'fixed'])

    def test_suppressed_group_does_not_remove_active_dof(self):
        base, a, _, _ = self.assembly()
        robot, _ = self.export(root([base, a], joints=[hinge('drive', a, base)],
            groups=[group('disabled_clamp', [base, a], suppressed=True)]))
        self.assertEqual(robot.find('joint').get('type'), 'continuous')
        self.assertEqual(robot.find('ros2_control/joint').get('name'), 'drive')

    def test_unrelated_ordinary_loop_remains_an_explicit_reduction(self):
        base, a, b, c = self.assembly()
        robot, destination = self.export(root([base, a, b, c],
            joints=[hinge('first', a, base), hinge('second', b, a), hinge('closing', b, base)],
            groups=[group('fixed_accessory', [base, c])]))
        self.assertEqual(len(robot.findall('joint')), 3)
        self.assertEqual(len(robot.findall('ros2_control/joint')), 2)
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        self.assertEqual(len(sidecar['closing_joints']), 1)
        self.assertIn(sidecar['closing_joints'][0]['joint'], ('first', 'second', 'closing'))


class InputGapTest(ExportFixture, unittest.TestCase):
    def test_active_motion_links_and_bodyless_constraint_frames_are_named(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        child = occurrence('arm:1', [Body('arm', 10)])
        component = root([base, child], joints=[hinge('drive', child, base)])
        component.motionLinks = [types.SimpleNamespace(name='gear_pair', isSuppressed=False)]
        with self.assertRaisesRegex(ValueError, 'MOTION_LINK_NOT_IMPLEMENTED.*gear_pair'):
            self.export(component)
        component.motionLinks[0].isSuppressed = True
        robot, _ = self.export(component)
        self.assertEqual(robot.find('joint').get('type'), 'continuous')
        child.component.motionLinks = [types.SimpleNamespace(name='nested_gears', isSuppressed=False)]
        with self.assertRaisesRegex(ValueError, 'MOTION_LINK_NOT_IMPLEMENTED.*nested_gears'):
            self.export(component)
        child.component.motionLinks = []
        frame = occurrence('reference_frame:1')
        component.allOccurrences.append(frame)
        component.allJoints = [fixed('frame_mount', frame, base)]
        with self.assertRaisesRegex(ValueError, 'frame_mount.*BODYLESS_FRAME_NOT_IMPLEMENTED.*reference_frame'):
            self.export(component)

    def test_bodyless_design_root_constraint_is_an_explicit_input_gap(self):
        child = occurrence('arm:1', [Body('arm', 10)])
        with self.assertRaisesRegex(ValueError, 'root_mount.*BODYLESS_FRAME_NOT_IMPLEMENTED.*design_root'):
            self.export(root([child], joints=[fixed('root_mount', child, None)]))

    def test_locked_joint_exports_fixed_current_pose_and_original_type_in_sidecar(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        child = occurrence('arm:1', [Body('arm', 10)])
        joint = hinge('locked_drive', child, base)
        joint.isLocked = True
        joint.geometryTwoTransform = FakeMatrix(identity_at([10, 20, 30]))
        robot, destination = self.export(root([base, child], joints=[joint]))
        self.assertEqual(robot.find('joint').get('type'), 'fixed')
        self.assertEqual(robot.find('joint/origin').get('xyz'), '0.1 0.2 0.3')
        self.assertIsNone(robot.find('ros2_control'))
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        self.assertEqual(sidecar['joints']['locked_drive']['original_motion']['type'], 'continuous')
        self.assertIn('locked', sidecar['joints']['locked_drive']['fixed_reason'])

    def test_locked_slider_without_travel_bounds_has_no_moving_dof(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        child = occurrence('carriage:1', [Body('carriage', 10)])
        joint = fixed('locked_slider', child, base, (10, 0, 0))
        joint.jointMotion = types.SimpleNamespace(jointType=2)
        joint.isLocked = True
        robot, destination = self.export(root([base, child], joints=[joint]))
        self.assertEqual(robot.find('joint').get('type'), 'fixed')
        self.assertIsNone(robot.find('ros2_control'))
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        self.assertEqual(sidecar['joints']['locked_slider']['original_motion']['type'], 'prismatic')

    def test_locked_generalized_joint_is_fixed_but_active_form_is_not_decomposed(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        child = occurrence('ball:1', [Body('ball', 10)])
        joint = fixed('ball_socket', child, base)
        joint.jointMotion = types.SimpleNamespace(jointType=6)
        joint.isLocked = True
        component = root([base, child], joints=[joint])
        robot, destination = self.export(component)
        self.assertEqual(robot.find('joint').get('type'), 'fixed')
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        self.assertEqual(sidecar['joints']['ball_socket']['original_motion']['fusion_type'], 'Ball')
        joint.isLocked = False
        with self.assertRaisesRegex(ValueError, 'UNSUPPORTED_FUSION_JOINT_TYPE.*ball_socket.*Ball'):
            self.export(component)

    def test_excluded_constraints_are_reported_and_crossing_constraints_fail_by_name(self):
        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        a, b = occurrence('hidden_a:1', [Body('a', 10)]), occurrence('hidden_b:1', [Body('b', 20)])
        a.isVisible = b.isVisible = False
        joint = fixed('hidden_mount', b, a)
        component = root([base, a, b], joints=[joint], groups=[group('hidden_group', [a, b])])
        robot, destination = self.export(component)
        self.assertEqual([link.get('name') for link in robot.findall('link')], ['base_link'])
        sidecar = json.loads((destination / 'config/robot_data.yaml').read_text())
        self.assertEqual(sorted(note['name'] for note in sidecar['source_notes']), ['hidden_group', 'hidden_mount'])
        joint.occurrenceTwo = base
        with self.assertRaisesRegex(ValueError, 'hidden_mount.*CONSTRAINT_TARGET_EXCLUDED.*hidden_b'):
            self.export(component)

    def test_native_repeated_endpoint_needs_context_and_uses_authoritative_proxy_pose(self):
        class WrappedOccurrence(types.SimpleNamespace):
            def __eq__(self, other):
                return isinstance(other, WrappedOccurrence) and self.api_identity == other.api_identity

        base = occurrence('base:1', [Body('base', 0)], component='base_link')
        native = WrappedOccurrence(**vars(occurrence('arm:1', [Body('native', 99)])), api_identity='native-arm')
        left = occurrence('left:1+arm:1', [Body('left', 10)], matrix=identity_at([10, 0, 0]))
        right = occurrence('right:1+arm:1', [Body('right', 20)], matrix=identity_at([20, 0, 0]))
        left.nativeObject = WrappedOccurrence(**vars(native))
        right.nativeObject = WrappedOccurrence(**vars(native))
        left_mount = fixed('left_mount', native, base)
        left_mount.assemblyContext = types.SimpleNamespace(fullPathName='left:1')
        left_mount.transform = None
        left_mount.geometry = None
        right_mount = fixed('right_mount', right, base)
        component = root([base, left, right], joints=[right_mount])
        component.allAsBuiltJoints = [left_mount]
        robot, _ = self.export(component)
        self.assertEqual(robot.find("joint[@name='left_1_left_mount']/origin").get('xyz'), '0.1 0 0')
        left_mount.assemblyContext = None
        with self.assertRaisesRegex(ValueError, 'left_mount.*AMBIGUOUS_CONSTRAINT_TARGET.*arm:1'):
            self.export(component)


if __name__ == '__main__':
    unittest.main()
