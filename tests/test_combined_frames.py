"""Analytic API-double exports, not reproductions of original issue CAD.

All input lengths are Fusion internal centimeters. Changing defaultLengthUnits
below changes a label only, not a native document's unit system or geometry.
"""

import importlib
import itertools
import math
import os
from pathlib import Path
import struct
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakePoint
from tests.test_fusion_inputs import design, fixed, occurrence, root


ROOT_BOXES = (((10, 20, 30), (185.42, 12, 16), 2),
              ((-20, -10, 5), (6, 10, 14), 3))
WHEEL_BOXES = (((7, -3, 4), (8, 10, 6), 1.25),
               ((-4, 5, -2), (5, 7, 9), .75))
SLIDER_BOXES = (((2, 4, 7), (3, 5, 9), .7),)
BOXES = {'base_link': ROOT_BOXES, 'left_1_wheel_1': WHEEL_BOXES,
         'right_1_wheel_1': WHEEL_BOXES, 'left_1_wheel_1_slider_1': SLIDER_BOXES}
CURRENT_ANGLE = math.pi / 6
SAMPLES = ({'left_hinge': 0, 'right_hinge': 0, 'slide': 0},
           {'left_hinge': .37, 'right_hinge': -.28, 'slide': .031})
TRIANGLES = ((0, 3, 2), (0, 2, 1), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def times(scale, vector):
    return tuple(scale * value for value in vector)


def turn_y(point, angle):
    x, y, z = point
    return (math.cos(angle)*x + math.sin(angle)*z, y,
            -math.sin(angle)*x + math.cos(angle)*z)


def analytic_point_cm(link, point, sample=SAMPLES[0]):
    """Mechanism specified independently of the URDF's chosen link frames."""
    x, y, z = point
    if link == 'base_link':
        return add((40, -30, 20), (x, -z, y))
    if link == 'left_1_wheel_1':
        unposed = add((80, 15, -5), (-y, x, z))
    elif link == 'right_1_wheel_1':
        unposed = add((-35, 40, 18), (z, y, -x))
        pivot = (-15, 25, -10)
        return add(pivot, turn_y(subtract(unposed, pivot),
                                -CURRENT_ANGLE - sample['right_hinge']))
    else:
        unposed = add((95, 20, 10), (x, -z, y))
        unposed = subtract(unposed, (7 + 100*sample['slide'], 0, 0))
    pivot = (62, -5, 12)
    return add(pivot, turn_y(subtract(unposed, pivot),
                            CURRENT_ANGLE + sample['left_hinge']))


def analytic_pivot_cm(joint, sample):
    if joint == 'left_hinge':
        return (62, -5, 12)
    if joint == 'right_hinge':
        return (-15, 25, -10)
    pivot = (62, -5, 12)
    unposed = (90 - 7 - 100*sample['slide'], -20, 6)
    return add(pivot, turn_y(subtract(unposed, pivot),
                            CURRENT_ANGLE + sample['left_hinge']))


def analytic_axes(link, sample):
    origin = analytic_point_cm(link, (0, 0, 0), sample)
    return [subtract(analytic_point_cm(link, axis, sample), origin)
            for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]


def corners(center, size):
    return [add(center, tuple(s*d/2 for s, d in zip(signs, size)))
            for signs in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                          (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))]


def binary_box(center, size):
    vertices = [times(10, point) for point in corners(center, size)]
    data = bytearray(b'analytic uniform box, mm'.ljust(80, b'\0') + struct.pack('<I', 12))
    for indices in TRIANGLES:
        a, b, c = [vertices[index] for index in indices]
        u, v = subtract(b, a), subtract(c, a)
        normal = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
        normal = times(1 / math.sqrt(sum(value*value for value in normal)), normal)
        data.extend(struct.pack('<12fH', *normal, *a, *b, *c, 0))
    return bytes(data)


class PhysicalBody:
    def __init__(self, name, link, box, native):
        center, size, mass = box
        self.name = name
        self.nativeObject = native
        self.isVisible = True
        self.isValid = True
        self.isSolid = True
        points = [analytic_point_cm(link, p) for p in corners(center, size)]
        self.boundingBox = types.SimpleNamespace(
            minPoint=FakePoint([min(p[i] for p in points) for i in range(3)]),
            maxPoint=FakePoint([max(p[i] for p in points) for i in range(3)]))
        # Two-point Gaussian quadrature integrates a uniform box's second
        # moments exactly, without using the exporter's parallel-axis formula.
        nodes = [analytic_point_cm(link, add(center, tuple(s*d/(2*math.sqrt(3))
                 for s, d in zip(signs, size))))
                 for signs in itertools.product((-1, 1), repeat=3)]
        tensor = [[sum(mass/8 * ((sum(v*v for v in p) if i == j else 0) - p[i]*p[j])
                       for p in nodes) for j in range(3)] for i in range(3)]
        self.moments = (True, tensor[0][0], tensor[1][1], tensor[2][2],
                        tensor[0][1], tensor[1][2], tensor[0][2])
        self.properties = types.SimpleNamespace(mass=mass,
            centerOfMass=FakePoint(analytic_point_cm(link, center)),
            getXYZMomentsOfInertia=lambda: self.moments)

    def getPhysicalProperties(self, accuracy):
        return self.properties


def analytic_scene(label='cm', fail_after=None):
    native_sets = {name: [types.SimpleNamespace(name=name + '_' + str(i),
                    mesh=binary_box(center, size))
                    for i, (center, size, _) in enumerate(boxes)]
                   for name, boxes in (('root', ROOT_BOXES), ('wheel', WHEEL_BOXES),
                                       ('slider', SLIDER_BOXES))}
    paths = {'base_link': 'assembly:1+anchor:1', 'left_1_wheel_1': 'left:1+wheel:1',
             'right_1_wheel_1': 'right:1+wheel:1',
             'left_1_wheel_1_slider_1': 'left:1+wheel:1+slider:1'}
    owners = {}
    for link, boxes in BOXES.items():
        kind = 'root' if link == 'base_link' else 'slider' if 'slider' in link else 'wheel'
        bodies = [PhysicalBody(kind + '_' + str(i), link, box, native_sets[kind][i])
                  for i, box in enumerate(boxes)]
        t, axes = analytic_point_cm(link, (0, 0, 0)), analytic_axes(link, SAMPLES[0])
        matrix = [value for i in range(3) for value in
                  (axes[0][i], axes[1][i], axes[2][i], t[i])] + [0, 0, 0, 1]
        owners[link] = occurrence(paths[link], bodies, component=kind + ' (v7)', matrix=matrix)
    owners['right_1_wheel_1'].component = owners['left_1_wheel_1'].component
    base, left, right, slider = [owners[name] for name in BOXES]
    left_joint = fixed('left_hinge', left, base, analytic_pivot_cm('left_hinge', SAMPLES[0]))
    right_joint = fixed('right_hinge', base, right, analytic_pivot_cm('right_hinge', SAMPLES[0]))
    for joint in (left_joint, right_joint):
        joint.jointMotion = types.SimpleNamespace(jointType=1,
            rotationAxisVector=FakePoint((0, 1, 0)), rotationValue=CURRENT_ANGLE,
            rotationLimits=types.SimpleNamespace(isMinimumValueEnabled=True,
                isMaximumValueEnabled=True, minimumValue=-math.pi/2, maximumValue=math.pi/2))
    slide = fixed('slide', left, slider, analytic_pivot_cm('slide', SAMPLES[0]))
    slide.jointMotion = types.SimpleNamespace(jointType=2, slideValue=7,
        slideDirectionVector=FakePoint(turn_y((1, 0, 0), CURRENT_ANGLE)),
        slideLimits=types.SimpleNamespace(isMinimumValueEnabled=True,
            isMaximumValueEnabled=True, minimumValue=0, maximumValue=20))
    for joint in (left_joint, right_joint, slide):
        joint.geometryOrOriginOne.origin = FakePoint((999, 888, 777))
        joint.geometryOrOriginTwo.origin = FakePoint((-999, -888, -777))
    component = root([right, slider, base, left], joints=[slide, right_joint, left_joint])
    fixture, exported = design(component)
    fixture.unitsManager = types.SimpleNamespace(defaultLengthUnits=label)
    fixture.timeline = types.SimpleNamespace(markerPosition=7, entityTokens=('sketch', 'extrude', 'joint-edit'))
    execute = fixture.exportManager.execute

    def export(option):
        if fail_after is not None and len(exported) == fail_after:
            return False
        return execute(option)

    fixture.exportManager.execute = export
    return fixture, exported, {'root_occurrence': paths['base_link']}


def source_snapshot(fixture):
    seen = set()

    def freeze(value):
        if isinstance(value, (str, int, float, bool, bytes, type(None))):
            return value
        identity = id(value)
        if identity in seen:
            return ('reference', identity)
        seen.add(identity)
        if isinstance(value, (list, tuple)):
            return (identity, tuple(freeze(item) for item in value))
        if callable(value):
            return ('callable', identity)
        return (identity, type(value).__name__,
                tuple((name, freeze(item)) for name, item in sorted(vars(value).items())))

    return (freeze(fixture.rootComponent), freeze(fixture.unitsManager), freeze(fixture.timeline))


def rpy_apply(point, rpy):
    x, y, z = point
    roll, pitch, yaw = rpy
    y, z = math.cos(roll)*y - math.sin(roll)*z, math.sin(roll)*y + math.cos(roll)*z
    x, z = math.cos(pitch)*x + math.sin(pitch)*z, -math.sin(pitch)*x + math.cos(pitch)*z
    return (math.cos(yaw)*x - math.sin(yaw)*y, math.sin(yaw)*x + math.cos(yaw)*y, z)


def vector(element, attribute):
    return tuple(map(float, element.get(attribute).split()))


def origin_apply(point, element):
    return add(vector(element, 'xyz'), rpy_apply(point, vector(element, 'rpy')))


def axis_rotate(point, axis, angle):
    dot = sum(a*b for a, b in zip(axis, point))
    cross = (axis[1]*point[2] - axis[2]*point[1],
             axis[2]*point[0] - axis[0]*point[2], axis[0]*point[1] - axis[1]*point[0])
    return tuple(math.cos(angle)*p + math.sin(angle)*c + (1 - math.cos(angle))*dot*a
                 for p, c, a in zip(point, cross, axis))


class ExportedRobot:
    def __init__(self, destination):
        self.destination = destination
        self.xml = ET.parse(destination / 'urdf/combined.urdf').getroot()
        self.links = {link.get('name'): link for link in self.xml.findall('link')}
        self.joints = {joint.get('name'): joint for joint in self.xml.findall('joint')}
        self.incoming = {joint.find('child').get('link'): joint for joint in self.joints.values()}

    def world(self, link, point, sample):
        if link not in self.incoming:
            return point
        joint = self.incoming[link]
        q, axis = sample[joint.get('name')], vector(joint.find('axis'), 'xyz')
        if joint.get('type') == 'prismatic':
            point = add(point, times(q, axis))
        else:
            point = axis_rotate(point, axis, q)
        point = origin_apply(point, joint.find('origin'))
        return self.world(joint.find('parent').get('link'), point, sample)

    def geometry(self, link, kind, sample):
        element = self.links[link].find(kind)
        mesh = element.find('geometry/mesh')
        filename = mesh.get('filename').split('/meshes/')[1]
        data = (self.destination / 'meshes' / filename).read_bytes()
        count, = struct.unpack_from('<I', data, 80)
        if len(data) != 84 + count*50:
            raise AssertionError('Binary STL length does not match triangle count')
        scale = vector(mesh, 'scale')
        points = []
        for index in range(count):
            values = struct.unpack_from('<12fH', data, 84 + index*50)
            for offset in (3, 6, 9):
                point = tuple(p*s for p, s in zip(values[offset:offset+3], scale))
                points.append(self.world(link, origin_apply(point, element.find('origin')), sample))
        return count, points


def expected_physics(link, sample):
    boxes = BOXES[link]
    mass = sum(box[2] for box in boxes)
    center = tuple(sum(m*c[i] for c, _, m in boxes) / mass for i in range(3))
    world_center = times(.01, analytic_point_cm(link, center, sample))
    axes = analytic_axes(link, sample)
    tensor = [[0.0]*3 for _ in range(3)]
    for c, size, m in boxes:
        principal = [m * sum((size[j]*.01)**2 for j in range(3) if j != i) / 12
                     for i in range(3)]
        displacement = times(.01, subtract(analytic_point_cm(link, c, sample),
                                          analytic_point_cm(link, center, sample)))
        for i in range(3):
            for j in range(3):
                tensor[i][j] += sum(principal[k]*axes[k][i]*axes[k][j] for k in range(3))
                tensor[i][j] += m * ((sum(v*v for v in displacement) if i == j else 0)
                                      - displacement[i]*displacement[j])
    return mass, world_center, tensor


class CombinedFrameTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        artifacts = os.environ.get('FRAMES_ARTIFACT_DIR')
        if artifacts:
            cls.output = Path(tempfile.mkdtemp(prefix='combined-frames-', dir=artifacts))
        else:
            temporary = tempfile.TemporaryDirectory(prefix='combined-frames-')
            cls.addClassCleanup(temporary.cleanup)
            cls.output = Path(temporary.name)
        cls.entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        cls.fixture, cls.exported, config = analytic_scene()
        cls.before = source_snapshot(cls.fixture)
        destination = cls.output / 'cm' / 'combined_description'
        cls.entry.export_design(cls.fixture, destination, config=config)
        cls.robot = ExportedRobot(destination)

    def assertVector(self, actual, expected, tolerance=2e-12):
        self.assertEqual(len(actual), len(expected))
        for i, (got, want) in enumerate(zip(actual, expected)):
            self.assertAlmostEqual(got, want, delta=tolerance, msg='coordinate %d' % i)

    def check_geometry(self, robot):
        self.assertEqual(set(robot.links), set(BOXES))
        for sample in SAMPLES:
            for link, boxes in BOXES.items():
                expected = [times(.01, analytic_point_cm(link, p, sample))
                            for center, size, _ in boxes for p in corners(center, size)]
                for kind in ('visual', 'collision'):
                    with self.subTest(sample=sample, link=link, geometry=kind):
                        count, actual = robot.geometry(link, kind, sample)
                        self.assertEqual(count, len(boxes)*12)
                        for bound in (min, max):
                            self.assertVector([bound(p[i] for p in actual) for i in range(3)],
                                              [bound(p[i] for p in expected) for i in range(3)], 2e-7)
                        for source, target in ((actual, expected), (expected, actual)):
                            for point in source:
                                self.assertLessEqual(min(max(abs(a-b) for a, b in zip(point, other))
                                                         for other in target), 2e-7)
        _, points = robot.geometry('base_link', 'visual', SAMPLES[0])
        self.assertAlmostEqual(max(p[0] for p in points) - min(p[0] for p in points),
                               1.8542, delta=2e-7)

    def check_physics(self, robot):
        for sample in SAMPLES:
            for link in BOXES:
                with self.subTest(sample=sample, link=link, physics=True):
                    element = robot.links[link].find('inertial')
                    mass, center, expected_tensor = expected_physics(link, sample)
                    self.assertAlmostEqual(float(element.find('mass').get('value')), mass, delta=1e-14)
                    self.assertVector(robot.world(link, vector(element.find('origin'), 'xyz'), sample), center)
                    inertia = element.find('inertia')
                    tensor = [[float(inertia.get(key)) for key in row] for row in
                              (('ixx', 'ixy', 'ixz'), ('ixy', 'iyy', 'iyz'), ('ixz', 'iyz', 'izz'))]
                    frame_origin = robot.world(link, (0, 0, 0), sample)
                    axes = [subtract(robot.world(link, rpy_apply(axis,
                            vector(element.find('origin'), 'rpy')), sample), frame_origin)
                            for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
                    for i in range(3):
                        for j in range(i, 3):
                            actual = sum(axes[k][i]*tensor[k][l]*axes[l][j]
                                         for k in range(3) for l in range(3))
                            self.assertAlmostEqual(actual, expected_tensor[i][j], delta=2e-12,
                                                   msg='tensor term %d,%d' % (i, j))

    def test_offset_pivots_mirrored_axes_reversed_slider_and_mesh_fk(self):
        self.check_geometry(self.robot)
        self.assertEqual(set(self.robot.joints), {'left_hinge', 'right_hinge', 'slide'})
        for name, joint in self.robot.joints.items():
            expected_limits = (-.07, .13) if name == 'slide' else (-2*math.pi/3, math.pi/3)
            self.assertVector([float(joint.find('limit').get(key)) for key in ('lower', 'upper')],
                              expected_limits)
            for sample in SAMPLES:
                with self.subTest(joint=name, sample=sample):
                    child = joint.find('child').get('link')
                    pivot = self.robot.world(child, (0, 0, 0), sample)
                    self.assertVector(pivot, times(.01, analytic_pivot_cm(name, sample)))
                    axis = subtract(self.robot.world(child, vector(joint.find('axis'), 'xyz'), sample), pivot)
                    expected_axis = ((0, 1, 0) if name == 'left_hinge' else (0, -1, 0)
                                     if name == 'right_hinge' else
                                     turn_y((-1, 0, 0), CURRENT_ANGLE + sample['left_hinge']))
                    self.assertVector(axis, expected_axis)

    def test_multibody_signed_com_and_all_six_tensor_terms(self):
        self.check_physics(self.robot)
        _, center, tensor = expected_physics('base_link', SAMPLES[0])
        self.assertVector(center, (.32, -.45, .22))
        self.assertTrue(all(abs(tensor[i][j]) > 1e-6 for i, j in ((0, 1), (0, 2), (1, 2))))

    def test_document_unit_labels_do_not_rescale_internal_centimeters(self):
        for label in ('mm', 'm', 'in'):
            with self.subTest(document_unit_label=label):
                fixture, _, config = analytic_scene(label)
                before = source_snapshot(fixture)
                destination = self.output / label / 'combined_description'
                self.entry.export_design(fixture, destination, config=config)
                self.assertEqual(source_snapshot(fixture), before)
                robot = ExportedRobot(destination)
                self.check_geometry(robot)
                self.check_physics(robot)

    def test_source_graph_unchanged_on_success_and_mid_mesh_failure(self):
        self.assertEqual(source_snapshot(self.fixture), self.before)
        self.assertEqual(len(self.exported), 7)
        fixture, exported, config = analytic_scene(fail_after=3)
        before = source_snapshot(fixture)
        destination = self.output / 'failure' / 'combined_description'
        with self.assertRaisesRegex(RuntimeError, 'Fusion mesh export failed'):
            self.entry.export_design(fixture, destination, config=config)
        self.assertEqual(len(exported), 3)
        self.assertFalse(destination.exists())
        self.assertEqual(list(destination.parent.iterdir()), [])
        self.assertEqual(source_snapshot(fixture), before)


if __name__ == '__main__':
    unittest.main()
