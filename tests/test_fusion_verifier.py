import copy
import importlib.util
import json
import math
from pathlib import Path
import struct
import tempfile
import types
import unittest
from unittest.mock import patch

from tools.verify_fusion.verification import (
    Box, Joint, QUARTER_TURN, Report, assert_entity_identity, assert_source_unchanged, bounds,
    box_corners, box_physics, capture_provenance, entity_evidence, loaded_code_provenance,
    owned_document, stl_vertices, verify_package,
)


ROBOT = '''<robot name="literal">
  <link name="base_link">
    <inertial>
      <origin xyz="0.03 0.05 0.07"/>
      <mass value="2"/>
      <inertia ixx="0.0008666666666666667" iyy="0.0006666666666666667" izz="0.0003333333333333333" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual><geometry><mesh filename="package://literal_description/meshes/base_link.stl" scale="0.001 0.001 0.001"/></geometry></visual>
    <collision><geometry><mesh filename="package://literal_description/meshes/base_link.stl" scale="0.001 0.001 0.001"/></geometry></collision>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.02 0.03 0.04"/>
      <mass value="3"/>
      <inertia ixx="0.00185" iyy="0.00145" izz="0.00085" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual><origin xyz="-0.2 0.1 -0.3" rpy="0 0 -1.5707963267948966"/><geometry><mesh filename="package://literal_description/meshes/arm.stl" scale="0.001 0.001 0.001"/></geometry></visual>
    <collision><origin xyz="-0.2 0.1 -0.3" rpy="0 0 -1.5707963267948966"/><geometry><mesh filename="package://literal_description/meshes/arm.stl" scale="0.001 0.001 0.001"/></geometry></collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base_link"/><child link="arm"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="2" effort="10" velocity="1"/>
  </joint>
</robot>'''


def write_box_stl(path, low, high):
    vertices = [(x, y, z) for x in (low[0], high[0])
                for y in (low[1], high[1]) for z in (low[2], high[2])]
    faces = ((0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),
             (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
             (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3))
    payload = bytearray(b'Independent literal box'.ljust(80, b'\0') + struct.pack('<I', 12))
    for face in faces:
        coordinates = [v for index in face for v in vertices[index]]
        payload.extend(struct.pack('<12fH', 0, 0, 0, *coordinates, 0))
    path.write_bytes(payload)


def literal_package(directory, xml=ROBOT):
    package = Path(directory) / 'literal_description'
    (package / 'urdf').mkdir(parents=True)
    (package / 'meshes').mkdir()
    (package / 'urdf/literal.xacro').write_text(xml, encoding='utf-8')
    write_box_stl(package / 'meshes/base_link.stl', (20, 30, 40), (40, 70, 100))
    write_box_stl(package / 'meshes/arm.stl', (45, 205, 305), (95, 235, 375))
    boxes = {'base_link': [Box((.02, .04, .06), (.03, .05, .07), mass_kg=2)],
             'arm': [Box((.03, .05, .07), (.07, .22, .34), QUARTER_TURN, 3)]}
    cad = {'base_link': ((.02, .03, .04), (.04, .07, .1)),
           'arm': ((.045, .205, .305), (.095, .235, .375))}
    joints = [Joint('hinge', 'base_link', 'arm', 'revolute', limits=(-1, 2),
                    pivot_world=(.1, .2, .3), sample_position=math.pi/2)]
    return package, boxes, cad, joints


def load_entrypoint_without_sdk():
    adsk = types.ModuleType('adsk')
    adsk.core, adsk.fusion = types.ModuleType('adsk.core'), types.ModuleType('adsk.fusion')
    adsk.fusion.DesignTypes = types.SimpleNamespace(ParametricDesignType=1)
    spec = importlib.util.spec_from_file_location('tools.verify_fusion._snapshot_test',
                Path(__file__).resolve().parents[1] / 'tools/verify_fusion/verify_fusion.py')
    native = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', {'adsk': adsk, 'adsk.core': adsk.core, 'adsk.fusion': adsk.fusion}):
        spec.loader.exec_module(native)
    return native


class VerificationMathTest(unittest.TestCase):
    def test_rotated_box_bounds_are_literal(self):
        box = Box((2, 4, 6), (10, 20, 30), QUARTER_TURN)
        self.assertEqual(bounds(box_corners(box)), ((8, 19, 27), (12, 21, 33)))

    def test_rotated_centroid_inertia_and_parallel_axis_sum(self):
        self.assertEqual(box_physics([Box((2, 4, 6), (10, 20, 30), QUARTER_TURN, 12)]),
                         (12, (10, 20, 30), (40, 0, 0, 0, 52, 0, 0, 0, 20)))
        mass, center, inertia = box_physics([Box((2, 2, 2), (-1, -1, 0), mass_kg=3),
                                            Box((2, 2, 2), (1, 1, 0), mass_kg=3)])
        self.assertEqual((mass, center, inertia), (6, (0, 0, 0), (10, -6, 0, -6, 10, 0, 0, 0, 16)))

    def test_literal_rotated_package_checks_real_mesh_bytes_and_fk(self):
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            evidence = verify_package(package, boxes, cad, joints)
            result = next(e for e in evidence if e['name'] == 'arm zero-pose visual world bounds')
            self.assertEqual(result['status'], 'pass')
            for actual, expected in zip(result['actual'][0], (.045, .205, .305)):
                self.assertAlmostEqual(actual, expected, places=8)
            inertia = next(e for e in evidence if e['name'] == 'arm world centroid inertia in kg m^2')
            for actual, expected in zip(inertia['actual'], (.00145, 0, 0, 0, .00185, 0, 0, 0, .00085)):
                self.assertAlmostEqual(actual, expected, places=12)

    def test_wrong_geometry_and_physics_cannot_pass(self):
        mutations = (
            ('visual pose', 'xyz="-0.2 0.1 -0.3"', 'xyz="-0.1 0.1 -0.3"'),
            ('scale', 'scale="0.001 0.001 0.001"', 'scale="1 1 1"'),
            ('axis', 'xyz="0 0 1"', 'xyz="1 0 0"'),
            ('range', 'lower="-1"', 'lower="0"'),
            ('COM', 'xyz="0.02 0.03 0.04"', 'xyz="0 0 0"'),
            ('mass', 'mass value="3"', 'mass value="30"'),
            ('inertia', 'ixx="0.00185"', 'ixx="0.0185"'),
            ('joint type', 'type="revolute"', 'type="fixed"'),
        )
        for name, old, new in mutations:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                arguments = literal_package(directory, ROBOT.replace(old, new))
                with self.assertRaises(AssertionError):
                    verify_package(*arguments)
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            write_box_stl(package / 'meshes/arm.stl', (0, 0, 0), (50, 30, 70))
            with self.assertRaisesRegex(AssertionError, 'raw root-frame STL bounds'):
                verify_package(package, boxes, cad, joints)

    def test_degenerate_triangles_with_correct_bounds_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            for path in (package / 'meshes').glob('*.stl'):
                low, high = bounds(stl_vertices(path))
                data = bytearray(path.read_bytes())
                for offset in range(84, len(data), 50):
                    struct.pack_into('<9f', data, offset + 12, *low, *high, *low)
                path.write_bytes(data)
            with self.assertRaisesRegex(AssertionError, 'degenerate facet'):
                verify_package(package, boxes, cad, joints)

    def test_compensated_perpendicular_hinge_shift_is_rejected(self):
        shifted = ROBOT.replace('xyz="0.1 0.2 0.3"', 'xyz="0.2 0.2 0.3"')
        shifted = shifted.replace('xyz="-0.2 0.1 -0.3"', 'xyz="-0.2 0.2 -0.3"')
        shifted = shifted.replace('xyz="0.02 0.03 0.04"', 'xyz="0.02 0.13 0.04"')
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(AssertionError, 'world axis line'):
                verify_package(*literal_package(directory, shifted))

    def test_equivalent_hinge_axis_line_and_literal_nonzero_fk(self):
        shifted = ROBOT.replace('xyz="0.1 0.2 0.3"', 'xyz="0.1 0.2 0.4"')
        shifted = shifted.replace('xyz="-0.2 0.1 -0.3"', 'xyz="-0.2 0.1 -0.4"')
        shifted = shifted.replace('xyz="0.02 0.03 0.04"', 'xyz="0.02 0.03 -0.06"')
        for xml in (ROBOT, shifted):
            with self.subTest(xml=xml), tempfile.TemporaryDirectory() as directory:
                evidence = verify_package(*literal_package(directory, xml))
                result = next(e for e in evidence if e['name'] == 'hinge sample arm visual moved world bounds')
                for actual, expected in zip(result['actual'][0] + result['actual'][1],
                                            (.065, .145, .305, .095, .195, .375)):
                    self.assertAlmostEqual(actual, expected, places=8)
                center = next(e for e in evidence if e['name'] == 'hinge sample arm moved world COM')
                for actual, expected in zip(center['actual'], (.08, .17, .34)):
                    self.assertAlmostEqual(actual, expected, places=8)

    def test_open_and_wrong_closed_surfaces_with_correct_bounds_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            path = package / 'meshes/arm.stl'
            data = bytearray(path.read_bytes()[:-50])
            struct.pack_into('<I', data, 80, 11)
            path.write_bytes(data)
            with self.assertRaisesRegex(AssertionError, 'closed oriented manifold'):
                verify_package(package, boxes, cad, joints)
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            path = package / 'meshes/arm.stl'
            corners = ((45, 205, 305), (95, 235, 305), (95, 205, 375), (45, 235, 375))
            data = bytearray(b'Closed tetrahedron'.ljust(80, b'\0') + struct.pack('<I', 4))
            for face in ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)):
                data.extend(struct.pack('<12fH', 0, 0, 0, *(v for i in face for v in corners[i]), 0))
            path.write_bytes(data)
            with self.assertRaisesRegex(AssertionError, 'analytic box surface ownership'):
                verify_package(package, boxes, cad, joints)

    def test_literal_slider_nonzero_fk(self):
        xml = ROBOT.replace('type="revolute"', 'type="prismatic"')
        xml = xml.replace('lower="-1" upper="2"', 'lower="-0.02" upper="0.04"')
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, _ = literal_package(directory, xml)
            joints = [Joint('hinge', 'base_link', 'arm', 'prismatic', limits=(-.02, .04), sample_position=.02)]
            evidence = verify_package(package, boxes, cad, joints)
            result = next(e for e in evidence if e['name'] == 'hinge sample arm collision moved world bounds')
            for actual, expected in zip(result['actual'][0] + result['actual'][1],
                                        (.045, .205, .325, .095, .235, .395)):
                self.assertAlmostEqual(actual, expected, places=8)
            center = next(e for e in evidence if e['name'] == 'hinge sample arm moved world COM')
            for actual, expected in zip(center['actual'], (.07, .22, .36)):
                self.assertAlmostEqual(actual, expected, places=8)

    def test_combined_revolute_slider_chain_uses_moved_parent_axis(self):
        tip = ROBOT[ROBOT.index('  <link name="arm">'):ROBOT.index('  <joint')]
        tip = tip.replace('name="arm"', 'name="tip"').replace('meshes/arm.stl', 'meshes/tip.stl')
        tip = tip.replace('xyz="0.02 0.03 0.04"', 'xyz="0.02 -0.07 0.04"')
        slide = '''<joint name="slide" type="prismatic"><parent link="arm"/><child link="tip"/>
          <axis xyz="0 -1 0"/><limit lower="-0.02" upper="0.04"/></joint>'''
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory, ROBOT.replace('</robot>', tip + slide + '</robot>'))
            write_box_stl(package / 'meshes/tip.stl', (145, 205, 305), (195, 235, 375))
            boxes['tip'] = [Box((.03, .05, .07), (.17, .22, .34), QUARTER_TURN, 3)]
            cad['tip'] = ((.145, .205, .305), (.195, .235, .375))
            joints.append(Joint('slide', 'arm', 'tip', 'prismatic', axis_world=(1, 0, 0),
                                limits=(-.02, .04), sample_position=.02))
            evidence = verify_package(package, boxes, cad, joints)
            result = next(e for e in evidence if e['name'] == 'combined sample tip visual moved world bounds')
            for actual, expected in zip(result['actual'][0] + result['actual'][1],
                                        (.065, .265, .305, .095, .315, .375)):
                self.assertAlmostEqual(actual, expected, places=8)
            center = next(e for e in evidence if e['name'] == 'combined sample tip moved world COM')
            for actual, expected in zip(center['actual'], (.08, .29, .34)):
                self.assertAlmostEqual(actual, expected, places=8)

    def test_closed_box_volume_is_checked_beyond_bounds_tolerance(self):
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            evidence = verify_package(package, boxes, cad, joints)
            volume = next(e for e in evidence if e['name'] == 'arm visual box 0 enclosed volume in m^3')
            self.assertAlmostEqual(volume['actual'], .000105, places=12)
            write_box_stl(package / 'meshes/arm.stl', (45.0015, 205, 305), (95, 235, 375))
            with self.assertRaisesRegex(AssertionError, 'enclosed volume'):
                verify_package(package, boxes, cad, joints)

    def test_duplicate_joint_and_incorrect_direct_cad_bounds_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            args = literal_package(directory, ROBOT.replace('</robot>',
                '<joint name="hinge" type="fixed"><parent link="base_link"/><child link="arm"/></joint></robot>'))
            with self.assertRaisesRegex(AssertionError, 'unique joint names'):
                verify_package(*args)
        with tempfile.TemporaryDirectory() as directory:
            package, boxes, cad, joints = literal_package(directory)
            cad['arm'] = ((0, 0, 0), (.05, .03, .07))
            with self.assertRaisesRegex(AssertionError, 'analytic versus direct CAD'):
                verify_package(package, boxes, cad, joints)

    def test_fixed_and_slider_joint_contracts(self):
        fixed_xml = ROBOT.replace('type="revolute"', 'type="fixed"')
        slider_xml = ROBOT.replace('type="revolute"', 'type="prismatic"').replace('lower="-1" upper="2"', 'lower="-0.02" upper="0.04"')
        for xml, expected in ((fixed_xml, Joint('hinge', 'base_link', 'arm', 'fixed')),
                              (slider_xml, Joint('hinge', 'base_link', 'arm', 'prismatic', limits=(-.02, .04)))):
            with self.subTest(kind=expected.kind), tempfile.TemporaryDirectory() as directory:
                package, boxes, cad, _ = literal_package(directory, xml)
                evidence = verify_package(package, boxes, cad, [expected])
                result = next(e for e in evidence if e['name'] == 'hinge type')
                self.assertEqual((result['status'], result['actual']), ('pass', expected.kind))

    def test_binary_stl_parser_rejects_truncated_and_nonfinite_vertices(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'box.stl'
            write_box_stl(path, (0, 0, 0), (2, 4, 6))
            self.assertEqual(bounds(stl_vertices(path)), ((0, 0, 0), (2, 4, 6)))
            original = path.read_bytes()
            for broken in (original[:-1], original[:84],
                           original[:96] + struct.pack('<f', math.nan) + original[100:]):
                path.write_bytes(broken)
                with self.assertRaises(ValueError):
                    stl_vertices(path)


class SourceSafetyTest(unittest.TestCase):
    def test_unavailable_proxy_token_is_evidence_not_an_identity_failure(self):
        class Entity:
            objectType = 'adsk::fusion::Occurrence'
            isValid = True

            @property
            def entityToken(self):
                raise RuntimeError('Tokens require a root-context proxy')

        entity = Entity()
        evidence = entity_evidence([('nested', entity)])
        self.assertIsNone(evidence[0]['token'])
        self.assertIn('root-context', evidence[0]['token_unavailable'])
        self.assertEqual(assert_entity_identity([('nested', entity)], [('nested', entity)])['status'], 'pass')

    def test_direct_snapshot_never_reads_parametric_only_collections(self):
        native = load_entrypoint_without_sdk()

        class Direct:
            designType = 0
            rootComponent = types.SimpleNamespace(name='direct', bRepBodies=[], allOccurrences=[],
                                                  allJoints=[], allAsBuiltJoints=[])

            @property
            def userParameters(self):
                raise RuntimeError('not parametric')

            @property
            def allParameters(self):
                raise RuntimeError('not parametric')

        snapshot = native.source_snapshot(Direct())
        self.assertEqual((snapshot['timeline'], snapshot['parameters']), (None, []))

    def test_fixture_setup_never_renames_fusion_root(self):
        native = load_entrypoint_without_sdk()

        class Root:
            @property
            def name(self):
                return 'Untitled'

        root = Root()
        body = types.SimpleNamespace(name='solid')
        with patch.object(native, '_add_box', return_value=body):
            bindings, joints = native._fixtures(types.SimpleNamespace(rootComponent=root), 'root_bodies')
        self.assertEqual((root.name, len(bindings), joints), ('Untitled', 2, []))

    def test_retained_entity_identity_accepts_changed_tokens_but_rejects_replacement(self):
        class Entity:
            objectType = 'adsk::fusion::ExtrudeFeature'
            isValid = True
            entityToken = 'first-token'
        feature = Entity()
        before = [('timeline[0]', feature)]
        first_evidence = entity_evidence(before)
        feature.entityToken = 'different-token-for-the-same-entity'
        after = [('timeline[0]', feature)]
        self.assertEqual((first_evidence[0]['token'], entity_evidence(after)[0]['token']),
                         ('first-token', 'different-token-for-the-same-entity'))
        self.assertEqual(assert_entity_identity(before, after)['retained_entities'], 1)
        replacement = Entity()
        replacement.entityToken = feature.entityToken
        with self.assertRaisesRegex(AssertionError, 'entity identity changed'):
            assert_entity_identity(before, [('timeline[0]', replacement)])
        feature.isValid = False
        with self.assertRaisesRegex(AssertionError, 'entity identity changed'):
            assert_entity_identity(before, after)

    def test_snapshot_rejects_expression_and_feature_edits_with_unchanged_counts(self):
        native = load_entrypoint_without_sdk()
        parameter = types.SimpleNamespace(name='proof_height', expression='60 mm', unit='cm', value=6., comment='Editable depth')
        feature = types.SimpleNamespace(name='ProofExtrude', objectType='adsk::fusion::ExtrudeFeature', isValid=True)
        timeline_object = types.SimpleNamespace(entity=feature, name='ProofExtrude', index=0, isSuppressed=False)
        design = types.SimpleNamespace(designType=1, designIntent=2,
                    timeline=types.SimpleNamespace(markerPosition=1, count=1, item=lambda i: timeline_object),
                    allParameters=[parameter], userParameters=[parameter],
                    rootComponent=types.SimpleNamespace(name='proof', bRepBodies=[], allOccurrences=[], allJoints=[], allAsBuiltJoints=[]))
        for entity, attribute, value, field in ((parameter, 'expression', '6 cm', 'expression'),
                                                (parameter, 'unit', 'mm', 'unit'),
                                                (parameter, 'value', 7., 'value'),
                                                (feature, 'name', 'ReplacementExtrude', 'name'),
                                                (timeline_object, 'isSuppressed', True, 'is_suppressed'),
                                                (design, 'designIntent', 0, 'design_intent')):
            with self.subTest(field=field):
                before = native.source_snapshot(design)
                original = getattr(entity, attribute)
                setattr(entity, attribute, value)
                after = native.source_snapshot(design)
                setattr(entity, attribute, original)
                with self.assertRaisesRegex(AssertionError, field):
                    assert_source_unchanged(before, after)

    def test_source_changes_fail_with_the_changed_field(self):
        before = dict(design_type=1, timeline=dict(marker=4, count=8),
                      occurrences=[dict(path='arm:1', transform_cm=[0., 10., 20.],
                                        bodies=[dict(component='arm', visible=True)])],
                      joints=[dict(name='slide', slideValue=1.25)])
        self.assertEqual(assert_source_unchanged(before, copy.deepcopy(before)),
                         {'status': 'pass', 'numeric_absolute_tolerance': 1e-10})
        changes = (
            ('design_type', lambda state: state.update(design_type=0)),
            ('timeline.marker', lambda state: state['timeline'].update(marker=3)),
            ('timeline.count', lambda state: state['timeline'].update(count=0)),
            ('transform_cm', lambda state: state['occurrences'][0]['transform_cm'].__setitem__(1, 11.)),
            ('component', lambda state: state['occurrences'][0]['bodies'][0].update(component='other')),
            ('visible', lambda state: state['occurrences'][0]['bodies'][0].update(visible=False)),
            ('slideValue', lambda state: state['joints'][0].update(slideValue=0.)),
            ('bodies', lambda state: state['occurrences'][0]['bodies'].clear()),
        )
        for field, mutate in changes:
            with self.subTest(field=field):
                after = copy.deepcopy(before)
                mutate(after)
                with self.assertRaisesRegex(AssertionError, field):
                    assert_source_unchanged(before, after)


class Document:
    def __init__(self, app, name):
        self.app, self.name = app, name
        self.isValid = True
        self.close_result = True
        self.close_argument = None

    def close(self, save):
        self.close_argument = save
        if self.close_result:
            self.isValid = False
        return self.close_result

    def activate(self):
        self.app.activeDocument = self
        return True


class Documents:
    def __init__(self, app):
        self.app = app
        self.created = []

    def add(self, document_type):
        document = Document(self.app, document_type)
        self.created.append(document)
        document.activate()
        return document


class App:
    def __init__(self):
        self.documents = Documents(self)
        self.activeDocument = Document(self, 'user document')


class OwnedDocumentTest(unittest.TestCase):
    def test_only_owned_document_closes_and_previous_document_restores_after_failure(self):
        app = App()
        user = app.activeDocument
        with self.assertRaisesRegex(RuntimeError, 'fixture failed'):
            with owned_document(app, 'owned fixture') as document:
                self.assertEqual(app.activeDocument.name, 'owned fixture')
                raise RuntimeError('fixture failed')
        self.assertEqual((document.name, document.isValid, document.close_argument), ('owned fixture', False, False))
        self.assertEqual((app.activeDocument.name, user.isValid, user.close_argument), ('user document', True, None))

    def test_restore_still_happens_when_owned_close_is_refused(self):
        app = App()
        with self.assertRaisesRegex(RuntimeError, 'refused to close'):
            with owned_document(app, 'owned fixture') as document:
                document.close_result = False
        self.assertEqual((app.activeDocument.name, document.isValid, document.close_argument), ('user document', True, False))

    def test_nested_owned_documents_restore_their_actual_previous_document(self):
        app = App()
        with owned_document(app, 'outer') as outer:
            with owned_document(app, 'inner') as inner:
                self.assertEqual(app.activeDocument.name, 'inner')
            self.assertEqual((app.activeDocument.name, inner.isValid), ('outer', False))
        self.assertEqual((app.activeDocument.name, outer.isValid), ('user document', False))

    def test_existing_document_returned_by_factory_is_never_closed(self):
        app = App()
        user = app.activeDocument
        app.documents.add = lambda document_type: user
        with self.assertRaisesRegex(RuntimeError, 'distinct owned document'):
            with owned_document(app, 'owned fixture'):
                self.fail('Existing user document was accepted as owned')
        self.assertEqual((app.activeDocument.name, user.isValid, user.close_argument), ('user document', True, None))

    def test_no_previous_document_requires_no_restore(self):
        app = App()
        app.activeDocument = None
        with owned_document(app, 'owned fixture') as document:
            self.assertEqual(app.activeDocument.name, 'owned fixture')
        self.assertEqual((document.isValid, document.close_argument), (False, False))


class ReportTest(unittest.TestCase):
    def test_cached_exporter_setup_failure_retains_provenance_without_creating_documents(self):
        native = load_entrypoint_without_sdk()
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict('sys.modules', {'URDF_Exporter': types.ModuleType('URDF_Exporter')}):
                path = native.run_verification(types.SimpleNamespace(version='offline setup test, not Fusion'), directory)
            result = json.loads(path.read_text())
            self.assertEqual((result['status'], result['cases'][0]['fixture']), ('fail', 'runner_setup'))
            self.assertIn('Restart Fusion before running the verifier', result['fatal_traceback'])
            self.assertIn('URDF_Exporter', result['provenance']['preexisting_exporter_modules'])
            self.assertEqual(result['provenance']['loaded_code']['native_runner']['status'], 'match')
            self.assertIn('URDF_Exporter/URDF_Exporter.py', result['provenance']['inputs_sha256'])
            self.assertEqual(result['coverage'][0]['status'], 'not_exercised')

    def test_hashes_config_and_unavailable_git_survive_partial_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            repository = Path(directory)
            (repository / 'URDF_Exporter').mkdir()
            (repository / 'URDF_Exporter/input.toml').write_text('hello', encoding='utf-8')
            (repository / 'tools/verify_fusion').mkdir(parents=True)
            (repository / 'tools/verify_fusion/verify_fusion.manifest').write_text('hello', encoding='utf-8')
            with patch('tools.verify_fusion.verification.subprocess.run', side_effect=FileNotFoundError('git unavailable in Fusion')):
                provenance = capture_provenance(repository, {'collision_method': 'visual'})
            report = Report(directory, 'version', 2, provenance=provenance)
            report.start_case('partial', 'parametric', directory).update(status='fail', traceback='literal failure')
            report.save(finished=True)
            result = json.loads(report.path.read_text())
            self.assertEqual((result['status'], result['provenance']['requested_config']),
                             ('fail', {'collision_method': 'visual'}))
            self.assertEqual(result['provenance']['git'], {'status': 'unavailable', 'reason': 'git unavailable in Fusion'})
            self.assertEqual(result['provenance']['inputs_sha256'], {
                'URDF_Exporter/input.toml': '2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824',
                'tools/verify_fusion/verify_fusion.manifest': '2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824'})

    def test_loaded_function_provenance_distinguishes_cached_code_from_disk(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'loaded.py'
            source = 'def value():\n    return 1\n\nclass Value:\n    def read(self):\n        return 1\n'
            path.write_text(source, encoding='utf-8')
            namespace = {'__name__': 'literal_loaded'}
            exec(compile(source, str(path), 'exec'), namespace)
            first = loaded_code_provenance(namespace, path)
            self.assertEqual((first['status'], first['functions']['value']['first_line'],
                              first['functions']['Value.read']['matches_disk']), ('match', 1, True))
            path.write_text(source.replace('return 1', 'return 2'), encoding='utf-8')
            cached = loaded_code_provenance(namespace, path)
            self.assertEqual((namespace['value'](), cached['status'], cached['functions']['value']['matches_disk'],
                              cached['functions']['Value.read']['matches_disk']), (1, 'mismatch', False, False))
            self.assertEqual(cached['functions']['value']['bytecode_sha256'], first['functions']['value']['bytecode_sha256'])

    def test_changed_provenance_cannot_finish_as_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Report(directory, 'version', 1, provenance={'status': 'fail', 'reason': 'Input files changed'})
            report.start_case('root_bodies', 'parametric', directory)['status'] = 'pass'
            report.save(finished=True)
            self.assertEqual(json.loads(report.path.read_text())['status'], 'fail')

    def test_missing_provenance_is_explicit_in_partial_report(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Report(directory, 'version', 1)
            report.start_case('root_bodies', 'parametric', directory)['status'] = 'fail'
            report.save(finished=True)
            result = json.loads(report.path.read_text())
            self.assertEqual(result['provenance']['status'], 'unavailable')

    def test_report_persists_partial_failure_evidence_and_keeps_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'partial_description'
            output.mkdir()
            mesh = output / 'partial.stl'
            mesh.write_bytes(b'evidence')
            report = Report(directory, 'literal Fusion version', 2)
            first = report.start_case('box', 'parametric', output)
            first.update(status='pass', source_preservation={'status': 'pass'})
            report.save()
            partial = json.loads(report.path.read_text())
            self.assertEqual((partial['status'], partial['cases'][0]['status']), ('running', 'pass'))
            second = report.start_case('box', 'direct', output)
            second.update(status='fail', traceback='Literal export failure', source_preservation={'status': 'pass'})
            report.save(finished=True)
            final = json.loads(report.path.read_text())
            self.assertEqual((final['status'], final['fusion_version']), ('fail', 'literal Fusion version'))
            self.assertEqual(final['cases'][1]['traceback'], 'Literal export failure')
            self.assertEqual(final['coverage'][0]['status'], 'not_exercised')
            self.assertEqual(mesh.read_bytes(), b'evidence')

    def test_incomplete_report_cannot_finish_as_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Report(directory, 'version', 2)
            report.start_case('root_bodies', 'parametric', directory)['status'] = 'pass'
            report.save(finished=True)
            self.assertEqual(json.loads(report.path.read_text())['status'], 'fail')

    def test_complete_report_keeps_unexercised_coverage_distinct(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Report(directory, 'version', 1)
            report.start_case('root_bodies', 'parametric', directory)['status'] = 'pass'
            report.save(finished=True)
            result = json.loads(report.path.read_text())
            self.assertEqual((result['status'], result['coverage'][0]['status']), ('pass', 'not_exercised'))


if __name__ == '__main__':
    unittest.main()
