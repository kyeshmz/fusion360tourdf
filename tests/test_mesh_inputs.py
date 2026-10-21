"""Public MeshBody exports using documented API doubles and analytic oracles."""

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

from tests.test_fusion_inputs import Body, design, fixed, hinge, occurrence, root
from tests.test_issue_bodyless import pose
from tests.test_combined_frames import add, axis_rotate, origin_apply, source_snapshot, subtract, vector
from tests.test_mesh_geometry import box_oracle, cell_surface, transformed
from URDF_Exporter.core import Link
from URDF_Exporter.utils import config as config_mod, utils
from URDF_Exporter.utils.mesh_geometry import CancelledError, ResourceLimitError


READS = []
IDENTITY_POSE = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


class MeshBody:
    def __init__(self, name, triangles, *, owner=None, native=None, visible=True):
        self.name, self.nativeObject, self.assemblyContext = name, native, owner
        self.isVisible, self.isValid, self.isClosed, self.isOriented = visible, True, True, True
        nodes = [p for t in triangles for p in t]
        self._display = types.SimpleNamespace(isValid=True, triangleCount=len(triangles), nodeCount=len(nodes),
            nodeCoordinatesAsDouble=[v for p in nodes for v in p], nodeIndices=list(range(len(nodes))))

    @property
    def displayMesh(self):
        READS.append(self)
        if self.nativeObject is not None:
            raise AssertionError('Proxy displayMesh must not be read')
        return self._display

    def getPhysicalProperties(self, *args):
        raise AssertionError('MeshBody has no getPhysicalProperties API')


def mesh_root(triangles, name='scan'):
    component = root()
    body = MeshBody(name, triangles)
    component.meshBodies = [body]
    return component, body


def density(value=1000, **controls):
    return {'mesh_physics': dict(density_kg_m3=value, **controls)}


def supplied(path='design_root', name='scan', frame='native'):
    return {'occurrence': path, 'body': name, 'mass_kg': 2,
            'center_of_mass_m': [.01, .02, 0], 'inertia_kg_m2': [.003, .004, .005, -.0002, .0001, -.0003],
            'frame': frame, 'provenance': 'Measured assembly model. kg, m, kg*m^2. Central tensor.'}


def snapshot_design(component):
    fixture, exported = design(component)
    fixture.unitsManager = types.SimpleNamespace(defaultLengthUnits='inch')
    fixture.timeline = types.SimpleNamespace(markerPosition=3, entityTokens=('import', 'mesh', 'joint'))
    return fixture, exported


class MeshInputsTest(unittest.TestCase):
    def setUp(self):
        READS.clear()
        artifacts = os.environ.get('MESH_INPUT_ARTIFACT_DIR')
        if artifacts:
            self.output = Path(tempfile.mkdtemp(prefix=self._testMethodName + '-', dir=artifacts))
        else:
            temporary = tempfile.TemporaryDirectory(prefix='mesh-inputs-')
            self.addCleanup(temporary.cleanup)
            self.output = Path(temporary.name)
        self.entry = importlib.import_module('URDF_Exporter.URDF_Exporter')

    def export(self, component, config=None, name='mesh', cancelled=None):
        fixture, exported = snapshot_design(component)
        before = source_snapshot(fixture)
        target = self.output / (name + '_description')
        self.entry.export_design(fixture, target, config=config, cancelled=cancelled)
        self.assertEqual(source_snapshot(fixture), before)
        return ET.parse(target / 'urdf' / (name + '.urdf')).getroot(), target, exported

    def reject(self, component, config, error=ValueError, pattern=None):
        fixture, exported = snapshot_design(component)
        before = source_snapshot(fixture)
        target = self.output / 'invalid_description'
        with self.assertRaises(error) if pattern is None else self.assertRaisesRegex(error, pattern):
            self.entry.export_design(fixture, target, config=config)
        self.assertEqual(source_snapshot(fixture), before)
        self.assertEqual(exported, [])
        self.assertFalse(target.exists())
        self.assertEqual(list(self.output.iterdir()), [])

    def assert_physics(self, link, mass, center, tensor, relative=2e-6):
        physics = link.find('inertial')
        self.assertAlmostEqual(float(physics.find('mass').get('value')) / mass, 1, delta=relative)
        for actual, expected in zip(vector(physics.find('origin'), 'xyz'), center):
            self.assertAlmostEqual(actual, expected, delta=max(1e-15, abs(expected)*relative))
        keys = (('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2), ('ixy', 0, 1), ('iyz', 1, 2), ('ixz', 0, 2))
        for key, i, j in keys:
            self.assertAlmostEqual(float(physics.find('inertia').get(key)), tensor[i][j],
                                   delta=max(abs(tensor[k][k]) for k in range(3))*relative)

    def test_root_tetrahedron_literal_mass_signed_com_and_all_six_inertia(self):
        a, b, c, d = (0, 0, 0), (2, 0, 0), (0, 3, 0), (0, 0, 5)
        triangles = transformed([(a, c, b), (a, b, d), (a, d, c), (b, c, d)], translation=(-4, -5, -6))
        component, body = mesh_root(triangles)
        robot, target, exported = self.export(component, density(4000000), 'tetra')
        tensor = [[.00255, .00015, .00025], [.00015, .002175, .000375], [.00025, .000375, .000975]]
        self.assert_physics(robot.find('link'), 20, (-.035, -.0425, -.0475), tensor, 1e-12)
        self.assertEqual(READS, [body])
        self.assertEqual(exported, [])
        data = (target / 'meshes/base_link.stl').read_bytes()
        self.assertEqual(struct.unpack_from('<I', data, 80)[0], 4)
        receipt = json.loads((target / 'config/robot_data.yaml').read_text())
        proof = receipt['mesh_physics']['base_link'][0]
        self.assertFalse(proof['physics_is_native'])
        self.assertEqual(proof['integration']['shell_policy'], 'single-embedded-shell')
        self.assertTrue(receipt['warnings'])
        (self.output / 'expected.json').write_text(json.dumps({'evidence_kind': 'analytic_api_double',
            'mass_kg': 20, 'center_of_mass_m': [-.035, -.0425, -.0475], 'central_inertia_kg_m2': tensor,
            'triangles': 4, 'bounds_m': [[-.04, -.05, -.06], [-.02, -.02, -.01]],
            'source_preserved': True, 'native_evidence': False,
            'verifier': {'zero': {'positions': {}, 'link_poses': {'base_link': IDENTITY_POSE},
                'bounds': {'base_link': [[-.04, -.05, -.06], [-.02, -.02, -.01]]},
                'link_kinds': {'base_link': 'physical'}}}}, indent=2) + '\n')

    def test_repeated_posed_native_mesh_snapshot_read_once_and_frame_linked(self):
        triangles = transformed(cell_surface([(0, 0, 0)]), scale=(2, 3, 5))
        native = MeshBody('shared scan', triangles)
        left = occurrence('module:1+scan:1', matrix=pose((-12, 8, -4), (.31, -.47, .23)))
        right = occurrence('module:2+scan:1', matrix=pose((27, -13, 19), (-.29, .38, -.41)))
        for owner in (left, right):
            owner.meshBodies = [MeshBody('shared scan', (), owner=owner, native=native)]
        frame = occurrence('reference:1', matrix=pose((3, -2, 7), (.17, .28, -.21)))
        joints = [fixed('left_mount', left, frame, (-12, 8, -4)), fixed('right_mount', right, frame, (27, -13, 19))]
        component = root([right, frame, left], joints=joints)
        config = dict(density(2000), root_occurrence='reference:1')
        robot, target, exported = self.export(component, config, 'posed')
        self.assertEqual(list(robot.find("link[@name='base_link']")), [])
        self.assertEqual(READS, [native])
        self.assertEqual(exported, [])
        entries, _ = Link.make_inertial_dict(utils.link_sources(component, config=config, joint_records=[(j, False) for j in joints]), 'ok')
        comparisons = 0
        expected_links = {}
        verifier = {'zero': {'positions': {}, 'link_poses': {'base_link': IDENTITY_POSE},
                            'bounds': {}, 'link_kinds': {'base_link': 'frame'}}}
        for owner in (left, right):
            name = owner.fullPathName.replace(':', '_').replace('+', '_')
            matrix = owner.transform2.asArray()
            rotation = [matrix[4*i:4*i+3] for i in range(3)]
            translation = [matrix[4*i+3] / 100 for i in range(3)]
            mass, center, tensor = box_oracle((.02, .03, .05), 2000, rotation, translation)
            entry = entries[name]
            self.assertAlmostEqual(entry['mass'] / mass, 1, delta=2e-6)
            for actual, expected in zip(entry['center_of_mass'], center):
                self.assertAlmostEqual(actual, expected, delta=2e-8)
            for actual, (i, j) in zip(entry['inertia'], ((0, 0), (1, 1), (2, 2), (0, 1), (1, 2), (0, 2))):
                self.assertAlmostEqual(actual, tensor[i][j], delta=2e-11)
            data = (target / 'meshes' / (name + '.stl')).read_bytes()
            visual = robot.find("link[@name='%s']/visual" % name)
            joint = next(j for j in robot.findall('joint') if j.find('child').get('link') == name)
            expected_points = []
            for n, triangle in enumerate(triangles):
                row = struct.unpack_from('<12fH', data, 84 + n*50)
                for k, p in zip((3, 6, 9), triangle):
                    expected = [sum(rotation[i][j]*p[j] for j in range(3)) / 100 + translation[i] for i in range(3)]
                    actual = origin_apply(origin_apply(tuple(v*.001 for v in row[k:k+3]), visual.find('origin')), joint.find('origin'))
                    for got, want in zip(actual, expected):
                        self.assertAlmostEqual(got, want, delta=3e-8)
                    expected_points.append(expected)
                    comparisons += 1
                self.assertAlmostEqual(math.hypot(*row[:3]), 1, delta=1e-6)
            expected_links[name] = {'mass_kg': mass, 'center_of_mass_m': center, 'inertia_kg_m2': tensor,
                                    'vertices_root_m': expected_points, 'triangles': 12}
            verifier['zero']['link_poses'][name] = [list(rotation[i])+[translation[i]] for i in range(3)]+[[0, 0, 0, 1]]
            verifier['zero']['link_kinds'][name] = 'physical'
            verifier['zero']['bounds'][name] = [[f(p[i] for p in expected_points) for i in range(3)] for f in (min, max)]
        self.assertEqual(comparisons, 72)
        (self.output / 'expected.json').write_text(json.dumps({'evidence_kind': 'analytic_api_double',
            'links': expected_links, 'vertex_comparisons': 72, 'frame_links': 1,
            'mesh_links': 2, 'native_display_reads_per_export': 1, 'verifier': verifier}, indent=2) + '\n')

    def test_mixed_brep_mesh_direct_ownership_hidden_and_aggregate_tensor(self):
        triangles = transformed(cell_surface([(0, 0, 0)]), scale=(2, 3, 5), translation=(-8, 6, -4))
        base, scan = mesh_root(triangles)
        solid = Body('CAD solid', 10, mass=3)
        base.bRepBodies = [solid, Body('hidden CAD', 99, mass=999, visible=False)]
        base.meshBodies.append(MeshBody('hidden scan', (), visible=False))
        robot, target, exported = self.export(base, density(2000), 'mixed')
        self.assertEqual(exported, ['CAD solid'])
        self.assertEqual(READS, [scan])
        m1, c1, t1 = box_oracle((.02, .03, .05), 2000, translation=(-.08, .06, -.04))
        m2, c2 = 3, [.1, 0, 0]
        center = [(m1*c1[i] + m2*c2[i])/(m1+m2) for i in range(3)]
        tensor = [[0.0]*3 for _ in range(3)]
        for mass, c, t in ((m1, c1, t1), (m2, c2, [[.0002, 0, 0], [0, .0002, 0], [0, 0, .0002]])):
            d = [c[i]-center[i] for i in range(3)]
            for i in range(3):
                for j in range(3):
                    tensor[i][j] += t[i][j] + mass*((sum(v*v for v in d) if i == j else 0)-d[i]*d[j])
        self.assert_physics(robot.find('link'), m1+m2, center, tensor, 1e-12)
        self.assertEqual(struct.unpack_from('<I', (target / 'meshes/base_link.stl').read_bytes(), 80)[0], 24)

    def test_supplied_open_planar_visual_has_no_fabricated_bounds_and_records_override(self):
        component, body = mesh_root([((0, 0, 0), (2, 0, 0), (0, 3, 0))])
        body.isClosed = False
        robot, target, exported = self.export(component, {'mesh_physics': {'bodies': [supplied()]}}, 'surface')
        self.assertEqual(exported, [])
        self.assertEqual(robot.find('link/visual/geometry/mesh').get('scale'), '0.001 0.001 0.001')
        self.assertIsNone(robot.find('.//box'))
        self.assert_physics(robot.find('link'), 2, [.01, .02, 0],
            [[.003, -.0002, -.0003], [-.0002, .004, .0001], [-.0003, .0001, .005]], 1e-12)
        receipt = json.loads((target / 'config/robot_data.yaml').read_text())
        self.assertTrue(any('deliberately override' in warning for warning in receipt['warnings']))
        self.assertEqual(receipt['mesh_physics']['base_link'][0]['supplied_provenance'], supplied()['provenance'])
        self.assertFalse(receipt['mesh_physics']['base_link'][0]['reported_is_closed'])
        installed = self.output / 'relocated/share/surface_description'
        shutil.copytree(target, installed)
        self.assertEqual((installed / 'meshes/base_link.stl').read_bytes(), (target / 'meshes/base_link.stl').read_bytes())
        (self.output / 'expected.json').write_text(json.dumps({'triangles': 1, 'mass_kg': 2,
            'bounds_m': [[0, 0, 0], [.02, .03, 0]], 'area_m2': .0003,
            'independent_loader_status': 'unverified; registered surface runtime uses independent 2D and exact triangle-count declarations'}, indent=2)+'\n')

    def test_supplied_native_tensor_rotates_once_and_root_frame_does_not_rotate(self):
        native = MeshBody('scan', [((0, 0, 0), (2, 0, 0), (0, 3, 0))])
        matrix = (0, -1, 0, 10, 1, 0, 0, -20, 0, 0, 1, 30, 0, 0, 0, 1)
        for frame in ('native', 'design_root'):
            owner = occurrence('scan:1', matrix=matrix)
            owner.meshBodies = [MeshBody('scan', (), owner=owner, native=native)]
            robot, _, _ = self.export(root([owner]), {'mesh_physics': {'bodies': [supplied('scan:1', frame=frame)]}}, frame)
            tensor = [[.004, .0002, -.0001], [.0002, .003, -.0003], [-.0001, -.0003, .005]] if frame == 'native' else [
                [.003, -.0002, -.0003], [-.0002, .004, .0001], [-.0003, .0001, .005]]
            center = [.08, -.19, .3] if frame == 'native' else [.01, .02, 0]
            self.assert_physics(robot.find('link'), 2, center, tensor, 1e-12)

    def test_tiny_representable_mesh_physics_is_not_clamped(self):
        triangles = transformed(cell_surface([(0, 0, 0)]), scale=(2**-12, 2**-11, 2**-10))
        component, _ = mesh_root(triangles)
        robot, _, _ = self.export(component, density(7800), 'tiny')
        self.assert_physics(robot.find('link'), *box_oracle((2**-12/100, 2**-11/100, 2**-10/100), 7800), relative=1e-12)
        self.assertGreater(float(robot.find('link/inertial/inertia').get('ixx')), 0)

    def test_explicit_density_is_required_and_material_labels_are_never_density(self):
        component, body = mesh_root(cell_surface([(0, 0, 0)]))
        body.material = types.SimpleNamespace(name='Steel 7850 kg/m3', density=7850)
        self.reject(component, None, pattern='MESH_PHYSICS_REQUIRED.*design_root.*scan')
        self.assertEqual(READS, [])

    def test_selectors_use_raw_context_not_alias_and_reject_hidden_unused_duplicate_or_ambiguous(self):
        for kind in ('alias', 'unknown', 'hidden', 'duplicate', 'ambiguous'):
            component, body = mesh_root(cell_surface([(0, 0, 0)]))
            policy = {'occurrence': 'design_root', 'body': 'scan', 'density_kg_m3': 1000}
            config = {'mesh_physics': {'bodies': [policy]}}
            if kind == 'alias':
                policy['occurrence'] = 'base_link'
            elif kind == 'unknown':
                policy['body'] = 'unknown'
            elif kind == 'hidden':
                body.isVisible = False
                component.bRepBodies = [Body('solid', 0)]
            elif kind == 'duplicate':
                config['mesh_physics']['bodies'].append(dict(policy))
            else:
                component.meshBodies.append(MeshBody('scan', cell_surface([(1, 0, 0)])))
            with self.subTest(kind=kind):
                self.reject(component, config)
        component, _ = mesh_root(cell_surface([(0, 0, 0)]))
        component.meshBodies.append(MeshBody('scan', transformed(cell_surface([(0, 0, 0)]), translation=(3, 0, 0))))
        robot, _, _ = self.export(component, density(), 'uniform')
        self.assertAlmostEqual(float(robot.find('link/inertial/mass').get('value')), .002)

    def test_per_context_density_handles_repeated_names_without_source_renaming(self):
        native = MeshBody('scan', cell_surface([(0, 0, 0)]))
        one, two = occurrence('module:1+part:1'), occurrence('module:2+part:1')
        for owner in (one, two):
            owner.meshBodies = [MeshBody('scan', (), owner=owner, native=native)]
        component = root([two, one], joints=[fixed('mount', two, one)])
        policies = [{'occurrence': one.fullPathName, 'body': 'scan', 'density_kg_m3': 1000},
                    {'occurrence': two.fullPathName, 'body': 'scan', 'density_kg_m3': 3000}]
        robot, _, _ = self.export(component, {'root_occurrence': one.fullPathName, 'mesh_physics': {'bodies': policies}}, 'selectors')
        self.assertEqual({link.get('name'): float(link.find('inertial/mass').get('value')) for link in robot.findall('link')},
                         {'base_link': .001, 'module_2_part_1': .003})

    def test_density_rejects_open_nonmanifold_crossing_multiple_shells_and_cavities_despite_flags(self):
        cube = cell_surface([(0, 0, 0)])
        crossing = [tuple(((-.5, .5, .5) if p == (1, 1, 1) else p) for p in t) for t in cube]
        cases = [cube[:-1], cube+[cube[0]], crossing, cube+transformed(cube, translation=(3, 0, 0)),
                 transformed(cube, scale=(4, 4, 4))+[tuple(reversed(t)) for t in transformed(cube, translation=(1, 1, 1))]]
        for triangles in cases:
            with self.subTest(triangles=len(triangles)):
                component, _ = mesh_root(triangles)
                self.reject(component, density())

    def test_invalid_geometry_indices_counts_validity_and_nonfinite_fail_before_publication(self):
        for case in ('negative_index', 'fraction_index', 'bool_index', 'out_of_range', 'count', 'nodes',
                     'nan', 'inf', 'bool_coordinate', 'invalid_body', 'invalid_display', 'degenerate', 'overflow', 'underflow'):
            component, body = mesh_root(cell_surface([(0, 0, 0)]))
            mesh = body._display
            if case == 'negative_index': mesh.nodeIndices[0] = -1
            elif case == 'fraction_index': mesh.nodeIndices[0] = .5
            elif case == 'bool_index': mesh.nodeIndices[0] = True
            elif case == 'out_of_range': mesh.nodeIndices[0] = mesh.nodeCount
            elif case == 'count': mesh.triangleCount += 1
            elif case == 'nodes': mesh.nodeCount += 1
            elif case == 'nan': mesh.nodeCoordinatesAsDouble[0] = math.nan
            elif case == 'inf': mesh.nodeCoordinatesAsDouble[0] = math.inf
            elif case == 'bool_coordinate': mesh.nodeCoordinatesAsDouble[0] = True
            elif case == 'invalid_body': body.isValid = False
            elif case == 'invalid_display': mesh.isValid = False
            elif case == 'degenerate': mesh.nodeIndices[:3] = [0, 0, 0]
            elif case == 'overflow': mesh.nodeCoordinatesAsDouble = [v*1e40 for v in mesh.nodeCoordinatesAsDouble]
            elif case == 'underflow': mesh.nodeCoordinatesAsDouble = [v*1e-60 for v in mesh.nodeCoordinatesAsDouble]
            with self.subTest(case=case):
                self.reject(component, {'mesh_physics': {'bodies': [supplied()]}})

    def test_invalid_placement_and_native_context_rejected(self):
        for case in ('missing_native', 'wrong_context', 'invalid_owner', 'scale', 'reflection', 'nan'):
            owner = occurrence('scan:1')
            native = MeshBody('scan', cell_surface([(0, 0, 0)]))
            body = MeshBody('scan', (), owner=owner, native=native)
            owner.meshBodies = [body]
            if case == 'missing_native': body.nativeObject = None
            elif case == 'wrong_context': body.assemblyContext = occurrence('scan:2')
            elif case == 'invalid_owner': owner.isValid = False
            else:
                matrix = list(owner.transform2.asArray())
                matrix[0] = {'scale': 2, 'reflection': -1, 'nan': math.nan}[case]
                owner.transform2 = types.SimpleNamespace(asArray=lambda: matrix)
            with self.subTest(case=case): self.reject(root([owner]), density())

    def test_config_one_owner_copies_input_and_toml_supplied_physics(self):
        options = {'mesh_physics': {'bodies': [supplied()]}}
        before = copy.deepcopy(options)
        parsed = config_mod.load_config(overrides=options)
        again = config_mod.load_config(overrides=parsed)
        parsed['mesh_physics']['bodies'][0]['center_of_mass_m'][0] = 99
        self.assertEqual(options, before)
        self.assertEqual(again['mesh_physics']['bodies'][0]['center_of_mass_m'][0], .01)
        folder = self.output / 'config'
        folder.mkdir()
        (folder / 'xacro_export.toml').write_text('''[mesh_physics]
max_triangles = 15
[[mesh_physics.bodies]]
occurrence = "design_root"
body = "scan"
mass_kg = 2
center_of_mass_m = [0.01, 0.02, 0]
inertia_kg_m2 = [0.003, 0.004, 0.005, -0.0002, 0.0001, -0.0003]
frame = "native"
provenance = "Independent SI measurements"
''')
        config = config_mod.load_config(folder)
        component, _ = mesh_root([((0, 0, 0), (1, 0, 0), (0, 1, 0))])
        self.export(component, config, 'toml')

    def test_bad_density_and_supplied_nonphysical_incomplete_unknown_options_reject(self):
        for value in (0, -1, math.nan, math.inf, True, '7850', 10**1000):
            with self.subTest(value=repr(value)[:30]):
                component, _ = mesh_root(cell_surface([(0, 0, 0)]))
                self.reject(component, density(value))
        for field, value in (('mass_kg', 0), ('center_of_mass_m', [0, 0, math.nan]),
                             ('inertia_kg_m2', [1, 1, 9, 0, 0, 0]), ('inertia_kg_m2', [1, 1, 1, 2, 0, 0]),
                             ('inertia_kg_m2', [0]*6), ('frame', 'link'), ('provenance', ''),
                             ('unknown', 1), ('density_kg_m3', 1000)):
            policy = supplied()
            policy[field] = value
            component, _ = mesh_root(cell_surface([(0, 0, 0)]))
            with self.subTest(field=field, value=value):
                self.reject(component, {'mesh_physics': {'bodies': [policy]}})
        for missing in ('mass_kg', 'frame', 'provenance', 'inertia_kg_m2'):
            policy = supplied()
            del policy[missing]
            component, _ = mesh_root(cell_surface([(0, 0, 0)]))
            self.reject(component, {'mesh_physics': {'bodies': [policy]}})

    def test_limits_cancel_and_forced_stage_failure_are_transactional(self):
        for options in ({'max_triangles': 1}, {'max_nodes': 2}, {'max_pair_checks': 1}):
            component, _ = mesh_root(cell_surface([(0, 0, 0)]))
            self.reject(component, density(**options), ResourceLimitError)
        component, _ = mesh_root(cell_surface([(0, 0, 0)]))
        fixture, _ = snapshot_design(component)
        before = source_snapshot(fixture)
        for stage in ('early', 'capture', 'staging'):
            target = self.output / (stage + '_description')
            if stage == 'early':
                with self.assertRaises(CancelledError):
                    self.entry.export_design(fixture, target, config=density(), cancelled=lambda: True)
            elif stage == 'capture':
                calls = []
                def cancel():
                    calls.append(1)
                    return len(calls) > 65
                with self.assertRaises(CancelledError):
                    self.entry.export_design(fixture, target, config=density(), cancelled=cancel)
            else:
                original = Path.write_bytes
                def fail(path, data):
                    if path.name == 'base_link.stl': raise OSError('forced staging failure')
                    return original(path, data)
                with patch.object(Path, 'write_bytes', fail), self.assertRaisesRegex(OSError, 'forced staging'):
                    self.entry.export_design(fixture, target, config=density())
            self.assertFalse(target.exists())
            self.assertEqual(source_snapshot(fixture), before)
            self.assertEqual(list(self.output.iterdir()), [])

    def test_geometry_snapshot_survives_native_changes_before_mesh_callback(self):
        component, body = mesh_root(cell_surface([(0, 0, 0)]))
        fixture, exported = snapshot_design(component)
        sources = utils.link_sources(component, config=density())
        entries, _ = Link.make_inertial_dict(sources, 'ok')
        body._display.nodeCoordinatesAsDouble[:] = [99]*len(body._display.nodeCoordinatesAsDouble)
        body.isVisible = False
        target = self.output / 'captured.stl'
        utils.export_stl(fixture, {'base_link': target}, config=density(), sources=sources)
        self.assertEqual(READS, [body])
        self.assertEqual(exported, [])
        self.assertEqual(struct.unpack_from('<3f', target.read_bytes(), 96), (0, 0, 0))
        self.assertEqual(entries['base_link']['mass'], .001)

    def test_planar_density_and_box_collision_reject_but_finite_surface_visual_is_allowed(self):
        triangle = [((0, 0, 0), (2, 0, 0), (0, 3, 0))]
        component, _ = mesh_root(triangle)
        self.reject(component, density())
        self.reject(component, {'mesh_physics': {'bodies': [supplied()]}, 'collision_method': 'primitive_box'},
                    pattern='no positive-volume mesh bounds')
        self.reject(component, {'mesh_physics': {'bodies': [supplied()]}, 'collision_method': 'convex_decomposition'},
                    pattern='open or nonmanifold')

    def test_unused_uniform_density_and_bad_controls_do_not_silently_succeed(self):
        self.reject(root(bodies=[Body('solid', 0)]), density(), pattern='UNUSED_MESH_DENSITY')
        component, _ = mesh_root(cell_surface([(0, 0, 0)]))
        for controls in ({'max_triangles': 0}, {'max_nodes': True}, {'max_pair_checks': 10000001}, {'unknown': 1}):
            self.reject(component, density(**controls))

    def test_aggregate_mesh_budgets_apply_across_distinct_bodies(self):
        for controls in ({'max_triangles': 20}, {'max_pair_checks': 200}):
            component, _ = mesh_root(cell_surface([(0, 0, 0)]))
            component.meshBodies.append(MeshBody('second', transformed(cell_surface([(0, 0, 0)]), translation=(3, 0, 0))))
            self.reject(component, density(**controls), ResourceLimitError, 'aggregate')

    def test_topology_flags_are_not_a_substitute_for_source_validation(self):
        component, body = mesh_root(cell_surface([(0, 0, 0)]))
        body.isClosed = body.isOriented = False
        robot, _, _ = self.export(component, density(), 'flags')
        self.assertEqual(float(robot.find('link/inertial/mass').get('value')), .001)

    def test_density_does_not_close_a_source_seam_by_stl_quantization(self):
        cube = cell_surface([(0, 0, 0)])
        seam = list(cube)
        point = seam[0][1]
        seam[0] = (seam[0][0], tuple(v+1e-10 if v == 1 else v for v in point), seam[0][2])
        component, _ = mesh_root(seam)
        self.reject(component, density(), pattern='open or nonmanifold')

    def test_hidden_ancestor_and_mesh_occurrence_do_not_need_unused_physics(self):
        base = occurrence('base:1', [Body('solid', 0)], component='base_link')
        parent = occurrence('hidden:1')
        parent.isVisible = False
        child = occurrence('hidden:1+scan:1')
        child.meshBodies = [MeshBody('unread mesh', (), owner=child, native=MeshBody('unread mesh', ()))]
        robot, _, exported = self.export(root([child, base, parent]), name='hidden')
        self.assertEqual(READS, [])
        self.assertEqual(exported, ['solid'])
        self.assertEqual([link.get('name') for link in robot.findall('link')], ['base_link'])

    def test_unreadable_native_display_is_transactional(self):
        class Unreadable(MeshBody):
            @property
            def displayMesh(self):
                raise RuntimeError('forced native display extraction failure')
        component = root()
        component.meshBodies = [Unreadable('scan', cell_surface([(0, 0, 0)]))]
        self.reject(component, density(), RuntimeError, 'forced native display extraction failure')

    def test_mesh_visual_collision_and_com_follow_moving_frame_at_two_samples(self):
        native_triangles = transformed(cell_surface([(0, 0, 0)]), scale=(2, 3, 5))
        native = MeshBody('scan', native_triangles)
        frame = occurrence('reference:1', matrix=pose((3, -4, 5), (.23, -.37, .19)))
        owner = occurrence('reference:1+scan:1', matrix=pose((-8, 12, -3), (.31, .27, -.41)))
        owner.meshBodies = [MeshBody('scan', (), owner=owner, native=native)]
        drive = hinge('drive', frame, None)
        from tests.test_exporter_logic import FakeMatrix
        drive.geometryTwoTransform = FakeMatrix(pose((3, -4, 5), (0, 0, 0)))
        component = root([owner, frame], joints=[fixed('mount', owner, frame, (-8, 12, -3)), drive])
        robot, target, _ = self.export(component, dict(density(2000), root_occurrence='design_root'), 'moving')
        name = 'reference_1_scan_1'
        matrix = owner.transform2.asArray()
        root_points = [[sum(matrix[4*i+j]*p[j] for j in range(3))*.01 + matrix[4*i+3]*.01 for i in range(3)]
                       for triangle in native_triangles for p in triangle]
        root_center = [sum(matrix[4*i+j]*p for j, p in enumerate((.01, .015, .025)))+matrix[4*i+3]*.01 for i in range(3)]
        pivot = (.03, -.04, .05)
        data = (target / 'meshes' / (name + '.stl')).read_bytes()

        def world(link, point, q):
            joint = next((j for j in robot.findall('joint') if j.find('child').get('link') == link), None)
            if joint is None:
                return point
            if joint.get('name') == 'drive':
                point = axis_rotate(point, vector(joint.find('axis'), 'xyz'), q)
            return world(joint.find('parent').get('link'), origin_apply(point, joint.find('origin')), q)

        comparisons = 0
        for q in (0, .37):
            for kind in ('visual', 'collision'):
                element = robot.find("link[@name='%s']/%s" % (name, kind))
                for n, point in enumerate(root_points):
                    row = struct.unpack_from('<12fH', data, 84 + (n//3)*50)
                    local = tuple(v*.001 for v in row[3+(n%3)*3:6+(n%3)*3])
                    actual = world(name, origin_apply(local, element.find('origin')), q)
                    expected = add(pivot, axis_rotate(subtract(point, pivot), (0, 0, 1), q))
                    for got, want in zip(actual, expected):
                        self.assertAlmostEqual(got, want, delta=3e-8)
                    comparisons += 1
            physics = robot.find("link[@name='%s']/inertial" % name)
            actual = world(name, vector(physics.find('origin'), 'xyz'), q)
            expected = add(pivot, axis_rotate(subtract(root_center, pivot), (0, 0, 1), q))
            for got, want in zip(actual, expected):
                self.assertAlmostEqual(got, want, delta=3e-8)
        self.assertEqual(comparisons, 144)
        self.assertEqual(READS, [native])
        (self.output / 'expected.json').write_text(json.dumps({'evidence_kind': 'analytic_api_double',
            'positions': [0, .37], 'vertex_comparisons': 144, 'com_vectors': 2,
            'axis_root': [0, 0, 1], 'pivot_m': list(pivot), 'mass_kg': .06,
            'source_preserved': True}, indent=2)+'\n')


if __name__ == '__main__':
    unittest.main()
