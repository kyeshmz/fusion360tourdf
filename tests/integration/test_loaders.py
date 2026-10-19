import os
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

from tests.fixtures import generate_fixture, oracle
from tools.verify_loaders import check_urdfdom, expand, load_and_check, mesh_path, verify_fixtures


class LoaderIntegrationTest(unittest.TestCase):
    def test_component_urdfs_load_their_shared_meshes_independently(self):
        package = generate_fixture(self.output / 'components')
        assembly = ET.parse(package / 'urdf/fixture_articulated.urdf').getroot()
        for link in assembly.findall('link'):
            name = link.get('name')
            component = package / 'urdf/components' / (name + '.urdf')
            check_urdfdom(component, self.output / (name + '-component-urdfdom.log'))
            report = load_and_check(component, package)
            self.assertEqual(set(report['zero']['visual']), {name})
            self.assertEqual(set(report['zero']['collision']), {name})

    @classmethod
    def setUpClass(cls):
        base = os.environ.get('VERIFY_OUTPUT') or os.environ.get('TMPDIR')
        if base:
            Path(base).mkdir(parents=True, exist_ok=True)
        cls.output = Path(tempfile.mkdtemp(prefix='fusion-loaders-', dir=base))
        print('Loader evidence at ' + str(cls.output), flush=True)

    def test_generated_packages_load_and_move(self):
        report = verify_fixtures(self.output / 'packages')
        self.assertEqual(set(report), {'articulated', 'fixed', 'boxes'})
        self.assertEqual(set(report['articulated']), {'zero', 'moved'})
        self.assertEqual(set(report['fixed']), {'zero'})

    def test_expanded_export_loads_without_a_fixture_oracle(self):
        folder = self.output / 'external-export'
        package = generate_fixture(folder)
        urdf = expand(package, folder / 'robot.urdf')
        check_urdfdom(urdf, folder / 'urdfdom.log')
        report = load_and_check(urdf, package)
        self.assertEqual(set(report), {'zero'})
        expected = [[.19, .11, .31], [.24, .14, .39]]
        for actual_row, expected_row in zip(report['zero']['visual']['arm'], expected):
            for actual, value in zip(actual_row, expected_row):
                self.assertAlmostEqual(actual, value, places=6)

    def test_shared_fusion_extraction_loads_and_moves_a_rigid_attachment(self):
        import importlib
        import math
        import numpy as np
        from yourdfpy import URDF
        from tests.test_exporter_logic import identity_at
        from tests.test_fusion_inputs import Body, design, group, hinge, occurrence, root

        for attachment in ('rigid-group', 'ground-to-parent'):
            with self.subTest(attachment=attachment):
                base = occurrence('base:1', [Body('base', 0)], component='base_link')
                b = occurrence('b:1', [Body('b', 20)], matrix=identity_at([20, 0, 0]))
                path = 'a:1' if attachment == 'rigid-group' else 'b:1+a:1'
                a = occurrence(path, [Body('a', 10)], component='a', matrix=identity_at([10, 0, 0]))
                if attachment == 'ground-to-parent':
                    a.assemblyContext = b
                    a.isGroundToParent = True
                groups = [group('rigid_pair', [a, b])] if attachment == 'rigid-group' else []
                fixture, _ = design(root([base, a, b], joints=[hinge('drive', b, base)], groups=groups))
                package = self.output / 'extraction' / attachment / 'extracted_description'
                importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(fixture, package)
                urdf = expand(package, package.parent / 'extracted.urdf')
                check_urdfdom(urdf, package.parent / 'urdfdom.log')
                load_and_check(urdf, package)
                robot = URDF.load(str(urdf), filename_handler=lambda fname: mesh_path(fname, package, urdf),
                                 build_collision_scene_graph=True, load_collision_meshes=True, force_mesh=True)
                a_name = 'a_1' if attachment == 'rigid-group' else 'b_1_a_1'
                for q, expected in ((0.0, {
                        'base_link': [[-.01, -.01, -.01], [.01, .01, .01]],
                        a_name: [[.09, -.01, -.01], [.11, .01, .01]],
                        'b_1': [[.19, -.01, -.01], [.21, .01, .01]]}),
                        (math.pi / 2, {
                        'base_link': [[-.01, -.01, -.01], [.01, .01, .01]],
                        a_name: [[-.01, .09, -.01], [.01, .11, .01]],
                        'b_1': [[-.01, .19, -.01], [.01, .21, .01]]})):
                    robot.update_cfg({'drive': q})
                    for scene in (robot.scene, robot.collision_scene):
                        seen = set()
                        for node in scene.graph.nodes_geometry:
                            transform, geometry = scene.graph.get(node)
                            vertices = scene.geometry[geometry].vertices
                            points = vertices @ transform[:3, :3].T + transform[:3, 3]
                            owner = scene.graph.transforms.parents[node]
                            np.testing.assert_allclose([points.min(axis=0), points.max(axis=0)],
                                                       expected[owner], atol=1e-7, rtol=0)
                            seen.add(owner)
                        self.assertEqual(seen, set(expected))

    def test_axis_mutation_changes_observed_fk(self):
        folder = self.output / 'axis-mutation'
        package = generate_fixture(folder)
        urdf = expand(package, folder / 'mutated.urdf')
        tree = ET.parse(urdf)
        tree.find("joint[@name='shoulder']/axis").set('xyz', '1 0 0')
        tree.write(urdf)
        check_urdfdom(urdf, folder / 'urdfdom.log')
        with self.assertRaises(AssertionError):
            load_and_check(urdf, package, oracle())

    def test_valid_but_shifted_mesh_is_rejected(self):
        folder = self.output / 'mesh-mutation'
        package = generate_fixture(folder)
        urdf = expand(package, folder / 'model.urdf')
        mesh = package / 'meshes' / 'arm.stl'
        data = bytearray(mesh.read_bytes())
        for triangle in range(struct.unpack_from('<I', data, 80)[0]):
            for vertex in range(3):
                offset = 84 + 50 * triangle + 12 + 12 * vertex
                struct.pack_into('<f', data, offset, struct.unpack_from('<f', data, offset)[0] + 100)
        mesh.write_bytes(data)
        with self.assertRaises(AssertionError):
            load_and_check(urdf, package, oracle())

    def test_missing_mesh_is_not_a_successful_empty_scene(self):
        folder = self.output / 'missing-mesh'
        package = generate_fixture(folder)
        urdf = expand(package, folder / 'model.urdf')
        (package / 'meshes' / 'arm.stl').unlink()
        with self.assertRaisesRegex(AssertionError, 'Missing or external mesh'):
            load_and_check(urdf, package, oracle())


if __name__ == '__main__':
    unittest.main()
