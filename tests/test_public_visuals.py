"""Public native-selection and transactional visual-bundle checks."""
import copy
import importlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

from tests.test_fusion_inputs import Body, design, fixed, occurrence, root
from tests.test_exporter_logic import FakeCollection
from tests.test_visual_assets import (body, face, mesh, snapshot, color_appearance, image_appearance,
                                     build_obj_assets, png_rgba, KNOWN_JPEG, progressive_gray_jpeg,
                                     decode_assets, with_properties, Frozen)
from tests.fixtures import model, write_tetrahedron, MESH_BOUNDS_MM
from URDF_Exporter.core.Write import write_ros2_package

ENTRY = importlib.import_module('URDF_Exporter.URDF_Exporter')


def colored_body(name='solid', x=0, extras=False):
    result = Body(name, x)
    appearance = color_appearance(255, 0, 0, 128)
    if extras:
        appearance = with_properties(appearance, Frozen(id='roughness', objectType='adsk::core::FloatProperty'))
    result.faces = FakeCollection([face(appearance)])
    result.visibleOpacity, result.appearance = .5, None
    return result


class PublicVisualTest(unittest.TestCase):
    def test_native_public_selection_safe_context_ids_and_transform_once(self):
        source = colored_body()
        native = source.faces[0]
        proxy = face(color_appearance(0, 0, 255), mesh(vertices=[(999, 0, 0), (999, 1, 0), (999, 0, 1)]), native=native)
        source.faces = FakeCollection([proxy])
        hidden = colored_body('hidden', 99)
        hidden.isVisible = False
        owner = occurrence('機構:1+腕:1', [source, hidden], component='base_link',
                           matrix=[0, -1, 0, 2, 1, 0, 0, 3, 0, 0, 1, 4, 0, 0, 0, 1])
        owner.appearance = None
        fixture, exported = design(root([owner]))
        before = source.mesh
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / 'visual_description'
            ENTRY.export_design(fixture, target, config={'visual_materials': 'strict'})
            robot = ET.parse(target / 'urdf/visual.urdf').getroot()
            self.assertIsNone(robot.find('link/visual/material'))
            self.assertEqual(robot.find('link/visual/geometry/mesh').get('scale'), '0.001 0.001 0.001')
            self.assertTrue(robot.find('link/collision/geometry/mesh').get('filename').endswith('/base_link.stl'))
            self.assertEqual(exported, ['solid'])
            data = json.loads((target / 'config/robot_data.yaml').read_text())
            descriptor = data['visual_assets']['base_link']
            files = {item['path']: (target / item['path']).read_bytes() for item in descriptor['files']}
            obj, materials, _ = decode_assets({'descriptor': descriptor, 'files': files})
            self.assertEqual(obj['vertices'], [(20., 30., 40.), (20., 40., 40.), (10., 30., 40.)])
            self.assertEqual(materials[obj['triangles'][0][0]]['Kd'], ['0', '0', '1'])
            self.assertEqual(descriptor['surfaces'][0]['context_id'], 'base_link_body_0')
            self.assertEqual(source.mesh, before)
            self.assertEqual(float(robot.find('link/inertial/mass').get('value')), 1.)

    def test_legacy_does_not_read_faces_and_strict_failure_keeps_prior_package(self):
        source = Body('solid', 0)
        fixture, _ = design(root(bodies=[source]))
        with tempfile.TemporaryDirectory() as temporary:
            prior = Path(temporary) / 'legacy_description'
            ENTRY.export_design(fixture, prior)
            old = (prior / 'urdf/legacy.urdf').read_bytes()
            self.assertIsNotNone(ET.fromstring(old).find('link/visual/material'))
            with self.assertRaises(ValueError):
                ENTRY.export_design(fixture, Path(temporary) / 'blocked_description', config={'visual_materials': 'strict'})
            self.assertEqual((prior / 'urdf/legacy.urdf').read_bytes(), old)
            self.assertEqual(sorted(p.name for p in Path(temporary).iterdir()), ['legacy_description'])

    def test_subset_native_loss_receipts_persist_without_changing_physics(self):
        source = colored_body(extras=True)
        fixture, exported = design(root(bodies=[source]))
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / 'subset_description'
            ENTRY.export_design(fixture, target, config={'visual_materials': 'diffuse-subset'})
            sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
            receipt = sidecar['visual_assets']['base_link']['surfaces'][0]['appearance']
            self.assertEqual(receipt['losses'], [{'id': 'roughness', 'type': 'adsk::core::FloatProperty', 'reason': 'omitted-shader-property'}])
            self.assertIn('lossy-diffuse-subset-not-shader-equivalent', sidecar['warnings'])
            robot = ET.parse(target / 'urdf/subset.urdf').getroot()
            self.assertEqual(float(robot.find('link/inertial/mass').get('value')), 1.)
            self.assertEqual(exported, ['solid'])

    def test_pure_writer_bundle_validation_and_png_jpeg_uv_provenance(self):
        joints, inertial = model(True)
        with tempfile.TemporaryDirectory() as temporary:
            images = []
            for name, payload in (('selected.png', png_rgba()), ('baseline.jpg', KNOWN_JPEG), ('progressive.jpg', progressive_gray_jpeg())):
                path = Path(temporary) / name
                path.write_bytes(payload)
                images.append(path)
            assets = build_obj_assets(snapshot(body([face(image_appearance(p)) for p in images])))
            bundle = {'base_link': assets, 'mount': assets}
            target = Path(temporary) / 'portable_description'
            write_ros2_package(target, 'portable', joints, inertial, visual_assets=bundle,
                               config={'visual_materials': 'strict'},
                               write_meshes=lambda paths: [write_tetrahedron(p, MESH_BOUNDS_MM[n]) for n, p in paths.items()])
            sidecar = json.loads((target / 'config/robot_data.yaml').read_text())
            self.assertEqual(len(sidecar['visual_assets']), 2)
            self.assertIn('jpeg-structure-only-not-entropy-decoded', sidecar['warnings'])
            for entry in assets['descriptor']['files']:
                self.assertEqual((target / entry['path']).read_bytes(), assets['files'][entry['path']])
            self.assertEqual([p.read_bytes() for p in images], [png_rgba(), KNOWN_JPEG, progressive_gray_jpeg()])
            robot = ET.parse(target / 'urdf/portable.urdf').getroot()
            for actual, expected in zip(map(float, robot.find("link[@name='mount']/visual/origin").get('xyz').split()), (.1, .2, -.3)):
                self.assertAlmostEqual(actual, expected, places=12)
            self.assertFalse(robot.findall('link/visual/material'))

    def test_invalid_public_assets_fail_before_callback_and_do_not_publish(self):
        joints, inertial = model(True)
        assets = build_obj_assets(snapshot(body([face(color_appearance(255, 0, 0))])))
        for mode in ('hash', 'size', 'escape', 'extra', 'missing', 'policy', 'triangle', 'receipt', 'membership', 'legacy'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as temporary:
                changed = copy.deepcopy(assets)
                bundles = dict.fromkeys(inertial, changed)
                config = {'visual_materials': 'strict'}
                if mode == 'hash':
                    changed['descriptor']['files'][0]['sha256'] = '0'*64
                elif mode == 'size':
                    changed['descriptor']['files'][0]['size'] += 1
                elif mode == 'escape':
                    changed['descriptor']['files'][0]['path'] = '../escape.obj'
                elif mode == 'extra':
                    changed['files']['meshes/extra.obj'] = b'v 0 0 0'
                elif mode == 'missing':
                    changed['files'].pop(changed['descriptor']['mesh_path'])
                elif mode == 'policy':
                    changed['descriptor']['urdf_material_override'] = [1, 0, 0, 1]
                elif mode == 'triangle':
                    changed['descriptor']['triangle_count'] += 1
                elif mode == 'receipt':
                    changed['descriptor']['surfaces'][0]['appearance'] = None
                elif mode == 'membership':
                    bundles.pop('mount')
                else:
                    config = {}
                with self.assertRaises(ValueError):
                    write_ros2_package(Path(temporary) / 'blocked_description', 'blocked', joints, inertial,
                                       config=config, visual_assets=bundles,
                                       write_meshes=lambda _: self.fail('Invalid bundle reached mesh staging'))
                self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_mesh_callback_conflicts_or_failure_roll_back_asset_stage(self):
        joints, inertial = model(True)
        assets = build_obj_assets(snapshot(body([face(color_appearance(255, 0, 0))])))
        with tempfile.TemporaryDirectory() as temporary:
            for mode in ('conflict', 'symlink', 'failure'):
                def callback(paths):
                    for name, path in paths.items():
                        write_tetrahedron(path, MESH_BOUNDS_MM[name])
                    stage = next(iter(paths.values())).parent.parent
                    path = stage / assets['descriptor']['mesh_path']
                    if mode == 'failure':
                        raise RuntimeError('mesh callback failed')
                    if mode == 'symlink':
                        path.symlink_to(Path(temporary) / 'outside')
                    else:
                        path.write_bytes(b'conflict')
                with self.subTest(mode=mode), self.assertRaises((ValueError, RuntimeError)):
                    write_ros2_package(Path(temporary) / 'blocked_description', 'blocked', joints, inertial,
                                       config={'visual_materials': 'strict'}, visual_assets=dict.fromkeys(inertial, assets),
                                       write_meshes=callback)
                self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_optional_material_fixture_and_installed_receipts_validate_all_resources(self):
        from tests.fixtures import generate_fixture, fixture_oracle
        from tools.verify_loaders import asset_receipts
        from tools.validate_urdf import lint_file
        with tempfile.TemporaryDirectory() as temporary:
            package = generate_fixture(temporary, 'materials')
            descriptors = asset_receipts(package)
            self.assertEqual(set(descriptors), {'base_link', 'mount'})
            for name, descriptor in descriptors.items():
                self.assertEqual(descriptor['triangle_count'], fixture_oracle('materials')['zero']['materials'][name]['triangles'])
                self.assertEqual(len(descriptor['images']), 3)
                self.assertEqual(sum(len(s['appearance']['losses']) for s in descriptor['surfaces']), 4)
            path = package / 'urdf/fixture_materials.urdf'
            self.assertEqual(lint_file(path, package / 'meshes'), ([], []))
            image = package / descriptors['base_link']['images'][0]['path']
            image.write_bytes(image.read_bytes()+b'mutation')
            with self.assertRaisesRegex(AssertionError, 'manifest'):
                asset_receipts(package)
            self.assertTrue(lint_file(path, package / 'meshes')[0])


if __name__ == '__main__':
    unittest.main()
