"""Surface BRep preflight is a named rejection policy, not invented physics."""

import importlib
from pathlib import Path
import struct
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

from tests.test_exporter_logic import FakePoint
from tests.test_fusion_inputs import Body, design, fixed, occurrence, root
from tests.test_combined_frames import source_snapshot


class SurfaceBody(Body):
    def __init__(self, name='sheet', visible=True):
        super().__init__(name, 0, mass=0, visible=visible)
        self.isValid, self.isSolid = True, False
        self.physics_reads = 0
        self.properties.centerOfMass = FakePoint([-1/3, -1/3, 0])
        self.properties.volume = 0.
        self.boundingBox = types.SimpleNamespace(minPoint=FakePoint([-1, -1, 0]), maxPoint=FakePoint([1, 1, 0]))
        self.mesh = (b'surface BRep API double'.ljust(80, b'\0') + struct.pack('<I', 1)
                     + struct.pack('<12fH', 0, 0, 1, -10, -10, 0, 10, -10, 0, -10, 10, 0, 0))

    def getPhysicalProperties(self, accuracy):
        self.physics_reads += 1
        return super().getPhysicalProperties(accuracy)


def fixture_with_history(component):
    fixture, exported = design(component)
    fixture.unitsManager = types.SimpleNamespace(defaultLengthUnits='cm')
    fixture.timeline = types.SimpleNamespace(markerPosition=3, entityTokens=('surface', 'joint'))
    return fixture, exported


class BrepSurfacePolicyTests(unittest.TestCase):
    def reject_surface(self, component, surface, *, config=None, path='design_root', pattern=None):
        fixture, exported = fixture_with_history(component)
        before = source_snapshot(fixture)
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/'surface_description'
            diagnostic = pattern or 'BREP_SURFACE_PHYSICS_UNSUPPORTED.*' + path + '.*sheet'
            with self.assertRaisesRegex(ValueError, diagnostic):
                entry.export_design(fixture, target, config=config)
            self.assertFalse(target.exists())
            self.assertEqual(list(Path(directory).iterdir()), [])
        self.assertEqual(exported, [])
        self.assertEqual(surface.physics_reads, 0)
        self.assertEqual(source_snapshot(fixture), before)

    def test_surface_only_brep_rejects_with_body_identity_before_physics(self):
        surface = SurfaceBody()
        self.reject_surface(root(bodies=[surface]), surface)

    def test_surface_is_not_hidden_by_positive_aggregate_solid_mass(self):
        surface = SurfaceBody()
        self.reject_surface(root(bodies=[Body('solid', 3, mass=2), surface]), surface)

    def test_nested_surface_diagnostic_preserves_raw_context_and_source(self):
        surface = SurfaceBody()
        child = occurrence('assembly:1+sheet:2', [surface])
        component = root([child], bodies=[Body('solid', 3, mass=2)],
                         joints=[fixed('attachment', child, None)])
        self.reject_surface(component, surface, path='assembly:1\\+sheet:2')

    def test_hidden_surface_is_not_selected_or_read(self):
        surface, solid = SurfaceBody(visible=False), Body('solid', 3, mass=2)
        fixture, exported = fixture_with_history(root(bodies=[solid, surface]))
        before = source_snapshot(fixture)
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        with tempfile.TemporaryDirectory() as directory:
            package = entry.export_design(fixture, Path(directory)/'solid_description')
            robot = ET.parse(package/'urdf/solid.urdf')
            self.assertEqual(float(robot.find('link/inertial/mass').get('value')), 2.)
            self.assertEqual(list(map(float, robot.find('link/inertial/origin').get('xyz').split())), [.03, 0., 0.])
            self.assertEqual((package/'meshes/base_link.stl').read_bytes()[84:], solid.mesh[84:])
        self.assertEqual(exported, ['solid'])
        self.assertEqual(surface.physics_reads, 0)
        self.assertEqual(source_snapshot(fixture), before)

    def test_mesh_density_does_not_silently_apply_to_surface_brep(self):
        surface = SurfaceBody()
        self.reject_surface(root(bodies=[surface]), surface,
                            config={'mesh_physics': {'density_kg_m3': 1000.}}, pattern='UNUSED_MESH_DENSITY')

    def test_invalid_or_missing_brep_metadata_fails_closed_with_body_name(self):
        entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        for mutation in ('invalid', 'missing_solid', 'nonbool_solid'):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                body = Body('named_body', 0)
                if mutation == 'invalid':
                    body.isValid = False
                elif mutation == 'missing_solid':
                    del body.isSolid
                else:
                    body.isSolid = 1
                fixture, exported = design(root(bodies=[body]))
                with self.assertRaisesRegex(ValueError, 'BREP_BODY_.*design_root.*named_body'):
                    entry.export_design(fixture, Path(directory)/'invalid_description')
                self.assertEqual(exported, [])
                self.assertEqual(list(Path(directory).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
