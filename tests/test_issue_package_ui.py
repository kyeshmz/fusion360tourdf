"""Run the real script boundary with UI/API doubles, never a native GUI receipt."""

import importlib
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from tests.test_fusion_inputs import Body, design, root
from tests.test_joint_settings import scene


class ScriptBoundaryTest(unittest.TestCase):
    def setUp(self):
        self.entry = importlib.import_module('URDF_Exporter.URDF_Exporter')
        temporary = tempfile.TemporaryDirectory(prefix='script-boundary-')
        self.addCleanup(temporary.cleanup)
        self.folder = Path(temporary.name) / 'space 雪 & !'
        self.folder.mkdir()
        self.messages = []
        self.dialogs = []
        self.dialog_result = 1

        def folder_dialog():
            dialog = types.SimpleNamespace(folder=str(self.folder), title='',
                                           showDialog=lambda: self.dialog_result)
            self.dialogs.append(dialog)
            return dialog

        self.ui = types.SimpleNamespace(createFolderDialog=folder_dialog,
                                        messageBox=lambda text, title: self.messages.append((text, title)))
        self.app = types.SimpleNamespace(userInterface=self.ui, activeProduct=None)
        self.patches = [
            patch.object(self.entry.adsk.core, 'Application',
                         types.SimpleNamespace(get=lambda: self.app), create=True),
            patch.object(self.entry.adsk.fusion, 'Design',
                         types.SimpleNamespace(cast=lambda product: product), create=True),
        ]
        sdk_cores = {id(core): core for core in (self.entry.adsk.core, self.entry.utils.adsk.core)}
        self.patches.extend(patch.object(core, 'DialogResults',
                            types.SimpleNamespace(DialogOK=1), create=True)
                            for core in sdk_cores.values())
        for context in self.patches:
            context.start()
            self.addCleanup(context.stop)

    def config_loader(self, values):
        original = self.entry.config_mod.load_config
        reads = []
        pending = iter(values)

        def load(script_dir=None, overrides=None):
            if script_dir is not None:
                reads.append(Path(script_dir))
                return original(overrides=next(pending))
            return original(overrides=overrides)

        return patch.object(self.entry.config_mod, 'load_config', side_effect=load), reads

    def test_cancel_returns_before_export_and_does_not_create_output(self):
        fixture, _ = design(root(bodies=[Body('base', 0)]))
        self.app.activeProduct = fixture
        self.dialog_result = 0
        loader, reads = self.config_loader([{}])
        with loader, patch.object(self.entry, 'export_design') as export:
            self.entry.run(None)
        export.assert_not_called()
        self.assertEqual(list(self.folder.iterdir()), [])
        self.assertEqual(len(reads), 1)
        self.assertEqual(len(self.dialogs), 1)
        self.assertIn('canceled', self.messages[-1][0])

    def test_no_active_design_does_not_read_config_or_open_dialog(self):
        with patch.object(self.entry.config_mod, 'load_config') as load:
            self.entry.run(None)
        load.assert_not_called()
        self.assertEqual(self.dialogs, [])
        self.assertEqual(self.messages, [('No active Fusion design', 'fusion360tourdf')])
        self.assertEqual(list(self.folder.iterdir()), [])

    def test_rerun_reads_new_config_and_active_design_without_stale_state(self):
        first_root, first_drive, _ = scene()
        second_root, second_drive, _ = scene()
        first_root.name, second_root.name = 'first robot', 'second robot'
        first, first_exports = design(first_root)
        second, second_exports = design(second_root)
        loader, reads = self.config_loader([
            {'joint_limits': {'drive_pin': {'effort': 4, 'velocity': .8}}},
            {'joint_limits': {'drive_pin': {'effort': 7, 'velocity': .3}}},
        ])
        with loader:
            for fixture in (first, second):
                self.app.activeProduct = fixture
                self.entry.run(None)
        for name, effort, velocity in (('first', 4, .8), ('second', 7, .3)):
            package = self.folder / (name + '_description')
            element = ET.parse(package / ('urdf/' + name + '.urdf')).find(
                "joint[@name='drive_pin']/limit")
            self.assertEqual(float(element.get('effort')), effort)
            self.assertEqual(float(element.get('velocity')), velocity)
            self.assertTrue((package / 'viewers/display.launch.py').is_file())
            self.assertTrue((package / 'launch/control.launch.py').is_file())
        self.assertEqual(len(reads), 2)
        self.assertEqual(set(reads), {Path(self.entry.__file__).resolve().parent})
        self.assertEqual(len(self.dialogs), 2)
        self.assertTrue(all('Successfully created' in text for text, _ in self.messages))
        self.assertEqual(first_exports, ['base', 'arm', 'tool'])
        self.assertEqual(second_exports, ['base', 'arm', 'tool'])
        self.assertEqual(first_drive.jointMotion.rotationValue, .25)
        self.assertEqual(second_drive.jointMotion.rotationValue, .25)

    def test_failed_mesh_export_can_retry_and_existing_output_is_preserved(self):
        component = root(bodies=[Body('base', 0)])
        component.name = 'retry robot'
        broken, _ = design(component, corrupt=True)
        valid, _ = design(component)
        marker = self.folder / 'keep.txt'
        marker.write_bytes(b'not owned by exporter')
        loader, _ = self.config_loader([{}, {}, {}])
        with loader:
            self.app.activeProduct = broken
            self.entry.run(None)
            self.assertIn('Export failed:', self.messages[-1][0])
            self.assertEqual(list(self.folder.iterdir()), [marker])
            self.app.activeProduct = valid
            self.entry.run(None)
            self.assertIn('Successfully created', self.messages[-1][0])
            package = self.folder / 'retry_description'
            before = {p.relative_to(package): p.read_bytes() for p in package.rglob('*') if p.is_file()}
            self.entry.run(None)
            self.assertIn('Export failed:', self.messages[-1][0])
            self.assertIn('FileExistsError', self.messages[-1][0])
            self.assertEqual({p.relative_to(package): p.read_bytes()
                              for p in package.rglob('*') if p.is_file()}, before)
        self.assertEqual(marker.read_bytes(), b'not owned by exporter')
        self.assertEqual(sorted(p.name for p in self.folder.iterdir()), ['keep.txt', 'retry_description'])


if __name__ == '__main__':
    unittest.main()
