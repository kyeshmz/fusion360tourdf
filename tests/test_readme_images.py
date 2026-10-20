import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from tests.test_fusion_verifier import load_entrypoint_without_sdk
from tools.verify_export import copy_fusion_image


REPOSITORY = Path(__file__).resolve().parents[1]


class ReadmeImagesTest(unittest.TestCase):
    def test_verification_images_are_valid_sized_pngs_with_matching_receipts(self):
        directory = REPOSITORY / 'docs/images'
        gallery = (REPOSITORY / 'docs/verification.md').read_text()
        render = json.loads((directory / 'images.json').read_text())
        images = list(render['images'].values())
        self.assertEqual(len(images), 3)
        for metadata in images:
            with self.subTest(path=metadata['path']):
                data = (directory / metadata['path']).read_bytes()
                self.assertEqual(data[:8], b'\x89PNG\r\n\x1a\n')
                self.assertEqual(struct.unpack('>II', data[16:24]), (1280, 960))
                self.assertEqual(hashlib.sha256(data).hexdigest(), metadata['sha256'])
                self.assertIn('](images/' + metadata['path'] + ')', gallery)

    def test_fusion_image_copy_checks_hash_and_supports_older_bundles(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory)
            output = bundle / 'images'
            output.mkdir()
            receipt = bundle / 'native.json'
            receipt.write_text('{}')
            self.assertIsNone(copy_fusion_image(bundle, output, 'demo'))
            source = bundle / 'fusion.png'
            source.write_bytes(b'offline screenshot double')
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            receipt.write_text(json.dumps({'images': {'fusion': {'sha256': digest}}}))
            metadata = copy_fusion_image(bundle, output, 'demo')
            self.assertEqual(metadata['path'], 'demo-fusion.png')
            self.assertEqual((output / metadata['path']).read_bytes(), source.read_bytes())
            source.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'changed since capture'):
                copy_fusion_image(bundle, output, 'changed')
            self.assertFalse((output / 'changed-fusion.png').exists())


class FusionCaptureTest(unittest.TestCase):
    def capture_module(self):
        adsk = types.ModuleType('adsk')
        core = types.ModuleType('adsk.core')
        fusion = types.ModuleType('adsk.fusion')
        core.Point3D = core.Vector3D = types.SimpleNamespace(create=lambda *values: values)
        adsk.core, adsk.fusion = core, fusion
        spec = importlib.util.spec_from_file_location('tools.create_demo._image_test',
            REPOSITORY / 'tools/create_demo/create_demo.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'adsk': adsk, 'adsk.core': core, 'adsk.fusion': fusion,
                                     'tools.verify_fusion.verify_fusion': load_entrypoint_without_sdk()}):
            spec.loader.exec_module(module)
        return module

    def test_capture_uses_native_viewport_and_requested_resolution(self):
        module = self.capture_module()
        calls = []

        def save(filename, width, height):
            calls.append((filename, width, height))
            Path(filename).write_bytes(b'offline native viewport double')
            return True

        viewport = types.SimpleNamespace(camera=types.SimpleNamespace(), refresh=lambda: True, saveAsImageFile=save)
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'nested/fusion.png'
            metadata = module.capture_image(types.SimpleNamespace(activeViewport=viewport), filename)
            self.assertEqual(calls, [(str(filename.resolve()), 1280, 960)])
            self.assertEqual(metadata['sha256'], hashlib.sha256(filename.read_bytes()).hexdigest())
            self.assertEqual(viewport.camera.upVector, (0, 0, 1))

    def test_failed_capture_does_not_report_success(self):
        module = self.capture_module()
        viewport = types.SimpleNamespace(camera=types.SimpleNamespace(), refresh=lambda: True,
                                         saveAsImageFile=lambda *arguments: False)
        with tempfile.TemporaryDirectory() as directory, self.assertRaisesRegex(RuntimeError, 'capture failed'):
            module.capture_image(types.SimpleNamespace(activeViewport=viewport), Path(directory) / 'fusion.png')


if __name__ == '__main__':
    unittest.main()
