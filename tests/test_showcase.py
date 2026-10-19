import json
import hashlib
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from tests.fixtures import oracle
from tools.capture_rviz import component_labels, rviz_config
from tools.make_walkthrough import chapter_plan, validate_capture


class ShowcaseTest(unittest.TestCase):
    def test_published_media_and_component_catalog_match_their_receipts(self):
        root = Path(__file__).resolve().parents[1]
        media = root/'docs/media'
        movie = json.loads((media/'walkthrough.json').read_text())
        capture = json.loads((media/'rviz-capture.json').read_text())
        digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
        self.assertEqual(movie['status'],'pass')
        self.assertEqual(digest(media/'walkthrough.mp4'),movie['movie']['sha256'])
        self.assertEqual(digest(media/'rviz-capture.json'),movie['rviz_receipt_sha256'])
        self.assertEqual((media/'walkthrough.mp4').read_bytes()[4:8],b'ftyp')
        self.assertEqual((movie['movie']['width'],movie['movie']['height']),(1280,720))
        self.assertGreater(movie['movie']['duration'],90)
        self.assertGreaterEqual(len(movie['chapters']),12)
        for item in capture['images'].values():
            self.assertEqual(digest(media/item['path']),item['sha256'])
        self.assertEqual(digest(media/capture['movie']['path']),capture['movie']['sha256'])
        for item in movie['screenshots'].values():
            self.assertEqual(digest(media/item['path']),item['sha256'])
        provenance=json.loads((root/'Example/rotary_slide_provenance.json').read_text())
        self.assertEqual(provenance['native_receipt_sha256'],movie['native_receipt_sha256'])
        package=root/'Example/rotary_slide_description'
        actual={path.relative_to(package).as_posix():digest(path) for path in package.rglob('*') if path.is_file()}
        self.assertEqual(actual,provenance['files_sha256'])
        for item in capture['components']:
            self.assertTrue((package/item['urdf']).is_file())

    def test_labels_are_locked_to_matching_urdf_frames_and_catalog_files(self):
        class Marker:
            TEXT_VIEW_FACING, LINE_LIST, ADD = 9, 5, 0

            def __init__(self):
                self.header = types.SimpleNamespace()
                self.pose = types.SimpleNamespace(orientation=types.SimpleNamespace())
                self.scale, self.color = types.SimpleNamespace(), types.SimpleNamespace()

        with patch.dict(sys.modules, {
                'geometry_msgs.msg': types.SimpleNamespace(Point=lambda **values: types.SimpleNamespace(**values)),
                'visualization_msgs.msg': types.SimpleNamespace(Marker=Marker, MarkerArray=lambda: types.SimpleNamespace(markers=[]))}):
            expected = oracle()
            labels, catalog = component_labels(expected)
        self.assertEqual(len(labels.markers),2*len(expected['zero']['link_poses']))
        for index, item in enumerate(catalog):
            text, line = labels.markers[2*index:2*index+2]
            self.assertEqual(text.header.frame_id,item['link'])
            self.assertEqual(line.header.frame_id,item['link'])
            self.assertTrue(text.frame_locked and line.frame_locked)
            self.assertNotIn(' ',text.text)
            self.assertEqual(item['urdf'],'urdf/components/'+item['link']+'.urdf')
            pose = expected['zero']['link_poses'][item['link']]
            center = line.points[0]
            actual = [sum(pose[row][column]*getattr(center,'xyz'[column]) for column in range(3))+pose[row][3] for row in range(3)]
            bounds = expected['zero']['bounds'][item['link']]
            for value, wanted in zip(actual,[(a+b)/2 for a,b in zip(*bounds)]):
                self.assertAlmostEqual(value,wanted,places=10)
        self.assertIn('Topic:\n        Value: /fusion360tourdf/component_labels',rviz_config())

    def test_walkthrough_uses_shared_installers_and_keeps_scope_and_credit_explicit(self):
        chapters = chapter_plan('rotary_slide_description',[])
        text = '\n'.join(str(chapter) for chapter in chapters)
        self.assertIn('bash install.sh',text)
        self.assertIn('install.cmd',text)
        self.assertIn('fusion360tourdf-install.pyz',text)
        self.assertIn('LICENSES.md',text)
        self.assertNotIn('install-exporter',text)
        self.assertNotIn('--symlink-install',text)
        self.assertIn('changelog.md',text)
        self.assertIn('CONTRIBUTING.md',text)
        self.assertIn('not a physics simulation',text)
        self.assertTrue(any(chapter[3]=='motion' for chapter in chapters))

    def test_walkthrough_rejects_non_rviz_or_other_native_receipts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'native.json').write_text('{}')
            (root/'rviz.json').write_text(json.dumps({'status':'pass','evidence_kind':'blender-preview'}))
            with patch('tools.make_walkthrough.read_bundle',return_value=(root,{})):
                with self.assertRaisesRegex(ValueError,'real RViz'):
                    validate_capture(root,root)
            (root/'rviz.json').write_text(json.dumps({'status':'pass',
                'evidence_kind':'real-rviz-with-verified-mock-control','native_receipt_sha256':'wrong'}))
            with patch('tools.make_walkthrough.read_bundle',return_value=(root,{})):
                with self.assertRaisesRegex(ValueError,'another native export'):
                    validate_capture(root,root)


if __name__ == '__main__':
    unittest.main()
