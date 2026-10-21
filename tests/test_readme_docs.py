from pathlib import Path
import hashlib
import re
import unittest
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET


REPOSITORY = Path(__file__).resolve().parents[1]


class ReadmeDocsTest(unittest.TestCase):
    def test_banner_is_static_accessible_and_self_contained(self):
        root = ET.parse(REPOSITORY / 'docs/images/banner.svg').getroot()
        namespace = '{http://www.w3.org/2000/svg}'
        self.assertEqual(root.tag, namespace + 'svg')
        self.assertEqual(root.get('role'), 'img')
        self.assertEqual(root.get('viewBox'), '0 0 1440 360')
        self.assertEqual(root.find(namespace + 'title').text, 'fusion360tourdf')
        self.assertIn('Autodesk', root.find(namespace + 'desc').text)
        self.assertIn('fusion360tourdf', [node.text for node in root.iter(namespace + 'text')])
        identifiers = {node.get('id'): node for node in root.iter() if node.get('id')}
        labels = root.get('aria-labelledby', '').split()
        self.assertTrue(labels)
        for label in labels:
            self.assertIn(label, identifiers)
            self.assertTrue(identifiers[label].text)
        for node in root.iter():
            self.assertNotIn(node.tag, (namespace + 'script', namespace + 'foreignObject', namespace + 'style'))
            for name, value in node.attrib.items():
                self.assertFalse(name.lower().startswith('on'))
                if name.endswith('href'):
                    self.assertTrue(value.startswith('#'))
                    self.assertIn(value[1:], identifiers)
                for reference in re.findall(r'url\(([^)]+)\)', value):
                    self.assertTrue(reference.startswith('#'))
                    self.assertIn(reference[1:], identifiers)

    def test_banner_preserves_official_autodesk_geometry_and_attribution(self):
        root = ET.parse(REPOSITORY / 'docs/images/banner.svg').getroot()
        namespace = '{http://www.w3.org/2000/svg}'
        logo = root.find(".//*[@id='autodesk-logo']")
        self.assertIsNotNone(logo)
        paths = list(logo.iter(namespace + 'path'))
        self.assertEqual(len(paths), 1)
        self.assertEqual(paths[0].get('fill'), '#ffffff')
        primary_path_sha256 = '9d88855644088bbb104a2a87249493ad7ed7122f5d7a32c38bca8cf266061adf'
        self.assertEqual(hashlib.sha256(paths[0].get('d').encode()).hexdigest(), primary_path_sha256)
        attribution = (REPOSITORY / 'docs/images/README.md').read_text()
        for source in (
                'https://brand.autodesk.com/visual-system/logo/',
                'https://brand.autodesk.com/wp-content/themes/autodesk-brand-hub/assets/images/autodesk-brand-logo-white.svg',
                '0a22763b25d1eab8086ce0437a41ec92e59142d500d436619dc1594ce27bcf66',
                primary_path_sha256):
            with self.subTest(source=source):
                self.assertIn(source, attribution)
        self.assertIn('Autodesk trademark', attribution)
        self.assertIn('MIT license does not cover the Autodesk logo', attribution)

    def test_front_page_has_banner_and_one_real_motion_demo(self):
        readme = (REPOSITORY / 'README.md').read_text()
        banner = '![fusion360tourdf project banner](docs/images/banner.svg)'
        self.assertTrue(readme.startswith(banner + '\n'))
        self.assertNotRegex(readme, r'(?im)^#\s+fusion360tourdf\s*$')
        self.assertIn('This independent community fork is not affiliated with or endorsed by Autodesk.', readme)
        self.assertLess(readme.index(banner), readme.index('\n## '))
        images = re.findall(r'!\[[^\]]*\]\(([^)]+)\)', readme)
        self.assertEqual(
            [target for target in images if not urlsplit(target).scheme],
            ['docs/images/banner.svg', 'docs/media/rviz-motion.gif'],
        )

    def test_front_page_omits_removed_walkthrough_and_verification_details(self):
        readme = (REPOSITORY / 'README.md').read_text()
        for removed in (
                'Verify exports', 'verify.yml/badge.svg',
                'walkthrough-poster.png', 'walkthrough.mp4', 'walkthrough.md',
                'Download the walkthrough (MP4)', 'chapter transcript',
                'tools/verify_export.py', 'requirements-test.txt',
                'unittest discover', '275 tests'):
            with self.subTest(removed=removed):
                self.assertNotIn(removed, readme)
        self.assertNotRegex(readme, r'(?im)^##\s+(Verification|Reproduce the verified demo)\b')

    def test_overview_and_detailed_guides_are_reachable(self):
        required_links = {
            'README.md': {
                'docs/verification.md', 'docs/installation.md', 'docs/cli.md',
                'docs/exporting.md', 'CONTRIBUTING.md', 'LICENSES.md',
                'Example/rotary_slide_description/',
                'Example/rotary_slide_README.md#available-urdf-files',
            },
            'docs/verification.md': {
                '../README.md', 'verification-results.md', 'testing.md',
                'demo-workflow.md', '../CONTRIBUTING.md', 'showcase.md',
                'media/README.md', 'images/images.json',
            },
        }
        for filename, expected in required_links.items():
            with self.subTest(document=filename):
                content = (REPOSITORY / filename).read_text()
                targets = set(re.findall(r'\[[^\]]*\]\(([^)]+)\)', content))
                self.assertTrue(expected <= targets, expected - targets)

    def test_local_document_links_and_fragments_resolve(self):
        for filename in ('README.md', 'docs/verification.md'):
            document = REPOSITORY / filename
            targets = re.findall(r'\[[^\]]*\]\(([^)]+)\)', document.read_text())
            for target in targets:
                link = urlsplit(target)
                if link.scheme or link.netloc:
                    continue
                destination = document.parent / unquote(link.path) if link.path else document
                with self.subTest(document=filename, target=target):
                    self.assertTrue(destination.exists(), destination)
                    if link.fragment and destination.suffix == '.md':
                        headings = re.findall(r'^#{1,6}\s+(.+)$', destination.read_text(), re.MULTILINE)
                        anchors = {
                            re.sub(r'[^\w -]', '', heading.lower()).replace(' ', '-')
                            for heading in headings
                        }
                        self.assertIn(unquote(link.fragment), anchors)

    def test_relocated_verification_preserves_original_dated_results(self):
        verification = (REPOSITORY / 'docs/verification.md').read_text()
        self.assertIn('October 19, 2026', verification)
        self.assertIn('`7bf5a13`', verification)
        results = {
            'Offline regressions': '275 tests on CPython 3.11.11 and 3.13.2: 262 passed, 13 platform skips per run',
            'Offline in Docker Jazzy / Python 3.12': 'Same 275 tests: 263 passed, 12 platform skips',
            'Single-file installer': 'Install, dry-run, safe update/backup, private-config exclusion, license inclusion, and POSIX execution passed',
            'Native six-link demo': '449 CAD/export assertions passed',
            'Component URDFs': 'All six loaded independently with their shared visual/collision meshes',
            'Independent loader and Blender': 'Seven integration tests and three Blender variants passed; wrong-axis/shifted/missing-mesh controls rejected',
            'ROS 2 Humble and Jazzy': 'Installed native assets matched hashes; controllers activated; trajectory, joint feedback, and six-link transforms passed',
        }
        for check, result in results.items():
            with self.subTest(check=check):
                self.assertIn(f'| {check} | {result} |', verification)
        self.assertIn('`not_exercised`', verification)
        self.assertIn('tools/verify_export.py /path/to/rotary-slide-bundle', verification)
        self.assertIn('**3.11, 3.12, 3.13, 3.14**', verification)
        self.assertIn('Fusion **2705.1.30**', verification)
        self.assertIn('Python **3.14.0**', verification)

    def test_incoming_readme_fragments_resolve_after_the_reorganization(self):
        readme = REPOSITORY / 'README.md'
        headings = re.findall(r'^#{1,6}\s+(.+)$', readme.read_text(), re.MULTILINE)
        anchors = {re.sub(r'[^\w -]', '', heading.lower()).replace(' ', '-')
                   for heading in headings}
        documents = [*REPOSITORY.glob('*.md'),
                     *(REPOSITORY / 'docs').rglob('*.md'),
                     *(REPOSITORY / 'Example').rglob('*.md')]
        for document in documents:
            for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)', document.read_text()):
                link = urlsplit(target)
                if link.scheme or link.netloc or not link.fragment:
                    continue
                destination = document.parent / unquote(link.path) if link.path else document
                if destination.resolve() == readme.resolve():
                    with self.subTest(document=document.relative_to(REPOSITORY), target=target):
                        self.assertIn(unquote(link.fragment), anchors)

    def test_verification_gallery_uses_relative_media_links(self):
        verification = (REPOSITORY / 'docs/verification.md').read_text()
        self.assertEqual(
            set(re.findall(r'!\[[^\]]*\]\(([^)]+)\)', verification)),
            {
                'images/rotary-slide-fusion.png',
                'images/rotary-slide-zero.png',
                'images/rotary-slide-moved.png',
                'media/ros-jazzy-zero.png',
                'media/ros-jazzy-moved.png',
            },
        )

    def test_linked_component_catalog_keeps_full_assembly_and_every_component(self):
        catalog = REPOSITORY / 'Example/rotary_slide_README.md'
        targets = re.findall(r'\[[^\]]*\]\(([^)]+\.urdf)\)', catalog.read_text())
        self.assertEqual(set(targets), {
            'rotary_slide_description/urdf/rotary_slide.urdf',
            'rotary_slide_description/urdf/components/base_link.urdf',
            'rotary_slide_description/urdf/components/mast_1.urdf',
            'rotary_slide_description/urdf/components/boom_1.urdf',
            'rotary_slide_description/urdf/components/boom_1_carriage_1.urdf',
            'rotary_slide_description/urdf/components/boom_1_carriage_1_sensor_1.urdf',
            'rotary_slide_description/urdf/components/boom_1_carriage_1_sensor_2.urdf',
        })
        for target in targets:
            with self.subTest(target=target):
                self.assertTrue((catalog.parent / target).is_file())


if __name__ == '__main__':
    unittest.main()
