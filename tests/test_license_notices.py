import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class LicenseNoticesTest(unittest.TestCase):
    def test_every_inspected_repository_has_a_pinned_license_and_relationship(self):
        notices = (ROOT/'URDF_Exporter/THIRD_PARTY_NOTICES.md').read_text(encoding='utf-8')
        census = json.loads((ROOT/'docs/fusion-repositories.json').read_text())
        for item in census['repositories']:
            self.assertIn(item['repository'], notices)
            self.assertIn('/blob/'+item['revision']+'/LICENSE', notices)
        self.assertIn('comparison-only', notices)
        self.assertIn('not line-by-line provenance', notices)

    def test_retained_license_texts_match_the_pinned_upstream_texts(self):
        # Upstream SHA256s checked October 19, 2026; only an EOF newline may differ.
        expected = {
            'fusion2urdf': '25843f9143b800e5a2f3657e6e5fa2ec1856e489848abdd7a2a0da3265bd4866',
            'Adriaeik': '57eb3ca89b0c5030cf7ea076a5a42da60de9829b886c9eeb540efc5fb95267cc',
            'ACDC4Robot': 'f12ee066bdcc5db210eeca6e840a11ef60d567df882f149b2c33ef2f0e8c825a',
            'dheena2k2': '4e60d61c9ef7e651cefc4bec1b6496e78798119ba1cb653340317668556001f8',
            'cadop': '99a56b9f71fa474e3c94845c84bbfa0bae8e3dd50b3c6b0cb885c7f07bfc0918',
            'runtimerobotics': 'd87ba91b45f51d9cbc61347e8ec521a666bc11de5fadad9317d0bdea4a5963a9',
            'david-dorf': 'b218a8734ed8ae564112b1c1f09176f21e198997b7f0fd577b0fe88e6b64ae63',
        }
        for name, wanted in expected.items():
            data = (ROOT/'URDF_Exporter/licenses'/(name+'-MIT.txt')).read_bytes()
            self.assertIn(wanted, {hashlib.sha256(data).hexdigest(), hashlib.sha256(data.rstrip(b'\n')).hexdigest()}, name)
        self.assertEqual((ROOT/'LICENSE').read_bytes(), (ROOT/'URDF_Exporter/LICENSE').read_bytes())


if __name__ == '__main__':
    unittest.main()
