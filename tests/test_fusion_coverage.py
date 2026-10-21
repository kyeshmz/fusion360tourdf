import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from tools.verify_fusion_coverage import main, render_catalog, verify


class FusionCoverageTest(unittest.TestCase):
    def run_case(self, *arguments):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        output = Path(temporary.name)
        result = subprocess.run([sys.executable, 'tools/verify_fusion_coverage.py',
            '--output', str(output), *arguments], capture_output=True, text=True,
            cwd=Path(__file__).resolve().parents[1])
        report = output / 'fusion-coverage.json'
        return result, json.loads(report.read_text()) if report.is_file() else None

    def test_selected_case_runs_real_regressions_without_native_claim(self):
        result, report = self.run_case('--case', 'current-pose-limits')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report['cases']['current-pose-limits']['tests_run'], 2)
        self.assertEqual(report['cases']['current-pose-limits']['offline_status'], 'passed')
        self.assertFalse(report['native_fusion_verified'])

    def test_require_complete_does_not_turn_offline_success_into_native_proof(self):
        result, report = self.run_case('--case', 'current-pose-limits', '--require-complete')
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertTrue(report['offline_checks_passed'])
        self.assertFalse(report['full_scope_verified'])

    def test_unknown_case_is_an_error(self):
        result, report = self.run_case('--case', 'not-a-real-case')
        self.assertEqual(result.returncode, 2)
        self.assertIn('Unknown Fusion use case', result.stderr)
        self.assertIsNone(report)

    def test_skipped_regression_is_not_coverage(self):
        class Probe(unittest.TestCase):
            @unittest.skip('Dependency unavailable')
            def test_behavior(self):
                self.fail('Must not be counted as executed evidence')

        module = types.ModuleType('fusion_coverage_skip_probe')
        module.Probe = Probe
        sys.modules[module.__name__] = module
        self.addCleanup(sys.modules.pop, module.__name__)
        catalog = {'scope': 'probe', 'cases': [{'id': 'probe', 'implementation': 'supported',
            'tests': [module.__name__ + '.Probe.test_behavior']}]}
        with tempfile.TemporaryDirectory() as directory:
            report = verify(catalog, directory)
            self.assertEqual(report['cases']['probe']['offline_status'], 'failed')
            self.assertEqual(report['cases']['probe']['tests_skipped'], 1)
            self.assertFalse(report['offline_checks_passed'])

    def test_supported_case_requires_an_executable_test(self):
        catalog = {'scope': 'probe', 'cases': [{'id': 'missing',
            'implementation': 'supported', 'tests': []}]}
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'no executable evidence: missing'):
                verify(catalog, directory)

    def test_rendered_catalog_tracks_every_declared_case(self):
        root = Path(__file__).resolve().parents[1]
        catalog = json.loads((root / 'docs/fusion-use-cases.json').read_text())
        rendered = render_catalog(catalog)
        for case in catalog['cases']:
            self.assertIn('| ' + case['title'] + ' | ' + case['implementation'] + ' |', rendered)
        self.assertEqual((root / 'docs/fusion-coverage.md').read_text(), rendered)

    def test_census_has_unique_pinned_in_scope_sources(self):
        root = Path(__file__).resolve().parents[1]
        census = json.loads((root / 'docs/fusion-repositories.json').read_text())
        repositories = census['repositories']
        names = [entry['repository'] for entry in repositories]
        self.assertEqual(len(names), len(set(names)))
        self.assertNotIn('yanshil/Fusion2PyBullet', names)
        for entry in repositories:
            self.assertRegex(entry['revision'], r'^[a-f0-9]{40}$')
            self.assertTrue(entry['sources'], entry['repository'])

    def test_unimplemented_case_without_checks_is_not_a_success(self):
        catalog = {'scope': 'probe', 'cases': [{'id': 'not-implemented',
            'implementation': 'unimplemented', 'tests': []}]}
        with tempfile.TemporaryDirectory() as directory:
            with patch('tools.verify_fusion_coverage.json.loads', return_value=catalog):
                self.assertEqual(main(['--output', directory]), 1)
            report = json.loads((Path(directory) / 'fusion-coverage.json').read_text())
            self.assertEqual(report['cases']['not-implemented']['offline_status'], 'not-covered')
            self.assertFalse(report['offline_checks_passed'])

    def test_rejection_checks_are_not_labeled_input_support(self):
        catalog = {'scope': 'probe', 'cases': [{'id': 'rejection-probe',
            'implementation': 'unimplemented', 'tests': [], 'diagnostic_tests': [
                'tests.test_fusion_inputs.InputGapTest.test_active_motion_links_and_bodyless_constraint_frames_are_named']}]}
        with tempfile.TemporaryDirectory() as directory:
            with patch('tools.verify_fusion_coverage.json.loads', return_value=catalog):
                self.assertEqual(main(['--output', directory]), 0)
            report = json.loads((Path(directory) / 'fusion-coverage.json').read_text())
        case = report['cases']['rejection-probe']
        self.assertEqual(case['implementation'], 'unimplemented')
        self.assertEqual(case['checked_behavior'], 'rejection-only')
        self.assertEqual(case['tests'], [])
        self.assertGreater(case['tests_run'], 0)
        self.assertFalse(report['full_scope_verified'])
