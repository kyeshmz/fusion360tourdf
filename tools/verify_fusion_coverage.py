"""Run the declared Fusion regression cases without claiming native Fusion proof."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
import re
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def render_catalog(catalog):
    lines = ['# Fusion input coverage', '',
        'This is the declared input-support matrix, not a native Fusion certification.',
        'The [case catalog](fusion-use-cases.json) is its source of truth.',
        'The [nine-repository census](fusion-repositories.json) records the audited scope.', '',
        'Run `python3 tools/verify_fusion_coverage.py --output artifacts/fusion-coverage`',
        'for actual offline results. `--require-complete` fails rather than treating',
        'offline fixtures as native or full-scope proof. A rejection is not input support.', '',
        '| Use case | Implementation | Offline evidence | Native Fusion |',
        '| --- | --- | --- | --- |']
    for case in catalog['cases']:
        declared = case['tests'] or case.get('diagnostic_tests', [])
        modules = sorted({'.'.join(test.split('.')[:2]) for test in declared})
        evidence = ', '.join('[' + module + '](../' + module.replace('.', '/') + '.py)'
                             for module in modules) or 'No input-support test'
        if not case['tests'] and case.get('diagnostic_tests'):
            evidence = 'Rejection only. ' + evidence
        lines.append('| ' + ' | '.join([case['title'], case['implementation'], evidence,
                                       case.get('native_status', 'unverified')]) + ' |')
    lines += ['', '## Limits and gaps', '']
    lines += ['- ' + case['title'] + '. ' + case['limitation'] + '.'
              for case in catalog['cases'] if case.get('limitation')]
    lines += ['', 'Native fixtures must be executed inside Autodesk Fusion. Docker, Blender,',
              'source inspection, and API doubles cannot replace that gate. See the',
              '[native verifier](../tools/verify_fusion/README.md) and',
              '[testing guide](testing.md).', '']
    return '\n'.join(lines)


def verify(catalog, output, selected=None):
    cases = catalog['cases']
    identifiers = [case['id'] for case in cases]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError('Duplicate Fusion use-case identifiers')
    if any(not re.fullmatch(r'[a-z0-9]+(?:-[a-z0-9]+)*', identifier) for identifier in identifiers):
        raise ValueError('Fusion use-case identifiers must be lowercase kebab-case')
    for case in cases:
        if case['implementation'] not in ('supported', 'partial', 'unimplemented', 'urdf-limited', 'rejected'):
            raise ValueError('Unknown input-support status: ' + case['id'])
    if selected and selected not in identifiers:
        raise ValueError('Unknown Fusion use case: ' + selected)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = {'scope': catalog['scope'], 'evidence_kind': 'offline-api-doubles',
              'native_fusion_verified': False, 'full_scope_verified': False, 'cases': {}}
    inputs = [*ROOT.joinpath('URDF_Exporter').rglob('*.py'),
              *ROOT.joinpath('tests').rglob('*.py'), ROOT / 'docs/fusion-use-cases.json',
              ROOT / 'docs/fusion-repositories.json',
              *ROOT.joinpath('tools').rglob('*.py')]
    report['source_sha256'] = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                              for path in sorted(inputs)}
    try:
        revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, check=True,
                                  capture_output=True, text=True).stdout.strip()
        status = subprocess.run(['git', 'status', '--porcelain'], cwd=ROOT, check=True,
                                capture_output=True, text=True).stdout
        report['git'] = {'revision': revision, 'dirty': bool(status)}
    except (OSError, subprocess.CalledProcessError):
        report['git'] = {'revision': None, 'dirty': None, 'status': 'unavailable'}
    failed = False
    for case in cases:
        if selected and case['id'] != selected:
            continue
        tests = case['tests'] + case.get('diagnostic_tests', [])
        supported = case['implementation'] == 'supported'
        if case['implementation'] in ('supported', 'partial') and not case['tests']:
            raise ValueError('Supported case has no executable evidence: ' + case['id'])
        behavior = {'supported': 'input-support', 'partial': 'partial-input-support',
                    'urdf-limited': 'tree-reduction'}.get(case['implementation'], 'rejection-only')
        result = {'implementation': case['implementation'], 'native_status': 'unverified',
                  'tests': case['tests'], 'diagnostic_tests': case.get('diagnostic_tests', []),
                  'checked_behavior': behavior,
                  'limitation': case.get('limitation')}
        if tests:
            stream = io.StringIO()
            suite = unittest.TestLoader().loadTestsFromNames(tests)
            executed = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
            (output / (case['id'] + '.log')).write_text(stream.getvalue(), encoding='utf-8')
            passed = (executed.wasSuccessful() and executed.testsRun > 0
                      and not executed.skipped and not executed.expectedFailures)
            result.update(offline_status='passed' if passed else 'failed',
                          tests_run=executed.testsRun, tests_skipped=len(executed.skipped),
                          expected_failures=len(executed.expectedFailures))
            failed = failed or not passed
        else:
            result.update(offline_status='not-covered', tests_run=0)
        report['cases'][case['id']] = result
    report['offline_checks_passed'] = (not failed and any(
        case['tests_run'] > 0 for case in report['cases'].values()))
    (output / 'fusion-coverage.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case')
    parser.add_argument('--write-markdown', type=Path,
                        help='Regenerate the human-readable catalog at the given path')
    parser.add_argument('--require-complete', action='store_true',
                        help='Reject offline-only evidence when full Fusion scope proof is required')
    args = parser.parse_args(argv)
    catalog = json.loads((ROOT / 'docs/fusion-use-cases.json').read_text())
    if args.write_markdown:
        args.write_markdown.write_text(render_catalog(catalog), encoding='utf-8')
    try:
        report = verify(catalog, args.output, args.case)
    except ValueError as error:
        parser.error(str(error))
    print('Offline checks ' + ('passed' if report['offline_checks_passed'] else 'failed'))
    print('Native Fusion and full-scope verification remain unverified')
    print('Report at ' + str(args.output / 'fusion-coverage.json'))
    if not report['offline_checks_passed']:
        return 1
    return 2 if args.require_complete else 0


if __name__ == '__main__':
    sys.exit(main())
