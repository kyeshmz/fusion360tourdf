import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from tests.fixtures import generate_fixture
from tools.export_bundle import bundle_hashes, read_bundle
from tools.verify_export import negative_controls, verify_export
from tools.verify_fusion.verification import Box, IDENTITY, Joint, pose_expectations
from URDF_Exporter.utils.stl import transform_binary_stl


REPOSITORY = Path(__file__).resolve().parents[1]


def offline_bundle(directory):
    directory = Path(directory)
    package = generate_fixture(directory)
    (directory / 'expected.json').write_bytes((directory / 'fixture_articulated.expected.json').read_bytes())
    (directory / 'assembly.f3d').write_bytes(b'Offline receipt-validation double, not native CAD evidence')
    report = dict(status='pass', evidence_kind='native-fusion', package_name=package.name,
                  source_preservation={'status': 'pass'}, entity_identity={'status': 'pass'},
                  outputs_sha256=bundle_hashes(directory, package.name))
    (directory / 'native.json').write_text(json.dumps(report))
    return package, report


class BundleTest(unittest.TestCase):
    def test_bundle_requires_preservation_identity_and_native_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            package, original = offline_bundle(directory)
            self.assertEqual(read_bundle(directory)[0], package.resolve())
            for key, value in (('status', 'running'), ('evidence_kind', 'offline-api-doubles'),
                               ('source_preservation', {}), ('entity_identity', {}),
                               ('package_name', '../foreign_description')):
                with self.subTest(key=key):
                    report = copy.deepcopy(original)
                    report[key] = value
                    (Path(directory) / 'native.json').write_text(json.dumps(report))
                    with self.assertRaises(ValueError):
                        read_bundle(directory)

    def test_changed_mesh_oracle_archive_and_extra_files_are_rejected(self):
        for relative in ('meshes/arm.stl', '../expected.json', '../assembly.f3d', 'extra.txt'):
            with self.subTest(relative=relative), tempfile.TemporaryDirectory() as directory:
                package, report = offline_bundle(directory)
                path = package / relative
                path.write_bytes(b'changed')
                with self.assertRaisesRegex(ValueError, 'changed or is incomplete'):
                    read_bundle(directory)

    def test_external_symlink_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryDirectory() as other:
            package, report = offline_bundle(directory)
            external = Path(other) / 'external.stl'
            external.write_bytes(b'not an owned mesh')
            (package / 'meshes/external.stl').symlink_to(external)
            with self.assertRaisesRegex(ValueError, 'escape'):
                read_bundle(directory)

    def test_directory_and_broken_symlinks_are_rejected(self):
        for broken in (False, True):
            with self.subTest(broken=broken), tempfile.TemporaryDirectory() as directory, tempfile.TemporaryDirectory() as other:
                package, report = offline_bundle(directory)
                target = Path(other) / 'unowned'
                if not broken:
                    target.mkdir()
                (package / 'meshes/unowned').symlink_to(target, target_is_directory=True)
                with self.assertRaisesRegex(ValueError, 'escape'):
                    read_bundle(directory)

    def test_partial_joint_configurations_are_not_accepted_as_full_poses(self):
        with tempfile.TemporaryDirectory() as directory:
            package, report = offline_bundle(directory)
            path = Path(directory) / 'expected.json'
            expected = json.loads(path.read_text())
            expected['zero']['positions'].pop('shoulder')
            path.write_text(json.dumps(expected))
            report['outputs_sha256'] = bundle_hashes(directory, package.name)
            (Path(directory) / 'native.json').write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, 'every actuated joint'):
                read_bundle(directory)

    def test_failure_retains_nonpassing_end_to_end_report(self):
        with tempfile.TemporaryDirectory() as directory:
            package, report = offline_bundle(directory)
            with patch('tools.verify_export.expand', return_value=package / 'urdf/fixture_articulated.xacro'), \
                 patch('tools.verify_export.subprocess.run'), \
                 patch('tools.verify_export.capture_provenance', return_value={'status': 'offline-unit-double'}), \
                 patch('tools.verify_export.load_and_check', side_effect=AssertionError('bad geometry')):
                with self.assertRaisesRegex(AssertionError, 'bad geometry'):
                    verify_export(directory, Path(directory) / 'output')
            paths = list((Path(directory) / 'output').glob('export-e2e-*/summary.json'))
            self.assertEqual(len(paths), 1)
            result = json.loads(paths[0].read_text())
            self.assertEqual(result['status'], 'fail')
            self.assertIn('bad geometry', result['traceback'])

    def test_negative_controls_fail_if_corruption_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            package, report = offline_bundle(directory)
            with patch('tools.verify_export.load_and_check', return_value={}):
                with self.assertRaisesRegex(AssertionError, 'Corrupted export was accepted'):
                    negative_controls(package / 'urdf/fixture_articulated.xacro', package, {},
                                      Path(directory) / 'negative-controls')


class PlacementTest(unittest.TestCase):
    def test_invalid_stl_placement_does_not_silently_become_identity(self):
        for matrix in ([], [float('nan')] * 16,
                       [2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
                       [-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]):
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                transform_binary_stl(b'', matrix)

    def test_independent_expectations_specify_every_joint_and_moved_bounds(self):
        boxes = {'base_link': [Box((.02, .02, .02), (0, 0, 0))],
                 'tool': [Box((.02, .04, .06), (.1, 0, .2))]}
        joints = [Joint('lift', 'base_link', 'tool', 'prismatic', limits=(0, .1), sample_position=.04)]
        frames = {'base_link': (IDENTITY, (0, 0, 0)), 'tool': (IDENTITY, (.1, 0, .2))}
        expected = pose_expectations(boxes, joints, frames)
        self.assertEqual(expected['zero']['positions'], {'lift': 0.0})
        self.assertEqual(expected['moved']['positions'], {'lift': .04})
        self.assertAlmostEqual(expected['moved']['link_poses']['tool'][2][3], .24)
        self.assertAlmostEqual(expected['moved']['bounds']['tool'][0][2], .21)

    def test_rigid_mount_anchor_and_offset_geometry_remain_independent(self):
        boxes = {'base_link': [Box((.02, .02, .02), (0, 0, 0))],
                 'tool': [Box((.02, .04, .06), (.1, 0, .2))],
                 'sensor': [Box((.01, .01, .01), (.1, .04, .22))]}
        joints = [Joint('lift', 'base_link', 'tool', 'prismatic', limits=(0, .1), sample_position=.04),
                  Joint('mount', 'tool', 'sensor', 'fixed')]
        frames = {'base_link': (IDENTITY, (0, 0, 0)), 'tool': (IDENTITY, (.1, 0, .2)),
                  'sensor': (IDENTITY, (.1, 0, .2))}
        expected = pose_expectations(boxes, joints, frames)
        self.assertEqual(expected['zero']['link_poses']['sensor'], expected['zero']['link_poses']['tool'])
        self.assertAlmostEqual(expected['zero']['bounds']['sensor'][0][1], .035)
        self.assertAlmostEqual(expected['moved']['bounds']['sensor'][0][2], .255)


class DockerDriverTest(unittest.TestCase):
    def test_mac_bash_empty_array_and_spaced_output_paths(self):
        for native in (False, True):
            with self.subTest(native=native), tempfile.TemporaryDirectory() as directory:
                directory = Path(directory)
                docker = directory / 'docker'
                docker.write_text('#!/bin/sh\nprintf "%s\\n" "$@" >> "$DOCKER_TEST_LOG"\n')
                docker.chmod(0o755)
                environment = dict(os.environ, PATH=str(directory) + os.pathsep + os.environ['PATH'],
                                   DOCKER_TEST_LOG=str(directory / 'calls.log'))
                command = ['bash', str(REPOSITORY / 'tools/verify_docker.sh'), 'jazzy', str(directory / 'output space')]
                if native:
                    bundle = directory / 'native bundle'
                    bundle.mkdir()
                    command.append(str(bundle))
                result = subprocess.run(command, env=environment, text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                calls = (directory / 'calls.log').read_text()
                self.assertIn('tools/verify_ros.sh', calls)
                self.assertEqual('/input' in calls, native)


if __name__ == '__main__':
    unittest.main()
