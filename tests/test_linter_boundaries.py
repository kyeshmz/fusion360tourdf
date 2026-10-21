import contextlib
import io
from pathlib import Path
import tempfile
import unittest

from tools.validate_urdf import lint_file, main


class LinterBoundariesTest(unittest.TestCase):
    def lint(self, body):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.urdf'
            path.write_text('<robot name="test">' + body + '</robot>')
            return lint_file(path)

    def test_collision_must_belong_to_link(self):
        errors, _ = self.lint('<link name="base"/><collision><geometry>'
                              '<box size="1 1 1"/></geometry></collision>')
        self.assertTrue(any('collision' in error for error in errors), errors)

    def test_multiple_roots_are_errors(self):
        errors, _ = self.lint('<link name="base"/><link name="loose"/>')
        self.assertTrue(any('roots' in error for error in errors), errors)

    def test_cycle_outside_root_component_is_detected(self):
        errors, _ = self.lint(
            '<link name="base"/><link name="a"/><link name="b"/>'
            '<joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>'
            '<joint name="ba" type="fixed"><parent link="b"/><child link="a"/></joint>'
        )
        self.assertTrue(any('cycle' in error for error in errors), errors)

    def test_nonfinite_and_malformed_physics_are_errors(self):
        for mass, inertia in [('nan', '0'), ('inf', '0'), ('1', 'nan'),
                              ('1', 'oops'), ('1', 'inf')]:
            with self.subTest(mass=mass, inertia=inertia):
                errors, _ = self.lint(
                    '<link name="base"><inertial><mass value="' + mass + '"/>'
                    '<inertia ixx="1" iyy="1" izz="1" ixy="' + inertia + '" '
                    'ixz="0" iyz="0"/></inertial></link>'
                )
                self.assertTrue(errors, (mass, inertia))

    def test_nonfinite_origins_and_axes_are_errors(self):
        for origin, axis in [('nan 0 0', '0 0 1'), ('0 0 0', '0 inf 1')]:
            with self.subTest(origin=origin, axis=axis):
                errors, _ = self.lint(
                    '<link name="base"/><link name="arm"/>'
                    '<joint name="hinge" type="continuous"><parent link="base"/>'
                    '<child link="arm"/><origin xyz="' + origin + '" rpy="0 0 0"/>'
                    '<axis xyz="' + axis + '"/></joint>'
                )
                self.assertTrue(errors, (origin, axis))

    def test_nonfinite_limits_are_errors(self):
        for limits in ['lower="nan" upper="1" effort="1" velocity="1"',
                       'lower="0" upper="inf" effort="1" velocity="1"',
                       'lower="0" upper="1" effort="inf" velocity="1"',
                       'lower="0" upper="1" effort="1" velocity="nan"']:
            with self.subTest(limits=limits):
                errors, _ = self.lint(
                    '<link name="base"/><link name="slide"/>'
                    '<joint name="rail" type="prismatic"><parent link="base"/>'
                    '<child link="slide"/><axis xyz="1 0 0"/><limit ' + limits + '/></joint>'
                )
                self.assertTrue(errors, limits)

    def test_omitted_origin_attributes_use_urdf_defaults(self):
        errors, _ = self.lint(
            '<link name="base"/><link name="tip"/>'
            '<joint name="mount" type="fixed"><parent link="base"/>'
            '<child link="tip"/><origin xyz="0 0 1"/></joint>'
        )
        self.assertEqual(errors, [])

    def test_cli_returns_failure_for_errors_and_strict_warnings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.urdf'
            with contextlib.redirect_stdout(io.StringIO()):
                path.write_text('<robot name="test"><link name="base"/></robot>')
                self.assertEqual(main([str(path)]), 0)
                self.assertEqual(main([str(path), '--strict']), 1)
                path.write_text('<robot name="test"><link name="a"/><link name="b"/></robot>')
                self.assertEqual(main([str(path)]), 1)

    def test_every_visual_and_collision_mesh_reference_is_checked(self):
        for section in ('visual', 'collision'):
            with self.subTest(section=section), tempfile.TemporaryDirectory() as directory:
                parent = Path(directory)
                meshes = parent / 'meshes'
                meshes.mkdir()
                (meshes / 'first.stl').write_bytes(b'existence-only lint fixture')
                path = parent / 'multiple.urdf'
                path.write_text('<robot name="multiple"><link name="base">' +
                    '<' + section + '><geometry><mesh filename="package://demo/meshes/first.stl"/>'
                    '</geometry></' + section + '>' +
                    '<' + section + '><geometry><mesh filename="package://demo/meshes/second.stl"/>'
                    '</geometry></' + section + '></link></robot>')
                errors, _ = lint_file(path, meshes)
                self.assertTrue(any('second.stl' in error for error in errors), errors)
                (meshes / 'second.stl').write_bytes(b'existence-only lint fixture')
                errors, _ = lint_file(path, meshes)
                self.assertFalse(any('mesh missing' in error for error in errors), errors)
