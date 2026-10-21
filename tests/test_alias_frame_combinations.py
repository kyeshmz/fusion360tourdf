"""Cross-feature identity/geometry checks, including case-insensitive stems."""

import importlib
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

from tests.test_fusion_inputs import Body, design, fixed, hinge, occurrence, root
from URDF_Exporter.utils.names import allocate_names


class CombinedAliasFrameTest(unittest.TestCase):
    def check_scene(self, paths, source_joint_names):
        frames = [occurrence(path) for path in paths]
        children = [occurrence(path + '+tool:1', [Body('tool_' + str(i), x, mass=i+2)])
                    for i, (path, x) in enumerate(zip(paths, (10, 20)))]
        joints = [hinge(name, frame, None) for name, frame in zip(source_joint_names, frames)]
        joints += [fixed('mount_' + str(i), child, frame, (x, 0, 0))
                   for i, (child, frame, x) in enumerate(zip(children, frames, (10, 20)))]
        component = root(frames + children, bodies=[Body('base', 0)], joints=joints)
        fixture, exported = design(component)
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / 'combined_description'
            importlib.import_module('URDF_Exporter.URDF_Exporter').export_design(fixture, package)
            robot = ET.parse(package / 'urdf/combined.urdf').getroot()
            data = json.loads((package / 'config/robot_data.yaml').read_text())
            aliases = {identity: alias for alias, identity in data['source_identifiers']['links'].items()}
            joint_aliases = {identity: alias for alias, identity in data['source_identifiers']['joints'].items()}
            self.assertEqual(set(aliases), {'design_root', *paths, *(p + '+tool:1' for p in paths)})
            self.assertEqual(len(robot.findall('link')), 5)
            self.assertEqual(set(exported), {'base', 'tool_0', 'tool_1'})
            self.assertEqual(sum(float(e.get('value')) for e in robot.findall('link/inertial/mass')), 6)
            for path in paths:
                frame = robot.find("link[@name='%s']" % aliases[path])
                self.assertEqual(list(frame), [])
                self.assertEqual(data['link_sources'][aliases[path]]['kind'], 'frame')
            files = [p.relative_to(package).as_posix() for p in package.rglob('*') if p.is_file()]
            self.assertEqual(len(files), len({name.casefold() for name in files}))
            self.assertEqual(len(joint_aliases), len({name.casefold() for name in joint_aliases.values()}))
            for path in paths:
                self.assertTrue((package / 'urdf/components' / (aliases[path] + '.urdf')).is_file())
            for i, (path, angle, x) in enumerate(zip(paths, (.2, -.3), (.1, .2))):
                child = robot.find("link[@name='%s']" % aliases[path + '+tool:1'])
                visual = child.find('visual')
                uri = visual.find('geometry/mesh').get('filename')
                mesh = package / uri.split('package://combined_description/', 1)[1]
                raw = struct.unpack_from('<3f', mesh.read_bytes(), 96)
                point = [value * float(scale) for value, scale in zip(raw, visual.find('geometry/mesh').get('scale').split())]
                offset = list(map(float, visual.find('origin').get('xyz').split()))
                mount = robot.find("joint[@name='mount_%d']" % i)
                self.assertEqual(mount.find('parent').get('link'), aliases[path])
                origin = list(map(float, mount.find('origin').get('xyz').split()))
                point = [a+b+c for a, b, c in zip(point, offset, origin)]
                observed = [math.cos(angle)*point[0]-math.sin(angle)*point[1],
                            math.sin(angle)*point[0]+math.cos(angle)*point[1], point[2]]
                expected = [math.cos(angle)*(x-.01)+math.sin(angle)*.01,
                            math.sin(angle)*(x-.01)-math.cos(angle)*.01, -.01]
                for got, want in zip(observed, expected):
                    self.assertAlmostEqual(got, want, places=11)
                hinge_element = robot.find("joint[@name='%s']" % joint_aliases[source_joint_names[i]])
                self.assertEqual(hinge_element.find('child').get('link'), aliases[path])
                self.assertEqual(list(map(float, hinge_element.find('axis').get('xyz').split())), [0, 0, 1])

    def test_unicode_frame_and_physical_alias_collisions_share_one_identity_policy(self):
        self.check_scene(('腕:1', '脚:1'), ('回転-left', '回転_left'))

    def test_case_variant_frame_joint_and_physical_stems_are_portable(self):
        self.check_scene(('Arm:1', 'arm:1'), ('Drive', 'drive'))

    def test_case_variant_reserved_names_and_collection_order(self):
        identities = ['root:1', 'Arm:1', 'arm:1', 'BASE_LINK',
                      'Fusion_Helper_part', 'RIGID_ATTACHMENT_part', 'ordinary:1']
        mappings = [allocate_names(order, 'link', root_identity='root:1')
                    for order in (identities, list(reversed(identities)))]
        self.assertEqual(*mappings)
        mapping = mappings[0]
        self.assertEqual(mapping['root:1'], 'base_link')
        self.assertEqual(mapping['ordinary:1'], 'ordinary_1')
        self.assertEqual(len(mapping), len({alias.casefold() for alias in mapping.values()}))
        for source, alias in mapping.items():
            if source != 'root:1':
                self.assertNotEqual(alias.casefold(), 'base_link')
                self.assertFalse(alias.casefold().startswith(('fusion_helper_', 'rigid_attachment_')))


if __name__ == '__main__':
    unittest.main()
