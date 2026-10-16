"""Check shipped descriptions against board inputs, including chip variants."""

import json
from pathlib import Path
import subprocess
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
from xml.etree import ElementTree

import test_board_to_urdf as conversion_tests

converter = conversion_tests.converter
xacro = conversion_tests.xacro

try:
    import yaml
except ImportError:
    yaml = None


@unittest.skipIf(xacro is None or yaml is None, 'Source ROS and install PyYAML')
class BoardCatalogTest(unittest.TestCase):
    descriptions = conversion_tests.ExistingUrdfTest.descriptions
    boards = conversion_tests.ExistingUrdfTest.boards
    assertVector = conversion_tests.ExistingUrdfTest.assertVector
    assertOrigin = conversion_tests.ExistingUrdfTest.assertOrigin

    @classmethod
    def setUpClass(cls):
        if not cls.boards.is_dir():
            raise unittest.SkipTest('depthai_boards checkout is unavailable')
        cls.catalog = yaml.safe_load((cls.descriptions / 'urdf/include/board_models.yaml').read_text())
        cls.overrides = yaml.safe_load((cls.descriptions / 'urdf/include/model_config.yaml').read_text())['models']

    def expand(self, model, description='depthai_descr.urdf.xacro', **args):
        with patch('ament_index_python.packages.get_package_share_directory',
                   return_value=str(self.descriptions)):
            doc = xacro.process_file(str(self.descriptions / 'urdf' / description),
                                     mappings=dict(camera_model=model, pose_source='board', **args))
        root = ElementTree.fromstring(doc.toxml())
        names = [node.attrib['name'] for node in root.findall('link')]
        self.assertEqual(len(names), len(set(names)))
        joints = root.findall('joint')
        self.assertEqual(len(joints), len({j.attrib['name'] for j in joints}))
        for joint in joints:
            self.assertIn(joint.find('parent').attrib['link'], names)
            self.assertIn(joint.find('child').attrib['link'], names)
        return root, {j.attrib['name']: j.find('origin').attrib for j in joints}

    def converted(self, model, mode):
        record = self.catalog['models'][model]
        board = json.loads((self.boards / record['board_file']).read_text())
        output = converter.convert(board, None, mode)
        return output

    def test_all_camera_poses_and_meshes(self):
        for model, record in self.catalog['models'].items():
            with self.subTest(model=model):
                output = self.converted(model, 'native')
                root, joints = self.expand(model)
                self.expand(model, rs_compat='true')  # Also reject duplicate compatibility frames.
                terminal = output['reference_camera']
                self.assertOrigin(output['cameras'][terminal], dict(xyz='0 0 0', rpy='0 0 0'))
                self.assertEqual(record['reference_camera'], terminal)
                mapped_sockets = set()
                for role in ('rgb', 'left', 'right'):
                    name = f'oak_{role}_camera_joint'
                    socket = record.get(role + '_socket')
                    if socket is None:
                        self.assertNotIn(name, joints)
                        continue
                    mapped_sockets.add(socket)
                    self.assertOrigin(joints[name], output['cameras'][socket])
                    self.assertOrigin(joints[f'oak_{role}_camera_optical_joint'],
                                      output['camera_optical_origin'])
                for name, pose in record['extra_cameras'].items():
                    mapped_sockets.add(pose['socket'])
                    self.assertOrigin(joints[f'oak_{name}_camera_joint'], output['cameras'][pose['socket']])
                self.assertEqual(mapped_sockets, set(output['cameras']))
                meshes = root.findall('.//mesh')
                self.assertEqual(len(meshes), int(record['mesh'] is not None or model in ('OAK-D-PRO-POE', 'OAK-D-PRO-W-POE', 'OAK-D-W', 'OAK-4-D', 'OAK-4-S')))
                for mesh in meshes:
                    filename = mesh.attrib['filename'].split('/urdf/models/')[1]
                    self.assertTrue((self.descriptions / 'urdf/models' / filename).is_file())

    def test_all_imu_variants_in_both_frame_modes(self):
        for model, record in self.catalog['models'].items():
            for mode in ('auto', 'native', 'optical'):
                expected = self.converted(model, 'native' if mode == 'auto' else mode)
                self.assertEqual(set(record['imu_variants']), set(expected['imus']))
                for sensor, pose in expected['imus'].items():
                    with self.subTest(model=model, mode=mode, sensor=sensor):
                        _, joints = self.expand(model, imu_sensor=sensor, imu_frame=mode)
                        self.assertOrigin(joints['oak_imu_joint'], pose)

    def test_default_imu_selection_uses_native_axes(self):
        for model, record in self.catalog['models'].items():
            with self.subTest(model=model):
                _, joints = self.expand(model)
                legacy = self.overrides.get(model, {})
                if legacy.get('imu_xyz') is not None:
                    self.assertOrigin(joints['oak_imu_joint'],
                                      {'xyz': legacy['imu_xyz'], 'rpy': legacy['imu_rpy']})
                elif len(record['imu_variants']) == 1:
                    pose = next(iter(self.converted(model, 'native')['imus'].values()))
                    self.assertOrigin(joints['oak_imu_joint'], pose)
                else:
                    self.assertNotIn('oak_imu_joint', joints)

    def test_base_description_and_realsense_names(self):
        _, joints = self.expand('OAK-D-S2', description='base_descr.urdf.xacro',
                                has_imu='true', imu_sensor='BMI')
        self.assertOrigin(joints['oak_imu_joint'], self.converted('OAK-D-S2', 'native')['imus']['BMI'])
        root, _ = self.expand('OAK-D-S2', rs_compat='true', imu_sensor='BNO')
        names = {link.attrib['name'] for link in root.findall('link')}
        self.assertTrue({'oak_color_frame', 'oak_infra2_frame', 'oak_infra1_frame'} <= names)

    def test_invalid_and_ambiguous_selections(self):
        for args in ({'imu_sensor': 'NOT_A_CHIP'}, {'imu_frame': 'invalid'}):
            with self.subTest(args=args), self.assertRaises(xacro.XacroException):
                self.expand('OAK-D-S2', **args)
        with self.assertRaises(xacro.XacroException):
            self.expand('OAK-D-S2', description='base_descr.urdf.xacro', has_imu='true')
        for model in self.catalog['unsupported']:
            with self.subTest(model=model), self.assertRaises(xacro.XacroException):
                self.expand(model)

    def test_pro_w_mesh_shares_pro_housing_reference(self):
        # Check the shipped geometry itself: a misplaced visual can pass all TF tests.
        def vertices(model):
            root, _ = self.expand(model)
            visual = root.find("link[@name='oak_model_origin']/visual")
            offset = list(map(float, visual.find('origin').get('xyz').split()))
            filename = visual.find('geometry/mesh').get('filename').split('/urdf/models/')[1]
            data = (self.descriptions / 'urdf/models' / filename).read_bytes()
            count = struct.unpack_from('<I', data, 80)[0]
            self.assertEqual(len(data), 84 + 50 * count)
            points = set()
            for triangle in struct.iter_unpack('<12fH', data[84:]):
                for index in (3, 6, 9):
                    points.add(tuple(round(triangle[index + axis] + offset[axis], 5)
                                     for axis in range(3)))
            return points
        common = vertices('OAK-D-PRO') & vertices('OAK-D-PRO-W')
        # Thousands of matching housing vertices across the enclosure, at 10 um
        # resolution, establish a translation rather than a bounding-box guess.
        self.assertGreater(len(common), 5000)
        for axis, minimum_span in enumerate((0.09, 0.025, 0.015)):
            self.assertGreater(max(p[axis] for p in common) - min(p[axis] for p in common), minimum_span)

    def test_catalog_regeneration(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'catalog.yaml'
            subprocess.run([sys.executable, str(self.descriptions / 'scripts/generate_board_models.py'),
                            '--boards', str(self.boards), '--output', str(output)],
                           check=True, capture_output=True, text=True)
            self.assertEqual(yaml.safe_load(output.read_text()), self.catalog)


if __name__ == '__main__':
    unittest.main()
