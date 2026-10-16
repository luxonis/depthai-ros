import importlib.util
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch
from xml.etree import ElementTree

try:
    import xacro
except ImportError:
    xacro = None


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'board_to_urdf.py'
SPEC = importlib.util.spec_from_file_location('board_to_urdf', SCRIPT)
converter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(converter)


def extrinsics(target, xyz, rpy=(0, 0, 0)):
    return {'to_cam': target, 'specTranslation': dict(zip('xyz', xyz)),
            'rotation': dict(zip('rpy', rpy))}


class BoardConversionTest(unittest.TestCase):
    def test_automatic_reference(self):
        for source, terminal in [('CAM_B', 'CAM_A'), ('CAM_A', 'CAM_E'), ('CAM_B', 'CAM_C')]:
            board = {'cameras': {
                source: {'extrinsics': extrinsics(terminal, (-2.5, 0, 0))}, terminal: {}}}
            output = converter.convert(board, None, 'native')
            self.assertEqual(output['reference_camera'], terminal)
            self.assertVector(output['cameras'][terminal]['xyz'], (0, 0, 0))
            self.assertVector(output['cameras'][source]['xyz'], (0, 0.025, 0))
            override = converter.convert(board, source, 'native')
            self.assertEqual(override['reference_camera'], source)
            self.assertVector(override['cameras'][terminal]['xyz'], (0, -0.025, 0))
        single = converter.convert({'cameras': {'CAM_A': {}}}, None, 'native')
        self.assertEqual(single['reference_camera'], 'CAM_A')

    def test_ambiguous_reference(self):
        with self.assertRaisesRegex(ValueError, 'found: A, B.*--reference'):
            converter.convert({'cameras': {'A': {}, 'B': {}}}, None, 'native')
        cycle = {'cameras': {
            'A': {'extrinsics': extrinsics('B', (1, 0, 0))},
            'B': {'extrinsics': extrinsics('A', (-1, 0, 0))}}}
        with self.assertRaisesRegex(ValueError, 'found: none.*--reference'):
            converter.convert(cycle, None, 'native')
        self.assertEqual(converter.convert(cycle, 'A', 'native')['reference_camera'], 'A')

    def assertVector(self, text, expected):
        actual_values = list(map(float, text.split()))
        self.assertEqual(len(actual_values), len(expected))
        for actual, value in zip(actual_values, expected):
            self.assertAlmostEqual(actual, value)

    def test_stereo_chain_and_imu(self):
        board = {'cameras': {
            'CAM_A': {},
            'CAM_B': {'extrinsics': extrinsics('CAM_C', (-7.5, 0, 0))},
            'CAM_C': {'extrinsics': extrinsics('CAM_A', (3.75, 0, 0))}},
            'imuExtrinsics': {'sensors': {'BNO': {'extrinsics': {
                'to_cam': 'CAM_A', 'specTranslation': dict(x=1.5, y=1.3662, z=0),
                'rotationMatrix': [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]}}}}}
        output = converter.convert(board, 'CAM_A', 'native')
        self.assertVector(output['cameras']['CAM_B']['xyz'], (0, 0.0375, 0))
        self.assertVector(output['cameras']['CAM_C']['xyz'], (0, -0.0375, 0))
        self.assertVector(output['imus']['BNO']['xyz'], (0, -0.015, -0.013662))
        # Preserve native chip axes: C multiplied by diag(-1, -1, 1).
        self.assertVector(output['imus']['BNO']['rpy'], (math.pi / 2, 0, math.pi / 2))
        optical = converter.convert(board, 'CAM_A', 'optical')
        self.assertVector(optical['imus']['BNO']['rpy'], (-math.pi / 2, 0, -math.pi / 2))

    def test_inverse_with_rotation_and_nonzero_translation(self):
        board = {'cameras': {'A': {'extrinsics': extrinsics('B', (10, 20, 30), (0, 0, 90))}, 'B': {}}}
        output = converter.convert(board, 'A', 'native')
        # Inverse translation in optical coordinates is (-20, 10, -30) cm.
        self.assertVector(output['cameras']['B']['xyz'], (-0.3, 0.2, -0.1))
        self.assertVector(output['cameras']['B']['rpy'], (-math.pi / 2, 0, 0))

    def test_rpy_round_trip_including_gimbal_lock(self):
        for angles in [(0.2, -0.4, 0.6), (0.3, math.pi / 2, 0.7), (0.3, -math.pi / 2, 0.7)]:
            rotation = converter.from_rpy(*angles)
            reconstructed = converter.from_rpy(*converter.to_rpy(rotation))
            for actual, expected in zip(sum(reconstructed, []), sum(rotation, [])):
                self.assertAlmostEqual(actual, expected)

    def test_invalid_graph(self):
        with self.assertRaisesRegex(ValueError, 'disconnected'):
            converter.convert({'cameras': {'A': {}, 'B': {}}}, 'A', 'native')
        with self.assertRaisesRegex(ValueError, 'Inconsistent'):
            converter.convert({'cameras': {
                'A': {'extrinsics': extrinsics('B', (1, 0, 0))},
                'B': {'extrinsics': extrinsics('A', (1, 0, 0))}}}, 'A', 'native')

    def test_reject_reflection_and_nonfinite_translation(self):
        data = extrinsics('A', (0, 0, 0))
        data['rotationMatrix'] = [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]
        with self.assertRaisesRegex(ValueError, 'right-handed'):
            converter.parse_transform(data)
        with self.assertRaisesRegex(ValueError, 'finite'):
            converter.parse_transform(extrinsics('A', (float('nan'), 0, 0)))


@unittest.skipIf(xacro is None, 'Source the ROS environment to compare expanded URDFs')
class ExistingUrdfTest(unittest.TestCase):
    """Compare real board fixtures with the current source-tree Xacro output.

    These focused regression cases use terminal-origin board mode. The catalog
    tests additionally cover every camera and IMU variant; legacy compatibility
    is checked separately against the pinned develop descriptions.
    """

    descriptions = SCRIPT.parents[1]
    boards = descriptions.parent / 'depthai-core/shared/depthai_boards/boards'

    def setUp(self):
        if not self.boards.is_dir():
            self.skipTest('depthai_boards checkout is unavailable')

    def board(self, filename, reference=None):
        path = self.boards / (filename + '.json')
        if not path.exists():
            self.skipTest(f'Board fixture is unavailable: {path.name}')
        return converter.convert(json.loads(path.read_text()), reference, 'native')

    def joints(self, model):
        # Resolve $(find ...) to this checkout, avoiding stale installed files.
        with patch('ament_index_python.packages.get_package_share_directory',
                   return_value=str(self.descriptions)):
            document = xacro.process_file(
                str(self.descriptions / 'urdf/depthai_descr.urdf.xacro'),
                mappings={'camera_model': model, 'pose_source': 'board', **({'imu_sensor': 'BNO'} if model in ('OAK-D', 'OAK-D-PRO') else {})})
        return {joint.attrib['name']: joint.find('origin').attrib
                for joint in ElementTree.fromstring(document.toxml()).findall('joint')}

    def assertOrigin(self, actual, expected):
        self.assertVector(actual['xyz'], list(map(float, expected['xyz'].split())))
        # Equivalent rotations can have different Euler angles, especially
        # at gimbal lock; compare their matrices instead of their RPY strings.
        for row_a, row_b in zip(
                converter.from_rpy(*map(float, actual['rpy'].split())),
                converter.from_rpy(*map(float, expected['rpy'].split()))):
            for a, b in zip(row_a, row_b):
                self.assertAlmostEqual(a, b)

    def assertVector(self, text, expected):
        actual_values = list(map(float, text.split()))
        self.assertEqual(len(actual_values), len(expected))
        for actual, value in zip(actual_values, expected):
            self.assertAlmostEqual(actual, value)

    def test_existing_camera_origins(self):
        for model in ('OAK-D', 'OAK-D-PRO', 'OAK-D-PRO-W', 'OAK-D-POE',
                      'OAK-D-S2', 'OAK-D-S2-POE', 'OAK-D-LITE', 'OAK-D-LR',
                      'OAK-1', 'OAK4-D', 'OAK4-S', 'OAK-D-CM4-POE'):
            with self.subTest(model=model):
                filename = {'OAK4-D': 'OAK-4-D', 'OAK4-S': 'OAK-4-S'}.get(model, model)
                output = self.board(filename)
                joints = self.joints(model)
                for socket, pose in output['cameras'].items():
                    sensor = {'CAM_A': 'rgb', 'CAM_B': 'left', 'CAM_C': 'right'}[socket]
                    self.assertOrigin(pose, joints[f'oak_{sensor}_camera_joint'])
                    self.assertOrigin(output['camera_optical_origin'],
                                      joints[f'oak_{sensor}_camera_optical_joint'])

    def test_thermal_origins_with_terminal_reference(self):
        # Both descriptions use terminal CAM_E as the origin.
        output = self.board('OAK-T')
        joints = self.joints('OAK-T')
        self.assertOrigin(output['cameras']['CAM_A'], joints['oak_rgb_camera_joint'])
        self.assertOrigin(output['cameras']['CAM_E'], joints['oak_right_camera_joint'])

    def test_short_range_relative_camera_origin(self):
        # Board mode places the terminal right sensor at the origin.
        output = self.board('OAK-D-SR')
        joints = self.joints('OAK-D-SR')
        left = list(map(float, joints['oak_left_camera_joint']['xyz'].split()))
        right = list(map(float, joints['oak_right_camera_joint']['xyz'].split()))
        self.assertVector(output['cameras']['CAM_B']['xyz'],
                          [a - b for a, b in zip(left, right)])
        self.assertVector(output['cameras']['CAM_C']['xyz'], (0, 0, 0))

    def test_short_range_poe_camera_origins(self):
        # The ToF camera is the base in both the board and the URDF.
        output = self.board('OAK-D-SR-POE')
        joints = self.joints('OAK-D-SR-POE')
        for socket, joint in (('CAM_B', 'oak_left_camera_joint'),
                              ('CAM_C', 'oak_right_camera_joint')):
            self.assertOrigin(output['cameras'][socket], joints[joint])

    def test_oak_d_pro_bno_imu_y(self):
        output = self.board('OAK-D-PRO')
        actual_y = float(output['imus']['BNO']['xyz'].split()[1])
        urdf_y = float(self.joints('OAK-D-PRO')['oak_imu_joint']['xyz'].split()[1])
        self.assertAlmostEqual(actual_y, urdf_y, places=12)

    def test_oak_d_imu_position(self):
        output = self.board('OAK-D')
        self.assertVector(output['imus']['BNO']['xyz'], list(map(
            float, self.joints('OAK-D')['oak_imu_joint']['xyz'].split())))


if __name__ == '__main__':
    unittest.main()
