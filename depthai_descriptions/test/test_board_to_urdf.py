import importlib.util
import math
from pathlib import Path
import unittest


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
        for actual, value in zip(map(float, text.split()), expected):
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


if __name__ == '__main__':
    unittest.main()
