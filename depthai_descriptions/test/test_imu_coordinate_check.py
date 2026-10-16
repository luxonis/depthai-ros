"""Synthetic motion tests; physical IMU verification is an interactive hardware test."""
import importlib.util
import math
from pathlib import Path
import unittest

SHARE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('imu_coordinate_check', SHARE / 'scripts/imu_coordinate_check.py')
check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check)
IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


class ImuCoordinateCheckTest(unittest.TestCase):
    def feed(self, detector, values):
        result = None
        for i, vector in enumerate(values):
            result = detector.update(i * 0.01, vector, [0, 0, 0])
        return result

    def test_first_acceleration_not_braking_or_return(self):
        detector = check.FirstPulse(0.5, 'acc')
        values = [[0, 0, 0]] * 20 + [[2, 0, 0]] * 12 + [[-2, 0, 0]] * 12 + [[0, 0, 0]] * 20 + [[-4, 0, 0]] * 50
        result = self.feed(detector, values)
        self.assertEqual(result['status'], 'MEASURED')
        self.assertGreater(result['vector'][0], 1)
        self.assertEqual(check.direction_result(result['vector'], [1, 0, 0], IDENTITY, 30)['status'], 'PASS')

    def test_wrong_initial_direction_cannot_pass_on_return(self):
        for vector in ([-2, 0, 0], [0, 2, 0], [0, 0, 2]):
            result = self.feed(check.FirstPulse(0.5, 'acc'), [vector] * 25 + [[2, 0, 0]] * 100)
            self.assertEqual(check.direction_result(result['vector'], [1, 0, 0], IDENTITY, 30)['status'], 'FAIL')

    def test_stationary_noise_does_not_trigger(self):
        detector = check.FirstPulse(0.5, 'acc')
        self.assertIsNone(self.feed(detector, [[0.05 * (-1)**i, 0.01, 0.02] for i in range(500)]))

    def test_gyro_axis_sign_and_board_rotation(self):
        rotation = check.from_rpy(0, 0, math.pi/2)
        result = self.feed(check.FirstPulse(0.15, 'gyro'), [[0.6, 0, 0]] * 30 + [[-1, 0, 0]] * 40)
        self.assertEqual(check.direction_result(result['vector'], [0, 1, 0], rotation, 30)['status'], 'PASS')
        self.assertEqual(check.direction_result(result['vector'], [0, -1, 0], rotation, 30)['status'], 'FAIL')

    def test_tilt_and_stream_gaps_are_inconclusive(self):
        detector = check.FirstPulse(0.5, 'acc')
        for i in range(20):
            result = detector.update(i * 0.01, [2, 0, 0], [1, 0, 0])
        self.assertEqual(result['status'], 'INCONCLUSIVE')
        detector = check.FirstPulse(0.5, 'acc')
        detector.update(0, [0, 0, 0], [0, 0, 0])
        self.assertEqual(detector.update(1, [2, 0, 0], [0, 0, 0])['status'], 'INCONCLUSIVE')

    def test_baseline_gravity_and_stationarity(self):
        samples = [dict(stamp=i*.01, acc=[0, 9.81, 0], gyro=[0, 0, 0]) for i in range(100)]
        acc, gyro, noise = check.baseline(samples)
        self.assertAlmostEqual(acc[1], 9.81)
        self.assertAlmostEqual(noise, 0)
        # Stationarity is independent of axis convention; gravity alignment is tested separately.
        self.assertEqual(check.direction_result(acc, [0, 0, 1], IDENTITY, 30)['status'], 'FAIL')
        samples[30]['acc'] = [3, 9.81, 0]
        with self.assertRaises(ValueError):
            check.baseline(samples)
        with self.assertRaises(ValueError):
            check.baseline([dict(stamp=i*.01, acc=[0, 0, 0], gyro=[0, 0, 0]) for i in range(100)])

    def test_board_selection_requires_real_chip_and_board(self):
        with self.assertRaisesRegex(ValueError, 'Select --imu-sensor'):
            check.select_board('OAK-D-S2', '', 'native', SHARE)
        with self.assertRaisesRegex(ValueError, 'No supported board geometry'):
            check.select_board('OAK-D-IOT-75', '', 'native', SHARE)
        sensor, _, optical = check.select_board('OAK-D-S2', 'BMI', 'optical', SHARE)
        self.assertEqual(sensor, 'BMI')
        # RDF forward Z maps to body forward X.
        for value, expected in zip(check.rotate(optical, [0, 0, 1]), [1, 0, 0]):
            self.assertAlmostEqual(value, expected)
        _, _, native = check.select_board('OAK-D-S2', 'BMI', 'native', SHARE)
        self.assertNotEqual(native, optical)
