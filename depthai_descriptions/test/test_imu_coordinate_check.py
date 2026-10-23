"""Synthetic motion tests; physical IMU verification is an interactive hardware test."""
import importlib.util
import json
import math
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
import unittest
import tempfile
from unittest.mock import MagicMock, patch

SHARE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('imu_coordinate_check', SHARE / 'scripts/imu_coordinate_check.py')
check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check)
IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


class ImuCoordinateCheckTest(unittest.TestCase):
    def test_standalone_abort_needs_no_ros_and_releases_source(self):
        source = MagicMock()
        def start(callback, timeout):
            callback(dict(stamp=10., age=0., frame='device_imu', acc=[0, -9.81, 0],
                          gyro=[0, 0, 0], valid=True))
        source.start.side_effect = start
        source.poll.side_effect = lambda: check.time.sleep(.005)
        previous_handler = check.signal.getsignal(check.signal.SIGINT)
        source.close.side_effect = lambda: check.signal.raise_signal(check.signal.SIGINT)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'report.json'
            urdf = Path(directory) / 'input.urdf'
            urdf.write_text('<robot name="test"><link name="oak"/><link name="oak_imu_frame"/>'
                            '<joint name="imu" type="fixed"><parent link="oak"/><child link="oak_imu_frame"/></joint></robot>')
            args = SimpleNamespace(urdf=urdf, body_link='oak', imu_link='oak_imu_frame', pose_source='board', model='OAK4-D', imu_sensor='LSM6DSV', imu_frame='optical',
                                   standalone=True, device='10.12.153.23', topic='/unused', timeout=1.,
                                   skip_gyro=False, angle=30., max_tilt=4., acc_threshold=.5, gyro_threshold=.15,
                                   still_acc_threshold=.5, still_gyro_threshold=.25,
                                   output=str(output))
            with patch.dict('sys.modules', rclpy=None, ament_index_python=None), \
                    patch.object(check, 'DirectImuSource', return_value=source), \
                    patch.object(check, 'RosImuSource', side_effect=AssertionError('ROS source used')), \
                    patch('builtins.input', return_value='q'), patch('builtins.print'):
                try:
                    self.assertEqual(check.run(args), 1)
                except KeyboardInterrupt:
                    self.fail('A repeated shutdown signal interrupted cleanup/report writing')
                self.assertEqual(check.signal.getsignal(check.signal.SIGINT), previous_handler)
            report = json.loads(output.read_text())
            self.assertFalse(report['passed'])
            self.assertFalse(report['complete'])
            self.assertEqual(report['source'], 'standalone')
            self.assertEqual(report['device'], args.device)
            self.assertIsNone(report['topic'])
        source.close.assert_called_once()

    def test_direct_source_report_modes_and_samples(self):
        for mode, acc_mode, gyro_mode in (
                ('optical', 'ACCELEROMETER_UNCALIBRATED', 'GYROSCOPE_UNCALIBRATED'),
                ('native', 'ACCELEROMETER_RAW', 'GYROSCOPE_RAW')):
            with self.subTest(mode=mode):
                dai = MagicMock()
                device = dai.Device.return_value.__enter__.return_value
                pipeline = dai.Pipeline.return_value.__enter__.return_value
                imu = pipeline.create.return_value
                acc = SimpleNamespace(x=1., y=2., z=3., getTimestamp=lambda: timedelta(seconds=10))
                gyro = SimpleNamespace(x=4., y=5., z=6., getTimestamp=lambda: timedelta(seconds=10.002))
                imu.out.createOutputQueue.return_value.tryGet.return_value = SimpleNamespace(
                    packets=[SimpleNamespace(acceleroMeter=acc, gyroscope=gyro)])
                dai.Clock.now.return_value = timedelta(seconds=10.01)
                samples = []
                source = check.DirectImuSource('10.12.153.23', mode)
                with patch.dict('sys.modules', depthai=dai):
                    try:
                        source.start(samples.append, 10)
                        self.assertAlmostEqual(source.now(), 10.01)
                        source.poll()
                    finally:
                        source.close()
                dai.DeviceInfo.assert_called_once_with('10.12.153.23')
                dai.Pipeline.assert_called_once_with(device)
                self.assertEqual(imu.enableIMUSensor.call_args_list[0].args, (getattr(dai.IMUSensor, acc_mode), 400))
                self.assertEqual(imu.enableIMUSensor.call_args_list[1].args, (getattr(dai.IMUSensor, gyro_mode), 400))
                self.assertEqual(samples[0]['acc'], [1., 2., 3.])
                self.assertEqual(samples[0]['gyro'], [4., 5., 6.])
                self.assertAlmostEqual(samples[0]['age'], .01)
                self.assertTrue(samples[0]['valid'])
                dai.Pipeline.return_value.__exit__.assert_called_once()
                dai.Device.return_value.__exit__.assert_called_once()

    def test_direct_source_start_failure_releases_device(self):
        dai = MagicMock()
        dai.Pipeline.return_value.__enter__.return_value.start.side_effect = RuntimeError('start failed')
        source = check.DirectImuSource('', 'optical')
        with patch.dict('sys.modules', depthai=dai):
            try:
                with self.assertRaisesRegex(RuntimeError, 'start failed'):
                    source.start(lambda _: None, 10)
            finally:
                source.close()
        dai.Pipeline.return_value.__exit__.assert_called_once()
        dai.Device.return_value.__exit__.assert_called_once()

    def test_ros_source_clock_matches_report_timestamps(self):
        source = check.RosImuSource('/oak/imu/data')
        source.node = MagicMock()
        source.node.get_clock.return_value.now.return_value.nanoseconds = 1700000000123456789
        self.assertAlmostEqual(source.now(), 1700000000.123456789, delta=1e-6)

    def run_timed_motion(self, standalone, delay, pulse_start):
        # Simulate delivery independently of measurement time, with a source
        # clock far from the monotonic clock used for receipt/deadlines.
        elapsed = 0.
        next_sample = 1
        epoch = 1000. if standalone else 1700000000.
        source = MagicMock()
        source.now.side_effect = lambda: epoch + elapsed

        def emit():
            measured = elapsed - delay
            moving = pulse_start <= measured < pulse_start + .16
            callback(dict(stamp=epoch + measured, age=delay, frame='oak_imu_frame',
                          acc=[2. if moving else 0., 0., 9.81], gyro=[0., 0., 0.], valid=True))

        def start(receive, timeout):
            nonlocal callback
            callback = receive
            emit()

        def sleep(duration):
            nonlocal elapsed, next_sample
            end = elapsed + duration
            while next_sample * .01 <= end:
                elapsed = next_sample * .01
                next_sample += 1
                emit()
            elapsed = end

        callback = None
        source.start.side_effect = start
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'report.json'
            args = SimpleNamespace(standalone=standalone, device='', imu_frame='optical', topic='/oak/imu/data',
                                   imu_link='oak_imu_frame', body_link='oak', model='OAK-D-S2', imu_sensor='BMI',
                                   timeout=1., skip_gyro=True, prepare_seconds=3, angle=30., max_tilt=4.,
                                   acc_threshold=.5, gyro_threshold=.15, still_acc_threshold=.5,
                                   still_gyro_threshold=.25, output=str(output))
            with patch.object(check, 'load_description', return_value=('BMI', IDENTITY, '<robot/>', dict(origin='test'))), \
                    patch.object(check, 'DirectImuSource', return_value=source), \
                    patch.object(check, 'RosImuSource', return_value=source), \
                    patch.object(check.threading, 'Thread'), \
                    patch.object(check.time, 'monotonic', side_effect=lambda: 10000. + elapsed), \
                    patch.object(check.time, 'sleep', side_effect=sleep), \
                    patch('builtins.input', side_effect=['', 'q']), patch('builtins.print'):
                self.assertEqual(check.run(args), 1)
            source.now.assert_called_once_with()
            return json.loads(output.read_text())

    def test_delayed_motion_measured_before_arming_is_not_scored(self):
        # Baseline ends at 1.2 s and the countdown arms at 4.2 s. This
        # pulse ends before arming but arrives entirely after MOVE NOW.
        for standalone in (True, False):
            with self.subTest(standalone=standalone):
                report = self.run_timed_motion(standalone, delay=.3, pulse_start=4.)
                self.assertEqual([r['name'] for r in report['results']], ['gravity'])

    def test_motion_measured_after_arming_is_scored(self):
        for standalone in (True, False):
            for delay in (0., .3):
                with self.subTest(standalone=standalone, delay=delay):
                    report = self.run_timed_motion(standalone, delay=delay, pulse_start=4.3)
                    self.assertEqual([r['name'] for r in report['results']], ['gravity', 'forward'])
                    self.assertEqual(report['results'][-1]['status'], 'PASS')

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

    def test_hand_tremor_does_not_accumulate_as_tilt(self):
        detector = check.FirstPulse(.5, 'acc')
        for i in range(200):
            self.assertIsNone(detector.update(i*.01, [0, 0, 0], [.5*(-1)**i, 0, 0]))
        for i in range(200, 225):
            result = detector.update(i*.01, [0, -2, 0], [.5*(-1)**i, 0, 0])
        self.assertEqual(result['status'], 'MEASURED')
        self.assertLess(math.degrees(detector.rotation), 1)

    def test_real_rotation_excursion_is_retained_and_limit_is_configurable(self):
        detector = check.FirstPulse(.5, 'acc')
        detector.integrate_rotation([1, 0, 0], .1)
        detector.integrate_rotation([-1, 0, 0], .1)
        self.assertAlmostEqual(detector.rotation, .1)
        self.assertEqual(detector.update(1, [0, 0, 0], [0, 0, 0])['status'], 'INCONCLUSIVE')
        relaxed = check.FirstPulse(.5, 'acc', max_tilt=8)
        relaxed.integrate_rotation([1, 0, 0], .1)
        self.assertIsNone(relaxed.update(1, [0, 0, 0], [0, 0, 0]))

    def test_baseline_gravity_and_stationarity(self):
        samples = [dict(stamp=i*.01, acc=[0, 9.81, 0], gyro=[0, 0, 0]) for i in range(100)]
        acc, gyro, noise = check.baseline(samples)
        self.assertAlmostEqual(acc[1], 9.81)
        self.assertAlmostEqual(noise, 0)
        # Stationarity is independent of axis convention; gravity alignment is tested separately.
        self.assertEqual(check.direction_result(acc, [0, 0, 1], IDENTITY, 30)['status'], 'FAIL')
        # Mild handheld tremor passes the relaxed defaults but not the old limits.
        handheld = [dict(stamp=i*.01, acc=[.3 * (-1)**i, 9.81, 0], gyro=[.15, 0, 0]) for i in range(100)]
        check.baseline(handheld)
        with self.assertRaises(ValueError):
            check.baseline(handheld, .18, .10)
        samples[30]['acc'] = [6, 9.81, 0]
        with self.assertRaises(ValueError):
            check.baseline(samples)
        with self.assertRaises(ValueError):
            check.baseline([dict(stamp=i*.01, acc=[0, 0, 0], gyro=[0, 0, 0]) for i in range(100)])

    def test_urdf_chain_and_wrong_description(self):
        xml = '<robot name="test"><link name="body"/><link name="mount"/><link name="imu"/>' + \
              '<joint name="mount" type="fixed"><parent link="body"/><child link="mount"/>' + \
              '<origin rpy="0 0 1.5707963267948966"/></joint>' + \
              '<joint name="imu" type="fixed"><parent link="mount"/><child link="imu"/>' + \
              '<origin rpy="1.5707963267948966 0 0"/></joint></robot>'
        rotation = check.urdf_rotation(xml, 'body', 'imu')
        for actual, expected in zip(check.rotate(rotation, [0, 0, 1]), [1, 0, 0]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(check.direction_result([0, 0, 1], [1, 0, 0], rotation, 30)['status'], 'PASS')
        wrong = xml.replace('0 0 1.5707963267948966', '0 0 -1.5707963267948966')
        self.assertEqual(check.direction_result([0, 0, 1], [1, 0, 0], check.urdf_rotation(wrong, 'body', 'imu'), 30)['status'], 'FAIL')
        with self.assertRaisesRegex(ValueError, 'only fixed'):
            check.urdf_rotation(xml.replace('type="fixed"', 'type="revolute"'), 'body', 'imu')
        with self.assertRaisesRegex(ValueError, 'must contain'):
            check.urdf_rotation(xml, 'missing', 'imu')
        with self.assertRaisesRegex(ValueError, 'No fixed'):
            check.urdf_rotation(xml, 'imu', 'body')

    def test_generated_description_uses_expanded_joint(self):
        args = SimpleNamespace(urdf=None, model='OAK-D-S2', imu_sensor='BMI', imu_frame='optical',
                               pose_source='board', body_link='oak', imu_link='oak_imu_frame')
        with patch('ament_index_python.packages.get_package_share_directory', return_value=str(SHARE)):
            sensor, rotation, xml, metadata = check.load_description(args)
        self.assertEqual(sensor, 'BMI')
        self.assertEqual(metadata['origin'], 'generated:OAK-D-S2:board')
        self.assertEqual(rotation, check.urdf_rotation(xml, 'oak', 'oak_imu_frame'))
        for actual, expected in zip(check.rotate(rotation, [0, -9.81, 0]), [0, 0, 9.81]):
            self.assertAlmostEqual(actual, expected)
