#!/usr/bin/env python3
"""Interactive board-only IMU axis check using short, manually armed movements.

This checks accelerometer/gyro directions, not IMU position, scale calibration,
or fused quaternion conventions. No TF is applied to incoming samples: the
expected rotation comes directly from the selected board/chip catalog entry.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import statistics
import threading
import time


TRANSLATIONS = [
    ('forward', (1, 0, 0), 'move AWAY from you'),
    ('backward', (-1, 0, 0), 'move TOWARD you'),
    ('left', (0, 1, 0), 'move to YOUR LEFT'),
    ('right', (0, -1, 0), 'move to YOUR RIGHT'),
    ('up', (0, 0, 1), 'move UP'),
    ('down', (0, 0, -1), 'move DOWN'),
]
ROTATIONS = [
    ('roll', (1, 0, 0), 'tilt the TOP toward YOUR RIGHT, around the viewing direction'),
    ('pitch', (0, 1, 0), 'tilt the lenses DOWN'),
    ('yaw', (0, 0, 1), 'turn the lenses toward YOUR LEFT'),
]


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def rotate(matrix, vector):
    return [dot(row, vector) for row in matrix]


def from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def select_board(model, sensor, mode, share=None):
    import yaml
    from ament_index_python.packages import get_package_share_directory
    share = Path(share or get_package_share_directory('depthai_descriptions'))
    catalog = yaml.safe_load((share / 'urdf/include/board_models.yaml').read_text())
    if model not in catalog['models']:
        raise ValueError(f'No supported board geometry for {model}. Legacy descriptions are not tested.')
    record = catalog['models'][model]
    variants = record['imu_variants']
    if not sensor and len(variants) == 1:
        sensor = next(iter(variants))
    if sensor not in variants:
        raise ValueError(f'Select --imu-sensor from: {", ".join(variants) or "none (board has no IMU)"}')
    return sensor, record, from_rpy(*map(float, variants[sensor][mode + '_rpy'].split()))


def baseline(samples, accel_limit=0.18, gyro_limit=0.10):
    """Stationarity does not depend on the expected axis convention."""
    if len(samples) < 20 or samples[-1]['stamp'] - samples[0]['stamp'] < 0.7:
        raise ValueError('Too few samples: provide a fresh IMU stream at >= 20 Hz.')
    if any(b['stamp'] - a['stamp'] > 0.15 for a, b in zip(samples, samples[1:])):
        raise ValueError('IMU stream has gaps; use a higher report rate / smaller batches.')
    acc = [statistics.mean(s['acc'][i] for s in samples) for i in range(3)]
    gyro = [statistics.mean(s['gyro'][i] for s in samples) for i in range(3)]
    noise = math.sqrt(statistics.mean(sum((s['acc'][i]-acc[i])**2 for i in range(3)) for s in samples))
    if noise > accel_limit or max(norm(s['gyro']) for s in samples) > gyro_limit:
        raise ValueError('Camera was moving during baseline. Hold it still and retry.')
    if not 7.5 < norm(acc) < 12:
        raise ValueError('Expected acceleration including gravity in m/s^2; do not use gravity-removed reports.')
    return acc, gyro, noise


class FirstPulse:
    """Latch the FIRST sustained motion, regardless of its sign/expected axis.

    Stop at the first lobe's end or after 180 ms. Braking and repositioning
    cannot replace a wrong first pulse with a later, matching one.
    """
    def __init__(self, threshold, kind, hold=0.03, window=0.18):
        self.threshold, self.kind, self.hold, self.window = threshold, kind, hold, window
        self.smooth = deque()
        self.candidate = []
        self.pulse = []
        self.initial = None
        self.result = None
        self.last_stamp = None
        self.rotation = 0.0

    def update(self, stamp, vector, gyro):
        if self.result is not None:
            return self.result
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if dt <= 0 or dt > 0.15:
                self.result = dict(status='INCONCLUSIVE', reason='Missing/out-of-order IMU samples')
                return self.result
            self.rotation += norm(gyro)*dt
        self.last_stamp = stamp
        self.smooth.append((stamp, vector))
        while self.smooth and stamp - self.smooth[0][0] > 0.03:
            self.smooth.popleft()
        value = [statistics.mean(v[i] for _, v in self.smooth) for i in range(3)]
        if self.initial is None:
            if norm(value) < self.threshold:
                self.candidate.clear()
                # Rotation before/during the measured translation invalidates the
                # gravity subtraction even if angular velocity subsequently stops.
                if self.kind == 'acc' and self.rotation > math.radians(4):
                    self.result = dict(status='INCONCLUSIVE', reason='Camera tilted before movement; retry without rotating')
                return self.result
            self.candidate.append((stamp, value))
            if stamp - self.candidate[0][0] < self.hold:
                return None
            first = self.candidate[0][1]
            self.initial = [v/norm(first) for v in first]
            self.pulse = list(self.candidate)
        elif dot(value, self.initial) <= self.threshold*0.25:
            return self.finish()
        else:
            self.pulse.append((stamp, value))
        if stamp - self.pulse[0][0] >= self.window:
            return self.finish()
        return None

    def finish(self):
        if self.pulse[-1][0] - self.pulse[0][0] < 0.05:
            self.result = dict(status='INCONCLUSIVE', reason='Pulse too short; use a smooth short movement')
        elif self.kind == 'acc' and self.rotation > math.radians(4):
            self.result = dict(status='INCONCLUSIVE', reason='Camera tilted during translation; keep it upright')
        else:
            vector = [statistics.mean(v[i] for _, v in self.pulse) for i in range(3)]
            self.result = dict(status='MEASURED', vector=vector, samples=len(self.pulse))
        return self.result


def direction_result(vector, expected, rotation, angle_limit):
    body = rotate(rotation, vector)
    magnitude = norm(body)
    if magnitude < 1e-9:
        return dict(status='INCONCLUSIVE', reason='No measurable motion')
    angle = math.degrees(math.acos(max(-1.0, min(1.0, dot(body, expected)/magnitude))))
    return dict(status='PASS' if angle <= angle_limit else 'FAIL', angle_deg=angle,
                measured_imu=list(vector), measured_body=body, expected_body=list(expected))


def run(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from depthai_ros_msgs.msg import ImuWithMagneticField
    sensor, record, rotation = select_board(args.model, args.imu_sensor, args.imu_frame)
    print(f'Board: {args.model}, IMU: {sensor}, expected input axes: {args.imu_frame}', flush=True)
    print('NATIVE requires *_RAW reports; OPTICAL requires RDF-aligned *_UNCALIBRATED/*_CALIBRATED reports.')
    print('Use the actual device and unscaled IMU topic, not gallery TFs. No firmware modes are changed.')
    print('Hold the camera UPRIGHT, top up, lenses pointing AWAY from you throughout translations.')
    print('At MOVE NOW, make one smooth 10–20 cm movement, then stop. Do not shake or rotate it.')
    print('Only the first acceleration pulse is scored. Return movements are ignored while DISARMED.')
    print('Gyro checks use a small 15–25 degree rotation. Ctrl-C quits.\n', flush=True)
    node = rclpy.create_node('imu_coordinate_check')
    buffer = deque(maxlen=10000)
    lock = threading.Lock()
    problems = deque(maxlen=10)
    frame_id = None
    last_stamp = None

    def callback(message):
        nonlocal frame_id, last_stamp
        msg = message.imu if hasattr(message, 'imu') else message
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        age = node.get_clock().now().nanoseconds*1e-9 - stamp
        with lock:
            if not -0.2 <= age <= 0.5:
                problems.append('Stale IMU timestamps or clocks are not synchronized (allowed age: -0.2 to 0.5 s)')
                return
            if (not all(math.isfinite(v) for v in acc + gyro) or stamp <= 0
                    or msg.linear_acceleration_covariance[0] < 0 or msg.angular_velocity_covariance[0] < 0):
                problems.append('Invalid sample, timestamps, or unavailable accelerometer/gyro')
                return
            if frame_id is not None and msg.header.frame_id != frame_id:
                problems.append('IMU frame_id changed; check for multiple publishers')
                return
            if last_stamp is not None and stamp <= last_stamp:
                problems.append('Duplicate/out-of-order timestamps; check stream/publishers')
                return
            frame_id, last_stamp = msg.header.frame_id, stamp
            buffer.append(dict(arrival=time.monotonic(), stamp=stamp, acc=acc, gyro=gyro))

    thread = None
    results = []
    try:
        deadline = time.monotonic() + args.timeout
        topic_type = None
        while time.monotonic() < deadline:
            topics = dict(node.get_topic_names_and_types())
            if args.topic in topics:
                types = topics[args.topic]
                if len(types) != 1 or types[0] not in ('sensor_msgs/msg/Imu', 'depthai_ros_msgs/msg/ImuWithMagneticField'):
                    raise ValueError(f'Unsupported/ambiguous topic type: {types}')
                topic_type = Imu if types[0] == 'sensor_msgs/msg/Imu' else ImuWithMagneticField
                break
            time.sleep(0.1)
        if topic_type is None:
            raise ValueError(f'No IMU publisher found on {args.topic}')
        subscription = node.create_subscription(topic_type, args.topic, callback, qos_profile_sensor_data)
        stop = threading.Event()
        def spin():
            while not stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        thread = threading.Thread(target=spin, daemon=True)
        thread.start()
        steps = [('acc', *step) for step in TRANSLATIONS]
        if not args.skip_gyro:
            steps += [('gyro', *step) for step in ROTATIONS]
        for kind, name, expected, instruction in steps:
            while True:
                print(f'\nDISARMED — Next: {instruction}.', flush=True)
                if input('Reposition now, hold upright and still, then Enter to arm (q to quit): ').strip().lower() == 'q':
                    raise KeyboardInterrupt
                # Discard all queued return movements before starting a fresh baseline.
                with lock:
                    buffer.clear()
                    problems.clear()
                print('Hold still: measuring gravity/bias for 1.2 seconds...', flush=True)
                time.sleep(1.2)
                with lock:
                    still = list(buffer)
                    faults = list(problems)
                try:
                    if faults:
                        raise ValueError(faults[-1])
                    acc, gyro, noise = baseline(still)
                    if not still or time.monotonic() - still[-1]['arrival'] > 0.3:
                        raise ValueError('IMU stream is stale')
                except ValueError as exc:
                    print(f'INCONCLUSIVE: {exc} (no result recorded).', flush=True)
                    continue
                if not any(r['name'] == 'gravity' for r in results):
                    gravity = direction_result(acc, (0, 0, 1), rotation, args.angle)
                    results.append(dict(name='gravity', kind='acc', **gravity))
                    print(f"Gravity alignment: {gravity['status']}, deviation {gravity['angle_deg']:.1f} degrees")
                raw_direction = rotate(list(zip(*rotation)), expected)
                print('Expected IMU-axis direction: ' + ', '.join(f'{a}={v:+.2f}' for a, v in zip('XYZ', raw_direction)))
                cutoff = time.monotonic()
                with lock:
                    buffer.clear()
                    problems.clear()
                detector = FirstPulse(max(args.acc_threshold, 4*noise) if kind == 'acc' else args.gyro_threshold, kind)
                print(f'MOVE NOW: {instruction}!', flush=True)
                deadline = cutoff + args.timeout
                result = None
                while time.monotonic() < deadline and result is None:
                    with lock:
                        batch = list(buffer)
                        buffer.clear()
                        faults = list(problems)
                    if faults:
                        result = dict(status='INCONCLUSIVE', reason=faults[-1])
                        break
                    for sample in batch:
                        if sample['arrival'] < cutoff:
                            continue
                        # Do not accept old, heavily batched samples from before arming.
                        if sample['stamp'] <= still[-1]['stamp']:
                            continue
                        bias = acc if kind == 'acc' else gyro
                        value = [v-b for v, b in zip(sample[kind], bias)]
                        result = detector.update(sample['stamp'], value, [g-b for g, b in zip(sample['gyro'], gyro)])
                        if result:
                            break
                    time.sleep(0.005)
                if result is None:
                    result = dict(status='INCONCLUSIVE', reason='No clear first pulse before timeout / stream stopped')
                if result['status'] == 'MEASURED':
                    result = direction_result(result['vector'], expected, rotation, args.angle)
                print('DISARMED — measurement complete; you may return to the starting position.', flush=True)
                if result['status'] == 'INCONCLUSIVE':
                    print('INCONCLUSIVE: ' + result['reason'] + '. Retry this step.')
                    continue
                results.append(dict(name=name, kind=kind, **result))
                print(f"{result['status']}: {name}, direction error {result['angle_deg']:.1f} degrees; "
                      f"body XYZ={['%.3f' % v for v in result['measured_body']]}", flush=True)
                break
    except (KeyboardInterrupt, EOFError):
        print('\nAborted; incomplete tests do not pass.')
    finally:
        if thread:
            stop.set()
            thread.join(timeout=2)
        node.destroy_node()
        expected_count = 7 if args.skip_gyro else 10
        passed = len(results) == expected_count and all(r['status'] == 'PASS' for r in results)
        report = dict(model=args.model, imu_sensor=sensor, imu_frame=args.imu_frame, topic=args.topic,
                      frame_id=frame_id, board_file=record['board_file'], rotation_imu_to_body=rotation,
                      angle_limit_deg=args.angle, acc_threshold=args.acc_threshold, gyro_threshold=args.gyro_threshold,
                      passed=passed, complete=len(results) == expected_count, results=results)
        if args.output:
            Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
        print(f"\nOverall: {'PASS' if passed else 'FAIL / INCOMPLETE'} ({len(results)}/{expected_count} checks)")
    return 0 if passed else 1


def main():
    import rclpy
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, help='Exact board catalog model, e.g. OAK-D-S2')
    parser.add_argument('--imu-sensor', default='', help='Board IMU key; required when several variants exist, e.g. BMI or BNO')
    parser.add_argument('--imu-frame', choices=['native', 'optical'], default='optical', help='Expected report axes, not a firmware setting; default matches ROS processed reports')
    parser.add_argument('--topic', default='/oak/imu/data', help='Absolute IMU topic (sensor_msgs/Imu or ImuWithMagneticField)')
    parser.add_argument('--output', help='Optional JSON report path')
    parser.add_argument('--skip-gyro', action='store_true', help='Test acceleration/gravity axes only')
    parser.add_argument('--angle', type=float, default=30, help='Maximum direction error in degrees')
    parser.add_argument('--acc-threshold', type=float, default=0.5, help='Motion threshold in m/s^2')
    parser.add_argument('--gyro-threshold', type=float, default=0.15, help='Rotation threshold in rad/s')
    parser.add_argument('--timeout', type=float, default=10, help='Seconds to wait for each first pulse / topic')
    args = parser.parse_args(remove_ros_args()[1:])
    for name in ('angle', 'acc_threshold', 'gyro_threshold', 'timeout'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    if args.angle >= 90:
        parser.error('--angle must be less than 90 degrees')
    if not args.topic.startswith('/'):
        parser.error('--topic must be absolute')
    rclpy.init()
    try:
        code = run(args)
    except ValueError as exc:
        print(f'ERROR: {exc}')
        code = 2
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
