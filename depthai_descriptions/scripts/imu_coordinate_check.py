#!/usr/bin/env python3
"""Interactive URDF IMU axis check using short, manually armed movements.

This checks accelerometer/gyro directions, not IMU position, scale calibration,
or fused quaternion conventions. The expected rotation is read from the
fixed-joint chain in an expanded URDF description.
Use --standalone --device IP to stream directly with DepthAI v3, or omit
--standalone to check an existing ROS IMU topic.
"""
import argparse
from contextlib import ExitStack
from collections import deque
import json
import hashlib
import importlib.util
from xml.etree import ElementTree
import math
from pathlib import Path
import statistics
import signal
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


def description_share():
    candidates = (Path(__file__).resolve().parents[1],
                  Path(__file__).absolute().parents[2] / 'share/depthai_descriptions')
    for path in candidates:
        if (path / 'urdf/include/board_models.yaml').is_file():
            return path
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('depthai_descriptions'))


def urdf_rotation(xml, body_link, imu_link):
    """Compose the fixed URDF joint chain mapping IMU vectors into body axes."""
    root = ElementTree.fromstring(xml)
    if root.tag != 'robot':
        raise ValueError('Expected an expanded URDF <robot>')
    links = [link.attrib['name'] for link in root.findall('link')]
    if len(set(links)) != len(links):
        raise ValueError('Duplicate URDF links')
    if body_link not in links or imu_link not in links:
        raise ValueError(f'URDF must contain body link {body_link!r} and IMU link {imu_link!r}')
    parents = {}
    for joint in root.findall('joint'):
        child_node, parent_node = joint.find('child'), joint.find('parent')
        if child_node is None or parent_node is None:
            raise ValueError('URDF joint is missing its parent or child')
        child, parent = child_node.get('link'), parent_node.get('link')
        if child in parents or child not in links or parent not in links:
            raise ValueError('Invalid or ambiguous URDF joint tree')
        parents[child] = (parent, joint)
    rotation = from_rpy(0, 0, 0)
    current, visited = imu_link, set()
    while current != body_link:
        if current in visited or current not in parents:
            raise ValueError(f'No fixed URDF chain from {body_link!r} to {imu_link!r}')
        visited.add(current)
        parent, joint = parents[current]
        if joint.get('type') != 'fixed':
            raise ValueError('IMU-to-body URDF chain must contain only fixed joints')
        origin = joint.find('origin')
        rpy = list(map(float, (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()))
        if len(rpy) != 3 or not all(math.isfinite(v) for v in rpy):
            raise ValueError('Invalid URDF joint rotation')
        local = from_rpy(*rpy)
        rotation = [[dot(row, col) for col in zip(*rotation)] for row in local]
        current = parent
    return rotation


def load_description(args):
    if args.urdf:
        xml = Path(args.urdf).read_text()
        origin = str(Path(args.urdf).resolve())
        sensor = args.imu_sensor
    else:
        share = description_share()
        spec = importlib.util.spec_from_file_location('imu_description_resolver', share / 'scripts/resolve_description.py')
        resolver = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(resolver)
        except ImportError as exc:
            hint = ('Install PyYAML in the active Python environment: python -m pip install PyYAML'
                    if exc.name == 'yaml' else
                    'Source ROS and the workspace, and ensure Xacro and its dependencies are available to this Python environment')
            raise ValueError(f'Cannot generate URDF: {exc}. {hint}; alternatively pass --urdf FILE') from exc
        result = resolver.expand(dict(camera_model=args.model, imu_sensor=args.imu_sensor,
                                      imu_frame=args.imu_frame, pose_source=args.pose_source,
                                      has_imu='true', use_base_descr='true', base_frame=args.body_link), share=share)
        xml, sensor = result['urdf'], result['imu_sensor']
        origin = f'generated:{result["model"]}:{result["pose_source"]}'
        if result['diagnostics'].strip():
            print(result['diagnostics'].strip())
    rotation = urdf_rotation(xml, args.body_link, args.imu_link)
    return sensor, rotation, xml, dict(origin=origin, sha256=hashlib.sha256(xml.encode()).hexdigest(),
                                     body_link=args.body_link, imu_link=args.imu_link)


def baseline(samples, accel_limit=0.5, gyro_limit=0.25):
    """Stationarity does not depend on the expected axis convention."""
    if len(samples) < 20 or samples[-1]['stamp'] - samples[0]['stamp'] < 0.7:
        raise ValueError('Too few samples: provide a fresh IMU stream at >= 20 Hz.')
    if any(b['stamp'] - a['stamp'] > 0.15 for a, b in zip(samples, samples[1:])):
        raise ValueError('IMU stream has gaps; use a higher report rate / smaller batches.')
    acc = [statistics.mean(s['acc'][i] for s in samples) for i in range(3)]
    gyro = [statistics.mean(s['gyro'][i] for s in samples) for i in range(3)]
    noise = math.sqrt(statistics.mean(sum((s['acc'][i]-acc[i])**2 for i in range(3)) for s in samples))
    peak_gyro = max(norm(s['gyro']) for s in samples)
    if noise > accel_limit or peak_gyro > gyro_limit:
        raise ValueError(f'Camera was moving during baseline: acceleration RMS {noise:.3f}/{accel_limit:.3f} m/s², '
                         f'peak gyro {peak_gyro:.3f}/{gyro_limit:.3f} rad/s. Hold still and retry.')
    if not 7.5 < norm(acc) < 12:
        raise ValueError('Expected acceleration including gravity in m/s^2; do not use gravity-removed reports.')
    return acc, gyro, noise


class FirstPulse:
    """Latch the FIRST sustained motion, regardless of its sign/expected axis.

    Stop at the first lobe's end or after 180 ms. Braking and repositioning
    cannot replace a wrong first pulse with a later, matching one.
    """
    def __init__(self, threshold, kind, hold=0.03, window=0.18, max_tilt=4):
        self.threshold, self.kind, self.hold, self.window = threshold, kind, hold, window
        self.smooth = deque()
        self.candidate = []
        self.pulse = []
        self.initial = None
        self.result = None
        self.last_stamp = None
        self.rotation = 0.0
        self.orientation = [1.0, 0.0, 0.0, 0.0]
        self.max_tilt = math.radians(max_tilt)

    def integrate_rotation(self, gyro, dt):
        # Accumulate orientation, not angular distance: back-and-forth hand
        # tremor must not add up to a large fictitious tilt.
        speed = norm(gyro)
        if speed == 0:
            return
        half = speed * dt / 2
        a = math.cos(half)
        b, c, d = [v * math.sin(half) / speed for v in gyro]
        w, x, y, z = self.orientation
        q = [w*a-x*b-y*c-z*d, w*b+x*a+y*d-z*c,
             w*c-x*d+y*a+z*b, w*d+x*c-y*b+z*a]
        length = norm(q)
        self.orientation = [v / length for v in q]
        angle = 2 * math.atan2(norm(self.orientation[1:]), abs(self.orientation[0]))
        # Retain the largest excursion so returning upright cannot hide tilt.
        self.rotation = max(self.rotation, angle)

    def tilt_reason(self, when):
        return (f'Camera rotated {math.degrees(self.rotation):.1f}° {when} '
                f'(limit {math.degrees(self.max_tilt):.1f}°); keep its orientation steady')

    def update(self, stamp, vector, gyro):
        if self.result is not None:
            return self.result
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if dt <= 0 or dt > 0.15:
                self.result = dict(status='INCONCLUSIVE', reason='Missing/out-of-order IMU samples')
                return self.result
            self.integrate_rotation(gyro, dt)
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
                if self.kind == 'acc' and self.rotation > self.max_tilt:
                    self.result = dict(status='INCONCLUSIVE', reason=self.tilt_reason('before movement'))
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
        elif self.kind == 'acc' and self.rotation > self.max_tilt:
            self.result = dict(status='INCONCLUSIVE', reason=self.tilt_reason('during translation'))
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


class DirectImuSource:
    """DepthAI v3 IMU-only pipeline; no ROS nodes or camera streams."""
    def __init__(self, device, mode):
        self.device_id, self.mode = device, mode
        self.resources = ExitStack()

    def start(self, callback, timeout):
        try:
            import depthai as dai
        except ImportError as exc:
            raise ValueError('Standalone mode requires DepthAI v3: python3 -m pip install "depthai>=3,<4"') from exc
        if not hasattr(dai.Pipeline, 'start'):
            raise ValueError('Standalone mode requires DepthAI v3 (depthai>=3,<4)')
        self.dai, self.callback = dai, callback
        device = self.resources.enter_context(dai.Device(dai.DeviceInfo(self.device_id)) if self.device_id else dai.Device())
        self.pipeline = self.resources.enter_context(dai.Pipeline(device))
        imu = self.pipeline.create(dai.node.IMU)
        raw = self.mode == 'native'
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW if raw else dai.IMUSensor.ACCELEROMETER_UNCALIBRATED, 400)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW if raw else dai.IMUSensor.GYROSCOPE_UNCALIBRATED, 400)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        self.queue = imu.out.createOutputQueue(maxSize=50, blocking=False)
        self.pipeline.start()
        print(f'Connected directly to {device.getDeviceName()}; IMU-only stream started.', flush=True)

    def poll(self):
        if not self.pipeline.isRunning():
            raise ValueError('Device IMU pipeline stopped')
        batch = self.queue.tryGet()
        if batch is None:
            time.sleep(0.005)
            return
        now = self.dai.Clock.now().total_seconds()
        for packet in batch.packets:
            acc, gyro = packet.acceleroMeter, packet.gyroscope
            stamp = acc.getTimestamp().total_seconds()
            gyro_stamp = gyro.getTimestamp().total_seconds()
            self.callback(dict(stamp=stamp, age=max(now-stamp, now-gyro_stamp),
                               frame='device_imu', acc=[acc.x, acc.y, acc.z],
                               gyro=[gyro.x, gyro.y, gyro.z], valid=abs(stamp-gyro_stamp) <= 0.05))

    def close(self):
        self.resources.close()

    def now(self):
        return self.dai.Clock.now().total_seconds()


class RosImuSource:
    def __init__(self, topic):
        self.topic, self.node = topic, None

    def start(self, callback, timeout):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
        from depthai_ros_msgs.msg import ImuWithMagneticField
        rclpy.init()
        self.node = rclpy.create_node('imu_coordinate_check')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            topics = dict(self.node.get_topic_names_and_types())
            if self.topic in topics:
                types = topics[self.topic]
                if len(types) != 1 or types[0] not in ('sensor_msgs/msg/Imu', 'depthai_ros_msgs/msg/ImuWithMagneticField'):
                    raise ValueError(f'Unsupported/ambiguous topic type: {types}')
                topic_type = Imu if types[0] == 'sensor_msgs/msg/Imu' else ImuWithMagneticField
                break
            time.sleep(0.1)
        else:
            raise ValueError(f'No IMU publisher found on {self.topic}; start the driver or use --standalone')

        def receive(message):
            msg = message.imu if hasattr(message, 'imu') else message
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            callback(dict(stamp=stamp, age=self.node.get_clock().now().nanoseconds*1e-9-stamp,
                          frame=msg.header.frame_id,
                          acc=[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                          gyro=[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                          valid=msg.linear_acceleration_covariance[0] >= 0 and msg.angular_velocity_covariance[0] >= 0))
        self.subscription = self.node.create_subscription(topic_type, self.topic, receive, qos_profile_sensor_data)

    def poll(self):
        import rclpy
        if not rclpy.ok():
            raise ValueError('ROS context stopped')
        rclpy.spin_once(self.node, timeout_sec=0.05)

    def now(self):
        return self.node.get_clock().now().nanoseconds*1e-9

    def close(self):
        if self.node is not None:
            import rclpy
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def run(args):
    sensor, rotation, urdf, description = load_description(args)
    print(f'Expected IMU input axes: {args.imu_frame}', flush=True)
    print('NATIVE requires *_RAW reports; OPTICAL requires RDF-aligned *_UNCALIBRATED/*_CALIBRATED reports.')
    print(f'URDF: {description["origin"]}; testing {args.imu_link} -> {args.body_link}', flush=True)
    print('Hold the camera UPRIGHT, top up, lenses pointing AWAY from you throughout translations.')
    print('At MOVE NOW, make one smooth 10–20 cm movement, then stop. Do not shake or rotate it.')
    print('Only the first acceleration pulse is scored. Return movements are ignored while DISARMED.')
    print('Gyro checks use a small 15–25 degree rotation. Ctrl-C quits.\n', flush=True)
    source = DirectImuSource(args.device, args.imu_frame) if args.standalone else RosImuSource(args.topic)
    buffer = deque(maxlen=10000)
    lock = threading.Lock()
    problems = deque(maxlen=10)
    frame_id = None
    last_stamp = None

    def callback(sample):
        nonlocal frame_id, last_stamp
        stamp, acc, gyro = sample['stamp'], sample['acc'], sample['gyro']
        with lock:
            if not -0.2 <= sample['age'] <= 0.5:
                problems.append('Stale IMU timestamps or clocks are not synchronized (allowed age: -0.2 to 0.5 s)')
                return
            if (not all(math.isfinite(v) for v in acc + gyro + [stamp, sample['age']])
                    or stamp <= 0 or not sample['valid']):
                problems.append('Invalid sample, timestamps, or unavailable/unsynchronized accelerometer/gyro')
                return
            if not args.standalone and sample['frame'] != args.imu_link:
                problems.append(f'IMU frame {sample["frame"]!r} does not match URDF link {args.imu_link!r}')
                return
            if frame_id is not None and sample['frame'] != frame_id:
                problems.append('IMU frame_id changed; check for multiple publishers')
                return
            if last_stamp is not None and stamp <= last_stamp:
                problems.append('Duplicate/out-of-order timestamps; check stream/publishers')
                return
            frame_id, last_stamp = sample['frame'], stamp
            buffer.append(dict(arrival=time.monotonic(), stamp=stamp, acc=acc, gyro=gyro))

    thread = None
    results = []
    try:
        source.start(callback, args.timeout)
        stop = threading.Event()
        stream_errors = []
        def spin():
            try:
                while not stop.is_set():
                    source.poll()
            except Exception as exc:
                stream_errors.append(str(exc))
        def check_stream():
            if stream_errors:
                raise ValueError('IMU stream failed: ' + stream_errors[-1])
        thread = threading.Thread(target=spin, daemon=True)
        thread.start()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            check_stream()
            with lock:
                if buffer:
                    break
            time.sleep(0.01)
        else:
            with lock:
                detail = problems[-1] if problems else 'No IMU samples received'
            raise ValueError(detail)
        steps = [('acc', *step) for step in TRANSLATIONS]
        if not args.skip_gyro:
            steps += [('gyro', *step) for step in ROTATIONS]
        for kind, name, expected, instruction in steps:
            while True:
                print(f'\nDISARMED — Next: {instruction}.', flush=True)
                if input('Hold still, then Enter to begin the stillness check (q to quit): ').strip().lower() == 'q':
                    raise KeyboardInterrupt
                # Discard all queued return movements before starting a fresh baseline.
                with lock:
                    buffer.clear()
                    problems.clear()
                print('Hold still: measuring gravity/bias for 1.2 seconds...', flush=True)
                time.sleep(1.2)
                check_stream()
                with lock:
                    still = list(buffer)
                    faults = list(problems)
                try:
                    if faults:
                        raise ValueError(faults[-1])
                    acc, gyro, noise = baseline(still, args.still_acc_threshold, args.still_gyro_threshold)
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
                for remaining in range(args.prepare_seconds, 0, -1):
                    print(f'Hold still: {instruction} in {remaining}s. Wait for MOVE NOW.', flush=True)
                    time.sleep(1)
                    check_stream()
                # Ensure the gravity/bias baseline remained valid during the countdown.
                with lock:
                    preparation = list(buffer)
                    faults = list(problems)
                try:
                    if faults:
                        raise ValueError(faults[-1])
                    baseline(preparation, args.still_acc_threshold, args.still_gyro_threshold)
                    if not preparation or time.monotonic() - preparation[-1]['arrival'] > 0.3:
                        raise ValueError('IMU stream is stale')
                except ValueError as exc:
                    print(f'INCONCLUSIVE: {exc}. Retry the stillness check.', flush=True)
                    continue
                detector = FirstPulse(max(args.acc_threshold, 4*noise) if kind == 'acc' else args.gyro_threshold,
                                      kind, max_tilt=args.max_tilt)
                with lock:
                    buffer.clear()
                    problems.clear()
                    cutoff = time.monotonic()
                    # Use the report clock: delayed pre-arming motion may arrive
                    # after MOVE NOW even when the stream passes its age check.
                    armed_stamp = source.now()
                print(f'MOVE NOW: {instruction}!', flush=True)
                deadline = cutoff + args.timeout
                result = None
                while time.monotonic() < deadline and result is None:
                    check_stream()
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
                        if sample['stamp'] < armed_stamp:
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
        # ros2 launch may forward a second SIGINT after the terminal signal.
        # Finish device/thread cleanup and save the incomplete report atomically
        # with respect to that repeated interrupt.
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if thread:
                stop.set()
                thread.join(timeout=2)
            source.close()
            expected_count = 7 if args.skip_gyro else 10
            passed = len(results) == expected_count and all(r['status'] == 'PASS' for r in results)
            report = dict(model=args.model, imu_sensor=sensor, imu_frame=args.imu_frame,
                          source='standalone' if args.standalone else 'ros', device=args.device,
                          topic=None if args.standalone else args.topic,
                          frame_id=frame_id, description=description, rotation_imu_to_body=rotation,
                          angle_limit_deg=args.angle, acc_threshold=args.acc_threshold, gyro_threshold=args.gyro_threshold,
                          still_acc_threshold=args.still_acc_threshold, still_gyro_threshold=args.still_gyro_threshold,
                          max_tilt_deg=args.max_tilt,
                          passed=passed, complete=len(results) == expected_count, results=results)
            if args.output:
                Path(str(args.output) + '.urdf').write_text(urdf)
                Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
            print(f"\nOverall: {'PASS' if passed else 'FAIL / INCOMPLETE'} ({len(results)}/{expected_count} checks)")
        finally:
            signal.signal(signal.SIGINT, previous)
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--standalone', action='store_true', help='Connect directly with DepthAI v3; no ROS driver needed')
    parser.add_argument('--device', default='', help='Device IP or ID for --standalone; empty selects an available device')
    parser.add_argument('--urdf', type=Path, help='Expanded URDF to test; otherwise generate the model description')
    parser.add_argument('--body-link', default='oak', help='URDF camera body link: X forward, Y left, Z up')
    parser.add_argument('--imu-link', default='oak_imu_frame', help='URDF link in which incoming IMU components are expressed')
    parser.add_argument('--pose-source', choices=['board', 'legacy'], default='board', help='Geometry source when generating URDF')
    parser.add_argument('--model', help='Exact board catalog model, e.g. OAK-D-S2')
    parser.add_argument('--imu-sensor', default='', help='Board IMU key; required when several variants exist, e.g. BMI or BNO')
    parser.add_argument('--imu-frame', choices=['native', 'optical'], default='optical', help='Expected report axes, not a firmware setting; default matches ROS processed reports')
    parser.add_argument('--topic', default='/oak/imu/data', help='Absolute IMU topic (sensor_msgs/Imu or ImuWithMagneticField)')
    parser.add_argument('--output', help='Optional JSON report path')
    parser.add_argument('--skip-gyro', action='store_true', help='Test acceleration/gravity axes only')
    parser.add_argument('--prepare-seconds', type=int, default=3, help='Countdown after each stillness check, before MOVE NOW (default: 3)')
    parser.add_argument('--angle', type=float, default=30, help='Maximum direction error in degrees')
    parser.add_argument('--max-tilt', type=float, default=4, help='Maximum orientation change during translation in degrees (default: 4)')
    parser.add_argument('--acc-threshold', type=float, default=0.5, help='Motion threshold in m/s^2')
    parser.add_argument('--gyro-threshold', type=float, default=0.15, help='Rotation threshold in rad/s')
    parser.add_argument('--still-acc-threshold', type=float, default=0.5, help='Allowed acceleration RMS during stillness/countdown in m/s² (default: 0.5)')
    parser.add_argument('--still-gyro-threshold', type=float, default=0.25, help='Allowed peak gyro during stillness/countdown in rad/s (default: 0.25)')
    parser.add_argument('--timeout', type=float, default=10, help='Seconds to wait for each first pulse / topic')
    import sys
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(sys.argv)[1:]
    args = parser.parse_args(argv)
    if args.prepare_seconds < 0:
        parser.error('--prepare-seconds must be nonnegative')
    if not args.urdf and not args.model:
        parser.error('Provide --model to generate a description, or --urdf to test an existing one')
    if args.device and not args.standalone:
        parser.error('--device requires --standalone')
    for name in ('angle', 'max_tilt', 'acc_threshold', 'gyro_threshold', 'still_acc_threshold', 'still_gyro_threshold', 'timeout'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    if args.angle >= 90:
        parser.error('--angle must be less than 90 degrees')
    if args.max_tilt >= 90:
        parser.error('--max-tilt must be less than 90 degrees')
    if not args.standalone and not args.topic.startswith('/'):
        parser.error('--topic must be absolute')
    try:
        code = run(args)
    except (ValueError, RuntimeError, ImportError, OSError, ElementTree.ParseError) as exc:
        print(f'ERROR: {exc}')
        code = 2
    raise SystemExit(code)


if __name__ == '__main__':
    main()
