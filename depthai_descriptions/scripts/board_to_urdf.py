#!/usr/bin/env python3
"""Convert DepthAI board extrinsics to URDF origins (metres and radians).

Example:
  python3 board_to_urdf.py path/to/OAK-D.json --imu-frame native

By default, the reference is the unique terminal camera (no outgoing
extrinsics). Use --reference SOCKET to choose it explicitly, including for
graphs with no unique terminal. Disconnected cameras cannot be placed without
additional extrinsics and are rejected even with an explicit reference.

The reference camera's body frame is the URDF base. Camera origins describe
body frames (X forward, Y left, Z up); their optical children have the fixed
RPY (-pi/2, 0, -pi/2). IMU 'native' preserves each chip's axes. IMU 'optical'
assumes messages were already rotated into the reference camera's optical axes,
as expected by the ROS bridge. Every IMU variant is emitted separately.

Output is JSON containing xyz/rpy strings usable in URDF <origin> attributes.
Mesh alignment is not available in board extrinsics and is not generated.
Only the Python standard library is required.
"""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys


IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
OPTICAL_TO_BODY = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]


def transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def multiply(a, b):
    return [[sum(x * y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def rotate(rotation, vector):
    return [sum(x * y for x, y in zip(row, vector)) for row in rotation]


def compose(a, b):
    ar, at = a
    br, bt = b
    return multiply(ar, br), [x + y for x, y in zip(at, rotate(ar, bt))]


def inverse(transform):
    rotation, translation = transform
    rotation = transpose(rotation)
    return rotation, rotate(rotation, [-v for v in translation])


def from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def to_rpy(rotation):
    # URDF uses Rz(yaw) Ry(pitch) Rx(roll). At gimbal lock choose yaw=0.
    cp = math.hypot(rotation[0][0], rotation[1][0])
    pitch = math.atan2(-rotation[2][0], cp)
    if cp < 1e-12:
        return [math.atan2(-rotation[1][2], rotation[1][1]), pitch, 0.0]
    return [math.atan2(rotation[2][1], rotation[2][2]), pitch,
            math.atan2(rotation[1][0], rotation[0][0])]


def parse_transform(extrinsics):
    translation = [float(extrinsics['specTranslation'][axis]) for axis in 'xyz']
    if 'rotationMatrix' in extrinsics:
        rotation = [[float(value) for value in row] for row in extrinsics['rotationMatrix']]
    else:
        angles = extrinsics['rotation']
        rotation = from_rpy(*(math.radians(float(angles[axis])) for axis in 'rpy'))
    if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
        raise ValueError('rotationMatrix must be 3x3')
    if not all(math.isfinite(v) for row in rotation for v in row) or not all(map(math.isfinite, translation)):
        raise ValueError('extrinsics must contain finite numbers')
    product = multiply(rotation, transpose(rotation))
    determinant = sum(rotation[0][i] * (
        rotation[1][(i + 1) % 3] * rotation[2][(i + 2) % 3]
        - rotation[1][(i + 2) % 3] * rotation[2][(i + 1) % 3]) for i in range(3))
    if any(abs(product[i][j] - IDENTITY[i][j]) > 1e-5 for i in range(3) for j in range(3)) or abs(determinant - 1) > 1e-5:
        raise ValueError('rotationMatrix must be an orthonormal, right-handed rotation')
    return rotation, translation


def origin(rotation, translation):
    def text(values):
        return ' '.join(format(0.0 if abs(v) < 1e-15 else v, '.17g') for v in values)
    return {'xyz': text([v / 100.0 for v in rotate(OPTICAL_TO_BODY, translation)]),
            'rpy': text(to_rpy(rotation))}


def convert(board, reference, imu_frame):
    if imu_frame not in ('native', 'optical'):
        raise ValueError('imu_frame must be native or optical')
    config = board.get('board_config', board)
    cameras = config['cameras']
    if reference is None:
        terminals = [socket for socket, camera in cameras.items()
                     if camera.get('extrinsics') is None]
        if len(terminals) != 1:
            found = ', '.join(terminals) if terminals else 'none'
            raise ValueError(f'Cannot select a unique terminal camera (found: {found}). '
                             'Use --reference SOCKET to select a reference explicitly; '
                             'disconnected cameras require additional extrinsics.')
        reference = terminals[0]
    if reference not in cameras:
        raise ValueError(f'Reference {reference!r} is absent; choose from {", ".join(cameras)}')
    graph = {socket: [] for socket in cameras}
    for socket, camera in cameras.items():
        extrinsics = camera.get('extrinsics')
        if extrinsics is None:
            continue
        target = extrinsics['to_cam']
        if target not in cameras:
            raise ValueError(f'{socket} references unknown camera {target}')
        transform = parse_transform(extrinsics)
        # Stored transform maps source points into target coordinates.
        graph[target].append((socket, transform))
        graph[socket].append((target, inverse(transform)))
    poses = {reference: (IDENTITY, [0.0, 0.0, 0.0])}
    pending = deque([reference])
    while pending:
        parent = pending.popleft()
        for child, transform in graph[parent]:
            candidate = compose(poses[parent], transform)
            if child in poses:
                old_r, old_t = poses[child]
                new_r, new_t = candidate
                if any(abs(a - b) > 1e-5 for a, b in zip(old_t, new_t)) or any(
                        abs(old_r[i][j] - new_r[i][j]) > 1e-5 for i in range(3) for j in range(3)):
                    raise ValueError(f'Inconsistent extrinsics cycle at {child}')
            else:
                poses[child] = candidate
                pending.append(child)
    disconnected = set(cameras) - set(poses)
    if disconnected:
        raise ValueError(f'Cameras disconnected from {reference}: {", ".join(sorted(disconnected))}')
    output = {'model': config.get('name'), 'reference_camera': reference,
              'units': {'xyz': 'metres', 'rpy': 'radians'}, 'imu_frame': imu_frame,
              'cameras': {}, 'imus': {},
              'camera_optical_origin': origin(OPTICAL_TO_BODY, [0, 0, 0])}
    for socket, (rotation, translation) in poses.items():
        body_rotation = multiply(multiply(OPTICAL_TO_BODY, rotation), transpose(OPTICAL_TO_BODY))
        output['cameras'][socket] = {'name': cameras[socket].get('name', socket),
                                     **origin(body_rotation, translation)}
    for sensor, description in config.get('imuExtrinsics', {}).get('sensors', {}).items():
        extrinsics = description['extrinsics']
        target = extrinsics['to_cam']
        if target not in poses:
            raise ValueError(f'IMU {sensor} references unknown camera {target}')
        rotation, translation = compose(poses[target], parse_transform(extrinsics))
        rotation = multiply(OPTICAL_TO_BODY, rotation) if imu_frame == 'native' else OPTICAL_TO_BODY
        output['imus'][sensor] = {'name': description.get('name', sensor), **origin(rotation, translation)}
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('board', type=Path, help='DepthAI board JSON file')
    parser.add_argument('--reference', help='Camera whose body frame becomes the URDF base (default: unique terminal camera)')
    parser.add_argument('--imu-frame', required=True, choices=['native', 'optical'], help='Axes of the IMU data to describe; see above')
    parser.add_argument('-o', '--output', type=Path, help='Write JSON to this file instead of stdout')
    args = parser.parse_args()
    try:
        result = convert(json.loads(args.board.read_text()), args.reference, args.imu_frame)
        text = json.dumps(result, indent=2, allow_nan=False) + '\n'
        if args.output:
            args.output.write_text(text)
        else:
            sys.stdout.write(text)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f'error: {exc}\n')


if __name__ == '__main__':
    main()
