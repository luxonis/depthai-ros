#!/usr/bin/env python3
"""Compare expanded DepthAI URDFs with converted board geometry.

Examples (requires xacro and its PyYAML dependency):
  python3 compare_board_urdf.py OAK-D OAK-D-PRO
  python3 compare_board_urdf.py OAK-D-PRO --imu-sensor BNO --imu-frame native

Without model arguments, compare all bundled mesh models. Each listed IMU
variant is explicitly selected in a separate URDF expansion, in both conventions.
--imu-sensor limits the comparison to one variant. All poses are aligned
with the selected board reference camera before comparison; base-origin shifts
are printed separately. Mesh alignment has no board counterpart and is skipped.
Exit status: 0 = all match, 1 = differences/missing frames, 2 = input errors.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import board_to_urdf as geometry


DESCRIPTIONS = Path(__file__).resolve().parents[1]
ALIASES = {'OAK4-D': 'OAK-4-D', 'OAK4-S': 'OAK-4-S'}


def pose(attributes):
    return (geometry.from_rpy(*map(float, attributes.get('rpy', '0 0 0').split())),
            list(map(float, attributes.get('xyz', '0 0 0').split())))


def difference(board_pose, urdf_pose, position_tolerance, angle_tolerance):
    br, bt = board_pose
    ur, ut = urdf_pose
    delta = [(b - u) * 1000 for b, u in zip(bt, ut)]
    relative = geometry.multiply(br, geometry.transpose(ur))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (sum(relative[i][i] for i in range(3)) - 1) / 2))))
    return delta, angle, max(map(abs, delta)) <= position_tolerance and angle <= angle_tolerance


def link_poses(robot):
    """Resolve the full fixed-joint tree rather than assuming direct children."""
    links = {link.attrib['name'] for link in robot.findall('link')}
    children, edges = set(), {}
    for joint in robot.findall('joint'):
        if joint.attrib['type'] != 'fixed':
            raise ValueError('Comparison requires a fixed-joint URDF')
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']
        origin = joint.find('origin')
        if child in children or parent not in links or child not in links:
            raise ValueError('Invalid URDF joint tree')
        children.add(child)
        edges.setdefault(parent, []).append((child, pose(origin.attrib if origin is not None else {})))
    roots = links - children
    if len(roots) != 1:
        raise ValueError('URDF must have one root link')
    root = roots.pop()
    poses = {root: (geometry.IDENTITY, [0, 0, 0])}
    pending = [root]
    while pending:
        parent = pending.pop()
        for child, transform in edges.get(parent, []):
            if child in poses:
                raise ValueError('Cycle in URDF')
            poses[child] = geometry.compose(poses[parent], transform)
            pending.append(child)
    if set(poses) != links:
        raise ValueError('Disconnected URDF links')
    return poses


def expand(descriptions, model, sensor, mode):
    try:
        import xacro
        import xacro.substitution_args
    except ImportError as exc:
        raise ValueError('Install xacro (python3 -m pip install xacro), or source your ROS environment') from exc
    # Resolve this source package without requiring an installed ROS workspace.
    original_find = xacro.substitution_args._eval_find
    xacro.substitution_args._eval_find = lambda name: str(descriptions) if name == 'depthai_descriptions' else original_find(name)
    try:
        xml = xacro.process_file(str(descriptions / 'urdf/depthai_descr.urdf.xacro'), mappings={
            'camera_model': model, 'pose_source': 'board', 'legacy_frame_aliases': 'false',
            'camera_name': 'oak', 'base_frame': 'oak',
            'imu_sensor': sensor or '', 'imu_frame': mode})
        return link_poses(ET.fromstring(xml.toxml()))
    except xacro.XacroException as exc:
        raise ValueError(str(exc)) from exc
    finally:
        xacro.substitution_args._eval_find = original_find


def camera_links(config):
    names = {'CAM_A': 'rgb', 'CAM_B': 'left', 'CAM_C': 'right',
             'CAM_D': 'left_back', 'CAM_E': 'right_back'}
    if config.get('name') == 'OAK-D-SR-POE':
        names['CAM_A'] = 'tof'
    elif config.get('name') == 'OAK-T':
        names['CAM_E'] = 'thermal'
    result = {}
    for socket in config['cameras']:
        name = names.get(socket, socket.lower())
        result[socket] = f'oak_{name}_camera_frame'
    return result


def vector(values):
    return '(' + ', '.join(f'{v:.6f}' for v in values) + ')'


def compare_model(model, args):
    path = args.boards / (ALIASES.get(model, model) + '.json')
    board = json.loads(path.read_text())
    config = board.get('board_config', board)
    links = camera_links(config)
    modes = ['native', 'optical'] if args.imu_frame == 'both' else [args.imu_frame]
    failed = False
    for index, mode in enumerate(modes):
        converted = geometry.convert(board, args.reference, mode)
        poses = expand(args.descriptions, model, args.imu_sensor, mode)
        reference = converted['reference_camera']
        reference_link = links[reference]
        if reference_link not in poses:
            raise ValueError(f'Missing URDF reference camera: {reference_link}')
        align = geometry.inverse(poses[reference_link])
        if index == 0:
            print(f'  Reference: {reference}; URDF IMU selection: {args.imu_sensor or "each listed variant explicitly"}')
            ref_in_base = geometry.compose(geometry.inverse(poses['oak']), poses[reference_link])
            print(f'  Reference in URDF base: xyz={vector(ref_in_base[1])} m, '
                  f'rpy={vector(geometry.to_rpy(ref_in_base[0]))} rad')
            print('  Sensor / convention                 Result   ΔXYZ mm (board − URDF)                  Δrotation deg')
        entries = []
        if index == 0:
            entries.extend((socket, value, links[socket], None) for socket, value in converted['cameras'].items())
            actual = {name for name in poses if name.endswith('_camera_frame')}
            for extra in sorted(actual - set(links.values())):
                print(f'  {extra}: EXTRA URDF CAMERA')
                failed = True
            if 'oak_imu_frame' in poses and not converted['imus']:
                print('  oak_imu_frame: EXTRA URDF IMU')
                failed = True
        if args.imu_sensor and args.imu_sensor not in converted['imus']:
            raise ValueError(f'Unknown board IMU variant: {args.imu_sensor}')
        entries.extend((f'IMU {sensor} / {mode}', value, 'oak_imu_frame', sensor)
                       for sensor, value in converted['imus'].items()
                       if not args.imu_sensor or sensor == args.imu_sensor)
        for label, value, link, sensor in entries:
            selected_poses = expand(args.descriptions, model, sensor, mode) if sensor else poses
            if reference_link not in selected_poses:
                raise ValueError(f'Missing URDF reference camera: {reference_link}')
            selected_align = geometry.inverse(selected_poses[reference_link]) if sensor else align
            if link not in selected_poses:
                print(f'  {label:<36} MISSING URDF FRAME ({link})')
                failed = True
                continue
            urdf = geometry.compose(selected_align, selected_poses[link])
            expected = pose(value)
            delta, angle, matches = difference(expected, urdf, args.position_tolerance, args.angle_tolerance)
            failed |= not matches
            print(f'  {label:<36} {"MATCH" if matches else "DIFF":<8} {vector(delta):<40} {angle:.6f}')
            if not matches:
                print(f'    board xyz={vector(expected[1])} m  rpy={vector(geometry.to_rpy(expected[0]))} rad')
                print(f'    URDF  xyz={vector(urdf[1])} m  rpy={vector(geometry.to_rpy(urdf[0]))} rad')
    return failed


def tolerance(value):
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError('Tolerance must be finite and nonnegative')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('models', nargs='*')
    parser.add_argument('--descriptions', type=Path, default=DESCRIPTIONS, help='Description package directory')
    parser.add_argument('--boards', type=Path, default=DESCRIPTIONS.parent / 'depthai-core/shared/depthai_boards/boards')
    parser.add_argument('--reference', help='Override automatic terminal camera selection')
    parser.add_argument('--imu-frame', choices=['native', 'optical', 'both'], default='both')
    parser.add_argument('--imu-sensor', help='Compare only this IMU variant (default: select and compare every listed variant)')
    parser.add_argument('--position-tolerance', type=tolerance, default=0.001, help='Per-axis tolerance in mm (default: 0.001)')
    parser.add_argument('--angle-tolerance', type=tolerance, default=0.0001, help='Rotation tolerance in degrees (default: 0.0001)')
    args = parser.parse_args()
    args.descriptions = args.descriptions.resolve()
    models = args.models or sorted(p.stem for p in (args.descriptions / 'urdf/models').glob('*.stl'))
    if not models:
        parser.error('No models found; specify model names or a valid --descriptions directory')
    differences = errors = 0
    for model in models:
        print(f'\n{model}')
        try:
            differences += bool(compare_model(model, args))
        except (OSError, ValueError, KeyError, TypeError, ET.ParseError) as exc:
            print(f'  ERROR: {exc}')
            errors += 1
    print(f'\n{len(models)} models: {len(models) - differences - errors} matched, '
          f'{differences} with differences, {errors} errors. Mesh offsets not compared.')
    return 2 if errors else 1 if differences else 0


if __name__ == '__main__':
    sys.exit(main())
