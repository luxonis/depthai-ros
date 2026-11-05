#!/usr/bin/env python3
"""Resolve a board/chip and expand Xacro for launch files and TFPublisher."""

import argparse
import contextlib
import io
import json
from pathlib import Path
import shlex
import sys
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml


def resolve_model(model, boards, overrides):
    if model in boards.get('unsupported', {}):
        raise ValueError(f"Unsupported board {model}: {boards['unsupported'][model]}")
    known = set(boards['models']) | set(overrides['models'])
    if model in known:
        return model
    target = overrides.get('aliases', {}).get(model)
    if target in known:
        return target
    raise ValueError(f'Unknown camera model {model!r}; select a catalog model or a custom URDF')


def resolve_imu(sensor, detected, variants, required):
    # RVC4 reports its LSM6DSV accelerometer using the generic name "accel".
    # Only apply this hardware alias on boards that describe that sensor.
    if not sensor and detected == 'accel' and 'LSM6DSV' in variants:
        detected = 'LSM6DSV'
    requested = sensor or detected
    if requested in variants:
        return requested
    matches = [key for key, pose in variants.items() if pose['name'] == requested]
    if len(matches) == 1:
        return matches[0]
    if requested:
        raise ValueError(f'IMU {requested!r} is not described by this board; available: {", ".join(variants)}')
    if len(variants) == 1:
        return next(iter(variants))
    if required:
        raise ValueError(f'Cannot select an IMU; set imu_sensor to one of: {", ".join(variants) or "none"}')
    return ''


def expand(mappings, description=None, detected_imu='', share=None):
    mappings = dict(mappings)
    model = mappings.get('camera_model', 'OAK-D')
    poses = {}
    warnings = []
    effective_source = 'custom'
    if description is None:
        share = Path(share or get_package_share_directory('depthai_descriptions'))
        boards = yaml.safe_load((share / 'urdf/include/board_models.yaml').read_text())
        overrides = yaml.safe_load((share / 'urdf/include/model_config.yaml').read_text())
        legacy = yaml.safe_load((share / 'urdf/include/legacy_models.yaml').read_text())
        requested_source = mappings.get('pose_source', 'legacy')
        selection = mappings.pop('model_selection', 'auto')
        if selection == 'auto':
            # Standalone Xacro/launch users historically selected an exact model.
            # TFPublisher explicitly requests the old driver family mapping.
            selection = 'exact'
        if selection not in ('exact', 'legacy'):
            raise ValueError('model_selection must be exact or legacy')
        if selection == 'legacy':
            # DEPRECATED family geometry substitutions, retained only for migration.
            target = legacy['model_aliases'].get(model, model)
            warnings.append(f'DEPRECATED model_selection=legacy: {model} -> {target}; use exact for board geometry.')
            model = target
            if model not in legacy['models'] and mappings.get('fallback_model'):
                fallback = mappings.pop('fallback_model')
                warnings.append(f'Legacy model fallback: {model} -> {fallback}')
                model = fallback
        try:
            model = resolve_model(model, boards, overrides)
        except ValueError:
            fallback = mappings.pop('fallback_model', '')
            if not fallback:
                raise
            warnings.append(f'Explicit fallback_model: {model} -> {fallback}')
            model = resolve_model(fallback, boards, overrides)
        effective_source = 'legacy' if requested_source == 'legacy' and model in legacy['models'] else 'board'
        mappings['pose_source'] = requested_source
        mesh = mappings.get('mesh_model', '')
        if mesh and (Path(mesh).name != mesh or not (share / 'urdf/models' / (mesh + '.stl')).is_file()):
            raise ValueError(f'Unknown mesh_model {mesh!r}; select an installed STL model')
        mappings['camera_model'] = model
        record = boards['models'].get(model, {})
        config = legacy['models'][model] if effective_source == 'legacy' else dict(record, **overrides['models'].get(model, {}))
        required = mappings.get('has_imu', 'false').lower() in ('true', '1')
        if effective_source == 'legacy' and required and (config.get('imu_xyz') is None or config.get('imu_rpy') is None):
            raise ValueError(f'Legacy description for {model} has no IMU transform. '
                             'Selecting imu_sensor cannot add one to frozen legacy geometry. '
                             'Use pose_source:=board, or provide a URDF containing the IMU transform you want to test.')
        # Detection is only relevant when this description owns the IMU TF.
        sensor = mappings.get('imu_sensor', '')
        if effective_source == 'board' and (sensor or required):
            mappings['imu_sensor'] = resolve_imu(sensor, detected_imu if required else '',
                                                  record.get('imu_variants', {}), required)
        if effective_source == 'legacy':
            record = {}  # Develop calibrated TF rooted its terminal sensor at zero.
        for role in ('rgb', 'left', 'right'):
            socket = record.get(role + '_socket')
            if socket and config.get(role + '_xyz') is not None:
                poses[socket] = {'xyz': list(map(float, config[role + '_xyz'].split())),
                                 'rpy': list(map(float, config.get(role + '_rpy', '0 0 0').split()))}
        for pose in config.get('extra_cameras', {}).values():
            poses[pose['socket']] = {key: list(map(float, pose[key].split())) for key in ('xyz', 'rpy')}
        base_only = mappings.pop('use_base_descr', 'false').lower() in ('true', '1')
        description = share / 'urdf' / ('base_descr.urdf.xacro' if base_only else 'depthai_descr.urdf.xacro')
    diagnostics = io.StringIO()
    try:
        with contextlib.redirect_stderr(diagnostics):
            document = xacro.process_file(str(description), mappings=mappings)
        xml = document.toxml()
        root = ElementTree.fromstring(xml)
        if root.tag != 'robot' or root.find('link') is None:
            raise ValueError('Xacro output must contain a robot with at least one link')
    except Exception as exc:
        raise ValueError(f'Failed to expand {description}: {exc}\n{diagnostics.getvalue()}') from exc
    return {'model': model, 'imu_sensor': mappings.get('imu_sensor', ''),
            'pose_source': effective_source, 'camera_poses': poses, 'urdf': xml,
            'diagnostics': '\n'.join(warnings) + ('\n' if warnings else '') + diagnostics.getvalue()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path, help='Custom Xacro; bypasses catalog resolution')
    parser.add_argument('--detected-imu', default='')
    parser.add_argument('--custom-args', help='Legacy Xacro mapping string, parsed with shlex (never executed)')
    parser.add_argument('--output', choices=('xml', 'json'), default='xml')
    parser.add_argument('mappings', nargs='*', help='Xacro name:=value mappings')
    args = parser.parse_args()
    try:
        tokens = iter(args.mappings + (shlex.split(args.custom_args) if args.custom_args is not None else []))
        mappings = {}
        for token in tokens:
            # DEPRECATED Xacro CLI compatibility; in-order processing is the default.
            if token in ('--inorder', '-i'):
                continue
            if token == '--verbosity':
                value = next(tokens, '')
                if value not in ('0', '1', '2', '3', '4'):
                    raise ValueError('--verbosity requires an integer from 0 to 4')
                continue
            if token.startswith('--verbosity=') and token.split('=', 1)[1] in ('0', '1', '2', '3', '4'):
                continue
            key, sep, value = token.partition(':=')
            if not sep or not key:
                raise ValueError(f'Expected name:=value, got {token!r}. Shell expressions and output/file flags are not supported; pass literal mappings or use --description.')
            mappings[key] = value
        result = expand(mappings, args.description, args.detected_imu)
        print(json.dumps(result) if args.output == 'json' else result['urdf'])
        if args.output == 'xml' and result['diagnostics']:
            print(result['diagnostics'], file=sys.stderr, end='')
    except Exception as exc:
        parser.exit(2, f'{exc}\n')


if __name__ == '__main__':
    main()
