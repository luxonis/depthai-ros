"""Run and analyze frozen-input tests for RVC2 vertical aggregation asymmetry.

A fixture contains left.png/right.png from an already rectified native pair and
configuration.json from stereo_half_turn_prototype's mesh-native mode. The run
subcommand never reads live camera images and never writes calibration EEPROM.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

import cv2
import numpy as np

PROFILES = ('stock', 'stock-no-v', 'raw', 'raw-no-v', 'raw-no-h')
ORIENTATIONS = ('native', 'vertical', 'horizontal', 'half-turn')
VERTICAL_KEYS = {'costAggregation.verticalPenaltyCostP1', 'costAggregation.verticalPenaltyCostP2'}
HORIZONTAL_KEYS = {'costAggregation.horizontalPenaltyCostP1', 'costAggregation.horizontalPenaltyCostP2'}


def normalized(image, orientation):
    return image if orientation == 'native' else cv2.flip(image, {'vertical': 0, 'horizontal': 1, 'half-turn': -1}[orientation])


def differences(a, b, prefix=''):
    result = {}
    for key in a.keys() | b.keys():
        path = prefix + key
        if key not in a or key not in b:
            result[path] = [a.get(key), b.get(key)]
        elif isinstance(a[key], dict) and isinstance(b[key], dict):
            result.update(differences(a[key], b[key], path + '.'))
        elif a[key] != b[key]:
            result[path] = [a[key], b[key]]
    return result


def analyze(root):
    cases, images = {}, {}
    for directory in sorted(root.iterdir()):
        result_file = directory / 'result.json'
        if not result_file.exists():
            continue
        config = json.loads(result_file.read_text())
        profile, orientation = config['profile'], config['orientation']
        key = (profile, orientation)
        images[key] = {}
        repeatable = True
        for stream in ('depth', 'disparity'):
            frames = [cv2.imread(str(directory / f'{stream}-{i}.png'), -1) for i in (3, 4, 5)]
            if any(frame is None for frame in frames):
                raise ValueError(f'Incomplete captures in {directory}')
            repeatable &= all(np.array_equal(frames[0], frame) for frame in frames[1:])
            images[key][stream] = normalized(frames[-1], orientation)
        input_images = [cv2.imread(str(directory / ('input-' + side + '.png')), 0) for side in ('left', 'right')]
        if any(image is None for image in input_images):
            raise ValueError(f'Missing frozen inputs in {directory}')
        if orientation in ('horizontal', 'half-turn'):
            input_images.reverse()
        images[key]['inputs'] = [normalized(image, orientation) for image in input_images]
        cases[key] = {'directory': str(directory), 'repeatable': bool(repeatable), 'config': config['config']}
    comparisons = {}
    for (profile, orientation), case in cases.items():
        native = images.get((profile, 'native'))
        if native is None:
            continue
        candidate = images[(profile, orientation)]
        row = {'repeatable': case['repeatable'], 'input_pair_exact': all(
            np.array_equal(a, b) for a, b in zip(native['inputs'], candidate['inputs']))}
        for stream in ('depth', 'disparity'):
            a, b = native[stream], candidate[stream]
            common = (a > 0) & (b > 0)
            common[:8] = common[-8:] = False
            common[:, :100] = common[:, -100:] = False
            delta = np.abs(a[common].astype(float) - b[common].astype(float))
            relative = delta / a[common]
            row[stream] = {'entire_raster_exact': bool(np.array_equal(a, b)),
                           'different_pixels': int(np.count_nonzero(a != b)),
                           'validity_disagreement_pixels': int(np.count_nonzero((a > 0) != (b > 0))),
                           'common_interior_pixels': int(common.sum()),
                           'median_relative_difference': float(np.median(relative)) if len(relative) else None,
                           'p90_relative_difference': float(np.percentile(relative, 90)) if len(relative) else None}
        comparisons[profile + '/' + orientation] = row

    def exact(profile, orientation):
        row = comparisons.get(profile + '/' + orientation)
        return row is not None and row['input_pair_exact'] and row['repeatable'] and all(
            row[stream]['entire_raster_exact'] and row[stream]['common_interior_pixels'] >= 100
            for stream in ('depth', 'disparity'))

    def differs(profile, orientation):
        row = comparisons.get(profile + '/' + orientation)
        return row is not None and row['input_pair_exact'] and row['repeatable'] and row['disparity']['different_pixels'] > 0

    config_changes = {}
    for original, intervention in [('stock', 'stock-no-v'), ('raw', 'raw-no-v'), ('raw', 'raw-no-h')]:
        if (original, 'native') in cases and (intervention, 'native') in cases:
            config_changes[intervention] = differences(cases[(original, 'native')]['config'], cases[(intervention, 'native')]['config'])
    def disabled_only(changes, keys):
        return set(changes) == keys and all(before > 0 and after == 0 for before, after in changes.values())

    all_repeatable = bool(cases) and all(case['repeatable'] for case in cases.values())
    full = (all_repeatable and disabled_only(config_changes.get('stock-no-v', {}), VERTICAL_KEYS)
            and exact('stock', 'horizontal') and differs('stock', 'vertical') and differs('stock', 'half-turn')
            and all(exact('stock-no-v', o) for o in ORIENTATIONS))
    isolated = (disabled_only(config_changes.get('raw-no-v', {}), VERTICAL_KEYS)
                and exact('raw', 'horizontal') and differs('raw', 'vertical') and differs('raw', 'half-turn')
                and all(exact('raw-no-v', o) for o in ORIENTATIONS))
    specificity = (disabled_only(config_changes.get('raw-no-h', {}), HORIZONTAL_KEYS)
                   and exact('raw-no-h', 'horizontal') and differs('raw-no-h', 'vertical')
                   and differs('raw-no-h', 'half-turn'))
    return {'root': str(root), 'case_count': len(cases), 'all_cases_repeatable': all_repeatable,
            'full_preset_confirmed': bool(full), 'isolated_matching_confirmed': bool(isolated),
            'horizontal_negative_control_confirmed': bool(specificity),
            'config_changes': config_changes, 'comparisons': comparisons}


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    fixture = args.output / 'frozen'
    fixture.mkdir()
    hashes = {}
    for name in ('left.png', 'right.png', 'configuration.json'):
        source = args.fixture / name
        hashes[name] = hashlib.sha256(source.read_bytes()).hexdigest()
        shutil.copy(source, fixture / name)
    (args.output / 'fixture-sha256.json').write_text(json.dumps(hashes, indent=2))
    records = []
    for profile in args.profiles:
        for orientation in ORIENTATIONS:
            directory = args.output / (profile + '-' + orientation)
            directory.mkdir()
            command = [str(args.binary.resolve()), args.device, str(fixture.resolve()), str(directory.resolve()), profile, orientation]
            print('START', profile, orientation, flush=True)
            with (directory / 'run.log').open('w') as log:
                try:
                    code = subprocess.run(command, env=dict(os.environ, DEPTHAI_TELEMETRY='0'), stdout=log,
                                          stderr=subprocess.STDOUT, timeout=80).returncode
                except subprocess.TimeoutExpired:
                    code = 124
            records.append({'command': command, 'exit_code': code})
            (args.output / 'runs.json').write_text(json.dumps(records, indent=2))
            print('RESULT', code, flush=True)
            time.sleep(3)
    return int(any(record['exit_code'] for record in records))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    execute = commands.add_parser('run')
    execute.add_argument('--binary', type=Path, required=True)
    execute.add_argument('--device', required=True)
    execute.add_argument('--fixture', type=Path, required=True)
    execute.add_argument('--output', type=Path, required=True)
    execute.add_argument('--profiles', nargs='+', choices=PROFILES, default=list(PROFILES))
    inspect = commands.add_parser('analyze')
    inspect.add_argument('root', type=Path)
    inspect.add_argument('--output', type=Path, required=True)
    inspect.add_argument('--require-isolated', action='store_true')
    args = parser.parse_args()
    if args.command == 'run':
        return run(args)
    result = analyze(args.root)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps({key: value for key, value in result.items() if key not in ('comparisons', 'config_changes')}, indent=2))
    return int(not result['full_preset_confirmed'] or (args.require_isolated and not (
        result['isolated_matching_confirmed'] and result['horizontal_negative_control_confirmed'])))


if __name__ == '__main__':
    raise SystemExit(main())
