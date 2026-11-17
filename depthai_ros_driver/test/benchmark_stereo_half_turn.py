"""Run matched, opt-in RVC2 stereo half-turn experiments sequentially.

Source ROS and the built workspace first; select the matching SDK library path.
This test never modifies the production driver configuration or device EEPROM.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--device', required=True)
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--streams', choices=('all', 'depth'), default='all')
    parser.add_argument('--modes', nargs='+', choices=('native', 'current', 'mesh-native', 'sensor-flip', 'fused-flip'),
                        default=['native', 'current', 'mesh-native', 'fused-flip'])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, DEPTHAI_STEREO_PROTOTYPE_STREAMS=args.streams, DEPTHAI_TELEMETRY='0')
    runs = []
    for index, mode in enumerate(args.modes):
        directory = args.output / f'{index}-{mode}'
        directory.mkdir()
        command = [str(args.binary.resolve()), args.device, mode, str(args.seconds), str(directory.resolve())]
        print('START', mode, flush=True)
        started = time.monotonic()
        with (directory / 'run.log').open('w') as log:
            try:
                completed = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=args.seconds + 90)
                code = completed.returncode
            except subprocess.TimeoutExpired:
                code = 124
        record = {'mode': mode, 'streams': args.streams, 'command': command, 'exit_code': code,
                  'elapsed_seconds': time.monotonic() - started, 'directory': str(directory.resolve())}
        if (directory / 'result.json').exists():
            record['measurement'] = json.loads((directory / 'result.json').read_text())
        runs.append(record)
        (args.output / 'runs.json').write_text(json.dumps(runs, indent=2))
        print('RESULT', mode, 'exit', code,
              {k: round(v['fps'], 3) for k, v in record.get('measurement', {}).get('streams', {}).items()}, flush=True)
        time.sleep(3)  # Device close/re-enumeration is asynchronous on USB.
    return int(any(run['exit_code'] for run in runs))


if __name__ == '__main__':
    raise SystemExit(main())
