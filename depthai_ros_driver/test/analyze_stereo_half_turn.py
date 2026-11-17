"""Offline geometry checks for stereo_half_turn_prototype output directories.

Pass directories from one matched run: native current mesh-native sensor-flip
fused-flip. The analysis rotates saved validation images, never pipeline pixels.
"""
import argparse
import json
import warnings
from pathlib import Path

import cv2
import numpy as np


def median_image(directory, stream):
    files = sorted(directory.glob(stream + '-*.png'))
    if not files:
        raise ValueError(f'No {stream} snapshots in {directory}')
    stack = np.stack([cv2.imread(str(p), cv2.IMREAD_UNCHANGED) for p in files])
    if stream != 'depth':
        return np.median(stack, axis=0)
    # Zero denotes invalid depth, not a distance. Including zeros can halve
    # the median at pixels valid in exactly half of an even-sized sample.
    valid = stack > 0
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)  # All-invalid pixels stay zero.
        result = np.nanmedian(np.where(valid, stack, np.nan), axis=0)
    result[(valid.sum(axis=0) * 2 < len(stack)) | ~np.isfinite(result)] = 0
    return result


def correlation(a, b):
    return float(cv2.matchTemplate(a.astype(np.float32), b.astype(np.float32), cv2.TM_CCOEFF_NORMED)[0, 0])


def epipolar(left, right, depth=None, focal_baseline_mm=None):
    sift = cv2.SIFT_create()
    kl, dl = sift.detectAndCompute(left.astype(np.uint8), None)
    kr, dr = sift.detectAndCompute(right.astype(np.uint8), None)
    if dl is None or dr is None:
        return {'matches': 0}
    pairs = cv2.BFMatcher().knnMatch(dl, dr, k=2)
    matches = [m for pair in pairs if len(pair) == 2 for m, n in [pair] if m.distance < 0.7 * n.distance]
    # No epipolar constraint is used when selecting these descriptor matches.
    dy = np.array([abs(kl[m.queryIdx].pt[1] - kr[m.trainIdx].pt[1]) for m in matches])
    result = {'matches': len(matches), 'median_vertical_error_px': float(np.median(dy)) if len(dy) else None,
              'p90_vertical_error_px': float(np.percentile(dy, 90)) if len(dy) else None}
    if depth is not None:
        errors, seen = [], set()
        for match in matches:
            l, r = kl[match.queryIdx].pt, kr[match.trainIdx].pt
            pixel = tuple(int(round(v)) for v in l)
            if pixel in seen or abs(l[1] - r[1]) > 1 or l[0] - r[0] <= 1:
                continue
            x, y = pixel
            if not (0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]) or depth[y, x] == 0:
                continue
            expected = focal_baseline_mm / (l[0] - r[0])
            errors.append(abs(float(depth[y, x]) - expected) / expected)
            seen.add(pixel)
        result['triangulated_features'] = len(errors)
        result['median_triangulated_depth_error'] = float(np.median(errors)) if errors else None
        result['p90_triangulated_depth_error'] = float(np.percentile(errors, 90)) if errors else None
    return result


def compare(reference, candidate, half_turn=True):
    images = {s: median_image(reference, s) for s in ('depth', 'left', 'right')}
    output = {s: (cv2.rotate(median_image(candidate, s), cv2.ROTATE_180) if half_turn else median_image(candidate, s)) for s in images}
    mask = (images['depth'] > 0) & (output['depth'] > 0)
    mask[:8] = mask[-8:] = False
    mask[:, :8] = mask[:, -8:] = False
    relative = np.abs(images['depth'][mask] - output['depth'][mask]) / images['depth'][mask]
    result = {'reference': str(reference), 'candidate': str(candidate), 'half_turn': half_turn, 'common_valid_pixels': int(mask.sum()),
              'reference_valid_fraction': float(np.mean(images['depth'] > 0)),
              'candidate_valid_fraction': float(np.mean(output['depth'] > 0)),
              'median_depth_relative_error': float(np.median(relative)) if len(relative) else None,
              'p90_depth_relative_error': float(np.percentile(relative, 90)) if len(relative) else None,
              'left_correlation': correlation(images['left'], output['left']),
              'right_correlation': correlation(images['right'], output['right']),
              'epipolar': epipolar(output['left'], output['right'])}
    config = json.loads((candidate / 'result.json').read_text())
    if config['custom_mesh']:
        focal_baseline_mm = config['custom_rect_K'][0] * config['baseline_cm'] * 10
        result['epipolar'] = epipolar(output['left'], output['right'], output['depth'], focal_baseline_mm)
    texture = images['left'].astype(np.float32)
    deviation = np.sqrt(np.maximum(0, cv2.blur(texture * texture, (7, 7)) - cv2.blur(texture, (7, 7)) ** 2))
    textured = mask & (deviation > 20)
    error = np.abs(images['depth'][textured] - output['depth'][textured]) / images['depth'][textured]
    result['textured_pixels'] = int(textured.sum())
    result['textured_depth_relative_error_p90'] = float(np.percentile(error, 90)) if len(error) else None
    result['passes'] = (result['common_valid_pixels'] > 1000
                        and result['median_depth_relative_error'] < 0.05
                        and result['p90_depth_relative_error'] < 0.15
                        and result['candidate_valid_fraction'] >= 0.8 * result['reference_valid_fraction']
                        and min(result['left_correlation'], result['right_correlation']) > 0.8
                        and result['epipolar']['matches'] >= 10
                        and result['epipolar']['median_vertical_error_px'] < 1.5)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeat-control', nargs=2, type=Path)
    args = parser.parse_args()
    modes = {}
    for directory in args.directories:
        result = json.loads((directory / 'result.json').read_text())
        modes[result['mode']] = (directory, result)
    results = {'performance': {k: v[1] for k, v in modes.items()}, 'geometry': []}
    for reference, candidate in [('native', 'current'), ('mesh-native', 'sensor-flip'), ('mesh-native', 'fused-flip')]:
        if reference in modes and candidate in modes:
            results['geometry'].append(compare(modes[reference][0], modes[candidate][0]))
    if args.repeat_control:
        results['repeat_control'] = compare(*args.repeat_control, half_turn=False)
    args.output.write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    return int(any(not item['passes'] for item in results['geometry'])
               or ('repeat_control' in results and not results['repeat_control']['passes']))


if __name__ == '__main__':
    raise SystemExit(main())
