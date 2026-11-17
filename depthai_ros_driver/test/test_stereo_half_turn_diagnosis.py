"""Guard against attributing a cause without controlled inputs/configuration."""
import json

import cv2
import numpy as np
import pytest

from diagnose_stereo_half_turn import analyze, normalized, PROFILES, ORIENTATIONS


@pytest.fixture
def experiment(tmp_path):
    pixels = np.arange(32 * 224, dtype=np.uint16).reshape(32, 224)
    inputs = [(pixels % 251).astype(np.uint8), ((pixels + 17) % 251).astype(np.uint8)]
    for profile in PROFILES:
        config = {'costAggregation': {'verticalPenaltyCostP1': 250, 'verticalPenaltyCostP2': 500,
                                     'horizontalPenaltyCostP1': 250, 'horizontalPenaltyCostP2': 500}}
        axis = 'vertical' if profile.endswith('no-v') else 'horizontal' if profile.endswith('no-h') else None
        if axis:
            for suffix in ('P1', 'P2'):
                config['costAggregation'][axis + 'PenaltyCost' + suffix] = 0
        for orientation in ORIENTATIONS:
            directory = tmp_path / (profile + '-' + orientation)
            directory.mkdir()
            (directory / 'result.json').write_text(json.dumps({'profile': profile, 'orientation': orientation, 'config': config}))
            pair = [normalized(image, orientation) for image in inputs]
            if orientation in ('half-turn', 'horizontal'):
                pair.reverse()
            for side, image in zip(('left', 'right'), pair):
                cv2.imwrite(str(directory / ('input-' + side + '.png')), image)
            depth = 1000 + pixels
            if orientation in ('vertical', 'half-turn') and not profile.endswith('no-v'):
                depth[12, 112] += 10
            for stream in ('depth', 'disparity'):
                for frame in (3, 4, 5):
                    cv2.imwrite(str(directory / f'{stream}-{frame}.png'), normalized(depth, orientation))
    return tmp_path


def test_confirmation_requires_specific_intervention_and_negative_control(experiment):
    result = analyze(experiment)
    assert result['full_preset_confirmed']
    assert result['isolated_matching_confirmed']
    assert result['horizontal_negative_control_confirmed']
    (experiment / 'raw-no-h-half-turn' / 'result.json').unlink()
    assert not analyze(experiment)['horizontal_negative_control_confirmed']


def test_other_configuration_changes_prevent_attribution(experiment):
    path = experiment / 'stock-no-v-native' / 'result.json'
    data = json.loads(path.read_text())
    data['config']['costAggregation']['horizontalPenaltyCostP1'] = 0
    path.write_text(json.dumps(data))
    assert not analyze(experiment)['full_preset_confirmed']


def test_changed_inputs_prevent_attribution(experiment):
    path = experiment / 'raw-no-v-vertical' / 'input-left.png'
    image = cv2.imread(str(path), 0)
    image[0, 0] ^= 1
    cv2.imwrite(str(path), image)
    assert not analyze(experiment)['isolated_matching_confirmed']
