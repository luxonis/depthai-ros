"""Regression checks for the prototype's offline depth validation."""
import cv2
import numpy as np
import pytest

from analyze_stereo_half_turn import median_image


def test_invalid_depth_is_not_averaged_as_zero_distance(tmp_path):
    frames = [np.array([[0, 1000, 2000], [3000, 0, 1000]], np.uint16),
              np.array([[0, 0, 0], [0, 4000, 2000]], np.uint16)]
    for i, frame in enumerate(frames):
        cv2.imwrite(str(tmp_path / f'depth-{i}.png'), frame)
        cv2.imwrite(str(tmp_path / f'left-{i}.png'), (frame // 20).astype(np.uint8))
    np.testing.assert_equal(median_image(tmp_path, 'depth'), [[0, 1000, 2000], [3000, 4000, 1500]])
    # Black image pixels are real data; only depth uses zero as invalid.
    np.testing.assert_equal(median_image(tmp_path, 'left'), [[0, 25, 50], [75, 100, 75]])


def test_missing_capture_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='No depth snapshots'):
        median_image(tmp_path, 'depth')
