"""Test robust multi-view floor texture fusion helpers."""

import numpy as np

from sirius_navigation.sam3_texture_fusion import (
    estimate_overlap_exposure_gain,
    fill_small_texture_holes,
    robust_color_residual_weights,
    weighted_texture_from_statistics,
)


def test_weighted_texture_prefers_high_quality_observation():
    """Decode RGB sums using their accumulated quality weights."""
    color_sum = np.array([[[220.0, 220.0, 220.0]]])
    weight_sum = np.array([[2.0]])

    result = weighted_texture_from_statistics(color_sum, weight_sum)

    assert result.tolist() == [[[110, 110, 110]]]


def test_overlap_exposure_gain_recovers_darker_second_view():
    """Match a dark revisit to the luminance of shared mapped cells."""
    existing = np.full((30, 3), 120.0)
    incoming = np.full((30, 3), 100.0)
    observed = np.ones(30, dtype=bool)

    gain = estimate_overlap_exposure_gain(existing, incoming, observed)

    assert np.isclose(gain, 1.2)


def test_overlap_exposure_gain_needs_enough_shared_cells():
    """Do not normalize exposure without enough geometric overlap."""
    existing = np.full((10, 3), 120.0)
    incoming = np.full((10, 3), 100.0)
    observed = np.ones(10, dtype=bool)

    assert estimate_overlap_exposure_gain(existing, incoming, observed) == 1.0


def test_conflicting_revisit_is_downweighted_but_not_discarded():
    """Protect mapped texture from a large misregistered revisit residual."""
    existing = np.array([[100.0, 100.0, 100.0], [100.0, 100.0, 100.0]])
    incoming = np.array([[105.0, 105.0, 105.0], [200.0, 200.0, 200.0]])
    observed = np.array([True, True])

    weights = robust_color_residual_weights(existing, incoming, observed)

    assert np.isclose(weights[0], 1.0)
    assert np.isclose(weights[1], 0.35)


def test_fill_small_texture_holes_only_fills_enclosed_floor_cell():
    """Fill an enclosed pinhole while preserving non-floor boundaries."""
    texture = np.full((5, 5, 3), 80, dtype=np.uint8)
    valid = np.ones((5, 5), dtype=bool)
    valid[2, 2] = False
    valid[0, 0] = False
    floor = np.ones((5, 5), dtype=bool)
    floor[0, 0] = False

    filled_texture, filled_valid = fill_small_texture_holes(
        texture,
        valid,
        floor,
    )

    assert filled_valid[2, 2]
    assert filled_texture[2, 2].tolist() == [80, 80, 80]
    assert not filled_valid[0, 0]
