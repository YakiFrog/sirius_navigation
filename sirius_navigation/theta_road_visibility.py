"""Conservative visibility of ground samples using a synchronized planar scan.

This rejects observations, not map geometry. Unknown beams do not establish that
the ground is visible. Objects need no semantic class, but must intersect LiDAR.
"""
import numpy as np
from functools import lru_cache


@lru_cache(maxsize=4)
def _beam_directions(count, increment):
    delta = (np.arange(count)[:, None] - np.arange(count)[None, :]) * increment
    return np.cos(delta), np.sin(delta)


def padded_obstacle_limits(ranges, increment, radius, range_min, range_max):
    """First ray intersection with a radius-metre disk around every scan hit.

    Padding is in metres rather than pixels or a fixed number of beams, so its
    physical size remains the same across scan resolutions and object ranges.
    Unknown readings never become obstacles or evidence of visible ground.
    """
    hits = np.isfinite(ranges) & (ranges >= range_min) & (ranges > 0) & (ranges <= range_max)
    if not np.any(hits):
        return np.full(ranges.shape, np.inf)
    distance = ranges[hits]
    if np.any(distance <= radius):
        return np.zeros(ranges.shape)
    cosine, sine = _beam_directions(ranges.size, increment)
    along = cosine[:, hits] * distance
    discriminant = radius ** 2 - (sine[:, hits] * distance) ** 2
    intersection = along - np.sqrt(np.maximum(discriminant, 0))
    intersection[(discriminant < 0) | (along <= 0)] = np.inf
    return intersection.min(axis=1)


def visible_ground(x, y, angle_min, angle_increment, ranges, range_min,
                   range_max, margin=0.1, obstacle_padding=0.0):
    ranges = np.asarray(ranges, dtype=np.float64)
    if not ranges.size or not np.isfinite(angle_increment) or angle_increment == 0:
        return np.zeros_like(x, dtype=bool)
    distance = np.hypot(x, y)
    angle = np.arctan2(y, x)
    delta = np.mod(np.sign(angle_increment) * (angle - angle_min), 2 * np.pi)
    # Floating point scan endpoints may differ from -pi/pi by a few ulps.
    delta = np.where(delta > 2 * np.pi - abs(angle_increment) / 2, 0, delta)
    index = np.rint(delta / abs(angle_increment)).astype(np.int64)
    inside = (index >= 0) & (index < ranges.size)
    beam = ranges[np.clip(index, 0, ranges.size - 1)]
    clear = np.isposinf(beam) | (np.isfinite(beam) & (beam > range_max))
    known = clear | (np.isfinite(beam) & (beam >= range_min) & (beam > 0))
    limit = np.where(clear, range_max, beam - margin)
    if obstacle_padding > 0:
        padded = padded_obstacle_limits(ranges, angle_increment, obstacle_padding,
                                        range_min, range_max)
        limit = np.minimum(limit, padded[np.clip(index, 0, ranges.size - 1)] - margin)
    return inside & known & np.isfinite(distance) & (distance <= limit)
