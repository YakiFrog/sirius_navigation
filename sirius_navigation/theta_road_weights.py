"""Direction and observation quality weights for planar road mapping."""
import numpy as np


def directional_weights(x, y, rear_weight=1.0, transition_degrees=30.0):
    """Front=1, rear=rear_weight, with a smooth transition around both sides.

    x/y are in the robot frame at exposure time, not the map frame. This is a
    recording-condition prior, not a probability that an object is static.
    """
    radius = np.hypot(x, y)
    cosine = np.divide(x, radius, out=np.zeros_like(radius, dtype=float), where=radius > 0)
    half = np.sin(np.deg2rad(transition_degrees / 2))
    front = (cosine >= 0).astype(float) if half <= 0 else np.clip(.5 + .5*cosine/half, 0, 1)
    return (rear_weight + (1-rear_weight)*front).astype(np.float32)


def reliable_observations(gx, gy, weight, best_weight, min_relative_weight):
    """Keep observations comparable to the best quality previously seen per cell.

    Low-quality observations can fill unseen cells, but cannot slowly overwrite
    a better view just by being repeated. Use point quality, not point density.
    """
    keep = np.isfinite(weight) & (weight > 0)
    keep &= weight >= best_weight[gy, gx]*min_relative_weight
    np.maximum.at(best_weight, (gy[keep], gx[keep]), weight[keep])
    return keep
