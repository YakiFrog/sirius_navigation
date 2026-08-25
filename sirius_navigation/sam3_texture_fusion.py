"""Pure NumPy/OpenCV helpers for robust 2D floor texture fusion."""

import cv2

import numpy as np


def weighted_texture_from_statistics(color_sum, weight_sum):
    """Convert weighted RGB accumulators to an 8-bit texture."""
    result = np.zeros(color_sum.shape, dtype=np.uint8)
    observed = weight_sum > 1e-6
    if np.any(observed):
        result[observed] = np.clip(
            np.round(color_sum[observed] / weight_sum[observed, None]),
            0,
            255,
        ).astype(np.uint8)
    return result


def estimate_overlap_exposure_gain(
    existing_rgb,
    incoming_rgb,
    observed,
    min_overlap_cells=24,
):
    """Estimate a robust brightness gain from cells visible in both views."""
    if not np.any(observed):
        return 1.0

    existing_luma = (
        0.299 * existing_rgb[:, 0]
        + 0.587 * existing_rgb[:, 1]
        + 0.114 * existing_rgb[:, 2]
    )
    incoming_luma = (
        0.299 * incoming_rgb[:, 0]
        + 0.587 * incoming_rgb[:, 1]
        + 0.114 * incoming_rgb[:, 2]
    )
    usable = (
        observed
        & (existing_luma > 20.0)
        & (existing_luma < 235.0)
        & (incoming_luma > 20.0)
        & (incoming_luma < 235.0)
    )
    if int(np.sum(usable)) < min_overlap_cells:
        return 1.0

    ratios = existing_luma[usable] / incoming_luma[usable]
    lower, upper = np.percentile(ratios, [15.0, 85.0])
    trimmed = ratios[(ratios >= lower) & (ratios <= upper)]
    if trimmed.size == 0:
        return 1.0
    return float(np.clip(np.median(trimmed), 0.75, 1.33))


def robust_color_residual_weights(
    existing_rgb,
    incoming_rgb,
    observed,
    delta=35.0,
    minimum_weight=0.08,
):
    """Return Huber-style weights for conflicting multi-view colors."""
    residual = np.sqrt(np.mean((incoming_rgb - existing_rgb) ** 2, axis=1))
    weights = np.ones_like(residual)
    outlier = observed & (residual > delta)
    weights[outlier] = np.maximum(
        minimum_weight,
        delta / residual[outlier],
    )
    return weights


def fill_small_texture_holes(texture_rgb, valid, floor_mask, min_neighbors=6):
    """Fill isolated one-cell holes without expanding across floor boundaries."""
    valid_u8 = valid.astype(np.uint8)
    kernel = np.ones((3, 3), dtype=np.float32)
    kernel[1, 1] = 0.0
    neighbors = cv2.filter2D(
        valid_u8,
        cv2.CV_32F,
        kernel,
        borderType=cv2.BORDER_CONSTANT,
    )
    fill = (~valid) & floor_mask & (neighbors >= float(min_neighbors))
    if not np.any(fill):
        return texture_rgb, valid

    result = texture_rgb.copy()
    denominator = np.maximum(neighbors, 1.0)
    for channel in range(3):
        channel_sum = cv2.filter2D(
            texture_rgb[:, :, channel].astype(np.float32) * valid_u8,
            cv2.CV_32F,
            kernel,
            borderType=cv2.BORDER_CONSTANT,
        )
        result[:, :, channel][fill] = np.clip(
            np.round(channel_sum[fill] / denominator[fill]),
            0,
            255,
        ).astype(np.uint8)
    return result, valid | fill
