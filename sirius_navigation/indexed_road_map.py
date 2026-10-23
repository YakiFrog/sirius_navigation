"""PGM palette indices: semantic IDs 0..6, quantized RGB indices 7..131.

Generation may use fused BGRA observations; consumers need only PGM and JSON.
Matches the existing THETA indexed-map five-level RGB palette convention.
"""
import numpy as np

RESERVED_COLORS = [[127, 127, 127], [0, 0, 0], [255, 255, 255],
                   [0, 255, 0], [255, 255, 0], [0, 0, 255], [128, 128, 128]]
RGB_STEPS = [0, 64, 128, 192, 255]


def road_palette():
    # Keep duplicates of reserved colors. Texture indices must never acquire
    # semantic costs just because their RGB equals a semantic class color.
    return np.asarray(RESERVED_COLORS + [
        [r, g, b] for r in RGB_STEPS for g in RGB_STEPS for b in RGB_STEPS
    ], dtype=np.uint8)


def class_labels(grid):
    """Separate reserved semantic/structural IDs from appearance-only indices."""
    grid = np.asarray(grid)
    if grid.ndim != 2 or grid.dtype != np.uint8:
        raise ValueError('Expected a 2D uint8 PGM index grid')
    return np.where(grid < 7, grid, 0).astype(np.uint8)


def encode_road_map(labels, texture_bgra):
    labels, texture = np.asarray(labels), np.asarray(texture_bgra)
    if labels.ndim != 2 or labels.dtype != np.uint8 or np.any(labels > 6):
        raise ValueError('Expected reserved class IDs 0..6 in a uint8 grid')
    if texture.shape != (*labels.shape, 4) or texture.dtype != np.uint8:
        raise ValueError('Texture must be a uint8 BGRA image on the same grid')
    result = labels.copy()
    observed = texture[:, :, 3] > 0
    fill = (labels == 0) & observed
    # Same rounding and channel order as ThetaIndexedMapNode. No spatial
    # interpolation, hole filling, coordinate changes or semantic replacement.
    rgb = texture[fill, :3][:, ::-1].astype(np.float32)
    bins = np.clip(np.rint(rgb / 63.75), 0, 4).astype(np.uint16)
    result[fill] = (7 + 25*bins[:, 0] + 5*bins[:, 1] + bins[:, 2]).astype(np.uint8)
    report = dict(format='semantic_ids_and_quantized_rgb',
                  semantic_ids=list(range(7)), rgb_index_range=[7, 131],
                  rgb_steps=RGB_STEPS, palette_entries=132,
                  observed_cells=int(observed.sum()),
                  texture_index_cells=int(fill.sum()),
                  semantic_cells=int(np.isin(labels, [3, 4, 5, 6]).sum()),
                  policy='semantic class colors; quantized RGB in observed unclassified cells',
                  consumer_files=['colored.pgm', 'colored.json'])
    return result, road_palette(), report


def indexed_rgb(grid, metadata, mode='indexed', include_structure=False):
    """Decode exactly what the PGM/JSON cloud displays; no texture sidecar."""
    grid = np.asarray(grid)
    palette = np.asarray(metadata.get('palette', []), dtype=np.uint8)
    if grid.ndim != 2 or grid.dtype != np.uint8:
        raise ValueError('Expected a 2D uint8 PGM index grid')
    if palette.ndim != 2 or palette.shape[1] != 3 or not 1 <= len(palette) <= 256:
        raise ValueError('JSON palette must contain 1..256 RGB entries')
    if grid.size and int(grid.max()) >= len(palette):
        raise ValueError('PGM contains an index missing from the JSON palette')
    if mode == 'indexed' or (mode == 'semantic' and include_structure):
        mask = grid > 0
    elif mode == 'semantic':
        ids = [int(k) for k, info in metadata.get('labels', {}).items()
               if str(info.get('name', '')).lower() not in ('unknown', 'wall', 'floor')]
        mask = np.isin(grid, ids)
    else:
        raise ValueError("Use visualization_mode='indexed' or 'semantic'; PNG loading is not supported")
    return palette[grid], mask
