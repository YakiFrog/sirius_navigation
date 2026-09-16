#!/usr/bin/env python3
"""THETA路面マップの color.png を自前データから生成する。

ZED用 sam3_map_colorizer.py は PLY(/cloud_map) の色を塗るが、RTAB-Map の
/cloud_map は色が壊れている（床グレーが赤チャンネル+64のピンクになる）ため、
THETAでは PLY を使わず、
  - <base>.colored.pgm / .colored.json … 代表色パレットのindexed地図
  - <base>.texture.png                … 観測した実RGB（alpha=有効）
から色付きマップを描く。

使い方: python3 theta_colorize_map.py <map_base_path>
"""
import json
import sys

import cv2
import numpy as np


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: python3 theta_colorize_map.py <map_base_path>")
        return 1
    base = sys.argv[1]
    indexed = cv2.imread(base + ".colored.pgm", cv2.IMREAD_GRAYSCALE)
    if indexed is None:
        print(f"Error: {base}.colored.pgm がありません")
        return 1
    with open(base + ".colored.json", "r", encoding="utf-8") as stream:
        meta = json.load(stream)
    palette = np.array(meta["palette"], dtype=np.uint8)
    height, width = indexed.shape

    out = np.full((height, width, 3), 127, dtype=np.uint8)  # unknown = gray (BGR)

    structure = cv2.imread(base + ".pgm", cv2.IMREAD_GRAYSCALE)
    if structure is not None and structure.shape == (height, width):
        out[structure == 0] = (0, 0, 0)        # occupied = black
        out[structure >= 250] = (255, 255, 255)  # free = white

    # 代表色パレット（index>=3）で塗る
    safe = np.clip(indexed, 0, len(palette) - 1)
    mapped = indexed >= 3
    out[mapped] = palette[:, ::-1][safe[mapped]]

    # 観測済みの実RGBテクスチャを優先
    texture = cv2.imread(base + ".texture.png", cv2.IMREAD_UNCHANGED)
    if texture is not None and texture.ndim == 3 and texture.shape[:2] == (height, width) and texture.shape[2] == 4:
        valid = texture[:, :, 3] > 0
        out[valid] = texture[:, :, :3][valid]

    cv2.imwrite(base + ".color.png", out)
    print(f"SUCCESS: Saved visual map to {base}.color.png")

    # セマンティック対象のみのPNG（背景は透明、予約ID>=3のクラスのみ描画）
    labels = meta.get("labels", {}) or {}
    target_ids = sorted(int(idx) for idx in labels if int(idx) >= 3) or [3, 4, 5, 6]
    semantic_only = np.zeros((height, width, 4), dtype=np.uint8)
    for class_id in target_ids:
        if class_id >= len(palette):
            continue
        mask = indexed == class_id
        if not np.any(mask):
            continue
        red, green, blue = (int(channel) for channel in palette[class_id])
        semantic_only[mask, 0] = blue
        semantic_only[mask, 1] = green
        semantic_only[mask, 2] = red
        semantic_only[mask, 3] = 255
    cv2.imwrite(base + ".semantic_only.png", semantic_only)
    print(f"SUCCESS: Saved semantic-only map to {base}.semantic_only.png (classes {target_ids})")

    # ZED風セマンティック地図: unknown=灰, wall=黒, floor=白, 予約クラス=予約色, 代表色はfloor扱い
    semantic_map = np.full((height, width, 3), 127, dtype=np.uint8)
    if structure is not None and structure.shape == (height, width):
        semantic_map[structure == 0] = (0, 0, 0)
        semantic_map[structure >= 250] = (255, 255, 255)
    semantic_map[indexed == 1] = (0, 0, 0)
    semantic_map[indexed == 2] = (255, 255, 255)
    semantic_map[indexed >= 7] = (255, 255, 255)  # 代表色=非セマンティック路面はfloor扱い
    for class_id in target_ids:
        if class_id < len(palette):
            mask = indexed == class_id
            if np.any(mask):
                semantic_map[mask] = palette[class_id][::-1]
    cv2.imwrite(base + ".semantic_map.png", semantic_map)
    print(f"SUCCESS: Saved semantic map to {base}.semantic_map.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
