#!/usr/bin/env python3
"""THETA SAM3 事前確認: rosbag代表フレーム推論 -> HTMLレポート。

選択済みrosbagの /theta/dual_fisheye/image_raw/compressed から等間隔にN枚を抽出し、
本番の theta_sam3_perspective_node と同じ経路（dual-fisheye->透視投影->SAM3->BEV逆投影）
で推論する。閾値スイープ・クラス別尤度統計・カバレッジ・透視オーバーレイ・BEVラベルを
自己完結HTMLにしてブラウザ表示する（SAM3サーバ :8080 が必要）。
"""
from __future__ import annotations

import argparse
import base64
import sys
import time
import webbrowser
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import requests
import yaml

try:
    from sirius_navigation.theta_perspective import (
        build_perspective_remap,
        build_bev_to_view_maps,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from theta_perspective import (  # type: ignore
        build_perspective_remap,
        build_bev_to_view_maps,
    )


CAMERA_TOPIC = '/theta/dual_fisheye/image_raw/compressed'
MAX_CLASS_ID = 6
CLASS_COLORS = {
    0: (127, 127, 127),
    1: (0, 0, 0),
    2: (255, 255, 255),
    3: (0, 255, 0),
    4: (255, 255, 0),
    5: (0, 0, 255),
    6: (128, 128, 128),
}
DEFAULT_SAM3_CONFIG = (
    Path.home() / 'sirius_jazzy_ws' / 'src' / 'sirius' / 'sirius_navigation'
    / 'config' / 'theta_sam3.yaml'
)
OVERLAY_CLASS_COLORS = {
    3: (0, 255, 0),
    4: (255, 255, 0),
    5: (0, 0, 255),
    6: (128, 128, 128),
}


def load_sam3_config(path: Path) -> dict:
    cfg = {}
    if path and Path(path).exists():
        with open(path, encoding='utf-8') as stream:
            cfg = yaml.safe_load(stream) or {}
    classes = cfg.get('classes') or {
        'grass': {'id': 3, 'color': [0, 255, 0]},
        'tactile paving': {'id': 4, 'color': [255, 255, 0]},
        'line-type tactile paving': {'id': 4, 'color': [255, 255, 0]},
    }
    return {
        'prompt': cfg.get('prompt', 'grass, tactile paving, line-type tactile paving'),
        'threshold': float(cfg.get('threshold', 0.3)),
        'score_min': float(cfg.get('score_min', 0.3)),
        'classes': classes,
        'class_thresholds': cfg.get('class_thresholds') or {},
    }


def inspect_bag(path: Path) -> dict:
    metadata = yaml.safe_load((path / 'metadata.yaml').read_text(encoding='utf-8'))
    info = metadata['rosbag2_bagfile_information']
    counts = {}
    for item in info.get('topics_with_message_count', []):
        topic_meta = item.get('topic_metadata', {})
        counts[topic_meta.get('name', '')] = int(item.get('message_count', 0))
    return {
        'duration': int(info.get('duration', {}).get('nanoseconds', 0)) / 1e9,
        'start_ns': int(info.get('starting_time', {}).get('nanoseconds_since_epoch', 0)),
        'storage_id': str(info.get('storage_identifier', 'mcap')),
        'counts': counts,
    }


def sample_frames(path: Path, storage_id: str, topic: str, targets: list[int]) -> list:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise RuntimeError(f'{topic} がbagにありません')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    wanted = set(targets)
    frames = {}
    index = 0
    while reader.has_next() and len(frames) < len(wanted):
        _, serialized, bag_ns = reader.read_next()
        if index in wanted:
            msg = deserialize_message(serialized, get_message(type_map[topic]))
            bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if bgr is not None:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frames[index] = (bgr, stamp, int(bag_ns))
        index += 1
    return [frames[i] for i in sorted(frames)]


class Sam3Client:
    def __init__(self, server: str):
        self.server = server.rstrip('/')

    def post(self, endpoint: str, payload: dict | None = None, data: bytes | None = None) -> bool:
        try:
            response = requests.post(
                self.server + endpoint, json=payload, data=data, timeout=5.0)
            return response.status_code == 200
        except Exception as error:
            print(f'  POST {endpoint} 失敗: {error}')
            return False

    def configure(self, config: dict) -> bool:
        ok = self.post('/source_mode', {'mode': 'network'})
        ok &= self.post('/class_registry', {'classes': config['classes']})
        ok &= self.post('/prompt', {'prompt': config['prompt']})
        ok &= self.post('/threshold', {'threshold': config['threshold']})
        if config.get('class_thresholds'):
            self.post('/class_thresholds',
                      {'default': config['threshold'], 'classes': config['class_thresholds']})
        return ok

    def _versions(self) -> tuple[int, int]:
        try:
            state = requests.get(self.server + '/debug_state', timeout=2.0).json()
            return (int(state.get('network_frame_version', -1)),
                    int(state.get('sam3_inference_frame_version', -1)))
        except Exception:
            return -1, -1

    def infer(self, frame_bgr, out_size: tuple[int, int], timeout: float):
        _, before = self._versions()
        ok, encoded = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok or not self.post('/upload_frame', data=encoded.tobytes()):
            return None, None
        deadline = time.time() + timeout
        while time.time() < deadline:
            net, inference = self._versions()
            if inference > before and inference >= net:
                break
            time.sleep(0.03)
        try:
            class_response = requests.get(self.server + '/semantic_map.png', timeout=3.0)
            score_response = requests.get(self.server + '/semantic_score.png', timeout=3.0)
        except Exception:
            return None, None
        if class_response.status_code != 200 or score_response.status_code != 200:
            return None, None
        class_map = cv2.imdecode(
            np.frombuffer(class_response.content, np.uint8), cv2.IMREAD_GRAYSCALE)
        score_map = cv2.imdecode(
            np.frombuffer(score_response.content, np.uint8), cv2.IMREAD_GRAYSCALE)
        if class_map is None or score_map is None:
            return None, None
        width, height = out_size
        if class_map.shape != (height, width):
            class_map = cv2.resize(class_map, (width, height), interpolation=cv2.INTER_NEAREST)
            score_map = cv2.resize(score_map, (width, height), interpolation=cv2.INTER_NEAREST)
        return class_map, score_map


def overlay_view(view_image, class_map):
    overlay = view_image.copy()
    for class_id, color_rgb in OVERLAY_CLASS_COLORS.items():
        mask = class_map == class_id
        if np.any(mask):
            overlay[mask] = (0.5 * overlay[mask] + 0.5 * np.array(color_rgb[::-1])).astype(np.uint8)
    return overlay


def make_montage(images: list, cols: int, cell: int = 300, labels: list | None = None):
    if not images:
        return None
    tiles = []
    for image in images:
        resized = cv2.resize(image, (cell, cell), interpolation=cv2.INTER_AREA)
        tiles.append(resized)
    rows = []
    for start in range(0, len(tiles), cols):
        row_tiles = tiles[start:start + cols]
        while len(row_tiles) < cols:
            row_tiles.append(np.zeros_like(tiles[0]))
        rows.append(np.hstack(row_tiles))
    montage = np.vstack(rows)
    if labels:
        for position, text in enumerate(labels[:len(tiles)]):
            row, col = divmod(position, cols)
            cv2.putText(montage, text, (col * cell + 8, row * cell + 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(montage, text, (col * cell + 8, row * cell + 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return montage


def colorize_label(label, palette: dict):
    output = np.zeros((*label.shape, 3), dtype=np.uint8)
    for class_id, color_rgb in palette.items():
        output[label == class_id] = color_rgb[::-1]
    return output


def encode_png_base64(image) -> str:
    ok, buffer = cv2.imencode('.png', image)
    if not ok:
        return ''
    return base64.b64encode(buffer.tobytes()).decode('ascii')


def encode_jpeg_base64(image, quality=70) -> str:
    ok, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return ''
    return base64.b64encode(buffer.tobytes()).decode('ascii')


def make_row(images: list, height: int = 240, gap: int = 4):
    tiles = []
    for image in images:
        scale = height / image.shape[0]
        width = max(1, int(round(image.shape[1] * scale)))
        tiles.append(cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA))
    if not tiles:
        return None
    row = tiles[0]
    for tile in tiles[1:]:
        row = np.hstack([row, np.full((height, gap, 3), 40, np.uint8), tile])
    return row


def draw_label(image, text):
    cv2.putText(image, text, (8, image.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, (8, image.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def build_video_base64(frames: list, fps: float) -> str:
    import shutil
    import subprocess
    import tempfile
    if not frames or shutil.which('ffmpeg') is None:
        return ''
    fps = max(0.5, float(fps))
    with tempfile.TemporaryDirectory(prefix='sam3-preview-video-') as directory:
        for index, frame in enumerate(frames):
            cv2.imwrite(f'{directory}/f{index:05d}.png', frame)
        output = f'{directory}/timeline.mp4'
        command = [
            'ffmpeg', '-y', '-loglevel', 'error',
            '-framerate', f'{fps}', '-i', f'{directory}/f%05d.png',
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
            output,
        ]
        try:
            subprocess.run(command, check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            return ''
        if not Path(output).is_file():
            return ''
        return base64.b64encode(Path(output).read_bytes()).decode('ascii')


def frame_stats(frame_samples, ground_union, ground_cells, bev_size, score_min, semantic_ids):
    per_class = {cid: 0 for cid in semantic_ids}
    if frame_samples:
        rows = np.concatenate([group[0] for group in frame_samples])
        cols = np.concatenate([group[1] for group in frame_samples])
        classes = np.concatenate([group[2] for group in frame_samples])
        scores = np.concatenate([group[3] for group in frame_samples])
        label, per_class = aggregate(rows, cols, classes, scores, bev_size, score_min, semantic_ids)
        semantic_cells = int(np.count_nonzero(label >= 3))
        unknown_cells = int(np.count_nonzero((label == 0) & ground_union))
    else:
        semantic_cells = 0
        unknown_cells = ground_cells
    return {
        'semantic_pct': 100.0 * semantic_cells / ground_cells,
        'unknown_pct': 100.0 * unknown_cells / ground_cells,
        'per_class': per_class,
        'per_class_pct': {cid: 100.0 * per_class.get(cid, 0) / ground_cells
                          for cid in semantic_ids},
    }


TIMELINE_TOTAL_COLOR = '#1a7f37'
TIMELINE_LINE_COLORS = ['#b8860b', '#c0392b', '#555555', '#1f6fb2', '#8e44ad', '#e67e22']


def build_timeline_svg(entries: list, semantic_ids: list, names: dict) -> tuple[str, list]:
    if not entries:
        return '', []
    width, height, left, top, right, bottom = 960, 250, 46, 40, 12, 30
    plot_w = width - left - right
    plot_h = height - top - bottom
    times = [entry['time_s'] for entry in entries]
    t_min, t_max = min(times), max(times)
    if t_max <= t_min:
        t_max = t_min + 1.0
    colors = {cid: TIMELINE_LINE_COLORS[i % len(TIMELINE_LINE_COLORS)]
              for i, cid in enumerate(semantic_ids)}

    def x_of(time_s):
        return left + (time_s - t_min) / (t_max - t_min) * plot_w

    def y_of(pct):
        return top + plot_h - (pct / 100.0) * plot_h

    def polyline(getter, color, dash=''):
        points = ' '.join(
            f'{x_of(entry["time_s"]):.1f},{y_of(getter(entry)):.1f}' for entry in entries)
        extra = f' stroke-dasharray="{dash}"' if dash else ''
        return (f'<polyline points="{points}" fill="none" stroke="{color}" '
                f'stroke-width="2"{extra}/>')

    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" height="{height}" '
             f'xmlns="http://www.w3.org/2000/svg" '
             f'style="background:#fafafa;border:1px solid #ddd">']
    for pct in (0, 25, 50, 75, 100):
        y_value = y_of(pct)
        parts.append(f'<line x1="{left}" y1="{y_value:.1f}" x2="{left + plot_w}" '
                     f'y2="{y_value:.1f}" stroke="#e6e6e6"/>')
        parts.append(f'<text x="{left - 6}" y="{y_value + 4:.1f}" font-size="11" '
                     f'text-anchor="end" fill="#666">{pct}</text>')
    parts.append(polyline(lambda entry: entry['semantic_pct'], TIMELINE_TOTAL_COLOR))
    for cid in semantic_ids:
        parts.append(polyline(
            (lambda cid: lambda entry: entry['per_class_pct'].get(cid, 0.0))(cid),
            colors[cid], dash='4 3'))
    for fraction in (0.0, 0.5, 1.0):
        time_value = t_min + fraction * (t_max - t_min)
        parts.append(f'<text x="{x_of(time_value):.1f}" y="{height - 8}" font-size="11" '
                     f'text-anchor="middle" fill="#666">{time_value:.0f}s</text>')

    legend = [('total', TIMELINE_TOTAL_COLOR)] + [(names.get(cid, str(cid)), colors[cid])
                                                  for cid in semantic_ids]
    lx = left
    for label, color in legend:
        parts.append(f'<rect x="{lx}" y="{top - 25}" width="10" height="10" fill="{color}"/>')
        parts.append(f'<text x="{lx + 14}" y="{top - 16}" font-size="11" fill="#444">{label}</text>')
        lx += 30 + 7 * len(str(label)) + 16
    parts.append('</svg>')
    return ''.join(parts), legend


def palette_from_config(config: dict) -> dict:
    palette = dict(CLASS_COLORS)
    for spec in (config.get('classes') or {}).values():
        if not isinstance(spec, dict):
            continue
        class_id = int(spec.get('id', 0))
        color = spec.get('color', [255, 255, 255])
        if len(color) >= 3:
            palette[class_id] = (int(color[0]), int(color[1]), int(color[2]))
    return palette


CANDIDATE_NAMES = {
    0: 'unknown', 1: 'wall', 2: 'floor',
    3: 'grass', 4: 'tactile paving', 5: 'roadway', 6: 'sidewalk',
}


def class_name_map(config: dict) -> dict:
    names = dict(CANDIDATE_NAMES)
    for name, spec in (config.get('classes') or {}).items():
        if isinstance(spec, dict):
            names[int(spec.get('id', -1))] = name
    return names


def run(args) -> int:
    bag = Path(args.bag).expanduser().resolve()
    if not (bag / 'metadata.yaml').is_file():
        raise RuntimeError(f'rosbagではありません（metadata.yaml無し）: {bag}')
    info = inspect_bag(bag)
    total = info['counts'].get(CAMERA_TOPIC, 0)
    if total == 0:
        raise RuntimeError(f'{CAMERA_TOPIC} がbagにありません')
    frame_count = max(1, min(args.frames, total))
    targets = np.unique(np.linspace(0, total - 1, frame_count).astype(int)).tolist()
    print(f'bag: {bag}')
    print(f'  {CAMERA_TOPIC}: {total} 件 -> {len(targets)} 枚を等間隔抽出')

    frames = sample_frames(bag, info['storage_id'], CAMERA_TOPIC, targets)
    if not frames:
        raise RuntimeError('フレームを取得できませんでした')
    print(f'  デコード成功: {len(frames)} 枚')

    config = load_sam3_config(Path(args.sam3_config).expanduser())
    calibration = yaml.safe_load(Path(args.calibration).expanduser().read_text(encoding='utf-8'))
    view_yaws = [float(value) for value in str(args.view_yaws).split(',') if value.strip()]
    out_size = (int(args.out_width), int(args.out_height))
    bev_size = int(calibration['bev_size'])

    image_size = frames[0][0].shape[:2]
    remaps = [build_perspective_remap(calibration, yaw, image_size, out_size, args.hfov)
              for yaw in view_yaws]
    bev_maps = [build_bev_to_view_maps(calibration, yaw, bev_size, out_size, args.hfov,
                                       args.min_radius, args.max_radius)
                for yaw in view_yaws]
    ground_union = np.zeros((bev_size, bev_size), dtype=bool)
    for _, _, valid in bev_maps:
        ground_union |= valid
    ground_cells = int(np.count_nonzero(ground_union))
    if ground_cells == 0:
        raise RuntimeError('BEVの有効領域がありません（キャリブレーションを確認）')

    client = Sam3Client(args.server)
    print(f'SAM3サーバ設定を適用: {args.server} (prompt="{config["prompt"]}")')
    if not client.configure(config):
        print('  警告: 一部の設定に失敗しました（サーバ起動を確認）')

    palette = palette_from_config(config)
    names = class_name_map(config)
    semantic_ids = [cid for cid in sorted(palette) if cid >= 3]

    sample_groups = []
    overlay_images = []
    overlay_labels = []
    overlay_budget = max(0, int(args.overlay_frames)) * len(view_yaws)
    timeline_entries = []
    video_frames = []
    filmstrip = []
    first_bag_ns = frames[0][2]
    for frame_index, (bgr, _stamp, bag_ns) in enumerate(frames):
        frame_views = []
        frame_samples = []
        for view_index, (map_x, map_y) in enumerate(remaps):
            view_image = cv2.remap(bgr, map_x, map_y, cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_CONSTANT)
            class_map, score_map = client.infer(view_image, out_size, args.timeout)
            if class_map is None:
                continue
            overlay = overlay_view(view_image, class_map)
            frame_views.append(overlay)
            if len(overlay_images) < overlay_budget:
                overlay_images.append(overlay)
                overlay_labels.append(f'f{frame_index + 1} view{int(view_yaws[view_index])}')
            u_map, v_map, valid = bev_maps[view_index]
            rows, cols = np.nonzero(valid)
            if rows.size == 0:
                continue
            uu = np.clip(np.round(u_map[rows, cols]).astype(np.int32), 0, out_size[0] - 1)
            vv = np.clip(np.round(v_map[rows, cols]).astype(np.int32), 0, out_size[1] - 1)
            classes = class_map[vv, uu].astype(np.int32)
            scores = score_map[vv, uu].astype(np.float32) / 255.0
            detected = classes >= 3
            if not np.any(detected):
                continue
            group = (
                rows[detected].astype(np.int64),
                cols[detected].astype(np.int64),
                classes[detected].astype(np.int64),
                scores[detected].astype(np.float32),
            )
            sample_groups.append(group)
            frame_samples.append(group)

        time_s = (bag_ns - first_bag_ns) / 1e9
        if frame_views:
            row = make_row(frame_views, height=args.video_height)
            draw_label(row, f'{time_s:6.1f}s   frame {frame_index + 1}/{len(frames)}')
            video_frames.append(row)
            filmstrip.append({
                'time_s': time_s,
                'jpg': encode_jpeg_base64(row, 60),
                'label': f'{time_s:.1f}s',
            })
        entry = frame_stats(frame_samples, ground_union, ground_cells, bev_size,
                            config['score_min'], semantic_ids)
        entry['time_s'] = time_s
        entry['frame_index'] = frame_index
        entry['has_views'] = bool(frame_views)
        timeline_entries.append(entry)
        print(f'  推論 {frame_index + 1}/{len(frames)} 完了'
              f'  semantic={entry["semantic_pct"]:.1f}%')

    if not sample_groups:
        print('警告: SAM3の検出が1件もありません（prompt/classes/thresholdを確認）')
        all_rows = np.zeros(0, dtype=np.int64)
        all_cols = np.zeros(0, dtype=np.int64)
        all_cls = np.zeros(0, dtype=np.int64)
        all_score = np.zeros(0, dtype=np.float32)
    else:
        all_rows = np.concatenate([group[0] for group in sample_groups])
        all_cols = np.concatenate([group[1] for group in sample_groups])
        all_cls = np.concatenate([group[2] for group in sample_groups])
        all_score = np.concatenate([group[3] for group in sample_groups])

    thresholds = sorted(set(
        [config['score_min']] + parse_thresholds(args.thresholds)))

    sweep = []
    label_images = []
    label_captions = []
    for threshold in thresholds:
        label, per_class = aggregate(
            all_rows, all_cols, all_cls, all_score, bev_size, threshold, semantic_ids)
        semantic_cells = int(np.count_nonzero(label >= 3))
        unknown_cells = int(np.count_nonzero((label == 0) & ground_union))
        sweep.append({
            'threshold': threshold,
            'semantic_pct': 100.0 * semantic_cells / ground_cells,
            'unknown_pct': 100.0 * unknown_cells / ground_cells,
            'per_class': per_class,
            'semantic_cells': semantic_cells,
        })
        label_images.append(colorize_label(label, palette))
        label_captions.append(f'score_min={threshold:.2f}  unknown={sweep[-1]["unknown_pct"]:.0f}%')

    stats = score_stats(all_cls, all_score, config['score_min'], semantic_ids)
    main_threshold = min(thresholds, key=lambda value: abs(value - config['score_min']))

    bev_montage = make_montage(label_images, cols=min(3, len(label_images)),
                               cell=320, labels=label_captions)
    overlay_montage = make_montage(overlay_images, cols=min(2, len(view_yaws) or 1),
                                   cell=380, labels=overlay_labels)

    timeline_svg, timeline_legend = build_timeline_svg(timeline_entries, semantic_ids, names)
    video_b64 = ''
    video_note = ''
    if args.video and len(video_frames) >= 2:
        print(f'  タイムライン動画を生成中（{len(video_frames)}フレーム, {args.video_fps}fps）...')
        video_b64 = build_video_base64(video_frames, args.video_fps)
        if not video_b64:
            video_note = 'ffmpegによる動画生成に失敗しました（映像素子は下のフィルムストリップで確認できます）。'
    elif not args.video:
        video_note = '--no-video 指定のため動画は生成していません。'

    output_dir = Path(args.output).expanduser().resolve() if args.output else (
        Path.home() / 'sirius_jazzy_ws' / 'sam3_preview_reports'
        / f'{time.strftime("%Y%m%d_%H%M%S")}_{bag.name}')
    output_dir.mkdir(parents=True, exist_ok=True)

    report = {
        'bag': str(bag),
        'sam3_config': str(Path(args.sam3_config).expanduser()),
        'calibration': str(Path(args.calibration).expanduser()),
        'server': args.server,
        'frames_sampled': len(frames),
        'frames_total': total,
        'prompt': config['prompt'],
        'score_min': config['score_min'],
        'threshold': config['threshold'],
        'class_thresholds': config['class_thresholds'],
        'classes': {name: spec for name, spec in (config.get('classes') or {}).items()},
        'ground_cells': ground_cells,
        'main_threshold': main_threshold,
        'sweep': sweep,
        'stats': stats,
        'names': names,
        'semantic_ids': semantic_ids,
        'bev_montage': encode_png_base64(bev_montage) if bev_montage is not None else '',
        'overlay_montage': encode_png_base64(overlay_montage) if overlay_montage is not None else '',
        'timeline_svg': timeline_svg,
        'timeline_legend': timeline_legend,
        'timeline': timeline_entries,
        'filmstrip': filmstrip,
        'video_b64': video_b64,
        'video_note': video_note,
        'video_fps': args.video_fps,
    }
    html_path = output_dir / 'report.html'
    html_path.write_text(build_html(report), encoding='utf-8')
    print(f'\nレポートを出力しました: {html_path}')
    if args.open:
        webbrowser.open(html_path.as_uri())
    return 0


def parse_thresholds(raw: str) -> list[float]:
    values = []
    for token in str(raw).split(','):
        token = token.strip()
        if not token:
            continue
        try:
            values.append(float(token))
        except ValueError:
            pass
    return values


def aggregate(rows, cols, classes, scores, bev_size, score_min, semantic_ids):
    votes = np.zeros((bev_size, bev_size, MAX_CLASS_ID + 1), dtype=np.float32)
    keep = scores >= score_min
    if np.any(keep):
        np.add.at(votes, (rows[keep], cols[keep], classes[keep]), scores[keep])
    best = np.argmax(votes, axis=2).astype(np.uint8)
    best_score = np.max(votes, axis=2)
    label = np.where(best_score > 0, best, 0).astype(np.uint8)
    per_class = {int(cid): int(np.count_nonzero(label == cid)) for cid in semantic_ids}
    return label, per_class


def score_stats(classes, scores, score_min, semantic_ids):
    stats = {}
    keep = scores >= score_min
    kept_classes = classes[keep]
    kept_scores = scores[keep]
    for class_id in semantic_ids:
        values = kept_scores[kept_classes == class_id]
        if values.size == 0:
            stats[int(class_id)] = {'n': 0}
            continue
        if values.size > 200000:
            generator = np.random.default_rng(0)
            values = generator.choice(values, 200000, replace=False)
        stats[int(class_id)] = {
            'n': int(values.size),
            'mean': float(np.mean(values)),
            'median': float(np.median(values)),
            'p25': float(np.percentile(values, 25)),
            'p75': float(np.percentile(values, 75)),
            'p90': float(np.percentile(values, 90)),
        }
    return stats


def build_html(report: dict) -> str:
    names = report['names']
    semantic_ids = report['semantic_ids']

    def color_swatch(value):
        return (f'<span style="display:inline-block;width:14px;height:14px;'
                f'border:1px solid #999;background:rgb({value[0]},{value[1]},{value[2]})"></span>')

    classes_rows = []
    for name, spec in report['classes'].items():
        if not isinstance(spec, dict):
            continue
        class_id = int(spec.get('id', -1))
        color = spec.get('color', [255, 255, 255])
        class_threshold = report['class_thresholds'].get(name, '')
        classes_rows.append(
            f'<tr><td>{name}</td><td>{class_id}</td>'
            f'<td>{color_swatch(color)} {color}</td>'
            f'<td>{class_threshold}</td></tr>')

    sweep_header = ''.join(f'<th>{names.get(cid, cid)}</th>' for cid in semantic_ids)
    sweep_rows = []
    for entry in report['sweep']:
        cells = ''.join(
            f'<td>{entry["per_class"].get(cid, 0):,}</td>' for cid in semantic_ids)
        marker = ' class="main"' if abs(entry['threshold'] - report['main_threshold']) < 1e-9 else ''
        sweep_rows.append(
            f'<tr{marker}><td>{entry["threshold"]:.2f}</td>'
            f'<td>{entry["semantic_pct"]:.1f}%</td>'
            f'<td>{entry["unknown_pct"]:.1f}%</td>{cells}</tr>')

    stats_rows = []
    for class_id in semantic_ids:
        entry = report['stats'].get(class_id, {'n': 0})
        if entry.get('n', 0) == 0:
            stats_rows.append(
                f'<tr><td>{names.get(class_id, class_id)}</td><td>0</td>'
                f'<td colspan="5">検出なし</td></tr>')
            continue
        stats_rows.append(
            f'<tr><td>{names.get(class_id, class_id)}</td><td>{entry["n"]:,}</td>'
            f'<td>{entry["mean"]:.3f}</td><td>{entry["median"]:.3f}</td>'
            f'<td>{entry["p25"]:.3f}</td><td>{entry["p75"]:.3f}</td>'
            f'<td>{entry["p90"]:.3f}</td></tr>')

    timeline_html = ''
    if report.get('timeline_svg'):
        filmstrip_items = ''.join(
            f'<figure><img src="data:image/jpeg;base64,{item["jpg"]}">'
            f'<figcaption>{item["label"]}</figcaption></figure>'
            for item in report.get('filmstrip', []) if item.get('jpg'))
        if report.get('video_b64'):
            video_block = (
                f'<video controls preload="metadata" '
                f'src="data:video/mp4;base64,{report["video_b64"]}"></video>'
                f'<p class="note">再生速度 {report.get("video_fps")}fps'
                f'（bagから等間隔に抽出したフレームを時系列で表示）</p>')
        else:
            video_block = f'<p class="note">{report.get("video_note", "")}</p>'
        timeline_html = f"""
<h2>タイムライン（セマンティック coverage 推移）</h2>
{report['timeline_svg']}
<h2>映像タイムライン（透視オーバーレイ）</h2>
{video_block}
<div class="filmstrip">{filmstrip_items}</div>
"""

    images = []
    if report['bev_montage']:
        images.append('<h2>BEVラベル（閾値スイープ）</h2>'
                      f'<img src="data:image/png;base64,{report["bev_montage"]}">')
    if report['overlay_montage']:
        images.append('<h2>透視オーバーレイ（SAM3検出）</h2>'
                      f'<img src="data:image/png;base64,{report["overlay_montage"]}">')

    return f"""<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8">
<title>THETA SAM3 事前確認</title>
<style>
 body {{ font-family: sans-serif; margin: 24px; color: #222; }}
 h1 {{ font-size: 20px; }} h2 {{ font-size: 16px; margin-top: 28px; }}
 table {{ border-collapse: collapse; margin: 8px 0; }}
 th, td {{ border: 1px solid #ccc; padding: 4px 10px; font-size: 13px; text-align: right; }}
 th {{ background: #f2f2f2; }} td:first-child, th:first-child {{ text-align: left; }}
 tr.main {{ background: #fff7d6; font-weight: bold; }}
 img {{ max-width: 100%; border: 1px solid #ccc; }}
 .meta td {{ text-align: left; }}
 code {{ background: #f4f4f4; padding: 1px 4px; }}
 video {{ max-width: 100%; border: 1px solid #ccc; background: #000; }}
 .note {{ color: #666; font-size: 12px; }}
 .filmstrip {{ display: flex; gap: 8px; overflow-x: auto; padding: 6px 0;
   border: 1px solid #eee; }}
 .filmstrip figure {{ margin: 0; flex: 0 0 auto; text-align: center; }}
 .filmstrip img {{ height: 120px; display: block; }}
 .filmstrip figcaption {{ font-size: 11px; color: #555; }}
</style></head><body>
<h1>THETA SAM3 事前確認レポート</h1>
<table class="meta">
 <tr><th>bag</th><td><code>{report['bag']}</code></td></tr>
 <tr><th>SAM3設定</th><td><code>{report['sam3_config']}</code></td></tr>
 <tr><th>校正</th><td><code>{report['calibration']}</code></td></tr>
 <tr><th>サーバ</th><td>{report['server']}</td></tr>
 <tr><th>フレーム</th><td>{report['frames_sampled']} / {report['frames_total']}</td></tr>
 <tr><th>prompt</th><td><code>{report['prompt']}</code></td></tr>
 <tr><th>threshold / score_min</th><td>{report['threshold']} / {report['score_min']}</td></tr>
 <tr><th>BEV有効セル</th><td>{report['ground_cells']:,}</td></tr>
</table>
<h2>セマンティック対象クラス</h2>
<table><tr><th>class</th><th>id</th><th>color</th><th>class threshold</th></tr>
{''.join(classes_rows)}</table>
<h2>閾値スイープ（score_min別）</h2>
<table><tr><th>score_min</th><th>semantic</th><th>unknown</th>{sweep_header}</tr>
{''.join(sweep_rows)}</table>
<h2>クラス別 尤度統計（score_min={report['score_min']}）</h2>
<table><tr><th>class</th><th>n</th><th>mean</th><th>median</th><th>p25</th><th>p75</th><th>p90</th></tr>
{''.join(stats_rows)}</table>
{timeline_html}
{''.join(images)}
</body></html>"""


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='rosbag2ディレクトリ（metadata.yaml）')
    parser.add_argument('--calibration', default=str(
        Path.home() / 'sirius_jazzy_ws' / 'src' / 'sirius' / 'sirius_navigation'
        / 'config' / 'theta_calibration.yaml'))
    parser.add_argument('--sam3-config', default=str(DEFAULT_SAM3_CONFIG))
    parser.add_argument('--server', default='http://localhost:8080')
    parser.add_argument('--frames', type=int, default=20, help='等間隔に抽出する枚数')
    parser.add_argument('--thresholds', default='0.1,0.2,0.3,0.4,0.5,0.6,0.7',
                        help='score_minスイープ値（カンマ区切り）')
    parser.add_argument('--view-yaws', default='0,180')
    parser.add_argument('--hfov', type=float, default=120.0)
    parser.add_argument('--out-width', type=int, default=640)
    parser.add_argument('--out-height', type=int, default=480)
    parser.add_argument('--min-radius', type=float, default=1.2)
    parser.add_argument('--max-radius', type=float, default=4.8)
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--overlay-frames', type=int, default=3,
                        help='オーバーレイ表示するフレーム数')
    parser.add_argument('--video-fps', type=float, default=2.0,
                        help='タイムライン動画の再生fps')
    parser.add_argument('--video-height', type=int, default=240,
                        help='タイムライン動画フレームの高さ[px]')
    parser.add_argument('--no-video', dest='video', action='store_false',
                        help='タイムライン動画を生成しない（フィルムストリップのみ）')
    parser.add_argument('--output', default='', help='出力ディレクトリ（既定 自動）')
    parser.add_argument('--no-open', dest='open', action='store_false')
    parser.set_defaults(open=True, video=True)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        return run(args)
    except KeyboardInterrupt:
        print('\n中止しました。')
        return 130
    except Exception as error:
        print(f'エラー: {error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
