#!/usr/bin/env python3
"""THETA dual-fisheye -> 透視投影ビュー生成と、BEV画素->透視ビュー対応の事前計算。

座標系はキャリブレーションと同じ ROS 規約（X前, Y左, Z上）。
- 魚眼モデルは theta_bev_projection.py と同じ等距離モデル（front/backレンズ）
- マウント姿勢 R_mount = Rz(yaw) @ Ry(-pitch) @ Rx(roll) （build_maps と同一）
- ビュー姿勢 R_view = R_mount @ Rz(yaw_view) （yaw_view: 0=前, +90=左, 180=後, -90=右）
"""
import numpy as np


def rpy_to_matrix(roll_deg, pitch_deg, yaw_deg, pitch_sign=-1.0):
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_sign * pitch_deg)
    yaw = np.deg2rad(yaw_deg)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def mount_matrix(calibration):
    roll, pitch, yaw = calibration['rpy_degrees']
    return rpy_to_matrix(roll, pitch, yaw)


def _fisheye_uv(rays_robot, calibration, image_size, mount):
    """robotフレームのレイ(...,3) -> dual-fisheye画素(...,2) と有効マスク。"""
    img_h, img_w = image_size
    scale = np.array([img_w, img_h], dtype=np.float64) / np.asarray(calibration['image_size'], dtype=np.float64)
    rays_cam = rays_robot @ mount  # v_cam = R_mount^T @ v_robot (row vectors)
    front = rays_cam[..., 0] >= 0
    sign = np.where(front, 1.0, -1.0)
    theta = np.arccos(np.clip(sign * rays_cam[..., 0], -1.0, 1.0))
    # build_maps と同じ規約（後レンズは right の符号が反転、down は共通）
    right = -sign * rays_cam[..., 1]
    down = -rays_cam[..., 2]
    transverse = np.maximum(np.hypot(right, down), 1e-12)
    direction = np.stack([right / transverse, down / transverse], axis=-1)
    uv = np.zeros_like(direction)
    valid = np.zeros(theta.shape, dtype=bool)
    for name, mask in (('front', front), ('back', ~front)):
        if not np.any(mask):
            continue
        lens = calibration[name]
        k = lens['distortion']
        radius = theta * (1.0 + sum(float(k[i]) * theta ** (2 * (i + 1)) for i in range(4)))
        uv_lens = (np.asarray(lens['center'], dtype=np.float64) + direction * radius[..., None] * np.asarray(lens['focal'], dtype=np.float64)) * scale
        lens_valid = mask & (theta <= np.deg2rad(float(lens['max_theta_degrees'])))
        uv[lens_valid] = uv_lens[lens_valid]
        valid |= lens_valid
    in_bounds = (uv[..., 0] >= 0) & (uv[..., 0] <= img_w - 1) & (uv[..., 1] >= 0) & (uv[..., 1] <= img_h - 1)
    return uv, valid & in_bounds


def build_perspective_remap(calibration, yaw_view_deg, image_size, out_size, hfov_deg=120.0):
    """dual-fisheye から透視ビューを切り出す cv2.remap 用マップを返す。

    Returns: (map_x, map_y) float32, shape=(out_h, out_w)
    """
    out_w, out_h = out_size
    mount = mount_matrix(calibration)
    view = mount @ rpy_to_matrix(0.0, 0.0, yaw_view_deg, pitch_sign=1.0)
    f = (out_w / 2.0) / np.tan(np.deg2rad(hfov_deg) / 2.0)
    u = np.arange(out_w, dtype=np.float64)
    v = np.arange(out_h, dtype=np.float64)
    uu, vv = np.meshgrid(u, v)
    dx = (uu + 0.5 - out_w / 2.0) / f
    dy = (vv + 0.5 - out_h / 2.0) / f
    rays_view = np.stack([np.ones_like(dx), -dx, -dy], axis=-1)
    rays_view /= np.linalg.norm(rays_view, axis=-1, keepdims=True)
    rays_robot = rays_view @ view.T
    uv, valid = _fisheye_uv(rays_robot, calibration, image_size, mount)
    map_x = np.where(valid, uv[..., 0], -1.0).astype(np.float32)
    map_y = np.where(valid, uv[..., 1], -1.0).astype(np.float32)
    return map_x, map_y


def build_bev_to_view_maps(calibration, yaw_view_deg, bev_size, out_size, hfov_deg=120.0, min_radius=0.0, max_radius=0.0):
    """各BEV画素(地面点)を透視ビューへ投影した画素座標と有効マスクを返す。

    Returns: (u_map, v_map, valid)  shape=(bev_h, bev_w)
    """
    out_w, out_h = out_size
    extent = float(calibration['bev_extent_m'])
    camera_position = np.asarray(calibration['camera_position'], dtype=np.float64)
    mount = mount_matrix(calibration)
    view = mount @ rpy_to_matrix(0.0, 0.0, yaw_view_deg, pitch_sign=1.0)
    view_inv = view.T
    f = (out_w / 2.0) / np.tan(np.deg2rad(hfov_deg) / 2.0)
    u = np.arange(bev_size, dtype=np.float64)
    v = np.arange(bev_size, dtype=np.float64)
    uu, vv = np.meshgrid(u, v)
    x = (0.5 - (vv + 0.5) / bev_size) * extent
    y = (0.5 - (uu + 0.5) / bev_size) * extent
    z = np.zeros_like(x)
    radius = np.hypot(x, y)
    ground_valid = np.ones_like(radius, dtype=bool)
    if min_radius > 0.0:
        ground_valid &= radius >= min_radius
    if max_radius > 0.0:
        ground_valid &= radius <= max_radius
    rays_robot = np.stack([x - camera_position[0], y - camera_position[1], z - camera_position[2]], axis=-1)
    rays_view = rays_robot @ view  # v_view = R_view^T @ v_robot (row vectors)
    xv = rays_view[..., 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        u_view = f * (-rays_view[..., 1] / xv) + out_w / 2.0
        v_view = f * (-rays_view[..., 2] / xv) + out_h / 2.0
    valid = ground_valid & (xv > 1e-6) & (u_view >= 0) & (u_view < out_w) & (v_view >= 0) & (v_view < out_h)
    return u_view.astype(np.float32), v_view.astype(np.float32), valid


DEFAULT_VIEW_YAWS = (0.0, 90.0, 180.0, -90.0)
