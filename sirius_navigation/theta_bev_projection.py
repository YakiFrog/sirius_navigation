"""Inverse ground-plane mapping. Input/output pixel origin is top-left."""
import math
import numpy as np


def _ray_grid(calibration):
    c = calibration
    size, extent = int(c['bev_size']), float(c['bev_extent_m'])
    if size <= 0 or extent <= 0 or float(c['camera_position'][2]) <= 0:
        raise ValueError('BEV size, extent and camera height must be positive')
    u, v = np.meshgrid((np.arange(size) + .5) / size, (np.arange(size) + .5) / size)
    ground = np.stack(((.5-v)*extent, (.5-u)*extent, np.zeros_like(u)), axis=-1)
    rays = ground - np.asarray(c['camera_position'])
    roll, pitch, yaw = np.deg2rad(c['rpy_degrees'])
    pitch = -pitch  # UI convention: positive pitches the front lens upward.
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rotation = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @ np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    rays = rays @ rotation  # inverse camera-to-robot rotation for row vectors
    rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
    return u, v, rays


def _lens_uv(lens, sign, rays, scale):
    theta = np.arccos(np.clip(sign*rays[...,0], -1, 1))
    k = lens['distortion']
    radius = theta * (1 + sum(float(k[i])*theta**(2*(i+1)) for i in range(4)))
    right, down = -sign*rays[...,1], -rays[...,2]
    # 実機THETAのレンズは実装上のマウントで光軸周りに回転している。
    # サンプリング座標を image_roll_degrees だけ回すことで、ピクセルを触らず向きを正す。
    roll = math.radians(float(lens.get('image_roll_degrees', 0.0)))
    if roll:
        cr, sr = math.cos(roll), math.sin(roll)
        right, down = right*cr - down*sr, right*sr + down*cr
    transverse = np.maximum(np.hypot(right, down), 1e-12)
    xy = np.stack((right, down), axis=-1) / transverse[...,None]
    uv = (np.asarray(lens['center']) + xy * radius[...,None] * np.asarray(lens['focal'])) * scale
    valid = theta <= np.deg2rad(lens['max_theta_degrees'])
    return uv, valid


def build_maps(calibration, width, height):
    u, v, rays = _ray_grid(calibration)
    scale = np.array([width, height]) / np.asarray(calibration['image_size'])
    front = rays[...,0] >= 0
    maps = np.full((*u.shape, 2), -1., dtype=np.float32)
    for name, sign in [('front', 1), ('back', -1)]:
        uv, ok = _lens_uv(calibration[name], sign, rays, scale)
        valid = (front if sign == 1 else ~front) & ok
        maps[valid] = uv[valid]
    return maps[...,0], maps[...,1]


def build_blend_maps(calibration, width, height, blend_half_deg=6.0):
    """前後レンズをクロスフェード合成するBEV逆投影マップ。

    ロボット真横（前後軸に直交する線）は両レンズとも画角外周(θ≈90°)になり、ここで
    中心/歪みの差が段差（つなぎ目）として出る。両レンズの有効域が重なる帯で前レンズ
    重み alpha を滑らかに変化させ、remap結果を alpha で合成して段差を消す。

    Returns (map_fx, map_fy, map_bx, map_by, alpha):
      map_* : float32 (H,W) 逆投影先座標（-1は無効）
      alpha : float32 (H,W) 前レンズの重み[0,1]（後レンズは 1-alpha）
    """
    u, v, rays = _ray_grid(calibration)
    scale = np.array([width, height]) / np.asarray(calibration['image_size'])
    uv_f, valid_f = _lens_uv(calibration['front'], 1, rays, scale)
    uv_b, valid_b = _lens_uv(calibration['back'], -1, rays, scale)

    # ray.x（=cos θ_front）を基準に前後をクロスフェード。帯の幅はブレンド角で指定。
    s = math.sin(math.radians(blend_half_deg))
    alpha = np.clip((rays[...,0] + s) / (2.0*s), 0.0, 1.0)
    alpha = np.where(valid_f, alpha, 0.0)   # 前レンズ無効なら後ろを使う
    alpha = np.where(valid_b, alpha, 1.0)   # 後レンズ無効なら前を使う
    alpha = np.where(~valid_f & ~valid_b, 0.0, alpha)

    mx_f = np.where(valid_f, uv_f[...,0], -1).astype(np.float32)
    my_f = np.where(valid_f, uv_f[...,1], -1).astype(np.float32)
    mx_b = np.where(valid_b, uv_b[...,0], -1).astype(np.float32)
    my_b = np.where(valid_b, uv_b[...,1], -1).astype(np.float32)
    return mx_f, my_f, mx_b, my_b, alpha.astype(np.float32)
