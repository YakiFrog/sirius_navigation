"""Inverse ground-plane mapping. Input/output pixel origin is top-left."""
import numpy as np


def build_maps(calibration, width, height):
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
    front = rays[...,0] >= 0
    maps = np.full((*u.shape, 2), -1., dtype=np.float32)
    scale = np.array([width, height]) / np.asarray(c['image_size'])
    for name, sign in [('front', 1), ('back', -1)]:
        lens = c[name]
        theta = np.arccos(np.clip(sign*rays[...,0], -1, 1))
        k = lens['distortion']
        radius = theta * (1 + sum(float(k[i])*theta**(2*(i+1)) for i in range(4)))
        right, down = -sign*rays[...,1], -rays[...,2]
        transverse = np.maximum(np.hypot(right, down), 1e-12)
        xy = np.stack((right, down), axis=-1) / transverse[...,None]
        uv = (np.asarray(lens['center']) + xy * radius[...,None] * np.asarray(lens['focal'])) * scale
        valid = (front if sign == 1 else ~front) & (theta <= np.deg2rad(lens['max_theta_degrees']))
        maps[valid] = uv[valid]
    return maps[...,0], maps[...,1]
