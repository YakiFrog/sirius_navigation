"""Shared geometry-based THETA coloring for live clouds and offline maps."""
from pathlib import Path

import cv2
import numpy as np
import yaml

from .theta_perspective import project_camera_frame_points, rpy_to_matrix


def load_color_policy(path):
    policy = yaml.safe_load(Path(path).read_text())
    policy.setdefault('color_distance_power', 2.)
    policy.setdefault('fusion_weighted_median', False)
    policy.setdefault('color_rotation_scale_deg_s', 0.)
    rotation_scale = policy['color_rotation_scale_deg_s']
    if (isinstance(rotation_scale, bool) or not isinstance(rotation_scale, (int, float))
            or not np.isfinite(rotation_scale) or rotation_scale < 0):
        raise ValueError('Invalid color_rotation_scale_deg_s (expected finite nonnegative number)')
    if (isinstance(policy['color_distance_power'], bool)
            or not isinstance(policy['color_distance_power'], (int, float))
            or not np.isfinite(policy['color_distance_power'])
            or not 0 <= policy['color_distance_power'] <= 8):
        raise ValueError('Invalid color_distance_power (expected 0..8)')
    if not isinstance(policy['fusion_weighted_median'], bool):
        raise ValueError('Invalid fusion_weighted_median')
    for name in ('max_image_age_s', 'max_uncompensated_age_s', 'depth_margin_m',
                 'depth_relative_margin', 'coplanar_margin_m', 'offline_image_interval_s', 'max_color_range_m'):
        if not np.isfinite(policy[name]) or policy[name] < 0:
            raise ValueError(f'Invalid color policy: {name}')
    if not np.isfinite(policy['image_time_offset_s']):
        raise ValueError('Invalid image time offset')
    for name, minimum, maximum in [('occlusion_cell_px', 1, 32), ('occlusion_neighbors', 0, 3),
                                    ('edge_margin_px', 0, 10), ('fusion_views', 1, 15)]:
        value = policy[name]
        if not isinstance(value, int) or isinstance(value, bool) or not minimum <= value <= maximum:
            raise ValueError(f'Invalid color policy: {name}')
    camera_in_lidar(policy)  # Validate an override immediately, not mid-processing.
    return policy


def camera_in_lidar(policy):
    """Optional robot calibration. ROS RPY uses right-hand positive pitch."""
    pose = policy.get('camera_in_lidar')
    if pose is None:
        return None
    xyz = np.asarray(pose['xyz_m'], float)
    rpy = np.asarray(pose['rpy_degrees'], float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
        raise ValueError('camera_in_lidar must contain finite xyz_m/rpy_degrees triples')
    result = np.eye(4)
    result[:3, :3] = rpy_to_matrix(*rpy, pitch_sign=1.)
    result[:3, 3] = xyz
    return result


def _project(points, calibration, shape):
    points = np.asarray(points, float)
    finite = np.isfinite(points).all(axis=1)
    safe = np.where(finite[:, None], points, [1., 0., 0.])
    uv, valid = project_camera_frame_points(safe, calibration, shape)
    distance = np.linalg.norm(safe, axis=1)
    return uv, valid & finite & (distance > 1.e-6), distance


def visible_colors(points_cam, image, calibration, policy, occluders_cam=None, normals_cam=None):
    """Do not paint background points through closer foreground surfaces.

    A conservative radial depth buffer works for both fisheye lenses. Sparse
    live scans cannot prove all occlusions; offline uses dense map + raw scans.
    Returns packed RGB, positive confidence for accepted samples, diagnostics.
    """
    height, width = image.shape[:2]
    uv, valid, ranges = _project(points_cam, calibration, image.shape[:2])
    valid &= ranges <= policy['max_color_range_m']
    size = policy['occlusion_cell_px']
    cols, rows = (width+size-1)//size, (height+size-1)//size
    depth = np.full((rows, cols), np.inf, np.float64)
    owners = np.full((rows, cols), -1, int)

    def insert(projected, mask, distance, offset=0):
        xy = np.floor(projected[mask]/size).astype(int)
        np.minimum.at(depth, (xy[:, 1], xy[:, 0]), distance[mask])
        nearest = distance[mask] <= depth[xy[:, 1], xy[:, 0]]+1.e-9
        owners[xy[nearest,1],xy[nearest,0]] = np.flatnonzero(mask)[nearest]+offset

    insert(uv, valid, ranges)
    if occluders_cam is not None and len(occluders_cam):
        other_uv, other_valid, other_range = _project(occluders_cam, calibration, image.shape[:2])
        other_valid &= other_range <= policy['max_color_range_m']
        insert(other_uv, other_valid, other_range, len(points_cam))
    neighbors = policy['occlusion_neighbors']
    if neighbors:
        padded = np.pad(depth, neighbors, constant_values=np.inf)
        padded_ids = np.pad(owners, neighbors, constant_values=-1)
        depth, owners = depth.copy(), owners.copy()
        for dy in range(2*neighbors+1):
            for dx in range(2*neighbors+1):
                candidate = padded[dy:dy+rows,dx:dx+cols]
                better = candidate < depth
                depth[better] = candidate[better]
                owners[better] = padded_ids[dy:dy+rows,dx:dx+cols][better]
    ids = np.flatnonzero(valid)
    xy = np.floor(uv[ids]/size).astype(int)
    visible = ranges[ids] <= depth[xy[:, 1], xy[:, 0]] + policy['depth_margin_m'] + policy['depth_relative_margin']*ranges[ids]
    plane_recovered = 0
    if normals_cam is not None and len(ids):
        # Grazing floor pixels differ in range. Relax only if the nearest trusted
        # occluder's plane contains P. Raw dynamic occluders have no such plane.
        normals = np.asarray(normals_cam)
        if normals.shape != np.asarray(points_cam).shape:
            raise ValueError('Normals and color points must have matching shapes')
        owner = owners[xy[:,1],xy[:,0]]
        trusted = (~visible) & (owner >= 0) & (owner < len(points_cam))
        candidate_ids = np.flatnonzero(trusted)
        foreground = owner[trusted]
        normal = normals[foreground]
        coplanar = (np.linalg.norm(normal,axis=1) > .9) & (np.abs(np.sum(
            (np.asarray(points_cam)[ids[trusted]]-np.asarray(points_cam)[foreground])*normal,axis=1)) <= policy['coplanar_margin_m'])
        visible[candidate_ids[coplanar]] = True
        plane_recovered = int(coplanar.sum())
    occluded = int((~visible).sum())
    ids = ids[visible]
    edge_rejected = 0
    if policy['edge_margin_px'] and len(ids):
        edge = cv2.Canny(image, 60, 140)
        radius = policy['edge_margin_px']
        edge = cv2.dilate(edge, np.ones((2*radius+1,)*2, np.uint8))
        xy = np.rint(uv[ids]).astype(int)
        reliable = edge[xy[:, 1], xy[:, 0]] == 0
        edge_rejected = int((~reliable).sum())
        ids = ids[reliable]
    rgb = np.full(len(points_cam), 0x808080, np.uint32)
    confidence = np.zeros(len(points_cam), np.float32)
    if len(ids):
        u, v = uv[ids].T
        u0, v0 = np.floor(u).astype(int), np.floor(v).astype(int)
        u1, v1 = np.minimum(u0+1, width-1), np.minimum(v0+1, height-1)
        du, dv = (u-u0)[:, None], (v-v0)[:, None]
        color = np.clip(np.round(image[v0,u0]*(1-du)*(1-dv) + image[v0,u1]*du*(1-dv)
                                + image[v1,u0]*(1-du)*dv + image[v1,u1]*du*dv), 0, 255).astype(np.uint32)
        rgb[ids] = (color[:, 2]<<16) | (color[:, 1]<<8) | color[:, 0]
        cosine = np.maximum(abs(np.asarray(points_cam)[ids, 0])/ranges[ids], .1)
        confidence[ids] = cosine**2/np.maximum(ranges[ids], 1.)**policy.get('color_distance_power', 2.)
    return rgb, confidence, {'projected': int(valid.sum()), 'occluded': occluded, 'coplanar_recovered': plane_recovered,
                              'image_edge_rejected': edge_rejected, 'accepted': len(ids)}
