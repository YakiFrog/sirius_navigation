"""Calibrated dual-fisheye <-> equirectangular ground correspondence.

The image center faces camera +X, increasing columns face right (-Y), the
top is +Z. Longitude wraps, latitude does not. Coordinates denote pixel centers.
"""
import numpy as np
from sirius_navigation.theta_perspective import _fisheye_uv, mount_matrix


def equirectangular_rays(width, height):
    lon = ((np.arange(width)+.5)/width-.5)*2*np.pi
    lat = (.5-(np.arange(height)+.5)/height)*np.pi
    lon,lat = np.meshgrid(lon,lat)
    return np.stack([np.cos(lat)*np.cos(lon),-np.cos(lat)*np.sin(lon),np.sin(lat)],axis=-1)


def rays_to_equirectangular(rays, width, height):
    lon = np.arctan2(-rays[...,1],rays[...,0])
    lat = np.arctan2(rays[...,2],np.hypot(rays[...,0],rays[...,1]))
    return np.mod((lon/(2*np.pi)+.5)*width-.5,width),(.5-lat/np.pi)*height-.5


def build_equirectangular_remap(calibration, image_size, out_size=(2048,1024)):
    width,height=out_size
    if width != 2*height:
        raise ValueError('Full equirectangular image must have a 2:1 aspect ratio')
    mount=mount_matrix(calibration)
    rays=equirectangular_rays(width,height)@mount.T
    uv,valid=_fisheye_uv(rays,calibration,image_size,mount)
    return tuple(np.where(valid,uv[...,i],-1).astype(np.float32) for i in (0,1))


def build_bev_to_equirectangular(calibration, bev_size, out_size=(2048,1024),
                                min_radius=1.2, max_radius=4.8):
    extent=float(calibration['bev_extent_m'])
    cols,rows=np.meshgrid(np.arange(bev_size),np.arange(bev_size))
    x=(.5-(rows+.5)/bev_size)*extent
    y=(.5-(cols+.5)/bev_size)*extent
    ground=np.stack([x,y,np.zeros_like(x)],axis=-1)
    rays=(ground-np.asarray(calibration['camera_position']))@mount_matrix(calibration)
    u,v=rays_to_equirectangular(rays,*out_size)
    radius=np.hypot(x,y)
    valid=(radius>=min_radius)&(radius<=max_radius)&(v>=0)&(v<out_size[1])
    return u.astype(np.float32),v.astype(np.float32),valid


def ground_tile(panorama, u, v, valid, yaw_degrees):
    """180 x 90 degree lower-hemisphere tile with periodic horizontal indexing."""
    height,width=panorama.shape[:2]
    start=int(round((.5-yaw_degrees/360)*width-width/4))
    tile=panorama[height//2:,(np.arange(width//2)+start)%width]
    tu=np.mod(u-start,width);tv=v-height//2
    inside=valid&(tu>=0)&(tu<width//2)&(tv>=0)&(tv<height//2)
    return tile,tu,tv,inside
