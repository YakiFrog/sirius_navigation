"""Reproject saved image masks to common camera views for paired visual review."""
import numpy as np
from sirius_navigation.theta_perspective import rpy_to_matrix
from sirius_navigation.theta_equirectangular import rays_to_equirectangular


def reference_rays(yaw, out_size=(640,480), hfov=120.):
    w,h=out_size;u,v=np.meshgrid(np.arange(w),np.arange(h))
    f=(w/2)/np.tan(np.deg2rad(hfov/2))
    rays=np.stack([np.ones_like(u),-(u+.5-w/2)/f,-(v+.5-h/2)/f],axis=-1)
    return rays@rpy_to_matrix(0,0,yaw,pitch_sign=1).T


def sample_view_maps(rays,kind,yaw=0,perspective_size=(640,480),panorama_size=(2048,1024)):
    if kind in ('p2','p4'):
        points=rays@rpy_to_matrix(0,0,yaw,pitch_sign=1)
        w,h=perspective_size;f=(w/2)/np.tan(np.deg2rad(60))
        with np.errstate(divide='ignore',invalid='ignore'):
            u=f*(-points[...,1]/points[...,0])+w/2-.5
            v=f*(-points[...,2]/points[...,0])+h/2-.5
        valid=(points[...,0]>1e-8)&(u>=-.001)&(u<w)&(v>=-.001)&(v<h)
        return u,v,valid,False
    w,h=panorama_size;u,v=rays_to_equirectangular(rays,w,h)
    if kind=='erp':return u,v,(v>=0)&(v<h),True
    start=int(round((.5-yaw/360)*w-w/4));u=np.mod(u-start,w);v=v-h//2
    return u,v,(u<w//2)&(v>=0)&(v<h//2),False
