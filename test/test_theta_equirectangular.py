import numpy as np
import pytest
from sirius_navigation.theta_equirectangular import (
    equirectangular_rays,rays_to_equirectangular,ground_tile,
    build_equirectangular_remap,build_bev_to_equirectangular,
)
from sirius_navigation.sam3_experiment_client import mono_transport
from sirius_navigation.theta_sam3_evaluation import reference_rays,sample_view_maps


def test_cardinal_directions_and_floor_depression():
    rays=np.array([[1,0,0],[0,-1,0],[0,1,0],[1,0,-1]],float)
    u,v=rays_to_equirectangular(rays,1024,512)
    np.testing.assert_allclose(u,[511.5,767.5,255.5,511.5])
    np.testing.assert_allclose(v,[255.5,255.5,255.5,383.5])


def test_full_sphere_pixel_center_round_trip():
    rays=equirectangular_rays(128,64)
    u,v=rays_to_equirectangular(rays,128,64)
    cols,rows=np.meshgrid(np.arange(128),np.arange(64))
    np.testing.assert_allclose(u,cols,atol=1e-10)
    np.testing.assert_allclose(v,rows,atol=1e-10)


def test_wide_mono_reaches_server_as_complete_left_image():
    original=np.arange(8*16*3,dtype=np.uint8).reshape(8,16,3)
    sent,wrapped=mono_transport(original)
    assert wrapped and sent.shape==(8,32,3)
    np.testing.assert_array_equal(sent[:,:16],original)
    np.testing.assert_array_equal(sent[:,16:],original)
    normal=original[:,:10]
    assert mono_transport(normal)[0] is normal


def test_rear_tile_wraps_seam_and_keeps_lower_hemisphere():
    pano=np.zeros((8,16,3),np.uint8);pano[:,:,0]=np.arange(16)
    tile,u,v,valid=ground_tile(pano,np.array([15.,0.,7.]),np.full(3,6.),
                              np.ones(3,bool),180.)
    assert tile[0,:,0].tolist()==[12,13,14,15,0,1,2,3]
    assert valid.tolist()==[True,True,False]
    np.testing.assert_allclose(u[:2],[3,4])


def test_common_camera_view_matches_original_pixel_centers():
    for yaw in (0,90,180,270):
        u,v,valid,_=sample_view_maps(reference_rays(yaw),'p4',yaw)
        cols,rows=np.meshgrid(np.arange(640),np.arange(480))
        assert valid.all()
        np.testing.assert_allclose(u,cols,atol=1e-8)
        np.testing.assert_allclose(v,rows,atol=1e-8)


def test_experiment_client_does_not_read_stale_results_after_timeout():
    from types import SimpleNamespace
    from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient
    client=Sam3ExperimentClient(timeout=0)
    client.get=lambda path:{'network_frame_version':4}
    client.http.post=lambda *a,**k:SimpleNamespace(raise_for_status=lambda:None)
    client.http.get=lambda *a,**k:pytest.fail('stale PNG was requested')
    with pytest.raises(TimeoutError):client.infer(np.zeros((4,4,3),np.uint8))
    client.http.close()


def test_experiment_client_rejects_mismatched_frame_png():
    from types import SimpleNamespace
    from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient
    client=Sam3ExperimentClient()
    states=iter([{'network_frame_version':4},{'network_frame_version':5,'sam3_inference_frame_version':5}])
    client.get=lambda path:next(states)
    client.http.post=lambda *a,**k:SimpleNamespace(raise_for_status=lambda:None)
    client.http.get=lambda *a,**k:SimpleNamespace(raise_for_status=lambda:None,headers={'X-Frame-Version':'3'})
    with pytest.raises(RuntimeError,match='stale'):client.infer(np.zeros((4,4,3),np.uint8))
    client.http.close()
