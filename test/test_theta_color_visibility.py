"""Parallax/occlusion must not paint a hidden floor with foreground RGB."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import yaml

from sirius_navigation.theta_color_projection import camera_in_lidar, load_color_policy, visible_colors
from sirius_navigation.theta_colorize_cloud_node import ThetaColorizeCloudNode


CONFIG = Path(__file__).resolve().parents[1]/'config/theta_colorization.yaml'


def camera():
    lens = dict(focal=[16.,16.], distortion=[0.]*4, max_theta_degrees=95.)
    return dict(image_size=[128,64], front=dict(lens,center=[32.,32.]), back=dict(lens,center=[96.,32.]))


def test_foreground_blocks_background_including_raw_dynamic_occluders():
    policy = load_color_policy(CONFIG)
    policy['edge_margin_px'] = 0
    image = np.full((64,128,3), [0,0,255], np.uint8)
    points = np.array([[2.,0,-1], [4.,0,-2], [-2.,0,0], [np.nan,0,0]])
    rgb, confidence, stats = visible_colors(points, image, camera(), policy)
    assert rgb[0] == 0xff0000 and confidence[0] > 0
    assert rgb[1] == 0x808080 and confidence[1] == 0
    assert confidence[2] > 0 and confidence[3] == 0
    assert stats['occluded'] >= 1
    # Removing the person from the map must NOT remove them from the image depth test.
    rgb, confidence, _ = visible_colors(points[1:2], image, camera(), policy, points[:1])
    assert rgb[0] == 0x808080 and confidence[0] == 0


def test_configurable_distance_confidence_preserves_projected_colors():
    policy = load_color_policy(CONFIG)
    policy.update(edge_margin_px=0,occlusion_neighbors=0)
    image = np.full((64,128,3),[10,20,30],np.uint8)
    points = np.array([[2.,0,0],[-8.,0,0]])
    rgb, before, _ = visible_colors(points,image,camera(),policy)
    assert before[0]/before[1] == pytest.approx(16.)
    policy['color_distance_power'] = 4.
    changed_rgb, after, _ = visible_colors(points,image,camera(),policy)
    np.testing.assert_array_equal(changed_rgb,rgb)
    assert after[0]/after[1] == pytest.approx(256.)
    policy['color_distance_power'] = 0.
    _, unweighted, _ = visible_colors(points,image,camera(),policy)
    np.testing.assert_array_equal(unweighted,[1.,1.])


@pytest.mark.parametrize('value',[-1.,9.,float('nan'),True,'2'])
def test_invalid_distance_confidence_rejected(tmp_path,value):
    policy = yaml.safe_load(CONFIG.read_text())
    policy['color_distance_power'] = value
    path = tmp_path/'policy.yaml'; path.write_text(yaml.safe_dump(policy))
    with pytest.raises(ValueError,match='color_distance_power'):
        load_color_policy(path)


@pytest.mark.parametrize('value',[-1.,float('nan'),True,'5'])
def test_invalid_rotation_confidence_rejected(tmp_path,value):
    policy = yaml.safe_load(CONFIG.read_text()); policy['color_rotation_scale_deg_s'] = value
    path = tmp_path/'policy.yaml'; path.write_text(yaml.safe_dump(policy))
    with pytest.raises(ValueError,match='color_rotation_scale_deg_s'):
        load_color_policy(path)


def test_robot_override_uses_ros_pitch_and_can_be_shared_across_bags():
    policy = load_color_policy(CONFIG)
    assert camera_in_lidar(policy) is None
    policy['camera_in_lidar'] = dict(xyz_m=[.1,.2,.285],rpy_degrees=[0,90,0])
    mount = camera_in_lidar(policy)
    np.testing.assert_allclose(mount[:3,0], [0,0,-1], atol=1.e-9)
    np.testing.assert_allclose(mount[:3,3], [.1,.2,.285])
    point = np.array([2,0,-.85])
    local = (point-mount[:3,3]) @ mount[:3,:3]
    np.testing.assert_allclose(local @ mount[:3,:3].T+mount[:3,3], point)
    policy['camera_in_lidar']['xyz_m'][2] = float('nan')
    with pytest.raises(ValueError):
        camera_in_lidar(policy)


def test_grazing_floor_is_not_mistaken_for_an_occluder_but_foreground_still_blocks_it():
    policy = load_color_policy(CONFIG)
    policy['edge_margin_px'] = 0
    image = np.full((64,128,3), [0,255,0], np.uint8)
    points = np.array([[2.,0,-1.], [4.,0,-1.]])
    normals = np.array([[0.,0,1.], [0.,0,1.]])
    _, before, _ = visible_colors(points, image, camera(), policy)
    _, after, stats = visible_colors(points, image, camera(), policy, normals_cam=normals)
    assert before[1] == 0 and after[1] > 0 and stats['coplanar_recovered'] > 0
    # A real foreground object has a different plane; it is not floor support.
    object_and_floor = np.array([[2.,0,-.5], [4.,0,-1.]])
    _, confidence, _ = visible_colors(object_and_floor, image, camera(), policy,
                                     normals_cam=np.array([[1.,0,0.], [0.,0,1.]]))
    assert confidence[1] == 0
    # Moving raw surfaces must block even if the candidate itself has a floor normal.
    _, confidence, _ = visible_colors(points[1:], image, camera(), policy, object_and_floor[:1], normals_cam=normals[1:])
    assert confidence[0] == 0


def test_live_uses_synchronized_images_preserves_scan_and_does_not_guess_stale_rgb(tmp_path, monkeypatch):
    calibration = tmp_path/'intrinsics.yaml'
    calibration.write_text(yaml.safe_dump(camera()))
    rclpy.init(args=['--ros-args','-p',f'calibration:={calibration}', '-p',f'colorization_config:={CONFIG}',
                    '-p','use_tf:=false'])
    node = None
    try:
        node = ThetaColorizeCloudNode()
        node.color_policy['edge_margin_px'] = 0
        node.images.append((10.,np.full((64,128,3),[0,0,255],np.uint8)))
        assert node._select_image(10.3) is None  # No unbounded latest-image fallback.
        assert node._select_image(10.)[0] == 10.
        header = Header(frame_id='sirius3/lidar_link')
        header.stamp.sec = 10
        fields = [PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                  for i,n in enumerate(('x','y','z','time'))]
        fields.append(PointField(name='ring',offset=16,datatype=PointField.UINT16,count=1))
        msg = create_cloud(header, fields, [(2.,0.,-.715,-.02,1),(4.,0.,-1.715,0.,2)])
        original = read_points(msg)
        messages = []
        monkeypatch.setattr(node,'pub',SimpleNamespace(publish=messages.append))
        node.receive_cloud(msg)
        colored = read_points(messages[-1])
        np.testing.assert_array_equal(colored['rgb'].view(np.uint32), [0xff0000,0x808080])
        for name in ('x','y','z','time','ring'):
            np.testing.assert_array_equal(colored[name],original[name])
        msg.header.stamp.nanosec = 300_000_000
        node.receive_cloud(msg)
        np.testing.assert_array_equal(read_points(messages[-1])['rgb'].view(np.uint32), [0x808080]*2)
        assert messages[-1].header == msg.header
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_live_motion_compensation_uses_camera_time_and_fixed_mount(tmp_path):
    calibration = tmp_path/'intrinsics.yaml'
    calibration.write_text(yaml.safe_dump(camera()))
    rclpy.init(args=['--ros-args','-p',f'calibration:={calibration}', '-p',f'colorization_config:={CONFIG}'])
    node = None
    try:
        node = ThetaColorizeCloudNode()
        fixed = TransformStamped()
        fixed.header.frame_id='sirius3/lidar_link';fixed.child_frame_id='sirius3/theta_link'
        fixed.transform.rotation.w=1.;fixed.transform.translation.z=.285
        node.tf_buffer.set_transform_static(fixed,'test')
        for stamp_ns,x in [(0,0.),(100_000_000,.1)]:
            dynamic=TransformStamped()
            dynamic.header.frame_id='sirius3/odom';dynamic.child_frame_id='sirius3/lidar_link'
            dynamic.header.stamp.sec=10;dynamic.header.stamp.nanosec=stamp_ns
            dynamic.transform.rotation.w=1.;dynamic.transform.translation.x=x
            node.tf_buffer.set_transform(dynamic,'test')
        cloud=SimpleNamespace(header=Header(frame_id='sirius3/lidar_link'))
        cloud.header.stamp.sec=10
        rotation,translation=node._image_transform(cloud,10.1)
        np.testing.assert_allclose(rotation,np.eye(3),atol=1.e-9)
        np.testing.assert_allclose(translation,[-.1,0,-.285],atol=1.e-8)
        assert node._image_transform(cloud,10.3) is None
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
