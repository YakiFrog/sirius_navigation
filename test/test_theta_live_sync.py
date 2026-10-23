"""Camera cadence and bounded waiting for temporally matching images."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import rclpy
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
import yaml

from sirius_navigation.theta_capture_node import ThetaCaptureNode
from sirius_navigation.theta_colorize_cloud_node import ThetaColorizeCloudNode


def test_capture_deadline_keeps_five_hz_when_ten_hz_input_has_jitter():
    node = SimpleNamespace(fps=5., next_publish=0.)
    stamps = np.arange(101) * .1 + np.sin(np.arange(101)) * .001 + 10
    accepted = [t for t in stamps if ThetaCaptureNode._should_publish(node, t)]
    assert 49 <= len(accepted) <= 51
    assert np.max(np.diff(accepted)) < .31
    # After a stall publish one frame, then resume normal pacing (no burst).
    assert ThetaCaptureNode._should_publish(node, 30.)
    assert not ThetaCaptureNode._should_publish(node, 30.001)


def test_pending_clouds_are_bounded_and_expire_without_camera():
    node = SimpleNamespace(image_wait_timeout=.3, pending_clouds=deque(), _colorize_cloud=Mock())
    for i in range(21):
        ThetaColorizeCloudNode.receive_cloud(node, i)
    assert len(node.pending_clouds) == 20
    node._colorize_cloud.assert_called_once_with(0)
    node.pending_clouds = deque((t-1, msg) for t, msg in node.pending_clouds)
    ThetaColorizeCloudNode._flush_pending_clouds(node)
    assert not node.pending_clouds
    assert node._colorize_cloud.call_count == 21


def test_image_arriving_after_cloud_is_used_without_altering_geometry(tmp_path):
    calibration = tmp_path/'intrinsics.yaml'
    lens = dict(focal=[16., 16.], distortion=[0.]*4, max_theta_degrees=95.)
    calibration.write_text(yaml.safe_dump(dict(image_size=[128,64],
        front=dict(lens, center=[32.,32.]), back=dict(lens, center=[96.,32.]))))
    config = Path(__file__).resolve().parents[1]/'config/theta_colorization.yaml'
    rclpy.init(args=['--ros-args', '-p', f'calibration:={calibration}',
                    '-p', f'colorization_config:={config}', '-p', 'use_tf:=false'])
    node = None
    try:
        node = ThetaColorizeCloudNode()
        node.color_policy['edge_margin_px'] = 0
        messages = []
        node.pub = SimpleNamespace(publish=messages.append)
        header = Header(frame_id='sirius3/lidar_link'); header.stamp.sec = 10
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x','y','z','time'))]
        cloud = create_cloud(header, fields, [(2., 0., -.715, -.02)])
        node.receive_cloud(cloud)
        node._flush_pending_clouds()
        assert not messages
        # The next image arrives after this scan, but is the closest in time.
        node.images.append((10.03, np.full((64,128,3), [0,0,255], np.uint8)))
        node.pending_clouds = deque((t-1, m) for t, m in node.pending_clouds)
        node._flush_pending_clouds()
        assert len(messages) == 1 and messages[0].header == header
        points = read_points(messages[0])
        assert points['rgb'].view(np.uint32)[0] == 0xff0000
        original = read_points(cloud)
        for name in ('x','y','z','time'):
            np.testing.assert_array_equal(points[name], original[name])
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
