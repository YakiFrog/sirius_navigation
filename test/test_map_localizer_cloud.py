"""Regression coverage for organized Velodyne clouds and row padding."""
import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from sirius_navigation.map_localizer_node import _pc2_to_xyz


@pytest.mark.parametrize('bigendian', [False, True])
def test_organized_cloud_preserves_every_xyz_with_padded_rows(bigendian):
    msg = PointCloud2(height=3, width=2, point_step=16, row_step=40,
                      is_bigendian=bigendian)
    msg.fields = [PointField(name=k, offset=i * 4,
                            datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(('x', 'y', 'z'))]
    data = bytearray(msg.height * msg.row_step)
    expected = np.arange(18, dtype=np.float64).reshape(6, 3) + 0.25
    for row in range(msg.height):
        for col in range(msg.width):
            offset = row * msg.row_step + col * msg.point_step
            data[offset:offset + 12] = expected[row * msg.width + col].astype(
                ('>' if bigendian else '<') + 'f4').tobytes()
    msg.data = bytes(data)
    np.testing.assert_array_equal(_pc2_to_xyz(msg), expected)


def test_tf_continues_after_registration_rejects_scan(tmp_path):
    import time
    import rclpy
    from tf2_msgs.msg import TFMessage
    from sirius_navigation.map_localizer_node import MapLocalizer

    points = np.random.default_rng(7).normal(size=(200, 3)).astype('<f4')
    path = tmp_path / 'map.npy'
    np.save(path, points)
    rclpy.init(args=['--ros-args', '-p', 'map_path:=' + str(path)])
    node = MapLocalizer()
    received = []
    node.create_subscription(TFMessage, '/tf', lambda m: received.extend(
        t for t in m.transforms if t.header.frame_id == 'map'), 20)
    try:
        node.T_map_odom = np.eye(4)
        node._lookup = lambda target, source, stamp=None: np.eye(4)
        cloud = PointCloud2(height=1, width=len(points), point_step=12,
                            row_step=12 * len(points))
        cloud.header.frame_id = 'lidar'
        cloud.header.stamp = node.get_clock().now().to_msg()
        cloud.fields = [PointField(name=k, offset=i * 4,
                                   datatype=PointField.FLOAT32, count=1)
                        for i, k in enumerate(('x', 'y', 'z'))]
        cloud.data = (points + np.array([20, 0, 0], dtype='<f4')).tobytes()
        node._cloud = cloud
        node._update()
        assert node._last_status.startswith('reject ')
        until = time.monotonic() + 1.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(received) >= 3
        assert all(t.child_frame_id == 'sirius3/odom' for t in received)
        assert all(t.transform.translation.x == 0.0 for t in received)
        stamps = [t.header.stamp.sec * 10**9 + t.header.stamp.nanosec for t in received]
        assert stamps[-1] > stamps[0]
    finally:
        node.destroy_node()
        rclpy.shutdown()
