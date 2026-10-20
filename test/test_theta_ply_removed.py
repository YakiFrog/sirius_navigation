"""Clean and removed maps must be separate, latched, aligned ROS messages."""
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

from sirius_navigation.theta_ply_publisher import PlyPublisher, read_ply


def write_ascii(path, points):
    path.write_text('ply\nformat ascii 1.0\nelement vertex '+str(len(points))+'\n'
                    'property float x\nproperty float y\nproperty float z\n'
                    'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'
                    +''.join(' '.join(map(str, p))+'\n' for p in points))


@pytest.mark.parametrize('with_removed', [True, False])
def test_late_subscriber_gets_clean_geometry_and_red_removed_with_same_header(tmp_path, with_removed):
    clean = tmp_path/'map_3d_colored.ply'
    write_ascii(clean, [[1, 2, 3, 10, 20, 30], [4, 5, 6, 40, 50, 60]])
    if with_removed:
        write_ascii(tmp_path/'dynamic_candidates.ply', [[7, 8, 9, 0, 255, 0]])
        write_ascii(tmp_path/'dynamic_fragments.ply', [[7, 8, 9, 0, 255, 0]])
    rclpy.init(args=['--ros-args', '-p', f'ply_file:={clean}', '-p', 'topic:=/test_clean',
                    '-p', 'geometry_topic:=/test_geometry', '-p', 'removed_topic:=/test_removed',
                    '-p', 'fragment_topic:=/test_fragments',
                    '-p', 'frame_id:=offline_map', '-p', 'period_sec:=60.0'])
    pub = sub = executor = None
    try:
        pub = PlyPublisher()  # Publishes BEFORE subscribers exist.
        sub = Node('ply_test_subscriber')
        received = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [sub.create_subscription(PointCloud2, '/test_'+name,
                         lambda msg, name=name: received.setdefault(name, msg), qos)
                         for name in ('clean', 'geometry', 'removed', 'fragments')]
        executor = SingleThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)
        deadline = time.monotonic()+5
        while len(received) < 4 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        assert len(received) == 4
        assert received['clean'].header == received['geometry'].header == received['removed'].header
        assert received['clean'].header == received['fragments'].header
        assert received['clean'].header.frame_id == 'offline_map'
        colored, geometry = read_points(received['clean']), read_points(received['geometry'])
        for name in ('x', 'y', 'z'):
            np.testing.assert_array_equal(colored[name], geometry[name])
        np.testing.assert_array_equal(colored['rgb'].view(np.uint32), [0x0a141e, 0x28323c])
        removed = received['removed']
        fragments = received['fragments']
        assert removed.width == int(with_removed)
        assert fragments.width == int(with_removed)
        if with_removed:
            values = read_points(removed)
            assert values['x'][0] == 7 and values['rgb'].view(np.uint32)[0] == 0xff3030
            assert read_points(fragments)['rgb'].view(np.uint32)[0] == 0xffa000
        else:
            assert bytes(removed.data) == b''  # Clear an overlay from a previous map.
    finally:
        if executor:
            executor.shutdown()
        if sub:
            sub.destroy_node()
        if pub:
            pub.destroy_node()
        rclpy.shutdown()


def test_empty_ascii_diagnostic_does_not_read_other_elements(tmp_path):
    path = tmp_path/'dynamic_candidates.ply'
    write_ascii(path, [])
    xyz, rgb = read_ply(path)
    assert xyz.shape == (0, 3) and rgb.shape == (0,)


def test_z_cut_updates_both_clouds_rejects_invalid_range_and_restores_points(tmp_path):
    clean = tmp_path/'map_3d_colored.ply'
    removed = tmp_path/'dynamic_candidates.ply'
    rows = [[1, 2, z, 10, 20, 30] for z in (-1, 0, 2, 3)]
    write_ascii(clean, rows)
    write_ascii(removed, rows)
    write_ascii(tmp_path/'dynamic_fragments.ply', rows)
    original = clean.read_bytes(), removed.read_bytes()
    rclpy.init(args=['--ros-args', '-p', f'ply_file:={clean}', '-p', 'removed_topic:=/test_removed',
                    '-p', 'fragment_topic:=/test_fragments',
                    '-p', 'z_min:=0.0', '-p', 'z_max:=2.0'])
    pub = None
    try:
        pub = PlyPublisher()
        np.testing.assert_array_equal(pub.data['z'], [0, 2])
        np.testing.assert_array_equal(pub.removed_data['z'], [0, 2])
        np.testing.assert_array_equal(pub.fragment_data['z'], [0, 2])
        result = pub.set_parameters_atomically([Parameter('z_min', value=3.0)])
        assert not result.successful
        assert pub.get_parameter('z_min').value == 0.
        assert not pub.set_parameters_atomically([Parameter('z_max', value=float('nan'))]).successful
        # Zero points is valid; an empty cloud clears the existing ceiling display.
        assert pub.set_parameters_atomically([Parameter('z_min', value=1.), Parameter('z_max', value=1.)]).successful
        assert len(pub.data) == len(pub.removed_data) == len(pub.fragment_data) == 0
        # Restore from full clouds rather than filtering previously cropped data.
        assert pub.set_parameters_atomically([Parameter('z_min', value=-1.), Parameter('z_max', value=3.)]).successful
        np.testing.assert_array_equal(pub.data['z'], [-1, 0, 2, 3])
        np.testing.assert_array_equal(pub.removed_data['z'], [-1, 0, 2, 3])
        np.testing.assert_array_equal(pub.fragment_data['z'], [-1, 0, 2, 3])
        assert original == (clean.read_bytes(), removed.read_bytes())
    finally:
        if pub:
            pub.destroy_node()
        rclpy.shutdown()
