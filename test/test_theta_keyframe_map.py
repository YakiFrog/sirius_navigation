"""Regressions for registering simulated RGB scans with FAST-LIO poses."""

from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

from sirius_navigation.theta_keyframe_map_node import ThetaKeyframeMapNode, write_ply


@pytest.fixture
def mapper():
    """Exercise the production queue without starting ROS or TF listeners."""
    node = SimpleNamespace(
        odom_buffer=deque(), pending=deque(), pose_tol=0.002,
        skipped_tf=0, body_frame='body', lidar_frame='lidar',
        lidar_extrinsic=(np.eye(3), np.array([0.05, 0.0, 0.55])),
        ext_cache={}, scans=[],
    )
    node._extrinsic = lambda src: ThetaKeyframeMapNode._extrinsic(node, src)
    node._add_keyframe = lambda xyz, colors, rot, trans: node.scans.append(
        (xyz @ rot.T + trans, colors))
    node.drain = lambda: ThetaKeyframeMapNode._drain_pending(node)
    return node


def enqueue(node, stamp, xyz, frame='lidar'):
    """Add a scan in its own sensor frame."""
    node.pending.append((stamp, frame, np.array([xyz]), np.ones((1, 3))))


def test_moving_scan_waits_for_its_own_pose(mapper):
    """A delayed FAST-LIO pose must not smear a stationary wall."""
    # The old algorithm used this pose (100 ms old) as soon as the scan arrived.
    mapper.odom_buffer.append((1_000_000_000, np.eye(3), np.zeros(3)))
    enqueue(mapper, 1_100_000_000, [3.95, 0.0, -0.55])
    mapper.drain()
    assert not mapper.scans
    assert len(mapper.pending) == 1

    mapper.odom_buffer.append((1_100_000_000, np.eye(3), np.array([1., 0., 0.])))
    mapper.drain()
    np.testing.assert_allclose(mapper.scans[0][0], [[5.0, 0.0, 0.0]], atol=1e-12)
    assert not mapper.pending


def test_delayed_cloud_uses_closest_pose(mapper):
    """Choose the matching pose even when several poses are already buffered."""
    mapper.pose_tol = 0.15
    for stamp, x in [(1_000_000_000, 0.), (1_100_000_000, 1.), (1_200_000_000, 2.)]:
        mapper.odom_buffer.append((stamp, np.eye(3), np.array([x, 0., 0.])))
    enqueue(mapper, 1_100_000_000, [3.95, 0.0, -0.55])
    mapper.drain()
    np.testing.assert_allclose(mapper.scans[0][0], [[5., 0., 0.]], atol=1e-12)


def test_fastlio_nanosecond_rounding_does_not_block(mapper):
    """FAST-LIO converts stamp through double seconds, losing a nanosecond."""
    mapper.odom_buffer.append((1_100_000_000, np.eye(3), np.zeros(3)))
    enqueue(mapper, 1_100_000_001, [1., 0., 0.])
    mapper.drain()
    assert len(mapper.scans) == 1


def test_missing_pose_drops_scan_instead_of_misregistering(mapper):
    """After the pose stream passes a lost scan, reject stale poses."""
    mapper.odom_buffer.extend([
        (1_000_000_000, np.eye(3), np.zeros(3)),
        (1_200_000_000, np.eye(3), np.ones(3)),
    ])
    enqueue(mapper, 1_100_000_000, [1., 0., 0.])
    mapper.drain()
    assert not mapper.scans
    assert not mapper.pending
    assert mapper.skipped_tf == 1


def test_extrinsic_rotates_with_body(mapper):
    """Compose LiDAR extrinsics with IMU pose just as LaserMap does."""
    rot = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    mapper.odom_buffer.append((1_000_000_000, rot, np.array([2., 3., 0.])))
    enqueue(mapper, 1_000_000_000, [1., 0., 0.])
    mapper.drain()
    np.testing.assert_allclose(mapper.scans[0][0], [[2., 4.05, 0.55]])


def test_pending_clouds_keep_their_own_frames(mapper):
    """A later cloud cannot change an earlier cloud's extrinsic."""
    mapper.odom_buffer.append((1_000_000_000, np.eye(3), np.zeros(3)))
    enqueue(mapper, 1_000_000_000, [1., 0., 0.], frame='lidar')
    enqueue(mapper, 1_000_000_000, [1., 0., 0.], frame='body')
    mapper.drain()
    np.testing.assert_allclose(mapper.scans[0][0], [[1.05, 0., 0.55]])
    np.testing.assert_allclose(mapper.scans[1][0], [[1., 0., 0.]])


def test_missing_tf_keeps_scan_until_transform_arrives(mapper):
    """An initially unavailable TF should not discard a matched scan."""
    mapper.odom_buffer.append((1_000_000_000, np.eye(3), np.zeros(3)))
    mapper._lookup = lambda *args: None
    enqueue(mapper, 1_000_000_000, [1., 0., 0.], frame='other_lidar')
    mapper.drain()
    assert len(mapper.pending) == 1
    mapper.ext_cache['other_lidar'] = (np.eye(3), np.zeros(3))
    mapper.drain()
    assert len(mapper.scans) == 1


@pytest.fixture
def geometry_mapper(mapper):
    """Bind geometry mode methods with a capturing publisher."""
    mapper.geometry_topic = '/Laser_map'
    mapper.geometry_xyz = None
    mapper.geometry_colors = None
    mapper.geometry_publisher = None
    mapper.color_max_distance = 0.5
    mapper.output_frame = 'camera_init'
    mapper.key_xyz = deque([np.array([[1., 0., 0.], [2., 0., 0.]])])
    mapper.key_col = deque([np.array([[1., 0., 0.], [0., 1., 0.]])])
    mapper.dirty = False
    mapper.scans = 0
    mapper.fields = []
    mapper.published = []
    mapper.pub = SimpleNamespace(publish=mapper.published.append)
    mapper.get_logger = lambda: SimpleNamespace(info=lambda *args, **kwargs: None)
    mapper._assemble = lambda: ThetaKeyframeMapNode._assemble(mapper)
    return mapper


def laser_map(node, points, publisher=b'fastlio'):
    """Deliver a LaserMap snapshot, preserving duplicate and nonfinite points."""
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header

    header = Header(frame_id='camera_init')
    header.stamp.sec = 123
    msg = create_cloud_xyz32(header, points)
    ThetaKeyframeMapNode.on_geometry(node, msg, {'publisher_gid': publisher})
    return msg


def test_geometry_and_saved_ply_exactly_match_laser_map(geometry_mapper, tmp_path):
    """Color observations must never add, remove, reorder or move map points."""
    node = geometry_mapper
    points = np.array([[2., 0., 0.], [1., 0., 0.], [1., 0., 0.],
                       [99., 0., 0.], [np.nan, 0., 0.]], dtype=np.float32)
    source = laser_map(node, points)
    xyz, rgb = node._assemble()
    np.testing.assert_array_equal(xyz, points)
    np.testing.assert_allclose(rgb[:4], [[0, 1, 0], [1, 0, 0], [1, 0, 0], [.5, .5, .5]])
    ThetaKeyframeMapNode.rebuild(node)
    output = node.published[-1]
    assert output.header == source.header
    assert (output.width, output.height, output.is_dense) == (
        source.width, source.height, source.is_dense)
    published = np.frombuffer(output.data, dtype='<f4').reshape(-1, 4)
    np.testing.assert_array_equal(published[:, :3], points)
    path = tmp_path / 'map.ply'
    write_ply(path, xyz, np.round(rgb * 255).astype(np.uint8))
    raw = path.read_bytes().split(b'end_header\n', 1)[1]
    saved = np.frombuffer(raw, dtype=[('xyz', '<f4', 3), ('rgb', 'u1', 3)])
    np.testing.assert_array_equal(saved['xyz'], points)


def test_new_laser_map_replaces_previous_geometry(geometry_mapper):
    """Shrinking or empty maps must remove old geometry immediately."""
    node = geometry_mapper
    laser_map(node, [[1., 0., 0.], [2., 0., 0.]])
    laser_map(node, [[3., 0., 0.]])
    np.testing.assert_array_equal(node._assemble()[0], [[3., 0., 0.]])
    laser_map(node, np.empty((0, 3), dtype=np.float32))
    assert node._assemble()[0].shape == (0, 3)


def test_fastlio_restart_discards_old_color_observations(geometry_mapper):
    """A new publisher's coordinate origin must not reuse previous colors."""
    node = geometry_mapper
    laser_map(node, [[1., 0., 0.]])
    laser_map(node, [[1., 0., 0.]], publisher=b'restarted_fastlio')
    assert not node.key_xyz
    np.testing.assert_array_equal(node._assemble()[1], [[.5, .5, .5]])


def test_missing_color_or_different_frame_preserves_map(geometry_mapper):
    """Unobserved points stay neutral rather than borrowing distant colors."""
    node = geometry_mapper
    laser_map(node, [[1., 0., 0.]])
    node.output_frame = 'other_origin'
    xyz, rgb = node._assemble()
    np.testing.assert_array_equal(xyz, [[1., 0., 0.]])
    np.testing.assert_array_equal(rgb, [[.5, .5, .5]])


def test_jazzy_message_info_without_gid(geometry_mapper):
    """Resolve the publisher from discovery when Jazzy only supplies timestamps."""
    node = geometry_mapper
    source = laser_map(node, [[1., 0., 0.]])
    node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(endpoint_gid=b'new')]
    ThetaKeyframeMapNode.on_geometry(node, source, {'source_timestamp': 1})
    assert node.geometry_publisher == b'new'
    assert not node.key_xyz


def test_rotating_cloud_uses_each_point_time_instead_of_one_end_pose(mapper):
    mapper.lidar_extrinsic = (np.eye(3), np.zeros(3))
    mapper.odom_buffer.extend([(1_000_000_000, np.eye(3), np.zeros(3)),
                               (1_100_000_000, np.eye(3), np.array([1., 0., 0.]))])
    mapper.pending.append((1_000_000_000, 'lidar', np.array([[5., 0., 0.], [4., 0., 0.]]),
                           np.ones((2, 3)), np.array([0., .1])))
    mapper.drain()
    assert len(mapper.scans) == 1
    np.testing.assert_allclose(mapper.scans[0][0], [[5., 0., 0.], [5., 0., 0.]], atol=1e-9)
