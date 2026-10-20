"""Saved base poses must align with the map and survive late subscription."""
import time
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sirius_navigation.theta_ply_publisher import PlyPublisher, read_trajectory


def write_csv(path, second='11.000000001'):
    path.write_text('stamp,x,y,z,qx,qy,qz,qw\n'
        '10.123456789,1,2,0.3,0,0,0,1\n'
        +second+',3,4,1.3,0,0,0.6,0.8\n')


def test_read_saved_base_xyz_orientation_and_nanosecond_time(tmp_path):
    p=tmp_path/'trajectory.csv';write_csv(p)
    poses=read_trajectory(p,'offline_map')
    assert len(poses)==2
    assert poses[0].header.stamp.sec==10 and poses[0].header.stamp.nanosec==123456789
    assert poses[1].header.stamp.nanosec==1
    assert poses[1].pose.position.z==1.3 and poses[1].pose.orientation.z==.6
    write_csv(p,'9.0')
    with pytest.raises(ValueError):read_trajectory(p,'offline_map')
    p.write_text('stamp,x,y,z,qx,qy,qz,qw\n10,nan,2,0,0,0,0,1\n')
    with pytest.raises(ValueError):read_trajectory(p,'offline_map')


@pytest.mark.parametrize('with_trajectory',[False,True])
def test_latched_path_shares_map_frame_and_publish_time_without_height_cut(tmp_path,with_trajectory):
    p=tmp_path/'map_3d_colored.ply'
    p.write_text('ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n1 2 0\n')
    if with_trajectory:write_csv(tmp_path/'trajectory.csv')
    original=p.read_bytes()
    rclpy.init(args=['--ros-args','-p',f'ply_file:={p}','-p','topic:=/trajectory_test/map',
        '-p','trajectory_topic:=/trajectory_test/path','-p','frame_id:=offline_map',
        '-p','period_sec:=60.0','-p','z_min:=-0.1','-p','z_max:=0.1'])
    pub=sub=executor=None
    try:
        pub=PlyPublisher();sub=Node('trajectory_test_subscriber');received={}
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions=[sub.create_subscription(Path,'/trajectory_test/path',lambda m:received.setdefault('path',m),qos),
            sub.create_subscription(PointCloud2,'/trajectory_test/map',lambda m:received.setdefault('map',m),qos)]
        executor=SingleThreadedExecutor();executor.add_node(pub);executor.add_node(sub)
        deadline=time.monotonic()+5
        while len(received)<2 and time.monotonic()<deadline:executor.spin_once(timeout_sec=.1)
        assert len(received)==2
        assert received['path'].header==received['map'].header
        assert len(received['path'].poses)==(2 if with_trajectory else 0)
        if with_trajectory:
            assert all(q.header.frame_id=='offline_map' for q in received['path'].poses)
            np.testing.assert_allclose([q.pose.position.z for q in received['path'].poses],[.3,1.3])
        assert p.read_bytes()==original
    finally:
        if executor:executor.shutdown()
        if sub:sub.destroy_node()
        if pub:pub.destroy_node()
        rclpy.shutdown()
