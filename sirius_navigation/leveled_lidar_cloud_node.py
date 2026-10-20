"""Transform raw LiDAR XYZ into the level frame without changing other fields.

This is a rigid frame conversion, not deskewing, filtering or floor flattening.
FAST-LIO must keep using the raw input and the calibrated LiDAR-to-IMU rotation.
"""
import copy

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import dtype_from_fields
from tf2_ros import Buffer, TransformException, TransformListener


def transform_cloud(msg, transform, target_frame):
    """Preserve organized layout, row padding, ring, intensity and point times."""
    result = copy.deepcopy(msg)
    result.header.frame_id = target_frame
    dtype = dtype_from_fields(msg.fields, point_step=msg.point_step).newbyteorder('>' if msg.is_bigendian else '<')
    if not all(name in dtype.names for name in ('x', 'y', 'z')):
        raise ValueError('Point cloud has no XYZ fields')
    data = bytearray(msg.data)
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=data,
                        strides=(msg.row_step, msg.point_step))
    xyz = np.stack([points[name] for name in ('x', 'y', 'z')], axis=-1)
    q = transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = quat/norm
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    t = transform.translation
    translated = xyz @ rotation.T + [t.x, t.y, t.z]
    for i, name in enumerate(('x', 'y', 'z')):
        points[name] = translated[..., i]
    result.data = bytes(data)
    return result


class LeveledLidarCloudNode(Node):
    def __init__(self):
        super().__init__('leveled_lidar_cloud')
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_leveled')
        self.declare_parameter('target_frame', 'sirius3/lidar_level')
        self.target = self.get_parameter('target_frame').value
        source, output = [self.get_parameter(name).value for name in ('input_topic', 'output_topic')]
        if source == output:
            raise ValueError('Raw and leveled point cloud topics must differ')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # Reliable output also supports the existing Velodyne LaserScan subscriber.
        self.publisher = self.create_publisher(PointCloud2, output, 2)
        self.subscription = self.create_subscription(PointCloud2, source, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f'{source} -> {output}, frame={self.target}; raw FAST-LIO input is preserved')

    def on_cloud(self, msg):
        try:
            transform = self.buffer.lookup_transform(self.target, msg.header.frame_id,
                                                     Time.from_msg(msg.header.stamp), Duration(seconds=0.01))
            self.publisher.publish(transform_cloud(msg, transform.transform, self.target))
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f'Cannot level LiDAR: {error}', throttle_duration_sec=5.)


def main():
    rclpy.init()
    node = LeveledLidarCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
