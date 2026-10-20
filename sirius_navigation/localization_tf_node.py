"""Publish accepted map corrections in a process independent of registration."""
import copy
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def stale_reason(now, monotonic, odom, correction, odom_received, correction_received,
                 odom_timeout, correction_timeout):
    """Use receipt and sensor clocks: a stopped publisher cannot become fresh by restamping."""
    if correction is None:
        return 'waiting_for_registration'
    if odom is None:
        return 'waiting_for_odometry'
    odom_age, correction_age = now-stamp_seconds(odom), now-stamp_seconds(correction)
    if min(odom_age, correction_age) < -0.1:
        return 'future_stamp'
    if monotonic-odom_received > odom_timeout or odom_age > odom_timeout:
        return 'stale_odometry'
    if monotonic-correction_received > correction_timeout or correction_age > correction_timeout:
        return 'stale_registration'
    return 'ready'


class LocalizationTF(Node):
    def __init__(self):
        super().__init__('localization_tf')
        for name, value in [('map_frame', 'map'), ('odom_frame', 'sirius3/odom'),
                            ('odometry_topic', '/Odometry'), ('publish_rate', 20.0),
                            ('odom_timeout', 0.3), ('correction_timeout', 3.0)]:
            self.declare_parameter(name, value)
        for name in ('publish_rate', 'odom_timeout', 'correction_timeout'):
            if float(self.get_parameter(name).value) <= 0:
                raise ValueError(f'{name} must be positive')
        self._correction = None
        self._odom_stamp = None
        self._correction_received = self._odom_received = 0.0
        self._last_now = None
        self._reason = None
        self._published = 0
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._ready_pub = self.create_publisher(Bool, '/localization_ready', transient)
        self._status_pub = self.create_publisher(String, '/map_localizer/tf_status', 10)
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(TransformStamped, '/map_localizer/correction', self._on_correction, transient)
        self.create_subscription(Odometry, self.get_parameter('odometry_topic').value,
                                 self._on_odom, sensor)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._reset, 1)
        self._ready_pub.publish(Bool(data=False))
        self.create_timer(1.0/float(self.get_parameter('publish_rate').value), self._tick)

    def _reset(self, _=None):
        self._correction = None
        self._ready_pub.publish(Bool(data=False))

    def _on_correction(self, msg):
        if (msg.header.frame_id != self.get_parameter('map_frame').value or
                msg.child_frame_id != self.get_parameter('odom_frame').value):
            self.get_logger().error('Rejecting correction with unexpected frames')
            return
        t, q = msg.transform.translation, msg.transform.rotation
        if not all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)):
            return
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 0.01:
            return
        self._correction = msg
        self._correction_received = time.monotonic()

    def _on_odom(self, msg):
        self._odom_stamp = msg.header.stamp
        self._odom_received = time.monotonic()

    def _tick(self):
        now = self.get_clock().now().nanoseconds/1e9
        mono = time.monotonic()
        if self._last_now is not None and now < self._last_now-0.1:
            self._reset()
            self._odom_stamp = None
        self._last_now = now
        reason = stale_reason(now, mono, self._odom_stamp,
                              None if self._correction is None else self._correction.header.stamp,
                              self._odom_received, self._correction_received,
                              float(self.get_parameter('odom_timeout').value),
                              float(self.get_parameter('correction_timeout').value))
        if reason == 'ready':
            msg = copy.deepcopy(self._correction)
            # map->odom is the last accepted correction held at the current time.
            # Actual robot motion stays on the independently stamped odometry link.
            # Keep the correction's measurement stamp in /map_localizer/correction,
            # and stop this heartbeat when either source exceeds its age limit.
            msg.header.stamp = self.get_clock().now().to_msg()
            self._broadcaster.sendTransform(msg)
            self._published += 1
        if reason != self._reason:
            self._ready_pub.publish(Bool(data=reason == 'ready'))
            self.get_logger().info(f'Localization TF: {reason}')
            self._reason = reason
        status = {'ready': reason == 'ready', 'reason': reason, 'tf_published': self._published,
                  'odom_age_s': None if self._odom_stamp is None else now-stamp_seconds(self._odom_stamp),
                  'correction_age_s': None if self._correction is None else now-stamp_seconds(self._correction.header.stamp)}
        self._status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
