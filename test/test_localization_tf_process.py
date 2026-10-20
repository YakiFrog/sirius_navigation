"""Exercise the real TF process, including sensor and registration outages."""
import json
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def test_separate_tf_process_and_stale_input_shutdown(monkeypatch):
    # Run this integration test in its own localhost-only ROS domain.
    monkeypatch.setenv('ROS_DOMAIN_ID', '222')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init()
    node = rclpy.create_node('tf_process_test')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    odom_pub = node.create_publisher(Odometry, '/Odometry', sensor)
    correction_pub = node.create_publisher(TransformStamped, '/map_localizer/correction', latched)
    events, statuses, sent_stamps = [], [], set()
    def tf(msg):
        for item in msg.transforms:
            if item.header.frame_id == 'map':
                events.append((time.monotonic(), item.header.stamp.sec, item.header.stamp.nanosec))
    tf_sub = node.create_subscription(TFMessage, '/tf', tf, sensor)
    status_sub = node.create_subscription(String, '/map_localizer/tf_status',
                                          lambda m: statuses.append(json.loads(m.data)), 10)
    child = subprocess.Popen([sys.executable, '-c',
                              'from sirius_navigation.localization_tf_node import main; main()',
                              '--ros-args', '-p', 'correction_timeout:=0.7'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    busy = None
    def pump(duration, odom=True, correction=True):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            if odom:
                msg = Odometry(); msg.header.stamp = stamp
                sent_stamps.add((stamp.sec, stamp.nanosec)); odom_pub.publish(msg)
            if correction:
                msg = TransformStamped(); msg.header.stamp = stamp
                msg.header.frame_id = 'map'; msg.child_frame_id = 'sirius3/odom'
                msg.transform.rotation.w = 1.; msg.transform.translation.x = 1.25
                correction_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=.02)
            time.sleep(.01)
    try:
        deadline = time.monotonic()+15
        while correction_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert child.poll() is None and correction_pub.get_subscription_count()
        pump(.4, correction=False)
        assert not events  # no identity map correction before successful registration
        busy = subprocess.Popen([sys.executable, '-c',
                                  'import time; end=time.monotonic()+2\nwhile time.monotonic()<end: sum(range(20000))'])
        pump(2)
        assert len(events) >= 15
        assert max(b[0]-a[0] for a, b in zip(events, events[1:])) < .25
        now_ns = node.get_clock().now().nanoseconds
        assert all(sec*10**9+ns <= now_ns for _, sec, ns in events)
        pump(.65, odom=False)
        count = len(events)
        pump(.3, odom=False)
        assert len(events) == count and statuses[-1]['reason'] == 'stale_odometry'
        pump(.4)
        assert statuses[-1]['ready']
        pump(1, correction=False)
        count = len(events)
        pump(.3, correction=False)
        assert len(events) == count and statuses[-1]['reason'] == 'stale_registration'
    finally:
        child.terminate(); child.wait(timeout=5)
        if busy is not None:
            busy.wait(timeout=5)
        node.destroy_node(); rclpy.try_shutdown()
