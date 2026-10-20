"""A slow registration must not starve the regular TF publication."""
import time
import threading

import numpy as np
import rclpy
from sirius_navigation.map_localizer_node import MapLocalizer, make_executor


def test_tf_heartbeat_continues_during_blocked_registration(monkeypatch):
    intervals = []
    entered = threading.Event()
    finished = threading.Event()

    def slow_registration(self):
        entered.set()
        time.sleep(.7)
        finished.set()

    monkeypatch.setattr(MapLocalizer, '_load_map', lambda self: None)
    monkeypatch.setattr(MapLocalizer, '_update', slow_registration)
    monkeypatch.setattr(MapLocalizer, '_publish_tf', lambda self: intervals.append(time.monotonic()))
    rclpy.init(args=[])
    node = MapLocalizer()
    node.T_map_odom = np.eye(4)
    executor = make_executor()
    executor.add_node(node)
    try:
        until = time.monotonic()+1.4
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.02)
        assert entered.is_set() and finished.is_set()
        assert len(intervals) >= 10
        assert max(np.diff(intervals)) < .25
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
