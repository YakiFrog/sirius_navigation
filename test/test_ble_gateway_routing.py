"""Tests for selecting exactly one BLE navigation delivery route."""

import json

from unittest.mock import Mock, patch

from std_msgs.msg import String

from sirius_navigation.sirius_ble_gateway import SiriusBleGateway


def _make_gateway(subscriber_count):
    gateway = SiriusBleGateway.__new__(SiriusBleGateway)
    gateway.instruction_pub = Mock()
    get_count = gateway.instruction_pub.get_subscription_count
    get_count.return_value = subscriber_count
    gateway.get_logger = Mock(return_value=Mock())
    gateway._send_to_nav_http = Mock()
    return gateway


def test_forwards_only_to_ros_topic_when_subscriber_is_available():
    """Prefer the ROS topic and do not also start an HTTP request."""
    gateway = _make_gateway(subscriber_count=1)

    thread_path = 'sirius_navigation.sirius_ble_gateway.threading.Thread'
    with patch(thread_path) as thread:
        route = gateway._forward_nav_instruction('前に進んで')

    assert route == 'ros_topic'
    gateway.instruction_pub.publish.assert_called_once()
    message = gateway.instruction_pub.publish.call_args.args[0]
    assert message.data == '前に進んで'
    thread.assert_not_called()


def test_falls_back_only_to_http_when_ros_topic_has_no_subscriber():
    """Use HTTP only when ROS discovery reports no instruction subscriber."""
    gateway = _make_gateway(subscriber_count=0)

    thread_path = 'sirius_navigation.sirius_ble_gateway.threading.Thread'
    with patch(thread_path) as thread:
        route = gateway._forward_nav_instruction('前に進んで')

    assert route == 'http'
    gateway.instruction_pub.publish.assert_not_called()
    thread.assert_called_once_with(
        target=gateway._send_to_nav_http,
        args=('前に進んで',),
        daemon=True,
    )
    thread.return_value.start.assert_called_once_with()


def test_confirmed_navigation_mode_is_in_remote_status():
    """Relay the robot-confirmed mode instead of relying on the UI selection."""
    gateway = SiriusBleGateway.__new__(SiriusBleGateway)
    gateway._navigation_mode = "unknown"
    gateway._last_remote_payload = ""
    gateway._last_remote_status = None
    gateway._emergency_stop_active = False
    gateway._last_battery_data = None
    gateway.advertise_name = "SiriusBleBridge"
    gateway.service_uuid = "test-service"
    gateway.remote_status_pub = Mock()
    gateway._people_status_fields = Mock(return_value={})

    gateway._on_navigation_mode(String(data="strict_normal"))
    gateway._publish_remote_status("connected", ble_link=True, active=True)

    message = gateway.remote_status_pub.publish.call_args.args[0]
    payload = json.loads(message.data)
    assert payload["navigation_mode"] == "strict_normal"
