import json
import threading
from unittest.mock import Mock

from std_msgs.msg import String

from sirius_navigation.sirius_network_gateway import (
    PairingAuthority,
    SiriusNetworkGateway,
    build_manual_stop_payload,
    parse_manual_motion,
)


def test_pairing_code_is_one_time_and_token_authenticates():
    authority = PairingAuthority(ttl_seconds=60.0)
    code = authority.issue_code(now=100.0)

    assert len(code) == 6
    assert authority.pair("incorrect", now=101.0) is None

    token = authority.pair(code, now=101.0)
    assert token
    assert authority.authenticate(token)
    assert authority.pair(code, now=102.0) is None


def test_expired_pairing_code_is_rejected():
    authority = PairingAuthority(ttl_seconds=10.0)
    code = authority.issue_code(now=100.0)

    assert authority.pair(code, now=111.0) is None


def test_manual_motion_and_stop_payload():
    moving_payload = '[nav]{"type":"manual_teleop","linear":0.2,"angular":0.0,"assisted":true}'
    assert parse_manual_motion(moving_payload) == (True, True)

    stop_payload = build_manual_stop_payload(assisted=False)
    assert parse_manual_motion(stop_payload) == (False, False)
    stop_data = json.loads(stop_payload[len("[nav]"):])
    assert stop_data["linear"] == 0.0
    assert stop_data["angular"] == 0.0


def test_network_status_contains_robot_confirmed_navigation_mode():
    gateway = SiriusNetworkGateway.__new__(SiriusNetworkGateway)
    gateway._last_status = {}
    gateway._battery = None
    gateway._navigation_mode = "unknown"
    gateway._emergency_stop = False
    gateway._stop_state_received = False
    gateway._pairing = PairingAuthority()
    gateway._pairing_lock = threading.Lock()
    gateway._active_socket = None
    gateway._active_controller = ""
    gateway._people_count_at = 0.0
    gateway._people_count = None
    gateway._tracked_people_count = None
    gateway._broadcast_status = Mock()

    gateway._on_navigation_mode(String(data="strict_normal"))

    assert gateway._status_payload()["navigation_mode"] == "strict_normal"
    gateway._broadcast_status.assert_called_once_with()
