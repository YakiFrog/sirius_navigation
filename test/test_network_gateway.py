import json

from sirius_navigation.sirius_network_gateway import (
    PairingAuthority,
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
