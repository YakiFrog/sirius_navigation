"""Freshness must depend on measurement time as well as transport arrival."""
from builtin_interfaces.msg import Time
from sirius_navigation.localization_tf_node import stale_reason


def at(t):
    return Time(sec=int(t), nanosec=round((t-int(t))*1e9))


def test_fresh_and_stopped_inputs():
    assert stale_reason(10, 20, at(9.95), at(9.8), 19.95, 19.8, .3, 3) == 'ready'
    assert stale_reason(10, 20, at(9), at(9.8), 20, 20, .3, 3) == 'stale_odometry'
    assert stale_reason(10, 20, at(9.95), at(6), 20, 20, .3, 3) == 'stale_registration'
    assert stale_reason(10, 20, at(9.95), at(9.8), 19, 20, .3, 3) == 'stale_odometry'
    assert stale_reason(10, 20, at(9.95), at(9.8), 20, 16, .3, 3) == 'stale_registration'


def test_no_identity_tf_before_first_accepted_registration_and_no_future_stamps():
    assert stale_reason(10, 20, at(9.95), None, 20, 20, .3, 3) == 'waiting_for_registration'
    assert stale_reason(10, 20, None, at(9.8), 20, 20, .3, 3) == 'waiting_for_odometry'
    assert stale_reason(10, 20, at(11), at(9.8), 20, 20, .3, 3) == 'future_stamp'
