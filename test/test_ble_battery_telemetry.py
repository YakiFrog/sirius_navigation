"""Device-derived port powers and valid full/empty time estimates."""
from enum import Enum
from types import SimpleNamespace
import math

import pytest

from sirius_navigation.sirius_ble_gateway import SiriusBleGateway as Gateway


class Status(Enum):
    IDLE = 0
    CHARGING = 1
    BOTH = 2


def build(monkeypatch, **values):
    monkeypatch.setattr('sirius_navigation.sirius_ble_gateway.time.time', lambda: 1000)
    gateway = SimpleNamespace(battery_mac='7C:E9:13:76:E9:58',
                              _format_remaining_time=lambda h: Gateway._format_remaining_time(None, h))
    station = SimpleNamespace(**{'battery_percentage':75, 'charging_status':Status.IDLE,
                                'power_in':0, 'power_out':64, 'time_remaining':2.25,
                                **values})
    return Gateway._build_battery_status(gateway, station, Status)


def test_discharge_runtime_does_not_become_charge_eta(monkeypatch):
    data = build(monkeypatch)
    assert data['runtime_hours'] == 2.25
    assert data['charge_remaining_hours'] is None and data['charge_complete_at'] is None
    assert data['time_remaining_str'] == '2時間15分'


def test_charge_duration_and_wall_clock(monkeypatch):
    data = build(monkeypatch, power_in=100, power_out=10, time_remaining=1.5,
                 charging_status=Status.CHARGING)
    assert data['time_remaining_kind'] == 'charging'
    assert data['runtime_hours'] is None and data['charge_remaining_hours'] == 1.5
    assert data['charge_complete_at'] == 6400


@pytest.mark.parametrize('remaining', [6553.5, -1, 0, math.nan, math.inf])
def test_unavailable_time_is_never_a_runtime_or_eta(monkeypatch, remaining):
    data = build(monkeypatch, time_remaining=remaining)
    assert data['runtime_hours'] is None and data['charge_remaining_hours'] is None
    assert data['time_remaining_str'] == ''


def test_equal_input_output_has_no_full_or_empty_estimate(monkeypatch):
    data = build(monkeypatch, power_in=64)
    assert data['time_remaining_kind'] == 'idle'
    assert data['runtime_hours'] is None and data['charge_remaining_hours'] is None


def test_port_zero_unknown_and_output_switch_are_distinct(monkeypatch):
    data = build(monkeypatch, ac_power_out=64, dc_power_out=0, usb_c1_power=-1,
                 usb_c2_power=math.inf, ac_output=SimpleNamespace(name='OUTPUT'),
                 dc_output=SimpleNamespace(name='NOT_CONNECTED'))
    assert data['ports']['ac_output'] == 64 and data['ports']['dc_output'] == 0
    assert data['ports']['usb_c1'] is None and data['ports']['usb_c2'] is None
    assert data['ports']['usb_c3'] is None
    assert data['output_states'] == {'ac':'on', 'dc':'off'}
    assert data['device_mac'] == '7C:E9:13:76:E9:58'


def test_time_format_carries_sixty_minutes_to_next_hour():
    assert Gateway._format_remaining_time(None, 1.999) == '2時間'
    assert Gateway._format_remaining_time(None, 0.001) == '1分未満'
