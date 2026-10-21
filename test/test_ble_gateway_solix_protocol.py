"""Select encrypted vs legacy C300 support without mixing their session keys."""
import asyncio
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest

from sirius_navigation.sirius_ble_gateway import SiriusBleGateway


@pytest.mark.parametrize('plaintext', [False, True])
def test_only_legacy_devices_receive_plaintext_protocol_patch(monkeypatch, plaintext):
    modules = {name: ModuleType(name) for name in [
        'bleak', 'bleak.backends', 'bleak.backends.device', 'SolixBLE',
        'SolixBLE.const', 'SolixBLE.device', 'SolixBLE.devices', 'SolixBLE.devices.c300']}
    scanner, device, station, charging, base = (object() for _ in range(5))
    modules['bleak'].BleakScanner = scanner
    modules['bleak.backends.device'].BLEDevice = device
    modules['SolixBLE'].C300 = station
    modules['SolixBLE'].ChargingStatus = charging
    modules['SolixBLE'].const = modules['SolixBLE.const']
    modules['SolixBLE.device'].SolixBLEDevice = base
    modules['SolixBLE.devices.c300'].C300 = station
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    gateway = SimpleNamespace(battery_plaintext=plaintext, get_logger=lambda: Mock(),
                              _patch_bleak_bluez_manager=Mock(), _patch_solix_c300=Mock())
    assert SiriusBleGateway._load_solix_support(gateway) == (station, charging, scanner, device)
    gateway._patch_bleak_bluez_manager.assert_called_once_with()
    if plaintext:
        gateway._patch_solix_c300.assert_called_once_with(modules['SolixBLE'], base, station)
    else:
        gateway._patch_solix_c300.assert_not_called()
    assert modules['SolixBLE.const'].RECONNECT_ATTEMPTS_MAX == 0


def test_scan_before_connect_uses_discovered_device():
    gateway = SimpleNamespace(battery_mac='7C:E9:13:76:E9:58', battery_scan_before_connect=True, get_logger=lambda: Mock())
    scanner = SimpleNamespace(find_device_by_address=AsyncMock(return_value='discovered device'))
    factory = Mock(return_value='station')
    assert asyncio.run(SiriusBleGateway._resolve_and_create_station(gateway, factory, scanner, None)) == 'station'
    factory.assert_called_once_with('discovered device')
    scanner.find_device_by_address.return_value = None
    with pytest.raises(RuntimeError, match='not found'):
        asyncio.run(SiriusBleGateway._resolve_and_create_station(gateway, factory, scanner, None))
