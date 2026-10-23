"""A surviving BlueZ connection must not require a new advertisement."""
import asyncio
import sys
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

from sirius_navigation.sirius_ble_gateway import SiriusBleGateway as Gateway


MAC = "7C:E9:13:76:E9:58"


def gateway():
    node = SimpleNamespace(battery_mac=MAC, battery_scan_before_connect=True,
                           get_logger=Mock(return_value=Mock()))
    node._find_connected_battery = AsyncMock(return_value=None)
    return node


def test_connected_device_bypasses_advertisement_scan():
    node = gateway()
    device = object()
    node._find_connected_battery.return_value = device
    scanner = SimpleNamespace(find_device_by_address=AsyncMock())
    station = Mock()
    asyncio.run(Gateway._resolve_and_create_station(node, station, scanner, Mock()))
    station.assert_called_once_with(device)
    scanner.find_device_by_address.assert_not_called()


def test_disconnected_device_uses_scan():
    node = gateway()
    scanner = SimpleNamespace(find_device_by_address=AsyncMock(return_value=object()))
    station = Mock()
    asyncio.run(Gateway._resolve_and_create_station(node, station, scanner, Mock()))
    scanner.find_device_by_address.assert_awaited_once_with(MAC, timeout=10.0)
    station.assert_called_once_with(scanner.find_device_by_address.return_value)


def test_bluez_failure_falls_back_to_scan():
    node = gateway()
    node._find_connected_battery.side_effect = RuntimeError("unavailable")
    scanner = SimpleNamespace(find_device_by_address=AsyncMock(return_value=object()))
    asyncio.run(Gateway._resolve_and_create_station(node, Mock(), scanner, Mock()))
    scanner.find_device_by_address.assert_awaited_once()


def test_lookup_matches_connected_address_and_preserves_real_properties():
    def props(address, connected):
        return {key: SimpleNamespace(value=value) for key, value in {
            "Address": address, "Connected": connected, "Name": "C300",
            "Alias": "C300", "Adapter": "/org/bluez/hci1",
        }.items()}
    objects = {
        "/wrong": {"org.bluez.Device1": props("00:00:00:00:00:00", True)},
        "/disconnected": {"org.bluez.Device1": props(MAC, False)},
        "/actual": {"org.bluez.Device1": props(MAC.lower(), True)},
    }
    reply = SimpleNamespace(message_type="ok", body=[objects])
    bus = Mock(call=AsyncMock(return_value=reply))
    factory = Mock(return_value=SimpleNamespace(connect=AsyncMock(return_value=bus)))
    dbus = SimpleNamespace(BusType=SimpleNamespace(SYSTEM=1), Message=Mock(),
                           MessageType=SimpleNamespace(ERROR="error"))
    device_factory = Mock()
    with patch.dict(sys.modules, {"dbus_fast": dbus,
                                 "dbus_fast.aio": SimpleNamespace(MessageBus=factory)}):
        with patch.object(sys, "platform", "linux"):
            asyncio.run(Gateway._find_connected_battery(gateway(), device_factory))
    details = device_factory.call_args.kwargs["details"]
    assert details["path"] == "/actual"
    assert details["props"]["Adapter"] == "/org/bluez/hci1"
    bus.disconnect.assert_called_once()


def test_lookup_disconnects_its_dbus_client_on_error():
    bus = Mock(call=AsyncMock(side_effect=RuntimeError("D-Bus failed")))
    factory = Mock(return_value=SimpleNamespace(connect=AsyncMock(return_value=bus)))
    dbus = SimpleNamespace(BusType=SimpleNamespace(SYSTEM=1), Message=Mock(),
                           MessageType=SimpleNamespace(ERROR="error"))
    with patch.dict(sys.modules, {"dbus_fast": dbus,
                                 "dbus_fast.aio": SimpleNamespace(MessageBus=factory)}):
        with patch.object(sys, "platform", "linux"):
            try:
                asyncio.run(Gateway._find_connected_battery(gateway(), Mock()))
            except RuntimeError as exc:
                assert str(exc) == "D-Bus failed"
            else:
                raise AssertionError("Expected lookup failure")
    bus.disconnect.assert_called_once()
