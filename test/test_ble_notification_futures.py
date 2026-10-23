"""Late/duplicate BLE responses must not complete a future twice."""
import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

from sirius_navigation.sirius_ble_gateway import SiriusBleGateway as Gateway


def test_completed_and_cancelled_waiters_do_not_block_live_waiters():
    async def run():
        class Device:
            _process_notification = AsyncMock()
        original = Device._process_notification
        Gateway._patch_solix_notification_futures(None, Device)
        Gateway._patch_solix_notification_futures(None, Device)  # idempotent
        client = object()
        completed, cancelled, pending = [asyncio.get_running_loop().create_future() for _ in range(3)]
        completed.set_result(b"old")
        cancelled.cancel()
        device = Device()
        device._client = client
        device._split_packet = Mock(return_value=(b"pattern", b"cmd", b"new"))
        device._packet_futures = {b"patterncmd": [completed, cancelled, pending]}
        await device._process_notification(client, 1, bytearray())
        await device._process_notification(client, 1, bytearray())  # duplicate
        assert completed.result() == b"old"
        assert cancelled.cancelled()
        assert pending.result() == b"new"
        original.assert_not_awaited()
        assert device._last_packet_timestamp > 0
    asyncio.run(run())


def test_unsolicited_packets_reach_original_parser_and_old_clients_are_ignored():
    async def run():
        class Device:
            _process_notification = AsyncMock()
        original = Device._process_notification
        Gateway._patch_solix_notification_futures(None, Device)
        client = object()
        device = SimpleNamespace(_client=client, _packet_futures={},
                                 _split_packet=Mock(return_value=(b"p", b"c", b"data")))
        await Device._process_notification(device, object(), 1, bytearray())
        device._split_packet.assert_not_called()
        await Device._process_notification(device, client, 1, bytearray())
        original.assert_awaited_once_with(device, client, 1, bytearray())
    asyncio.run(run())


def test_old_encrypted_telemetry_waits_for_new_session_key():
    async def run():
        class Device:
            _process_notification = AsyncMock()
        original = Device._process_notification
        Gateway._patch_solix_notification_futures(None, Device)
        client = object()
        device = SimpleNamespace(_client=client, _packet_futures={}, _shared_secret=None,
                                 _split_packet=Mock(return_value=(bytes.fromhex('03010f'),
                                                                 bytes.fromhex('c402'), b'ciphertext')))
        await Device._process_notification(device, client, 1, bytearray())
        original.assert_not_awaited()
        device._shared_secret = b'new key'
        await Device._process_notification(device, client, 1, bytearray())
        original.assert_awaited_once()
    asyncio.run(run())
