#!/usr/bin/env python3
import asyncio
import json
import math
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


SIRIUS_SERVICE_UUID = "A07498CA-AD5B-474E-940D-16F1F1E0A123"
SIRIUS_CHAR_UUID = "A07498CA-AD5B-474E-940D-16F1F1E0A124"


def _append_face_stubs_dir(stubs_dir: str):
    if stubs_dir and stubs_dir not in sys.path:
        sys.path.append(stubs_dir)


class SiriusBleGateway(Node):
    """Own all Sirius BLE traffic from one ROS 2 process."""

    def __init__(self):
        super().__init__("sirius_ble_gateway")

        self.declare_parameter("enable_remote_server", True)
        self.declare_parameter("enable_battery_client", False)
        self.declare_parameter("battery_mac", "")
        self.declare_parameter("battery_poll_interval", 2.0)
        self.declare_parameter("battery_scan_before_connect", False)
        self.declare_parameter("advertise_name", "SiriusBleBridge")
        self.declare_parameter("service_uuid", SIRIUS_SERVICE_UUID)
        self.declare_parameter("characteristic_uuid", SIRIUS_CHAR_UUID)
        self.declare_parameter("nav_http_target", "http://localhost:50060/instruction")
        self.declare_parameter("face_speak_grpc_target", "localhost:50052")
        self.declare_parameter("face_status_grpc_target", "localhost:50051")
        self.declare_parameter(
            "face_stubs_dir",
            "/home/kotantu-desktop/sirius_face_anim2/scripts/stubs",
        )
        self.declare_parameter("publish_face_battery_params", True)

        self.enable_remote_server = self._bool_param("enable_remote_server")
        self.enable_battery_client = self._bool_param("enable_battery_client")
        self.battery_mac = self._str_param("battery_mac")
        self.battery_poll_interval = self._float_param("battery_poll_interval")
        self.battery_scan_before_connect = self._bool_param("battery_scan_before_connect")
        self.advertise_name = self._str_param("advertise_name")
        self.service_uuid = self._str_param("service_uuid")
        self.characteristic_uuid = self._str_param("characteristic_uuid")
        self.nav_http_target = self._str_param("nav_http_target")
        self.face_speak_grpc_target = self._str_param("face_speak_grpc_target")
        self.face_status_grpc_target = self._str_param("face_status_grpc_target")
        self.face_stubs_dir = self._str_param("face_stubs_dir")
        self.publish_face_battery_params = self._bool_param("publish_face_battery_params")

        _append_face_stubs_dir(self.face_stubs_dir)

        self.remote_command_pub = self.create_publisher(String, "/sirius/remote_command", 10)
        self.remote_status_pub = self.create_publisher(String, "/sirius/remote_status", 10)
        self.battery_json_pub = self.create_publisher(String, "/sirius/battery_status", 10)
        self.battery_state_pub = self.create_publisher(BatteryState, "/battery_state", 10)

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._tasks = []
        self._stopping = threading.Event()

        self._last_face_battery_update = 0.0
        self._last_remote_status = None
        self._last_remote_payload = ""
        self._remote_server = None
        self._battery_device = None

        self._thread.start()
        self._schedule_gateway_tasks()

        self.get_logger().info(
            "Sirius BLE Gateway started "
            f"(remote_server={self.enable_remote_server}, "
            f"battery_client={self.enable_battery_client})"
        )

    def _bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _schedule_gateway_tasks(self):
        if self.enable_remote_server:
            self._tasks.append(
                asyncio.run_coroutine_threadsafe(self._run_remote_ble_server(), self._loop)
            )
        if self.enable_battery_client:
            if not self.battery_mac:
                self.get_logger().error(
                    "enable_battery_client=true but battery_mac is empty. "
                    "Battery BLE client will not start."
                )
            else:
                self._tasks.append(
                    asyncio.run_coroutine_threadsafe(self._run_battery_monitor(), self._loop)
                )

    def destroy_node(self):
        self._stopping.set()
        for task in self._tasks:
            task.cancel()
        if self._remote_server:
            asyncio.run_coroutine_threadsafe(self._stop_remote_server(), self._loop)
        if self._battery_device:
            asyncio.run_coroutine_threadsafe(self._disconnect_battery(), self._loop)
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=3.0)
        super().destroy_node()

    async def _stop_remote_server(self):
        try:
            await self._remote_server.stop()
        except Exception as exc:
            self.get_logger().warning(f"Failed to stop BLE remote server: {exc}")

    async def _disconnect_battery(self):
        try:
            await self._battery_device.disconnect()
        except Exception:
            pass

    async def _run_remote_ble_server(self):
        try:
            from bless import (
                BlessServer,
                GATTAttributePermissions,
                GATTCharacteristicProperties,
            )
        except Exception as exc:
            self.get_logger().error(
                "BLE remote server requires 'bless'. "
                f"Install scripts/requirements_ble.txt dependencies. Error: {exc}"
            )
            return

        server = BlessServer(name=self.advertise_name)
        self._remote_server = server
        server.read_request_func = lambda characteristic, **kwargs: characteristic.value
        server.write_request_func = self._handle_remote_ble_write

        await server.add_new_service(self.service_uuid)
        char_flags = (
            GATTCharacteristicProperties.write
            | GATTCharacteristicProperties.read
            | GATTCharacteristicProperties.write_without_response
        )
        permissions = GATTAttributePermissions.writeable | GATTAttributePermissions.readable
        await server.add_new_characteristic(
            self.service_uuid,
            self.characteristic_uuid,
            char_flags,
            None,
            permissions,
        )

        self.get_logger().info(
            f"Remote BLE server advertising as '{self.advertise_name}' "
            f"service={self.service_uuid}"
        )
        await server.start()
        self._publish_remote_status("advertising", connected=False)

        try:
            while not self._stopping.is_set():
                is_connected = bool(server.is_connected())
                status = "connected" if is_connected else "advertising"
                self._publish_remote_status(status, connected=is_connected)
                await asyncio.sleep(1.0)
        finally:
            await server.stop()
            self._publish_remote_status("stopped", connected=False)

    def _handle_remote_ble_write(self, characteristic, value: bytes, **kwargs):
        characteristic.value = value
        try:
            text = bytes(value).decode("utf-8").strip()
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode BLE remote payload: {exc}")
            return

        if not text:
            return
        if text == "[ping]":
            self.get_logger().debug("Remote BLE ping received")
            self._publish_remote_status("connected", connected=True, last_payload="[ping]")
            return

        self._publish_remote_status("connected", connected=True, last_payload=text)

        msg = String()
        msg.data = text
        self.remote_command_pub.publish(msg)

        if text.startswith("[nav]"):
            instruction = text[len("[nav]"):].strip()
            self.get_logger().info(f"Remote BLE nav command: {instruction}")
            threading.Thread(
                target=self._send_to_nav_http,
                args=(instruction,),
                daemon=True,
            ).start()
            return

        self.get_logger().info(f"Remote BLE speak/chat command: {text}")
        threading.Thread(target=self._send_to_face_speak, args=(text,), daemon=True).start()

    def _send_to_nav_http(self, instruction: str):
        if not instruction:
            return
        try:
            data = json.dumps({"instruction": instruction}).encode("utf-8")
            req = urllib.request.Request(
                self.nav_http_target,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=3.0) as response:
                self.get_logger().info(
                    f"Forwarded nav command via HTTP status={response.status}: {instruction}"
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to forward nav command: {exc}")

    def _send_to_face_speak(self, text: str):
        try:
            import grpc
            import face_control_pb2
            import face_control_pb2_grpc

            with grpc.insecure_channel(self.face_speak_grpc_target) as channel:
                stub = face_control_pb2_grpc.PythonControlServiceStub(channel)
                req = face_control_pb2.SpeakRequest(text=text)
                stub.Speak(req, timeout=3.0)
        except Exception as exc:
            self.get_logger().error(f"Failed to forward speak command to face: {exc}")

    def _publish_remote_status(self, status: str, connected: bool, last_payload: str = ""):
        if last_payload:
            self._last_remote_payload = last_payload[:80]

        data = {
            "status": status,
            "connected": connected,
            "advertise_name": self.advertise_name,
            "service_uuid": self.service_uuid,
            "stamp": time.time(),
        }
        if self._last_remote_payload:
            data["last_payload"] = self._last_remote_payload

        cache_key = (
            data["status"],
            data["connected"],
            data.get("last_payload", ""),
        )
        if cache_key == self._last_remote_status and not last_payload:
            return
        self._last_remote_status = cache_key

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.remote_status_pub.publish(msg)

    async def _run_battery_monitor(self):
        solix = self._load_solix_support()
        if not solix:
            return

        C300, ChargingStatus, BleakScanner, BLEDevice = solix
        self._publish_battery_json({"status": "connecting"})

        while not self._stopping.is_set():
            try:
                if not self._battery_device or not self._battery_device.connected:
                    self._publish_battery_json({"status": "connecting"})
                    if self._battery_device:
                        await self._disconnect_battery()
                    self._battery_device = await self._resolve_and_create_station(
                        C300,
                        BleakScanner,
                        BLEDevice,
                    )
                    self.get_logger().info(f"Connecting to battery BLE: {self.battery_mac}")
                    if not await self._battery_device.connect():
                        self.get_logger().warning("Battery BLE connection failed; retrying")
                        await asyncio.sleep(5.0)
                        continue

                try:
                    await self._battery_device.get_status_update()
                except Exception as exc:
                    self.get_logger().warning(f"Battery polling failed; reconnecting: {exc}")
                    await self._disconnect_battery()
                    self._battery_device = None
                    await asyncio.sleep(2.0)
                    continue

                data = self._build_battery_status(self._battery_device, ChargingStatus)
                self._publish_battery_json(data)
                self._publish_battery_state(data)
                self._update_face_battery_params(data)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                self.get_logger().error(f"Battery monitor error: {exc}")
                await asyncio.sleep(5.0)

            await asyncio.sleep(max(0.5, self.battery_poll_interval))

        self._publish_battery_json({"status": "disconnected"})
        await self._disconnect_battery()

    def _load_solix_support(self):
        try:
            from bleak import BleakScanner
            from bleak.backends.device import BLEDevice
            from SolixBLE import C300, ChargingStatus
            import SolixBLE.const
            from SolixBLE.device import SolixBLEDevice
            from SolixBLE.devices.c300 import C300 as OriginalC300
        except Exception as exc:
            self.get_logger().error(
                "Battery BLE client requires 'bleak' and 'SolixBLE'. "
                f"Error: {exc}"
            )
            return None

        SolixBLE.const.RECONNECT_ATTEMPTS_MAX = 0
        self._patch_bleak_bluez_manager()
        self._patch_solix_c300(SolixBLE, SolixBLEDevice, OriginalC300)
        return C300, ChargingStatus, BleakScanner, BLEDevice

    def _patch_bleak_bluez_manager(self):
        try:
            from bleak.backends.bluezdbus.manager import BlueZManager
        except Exception as exc:
            self.get_logger().debug(f"Bleak BlueZ manager patch skipped: {exc}")
            return

        if getattr(BlueZManager, "_sirius_missing_device_patch_applied", False):
            return

        original_parse_msg = BlueZManager._parse_msg

        def patched_parse_msg(manager_self, message):
            try:
                return original_parse_msg(manager_self, message)
            except KeyError as exc:
                if exc.args == ("Device",):
                    return None
                raise

        BlueZManager._parse_msg = patched_parse_msg
        BlueZManager._sirius_missing_device_patch_applied = True
        self.get_logger().debug("Applied Bleak BlueZ missing Device compatibility patch")

    def _patch_solix_c300(self, SolixBLE, SolixBLEDevice, OriginalC300):
        import hashlib

        if getattr(SolixBLEDevice, "_sirius_c300_patch_applied", False):
            return

        original_reset_session = SolixBLEDevice._reset_session

        def patched_reset_session(device_self, reset_data=True):
            original_reset_session(device_self, reset_data)
            device_self._c300_negotiated = False

        original_encrypt_payload = SolixBLEDevice._encrypt_payload

        def patched_encrypt_payload(device_self, payload: bytes) -> bytes:
            if device_self._shared_secret is None:
                return payload
            return original_encrypt_payload(device_self, payload)

        async def patched_send_encrypted_packet(device_self, cmd: bytes, payload: bytes) -> None:
            packet = device_self._build_packet(bytes.fromhex("03000f"), cmd, payload)
            await device_self._client.write_gatt_char(SolixBLE.const.UUID_COMMAND, packet)

        async def patched_process_telemetry_packet(device_self, payload: bytes, cmd: bytes = None):
            fragment_index = (payload[0] >> 4) & 0x0F
            fragment_total = payload[0] & 0x0F

            if fragment_total > 1:
                fragment_data = payload[1:]
                cmd_key = bytes(cmd)
                if fragment_index > fragment_total:
                    swapped_index = payload[0] & 0x0F
                    swapped_total = (payload[0] >> 4) & 0x0F
                    if swapped_index <= swapped_total and swapped_total > 0:
                        fragment_index = swapped_index
                        fragment_total = swapped_total
                        fragment_data = payload[1:]
                    else:
                        payload = payload[1:]
                        fragment_total = 1

                if cmd_key not in device_self._fragment_buffers or fragment_index == 1:
                    device_self._fragment_buffers[cmd_key] = {}
                    device_self._fragment_totals[cmd_key] = fragment_total

                device_self._fragment_buffers[cmd_key][fragment_index] = fragment_data
                if fragment_total > 1 and len(device_self._fragment_buffers[cmd_key]) < fragment_total:
                    return

                payload = b"".join(
                    device_self._fragment_buffers[cmd_key][i]
                    for i in sorted(device_self._fragment_buffers[cmd_key])
                )
                del device_self._fragment_buffers[cmd_key]
                del device_self._fragment_totals[cmd_key]
            else:
                payload = payload[1:]

            if cmd and cmd.hex() != "8402":
                return
            parameters = device_self._parse_payload(payload)
            return await device_self._process_telemetry(parameters)

        original_process_negotiation = SolixBLEDevice._process_negotiation

        async def patched_process_negotiation(device_self, cmd: bytes, payload: bytes) -> None:
            if cmd.hex() == "0821":
                device_self._c300_negotiated = True
                device_self._shared_secret = hashlib.sha256(b"Solix").digest()
                await device_self._client.write_gatt_char(
                    SolixBLE.const.UUID_COMMAND,
                    bytes.fromhex(SolixBLE.const.NEGOTIATION_COMMAND_5),
                )
                return
            return await original_process_negotiation(device_self, cmd, payload)

        original_process_notification = SolixBLEDevice._process_notification

        async def patched_process_notification(device_self, client, handle, data) -> None:
            if device_self._client is not client:
                return
            try:
                pattern, cmd, payload = device_self._split_packet(data)
                pattern_hex = pattern.hex()
                cmd_hex = cmd.hex()
                if pattern_hex == "030001" and getattr(device_self, "_c300_negotiated", False):
                    return
                if cmd_hex == "8402" or pattern_hex in ("03000f", "030002", "03000c"):
                    return await device_self._process_telemetry_packet(payload, cmd)
                if cmd_hex == "4840":
                    return
            except Exception as exc:
                self.get_logger().debug(f"C300 notification patch fallback: {exc}")
            return await original_process_notification(device_self, client, handle, data)

        async def patched_get_status_update(device_self):
            await device_self._send_command(
                cmd=bytes.fromhex("4040"),
                payload=bytes.fromhex("a10121"),
            )
            return device_self._data or {}

        SolixBLEDevice._reset_session = patched_reset_session
        SolixBLEDevice._encrypt_payload = patched_encrypt_payload
        SolixBLEDevice._send_encrypted_packet = patched_send_encrypted_packet
        SolixBLEDevice._process_telemetry_packet = patched_process_telemetry_packet
        SolixBLEDevice._process_negotiation = patched_process_negotiation
        SolixBLEDevice._process_notification = patched_process_notification
        OriginalC300.get_status_update = patched_get_status_update
        SolixBLEDevice._sirius_c300_patch_applied = True

    async def _resolve_and_create_station(self, C300, BleakScanner, BLEDevice):
        self.get_logger().info(f"Resolving battery BLE device {self.battery_mac}")
        if self.battery_scan_before_connect:
            try:
                ble_device = await BleakScanner.find_device_by_address(
                    self.battery_mac,
                    timeout=10.0,
                )
                if ble_device:
                    return C300(ble_device)
            except Exception as exc:
                self.get_logger().warning(
                    "Battery scan failed; falling back to direct BlueZ path: "
                    f"{exc}"
                )

        self.get_logger().info("Using direct BlueZ path for known battery MAC")
        details = {
            "path": f"/org/bluez/hci0/dev_{self.battery_mac.replace(':', '_')}",
            "props": {
                "Address": self.battery_mac,
                "AddressType": "public",
                "Name": "Anker SOLIX C300",
                "Alias": "Anker SOLIX C300",
                "Paired": False,
                "Bonded": False,
                "Trusted": False,
                "Blocked": False,
                "LegacyPairing": False,
                "RSSI": -99,
                "Connected": False,
                "UUIDs": [],
                "Adapter": "/org/bluez/hci0",
                "ServicesResolved": False,
            },
        }
        ble_device = BLEDevice(
            address=self.battery_mac,
            name="Anker SOLIX C300",
            details=details,
            rssi=-99,
        )
        return C300(ble_device)

    def _build_battery_status(self, power_station, ChargingStatus) -> dict:
        percentage = getattr(power_station, "battery_percentage", -1)
        charging_status = getattr(power_station, "charging_status", None)
        status_str = str(charging_status)
        if "." in status_str:
            status_str = status_str.split(".")[-1]

        is_charging = False
        if charging_status == ChargingStatus.CHARGING:
            is_charging = True
        elif hasattr(ChargingStatus, "BOTH") and charging_status == ChargingStatus.BOTH:
            is_charging = True
        elif type(charging_status).__name__ == "ChargingStatusF3800":
            is_charging = getattr(charging_status, "name", "") == "BOTH"

        time_rem = float(getattr(power_station, "time_remaining", 0.0) or 0.0)
        return {
            "status": "connected" if percentage >= 0 else "connecting",
            "battery_level": float(percentage),
            "charging_status": status_str,
            "is_charging": is_charging,
            "total_input": float(getattr(power_station, "power_in", 0.0) or 0.0),
            "total_output": float(getattr(power_station, "power_out", 0.0) or 0.0),
            "temperature": float(getattr(power_station, "temperature", 0.0) or 0.0),
            "time_remaining": time_rem,
            "time_remaining_str": self._format_remaining_time(time_rem),
            "stamp": time.time(),
        }

    def _format_remaining_time(self, hours_float: float) -> str:
        if hours_float <= 0 or hours_float >= 200:
            return ""
        hours = int(hours_float)
        minutes = int(round((hours_float - hours) * 60))
        if hours > 0 and minutes > 0:
            return f"{hours}時間{minutes}分"
        if hours > 0:
            return f"{hours}時間"
        return f"{minutes}分"

    def _publish_battery_json(self, data: dict):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.battery_json_pub.publish(msg)

    def _publish_battery_state(self, data: dict):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        if data.get("is_charging"):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif data.get("status") == "connected":
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        level = float(data.get("battery_level", -1.0))
        msg.percentage = level / 100.0 if level >= 0 else math.nan
        msg.voltage = math.nan
        msg.current = math.nan
        msg.temperature = float(data.get("temperature", math.nan))
        msg.present = data.get("status") == "connected"
        self.battery_state_pub.publish(msg)

    def _update_face_battery_params(self, data: dict):
        if not self.publish_face_battery_params or data.get("status") != "connected":
            return
        now = time.time()
        if now - self._last_face_battery_update < 2.0:
            return
        self._last_face_battery_update = now

        def worker():
            try:
                import grpc
                import face_control_pb2
                import face_control_pb2_grpc

                charging_value = 1.0 if data.get("is_charging") else 2.0
                with grpc.insecure_channel(self.face_status_grpc_target) as channel:
                    stub = face_control_pb2_grpc.FaceServiceStub(channel)
                    stub.UpdateParameters(
                        face_control_pb2.ParameterRequest(
                            values={
                                "batteryStatus": 1.0,
                                "batteryLevel": float(data.get("battery_level", -1.0)),
                                "batteryCharging": charging_value,
                                "batteryInput": float(data.get("total_input", 0.0)),
                                "batteryOutput": float(data.get("total_output", 0.0)),
                                "batteryTemperature": float(data.get("temperature", 0.0)),
                                "batteryTimeRemaining": float(data.get("time_remaining", 0.0)),
                            }
                        ),
                        timeout=1.0,
                    )
            except Exception as exc:
                self.get_logger().debug(f"Failed to update face battery params: {exc}")

        threading.Thread(target=worker, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = SiriusBleGateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
