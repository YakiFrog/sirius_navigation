# -*- coding: utf-8 -*-
"""Paired LAN WebSocket transport for the Sirius remote controller."""

import asyncio
import json
import queue
import secrets
import threading
import time
from dataclasses import dataclass
from typing import Optional

from aiohttp import WSMsgType, web
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String

try:
    from .navigation_modes import navigation_mode_controller, normalize_navigation_mode
except ImportError:  # pragma: no cover - direct script execution
    from navigation_modes import navigation_mode_controller, normalize_navigation_mode


@dataclass
class PairingAuthority:
    """Own a short-lived one-time code and one controller token."""

    ttl_seconds: float = 300.0
    code: str = ""
    expires_at: float = 0.0
    token: str = ""

    def issue_code(self, now: Optional[float] = None) -> str:
        timestamp = time.time() if now is None else now
        self.code = f"{secrets.randbelow(1_000_000):06d}"
        self.expires_at = timestamp + self.ttl_seconds
        return self.code

    def pair(self, candidate: str, now: Optional[float] = None) -> Optional[str]:
        timestamp = time.time() if now is None else now
        if not self.code or timestamp > self.expires_at:
            return None
        if not secrets.compare_digest(str(candidate), self.code):
            return None
        self.token = secrets.token_urlsafe(32)
        self.code = ""
        self.expires_at = 0.0
        return self.token

    def authenticate(self, candidate: str) -> bool:
        return bool(self.token and secrets.compare_digest(str(candidate), self.token))


def parse_manual_motion(payload: str):
    """Return (moving, assisted) for a manual teleop payload, otherwise None."""
    if not payload.startswith("[nav]"):
        return None
    try:
        data = json.loads(payload[len("[nav]"):].strip())
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if data.get("type") != "manual_teleop":
        return None
    linear = float(data.get("linear", 0.0) or 0.0)
    angular = float(data.get("angular", 0.0) or 0.0)
    assisted = bool(data.get("assisted", True))
    return abs(linear) > 0.001 or abs(angular) > 0.001, assisted


def build_manual_stop_payload(assisted: bool = True) -> str:
    data = {
        "type": "manual_teleop",
        "linear": 0.0,
        "angular": 0.0,
        "assisted": bool(assisted),
    }
    return "[nav]" + json.dumps(data, separators=(",", ":"))


class SiriusNetworkGateway(Node):
    """Bridge an authenticated WebSocket controller into the BLE gateway topic."""

    def __init__(self):
        super().__init__("sirius_network_gateway")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 8766)
        self.declare_parameter("pairing_ttl", 300.0)
        self.declare_parameter("deadman_timeout", 0.45)

        self.listen_host = str(self.get_parameter("listen_host").value)
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.deadman_timeout = max(
            0.2,
            float(self.get_parameter("deadman_timeout").value),
        )
        pairing_ttl = max(30.0, float(self.get_parameter("pairing_ttl").value))

        self.command_pub = self.create_publisher(String, "/sirius/remote_command", 10)
        self.network_status_pub = self.create_publisher(
            String,
            "/sirius/network_remote_status",
            10,
        )
        self.create_subscription(String, "/sirius/remote_status", self._on_remote_status, 10)
        self.create_subscription(String, "/sirius/battery_status", self._on_battery, 10)
        navigation_mode_qos = QoSProfile(depth=1)
        navigation_mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        navigation_mode_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            String,
            "/sirius/navigation_mode",
            self._on_navigation_mode,
            navigation_mode_qos,
        )
        self.create_subscription(
            String,
            "/controller_selector",
            self._on_controller_selector,
            navigation_mode_qos,
        )
        self.create_subscription(Bool, "/stop", self._on_stop, 10)
        self.create_subscription(
            Int32,
            "/target_detector/people_count",
            self._on_people_count,
            10,
        )
        self.create_subscription(
            Int32,
            "/target_detector/tracked_people_count",
            self._on_tracked_people_count,
            10,
        )

        self._pairing = PairingAuthority(ttl_seconds=pairing_ttl)
        self._pairing_lock = threading.Lock()
        self._inbound_commands = queue.SimpleQueue()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_server_thread, daemon=True)
        self._runner = None
        self._active_socket = None
        self._active_controller = ""
        self._last_status = {}
        self._battery = None
        self._navigation_mode = "unknown"
        self._controller_selector = ""
        self._emergency_stop = False
        self._stop_state_received = False
        self._people_count = None
        self._tracked_people_count = None
        self._people_count_at = 0.0
        self._last_manual_at = 0.0
        self._manual_is_moving = False
        self._last_assisted = True
        self._stopping = threading.Event()
        self._deadman_task = None

        self.create_timer(0.02, self._drain_commands)
        self.create_timer(1.0, self._publish_status)
        code = self._pairing.issue_code()
        self._log_pairing_code(code)
        self._thread.start()

    def _log_pairing_code(self, code: str):
        self.get_logger().info("=" * 56)
        self.get_logger().info(f"SIRIUS NETWORK PAIRING CODE: {code}")
        self.get_logger().info(
            f"Expires in {int(self._pairing.ttl_seconds)} seconds; "
            "enter it in the remote controller."
        )
        self.get_logger().info("=" * 56)

    def _ensure_pairing_code(self):
        with self._pairing_lock:
            if self._pairing.token:
                return
            if self._pairing.code and time.time() <= self._pairing.expires_at:
                return
            code = self._pairing.issue_code()
        self._log_pairing_code(code)

    def _run_server_thread(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._start_server())
        self._deadman_task = self._loop.create_task(self._deadman_loop())
        try:
            self._loop.run_forever()
        finally:
            self._loop.run_until_complete(self._stop_server())
            self._loop.close()

    async def _start_server(self):
        app = web.Application(client_max_size=128 * 1024)
        app.router.add_get("/ws", self._handle_websocket)
        app.router.add_get("/health", self._handle_health)
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        site = web.TCPSite(self._runner, self.listen_host, self.listen_port)
        await site.start()
        self.get_logger().info(
            f"Network remote gateway listening on ws://{self.listen_host}:{self.listen_port}/ws"
        )

    async def _stop_server(self):
        if self._deadman_task:
            self._deadman_task.cancel()
            try:
                await self._deadman_task
            except asyncio.CancelledError:
                pass
        if self._runner:
            await self._runner.cleanup()

    async def _handle_health(self, _request):
        return web.json_response({
            "service": "sirius-network-gateway",
            "status": "ok",
            "paired": bool(self._pairing.token),
            "controller_connected": self._active_socket is not None,
        })

    async def _handle_websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=15.0, receive_timeout=45.0)
        await ws.prepare(request)
        authenticated = False
        controller_name = ""
        failed_pairing_attempts = 0

        await ws.send_json({"type": "hello", "requires_pairing": True})
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    if message.type in (WSMsgType.ERROR, WSMsgType.CLOSE):
                        break
                    continue
                try:
                    data = json.loads(message.data)
                except (TypeError, ValueError, json.JSONDecodeError):
                    await ws.send_json({"type": "error", "code": "invalid_json"})
                    continue

                msg_type = data.get("type")
                if not authenticated and msg_type == "pair":
                    with self._pairing_lock:
                        token = self._pairing.pair(str(data.get("code", "")))
                    if not token:
                        failed_pairing_attempts += 1
                        await ws.send_json({"type": "error", "code": "pairing_failed"})
                        if failed_pairing_attempts >= 5:
                            await ws.close(code=4004, message=b"too many pairing attempts")
                            break
                        continue
                    authenticated = True
                    controller_name = str(data.get("controller", "browser"))[:64]
                    await ws.send_json({"type": "paired", "token": token})
                elif not authenticated and msg_type == "auth":
                    with self._pairing_lock:
                        token_is_valid = self._pairing.authenticate(str(data.get("token", "")))
                    if not token_is_valid:
                        await ws.send_json({"type": "error", "code": "auth_failed"})
                        continue
                    authenticated = True
                    controller_name = str(data.get("controller", "browser"))[:64]
                    await ws.send_json({"type": "authenticated"})
                elif not authenticated:
                    await ws.send_json({"type": "error", "code": "authentication_required"})
                    continue

                if self._active_socket not in (None, ws):
                    await ws.send_json({"type": "error", "code": "controller_busy"})
                    await ws.close(code=4003, message=b"controller busy")
                    break

                self._active_socket = ws
                self._active_controller = controller_name
                if msg_type in ("pair", "auth"):
                    await self._send_status(ws)
                elif msg_type == "ping":
                    await ws.send_json({"type": "pong", "stamp": time.time()})
                    await self._send_status(ws)
                elif msg_type == "command":
                    payload = str(data.get("payload", ""))
                    if not payload or len(payload.encode("utf-8")) > 65536:
                        await ws.send_json({"type": "error", "code": "invalid_payload"})
                        continue
                    self._track_manual_payload(payload)
                    self._inbound_commands.put(payload)
                    await ws.send_json({
                        "type": "ack",
                        "seq": data.get("seq"),
                    })
        finally:
            if self._active_socket is ws:
                self._active_socket = None
                self._active_controller = ""
                self._queue_manual_stop("controller disconnected")
        return ws

    def _track_manual_payload(self, payload: str):
        manual = parse_manual_motion(payload)
        if manual is None:
            return
        moving, assisted = manual
        self._last_manual_at = time.monotonic()
        self._manual_is_moving = moving
        self._last_assisted = assisted

    def _queue_manual_stop(self, reason: str):
        if not self._manual_is_moving:
            return
        self._manual_is_moving = False
        self._inbound_commands.put(build_manual_stop_payload(self._last_assisted))
        self.get_logger().warning(f"Network remote deadman stop: {reason}")

    async def _deadman_loop(self):
        while not self._stopping.is_set():
            if (
                self._manual_is_moving
                and time.monotonic() - self._last_manual_at > self.deadman_timeout
            ):
                self._queue_manual_stop("manual command timeout")
            await asyncio.sleep(0.05)

    def _drain_commands(self):
        for _ in range(64):
            try:
                payload = self._inbound_commands.get_nowait()
            except queue.Empty:
                break
            msg = String()
            msg.data = payload
            self.command_pub.publish(msg)

    def _on_remote_status(self, msg: String):
        try:
            self._last_status = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._last_status = {"raw": msg.data}
        self._broadcast_status()

    def _on_battery(self, msg: String):
        try:
            self._battery = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self._broadcast_status()

    def _on_navigation_mode(self, msg: String):
        mode = str(msg.data or "").strip()
        if mode:
            self._navigation_mode = mode
            self._broadcast_status()

    def _on_controller_selector(self, msg: String):
        controller = str(msg.data or "").strip()
        if controller:
            self._controller_selector = controller
            self._broadcast_status()

    @staticmethod
    def _effective_navigation_mode(requested, controller_selector):
        """Return the mode implied by the currently active controller.

        The behavior tree resets its controller selection to the default
        (FollowPath) whenever it is rebuilt, while /sirius/navigation_mode
        keeps the last requested value. Deriving the effective mode from the
        live /controller_selector keeps the remote display consistent with the
        robot's actual behaviour.
        """
        requested_mode = normalize_navigation_mode(requested) or "normal"
        if not controller_selector:
            return requested_mode
        if controller_selector == navigation_mode_controller(requested_mode):
            return requested_mode
        # Selection diverged from the requested mode: trust the live controller.
        return "wait_normal" if controller_selector == "WaitPath" else "normal"

    def _on_stop(self, msg: Bool):
        self._emergency_stop = bool(msg.data)
        self._stop_state_received = True
        self._broadcast_status()

    def _on_people_count(self, msg: Int32):
        people_count = max(0, int(msg.data))
        changed = people_count != self._people_count
        self._people_count = people_count
        self._people_count_at = time.monotonic()
        if changed:
            self._broadcast_status()

    def _on_tracked_people_count(self, msg: Int32):
        tracked_people_count = max(0, int(msg.data))
        changed = tracked_people_count != self._tracked_people_count
        self._tracked_people_count = tracked_people_count
        self._people_count_at = time.monotonic()
        if changed:
            self._broadcast_status()

    def _status_payload(self):
        battery = self._battery or self._last_status.get("battery")
        emergency_stop = (
            self._emergency_stop
            if self._stop_state_received
            else bool(self._last_status.get("emergency_stop", False))
        )
        with self._pairing_lock:
            pairing_code = self._pairing.code
            pairing_expires_in = max(0, int(self._pairing.expires_at - time.time()))
            paired = bool(self._pairing.token)
        people_detection_active = (
            self._people_count_at > 0.0
            and time.monotonic() - self._people_count_at <= 3.0
        )
        navigation_mode = self._navigation_mode
        if navigation_mode == "unknown":
            navigation_mode = self._last_status.get("navigation_mode", "unknown")
        controller_selector = getattr(self, '_controller_selector', "") or self._last_status.get(
            "controller_selector", ""
        )
        navigation_mode_effective = self._effective_navigation_mode(
            navigation_mode, controller_selector
        )
        return {
            "type": "status",
            "status": "connected" if self._active_socket else "waiting",
            "transport": "network",
            "active": self._active_socket is not None,
            "controller": self._active_controller,
            "paired": paired,
            "pairing_code": pairing_code,
            "pairing_expires_in": pairing_expires_in if pairing_code else 0,
            "emergency_stop": emergency_stop,
            "battery": battery,
            "navigation_mode": navigation_mode,
            "navigation_mode_effective": navigation_mode_effective,
            "controller_selector": controller_selector,
            "people_detection_active": people_detection_active,
            "people_count": (
                self._people_count if people_detection_active else None
            ),
            "tracked_people_count": (
                self._tracked_people_count if people_detection_active else None
            ),
            "stamp": time.time(),
        }

    async def _send_status(self, ws):
        if ws and not ws.closed:
            await ws.send_json(self._status_payload())

    async def _broadcast_status_async(self):
        await self._send_status(self._active_socket)

    def _broadcast_status(self):
        if self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast_status_async(), self._loop)

    def _publish_status(self):
        self._ensure_pairing_code()
        msg = String()
        msg.data = json.dumps(self._status_payload(), ensure_ascii=False)
        self.network_status_pub.publish(msg)

    def destroy_node(self):
        self._stopping.set()
        self._queue_manual_stop("gateway shutdown")
        self._drain_commands()
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SiriusNetworkGateway()
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
