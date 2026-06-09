#!/usr/bin/env python3
import json
import signal
import sys
import time

try:
    from PySide6.QtCore import QProcess, QThread, QTimer, Signal, Slot
    from PySide6.QtGui import QColor, QTextCharFormat, QTextCursor
    from PySide6.QtWidgets import (
        QApplication,
        QComboBox,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except Exception:
    from PyQt5.QtCore import QProcess, QThread, QTimer, pyqtSignal as Signal, pyqtSlot as Slot
    from PyQt5.QtGui import QColor, QTextCharFormat, QTextCursor
    from PyQt5.QtWidgets import (
        QApplication,
        QComboBox,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


BATTERY_OPTIONS = [
    ("RemoteControllerのみ", ""),
    ("1台目  F4:9D:8A:7E:97:24", "F4:9D:8A:7E:97:24"),
    ("2台目  F4:9D:8A:57:90:E8", "F4:9D:8A:57:90:E8"),
]


class BatteryStatusNode(Node):
    def __init__(self, battery_callback, remote_callback):
        super().__init__("sirius_ble_gateway_ui_status")
        self.create_subscription(String, "/sirius/battery_status", self._on_status, 10)
        self.create_subscription(String, "/sirius/remote_status", self._on_remote_status, 10)
        self._battery_callback = battery_callback
        self._remote_callback = remote_callback

    def _on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"status": "invalid", "raw": msg.data}
        self._battery_callback(data)

    def _on_remote_status(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"status": "invalid", "raw": msg.data}
        self._remote_callback(data)


class RosStatusThread(QThread):
    battery_status_received = Signal(dict)
    remote_status_received = Signal(dict)
    ros_error = Signal(str)

    def __init__(self):
        super().__init__()
        self._executor = None
        self._node = None

    def run(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = BatteryStatusNode(
                self.battery_status_received.emit,
                self.remote_status_received.emit,
            )
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._executor.spin()
        except Exception as exc:
            self.ros_error.emit(str(exc))
        finally:
            if self._executor:
                self._executor.shutdown()
            if self._node:
                self._node.destroy_node()

    def stop(self):
        if self._executor:
            self._executor.shutdown()
        self.wait(1500)


class SiriusBleGatewayWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sirius BLE Gateway")
        self.resize(860, 620)

        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read_process_output)
        self.process.started.connect(self._on_process_started)
        self.process.finished.connect(self._on_process_finished)
        self.process.errorOccurred.connect(self._on_process_error)

        self.last_status_time = None
        self.last_status_data = {}
        self.last_remote_status_time = None
        self.last_remote_status_data = {}

        self.ros_thread = RosStatusThread()
        self.ros_thread.battery_status_received.connect(self._update_battery_status)
        self.ros_thread.remote_status_received.connect(self._update_remote_status)
        self.ros_thread.ros_error.connect(lambda text: self._append_log(f"[ROS UI] {text}", "warn"))
        self.ros_thread.start()

        self._build_ui()

        self.stale_timer = QTimer(self)
        self.stale_timer.timeout.connect(self._update_stale_state)
        self.stale_timer.start(1000)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        controls_box = QGroupBox("接続")
        controls = QGridLayout(controls_box)

        self.battery_combo = QComboBox()
        for label, mac in BATTERY_OPTIONS:
            self.battery_combo.addItem(label, mac)

        self.start_button = QPushButton("起動")
        self.stop_button = QPushButton("停止")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_gateway)
        self.stop_button.clicked.connect(self.stop_gateway)

        self.gateway_state = QLabel("停止中")
        self.gateway_state.setStyleSheet("font-weight: bold; color: #a33;")

        controls.addWidget(QLabel("バッテリー接続先"), 0, 0)
        controls.addWidget(self.battery_combo, 0, 1)
        controls.addWidget(self.start_button, 0, 2)
        controls.addWidget(self.stop_button, 0, 3)
        controls.addWidget(QLabel("Gateway"), 1, 0)
        controls.addWidget(self.gateway_state, 1, 1, 1, 3)
        controls.setColumnStretch(1, 1)
        layout.addWidget(controls_box)

        remote_box = QGroupBox("RemoteController")
        remote = QGridLayout(remote_box)
        self.remote_labels = {}
        remote_rows = [
            ("status", "接続状態"),
            ("advertise_name", "広告名"),
            ("service_uuid", "Service UUID"),
            ("last_payload", "最終受信"),
            ("last_update", "最終更新"),
        ]
        for row, (key, label) in enumerate(remote_rows):
            name = QLabel(label)
            value = QLabel("-")
            value.setStyleSheet("font-weight: bold;")
            self.remote_labels[key] = value
            remote.addWidget(name, row, 0)
            remote.addWidget(value, row, 1)
        remote.setColumnStretch(1, 1)
        layout.addWidget(remote_box)

        status_box = QGroupBox("バッテリー状態")
        status = QGridLayout(status_box)
        self.status_labels = {}
        rows = [
            ("status", "接続状態"),
            ("battery_level", "残量"),
            ("charging_status", "充電状態"),
            ("total_input", "入力"),
            ("total_output", "出力"),
            ("temperature", "温度"),
            ("time_remaining_str", "残り時間"),
            ("last_update", "最終更新"),
        ]
        for row, (key, label) in enumerate(rows):
            name = QLabel(label)
            value = QLabel("-")
            value.setStyleSheet("font-weight: bold;")
            self.status_labels[key] = value
            status.addWidget(name, row, 0)
            status.addWidget(value, row, 1)
        status.setColumnStretch(1, 1)
        layout.addWidget(status_box)

        log_header = QHBoxLayout()
        log_header.addWidget(QLabel("ログ"))
        clear_button = QPushButton("クリア")
        clear_button.clicked.connect(self._clear_log)
        log_header.addStretch(1)
        log_header.addWidget(clear_button)
        layout.addLayout(log_header)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("font-family: monospace;")
        layout.addWidget(self.log_view, 1)

        self._append_log("GUIを起動しました。接続先を選んで「起動」を押してください。", "info")

    @Slot()
    def start_gateway(self):
        if self.process.state() != QProcess.NotRunning:
            QMessageBox.information(self, "起動中", "BLE Gatewayはすでに起動中です。")
            return

        battery_mac = self.battery_combo.currentData()
        enable_battery = "true" if battery_mac else "false"
        args = [
            "launch",
            "sirius_navigation",
            "sirius_ble_gateway.launch.py",
            "enable_remote_server:=true",
            f"enable_battery_client:={enable_battery}",
            "battery_scan_before_connect:=false",
            "publish_face_battery_params:=true",
        ]
        if battery_mac:
            args.append(f"battery_mac:={battery_mac}")

        self._append_log("起動コマンド: ros2 " + " ".join(args), "info")
        self.process.start("ros2", args)

    @Slot()
    def stop_gateway(self):
        if self.process.state() == QProcess.NotRunning:
            return
        self._append_log("BLE Gatewayを停止します。", "warn")
        self.process.terminate()
        QTimer.singleShot(3000, self._kill_if_running)

    def _kill_if_running(self):
        if self.process.state() != QProcess.NotRunning:
            self._append_log("通常停止できなかったため強制終了します。", "warn")
            self.process.kill()

    @Slot()
    def _on_process_started(self):
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.battery_combo.setEnabled(False)
        self.gateway_state.setText("起動中")
        self.gateway_state.setStyleSheet("font-weight: bold; color: #286b2d;")

    @Slot(int, QProcess.ExitStatus)
    def _on_process_finished(self, code, status):
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.battery_combo.setEnabled(True)
        self.gateway_state.setText(f"停止中 (code={code})")
        self.gateway_state.setStyleSheet("font-weight: bold; color: #a33;")
        if status == QProcess.CrashExit:
            self._append_log("BLE Gatewayがクラッシュ終了しました。", "error")

    @Slot(QProcess.ProcessError)
    def _on_process_error(self, error):
        self._append_log(f"プロセスエラー: {error}", "error")

    @Slot()
    def _read_process_output(self):
        text = bytes(self.process.readAllStandardOutput()).decode(errors="replace")
        for line in text.splitlines():
            kind = "info"
            if "[ERROR]" in line or "Traceback" in line or "RuntimeError" in line:
                kind = "error"
            elif "[WARN]" in line or "warning" in line.lower():
                kind = "warn"
            self._append_log(line, kind)

    @Slot(dict)
    def _update_battery_status(self, data):
        self.last_status_time = time.time()
        self.last_status_data = data
        status = data.get("status", "-")
        self.status_labels["status"].setText(str(status))
        self.status_labels["battery_level"].setText(self._fmt_percent(data.get("battery_level")))
        self.status_labels["charging_status"].setText(str(data.get("charging_status", "-")))
        self.status_labels["total_input"].setText(self._fmt_watt(data.get("total_input")))
        self.status_labels["total_output"].setText(self._fmt_watt(data.get("total_output")))
        self.status_labels["temperature"].setText(self._fmt_temp(data.get("temperature")))
        self.status_labels["time_remaining_str"].setText(str(data.get("time_remaining_str") or "-"))
        self.status_labels["last_update"].setText(time.strftime("%H:%M:%S"))

        if status == "connected":
            self.status_labels["status"].setStyleSheet("font-weight: bold; color: #286b2d;")
        elif status == "connecting":
            self.status_labels["status"].setStyleSheet("font-weight: bold; color: #b36b00;")
        else:
            self.status_labels["status"].setStyleSheet("font-weight: bold; color: #a33;")

    @Slot(dict)
    def _update_remote_status(self, data):
        self.last_remote_status_time = time.time()
        self.last_remote_status_data = data
        status = data.get("status", "-")
        self.remote_labels["status"].setText(str(status))
        self.remote_labels["advertise_name"].setText(str(data.get("advertise_name", "-")))
        self.remote_labels["service_uuid"].setText(str(data.get("service_uuid", "-")))
        self.remote_labels["last_payload"].setText(str(data.get("last_payload", "-")))
        self.remote_labels["last_update"].setText(time.strftime("%H:%M:%S"))

        if data.get("connected"):
            self.remote_labels["status"].setStyleSheet("font-weight: bold; color: #286b2d;")
        elif status == "advertising":
            self.remote_labels["status"].setStyleSheet("font-weight: bold; color: #b36b00;")
        else:
            self.remote_labels["status"].setStyleSheet("font-weight: bold; color: #a33;")

    def _update_stale_state(self):
        if self.last_status_time is None:
            self.status_labels["last_update"].setText("未受信")
        else:
            age = time.time() - self.last_status_time
            if age > 6.0:
                self.status_labels["last_update"].setText(f"{int(age)}秒前")

        if self.last_remote_status_time is None:
            self.remote_labels["last_update"].setText("未受信")
            return
        age = time.time() - self.last_remote_status_time
        if age > 6.0:
            self.remote_labels["last_update"].setText(f"{int(age)}秒前")

    def _append_log(self, text, kind="info"):
        colors = {
            "info": QColor("#e8e8e8"),
            "warn": QColor("#ffd27d"),
            "error": QColor("#ff8f8f"),
        }
        fmt = QTextCharFormat()
        fmt.setForeground(colors.get(kind, colors["info"]))
        self.log_view.setCurrentCharFormat(fmt)
        self.log_view.append(text)
        self.log_view.moveCursor(QTextCursor.End)

    def _clear_log(self):
        self.log_view.clear()

    @staticmethod
    def _fmt_percent(value):
        try:
            return f"{float(value):.0f}%"
        except (TypeError, ValueError):
            return "-"

    @staticmethod
    def _fmt_watt(value):
        try:
            return f"{float(value):.1f} W"
        except (TypeError, ValueError):
            return "-"

    @staticmethod
    def _fmt_temp(value):
        try:
            return f"{float(value):.1f} ℃"
        except (TypeError, ValueError):
            return "-"

    def closeEvent(self, event):
        if self.process.state() != QProcess.NotRunning:
            self.stop_gateway()
            self.process.waitForFinished(1500)
        self.ros_thread.stop()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    app.setApplicationName("Sirius BLE Gateway")
    window = SiriusBleGatewayWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
