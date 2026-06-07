# -*- coding: utf-8 -*-
import os
import sys
import time
import logging

class FaceClient:
    """sirius_face_anim2 の gRPC サーバーと通信するクライアントラッパー。
    サーバーが起動していない場合でもプロセスが遅延しないように、接続状態をキャッシュして制御します。
    """
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger("FaceClient")
        self.face_server_active = True
        self.last_connect_retry = 0.0
        self.retry_interval = 30.0  # 切断時の再試行インターバル(秒)
        self._init_paths()

    def _init_paths(self):
        home_dir = os.path.expanduser("~")
        venv_packages = os.path.join(home_dir, "sirius_face_anim2/venv/lib/python3.12/site-packages")
        stubs_path = os.path.join(home_dir, "sirius_face_anim2/scripts/stubs")
        if venv_packages not in sys.path:
            sys.path.insert(0, venv_packages)
        if stubs_path not in sys.path:
            sys.path.insert(0, stubs_path)

    def _should_attempt_connection(self):
        if self.face_server_active:
            return True
        now = time.time()
        if now - self.last_connect_retry >= self.retry_interval:
            self.last_connect_retry = now
            return True
        return False

    def send_speak(self, text):
        """指定したテキストを喋らせる (gRPC ポート 50052)"""
        print(f"Command > [Speech] {text}")
        if not self._should_attempt_connection():
            return False

        try:
            import grpc
            import face_control_pb2
            import face_control_pb2_grpc

            # 表情・パース制御のフラグをリセット (50051)
            self.reset_indicators()

            with grpc.insecure_channel('localhost:50052') as channel:
                stub = face_control_pb2_grpc.PythonControlServiceStub(channel)
                req = face_control_pb2.SpeakRequest(text=text)
                stub.Speak(req, timeout=3.0)
                self.face_server_active = True
                return True
        except Exception as e:
            if self.face_server_active:
                self.logger.warning(f"Face speech server offline (gRPC port 50052): {e}")
                self.face_server_active = False
                self.last_connect_retry = time.time()
            return False

    def set_expression(self, expression_state):
        """表情を変更する (gRPC ポート 50051)"""
        print(f"Command > [Expression Change] -> {expression_state}")
        if not self._should_attempt_connection():
            return False

        try:
            import grpc
            import face_control_pb2
            import face_control_pb2_grpc

            with grpc.insecure_channel('localhost:50051') as channel:
                stub = face_control_pb2_grpc.FaceServiceStub(channel)
                req = face_control_pb2.ExpressionRequest(state=expression_state)
                stub.SetExpression(req, timeout=1.0)
                self.face_server_active = True
                return True
        except Exception as e:
            if self.face_server_active:
                self.logger.warning(f"Face parameters server offline (gRPC port 50051): {e}")
                self.face_server_active = False
                self.last_connect_retry = time.time()
            return False

    def reset_indicators(self):
        """思考状態(Thinking)などの表示フラグをクリアする (gRPC ポート 50051)"""
        if not self._should_attempt_connection():
            return False

        try:
            import grpc
            import face_control_pb2
            import face_control_pb2_grpc

            with grpc.insecure_channel('localhost:50051') as face_channel:
                face_stub = face_control_pb2_grpc.FaceServiceStub(face_channel)
                face_stub.UpdateParameters(face_control_pb2.ParameterRequest(values={
                    "isThinking": 0.0,
                    "isParseControl": 0.0
                }), timeout=1.0)
                self.face_server_active = True
                return True
        except Exception as e:
            # 他の呼び出しでハンドリングするため、ここでは活性フラグの更新は最小限に
            return False

    def get_battery_level_string(self):
        """現在のバッテリー残量を取得し、報告文を返す (gRPC ポート 50051)"""
        if not self._should_attempt_connection():
            return "[sad]バッテリー状態が確認できないのだ。"

        try:
            import grpc
            import face_control_pb2
            import face_control_pb2_grpc
            from google.protobuf import empty_pb2

            with grpc.insecure_channel('localhost:50051') as channel:
                stub = face_control_pb2_grpc.FaceServiceStub(channel)
                status = stub.GetStatus(empty_pb2.Empty(), timeout=2.0)
                level = status.current_parameters.get("batteryLevel", -1.0)
                charging_val = status.current_parameters.get("batteryCharging", 0.0)
                
                charging_str = "未充電"
                if charging_val == 1.0:
                    charging_str = "充電中"
                elif charging_val == 2.0:
                    charging_str = "放電中（使用中）"
                
                if level >= 0.0:
                    return f"[happy]現在のバッテリー残量は {level:.1f}% なのだ！状態は {charging_str} なのだ。"
                else:
                    return "[sad]バッテリー残量データが不正なのだ。"
        except Exception as e:
            if self.face_server_active:
                self.logger.warning(f"Face server offline during battery query: {e}")
                self.face_server_active = False
                self.last_connect_retry = time.time()
            return "[sad]バッテリー状態が確認できないのだ。"
