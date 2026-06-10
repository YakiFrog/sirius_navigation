# -*- coding: utf-8 -*-
import os
import json
import math
import yaml
import rclpy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from ..local_parser import normalize_instruction_text
except ImportError:
    from local_parser import normalize_instruction_text

JAPANESE_TO_ROMAJI = {
    "リビング": "living",
    "庭": "garden",
    "充電ステーション": "charging_station",
    "充電器": "charger",
    "充電": "charging",
    "キッチン": "kitchen",
    "台所": "kitchen",
    "寝室": "bedroom",
    "玄関": "entrance",
    "廊下": "corridor",
    "トイレ": "toilet",
    "洗面所": "washroom",
    "お風呂": "bathroom",
    "風呂": "bathroom",
    "書斎": "study",
    "子供部屋": "kids_room",
    "和室": "washitsu",
    "バルコニー": "balcony",
    "ベランダ": "veranda",
    "食堂": "dining_room",
    "ダイニング": "dining",
    "応接室": "parlor",
}

class LandmarkManager:
    def __init__(self, node):
        self.node = node
        self.landmarks = {}
        self.landmark_aliases = {}
        self.landmark_file_path = None
        self.landmark_file_mtime = None
        self.current_landmark_map_name = None
        self.last_landmark_marker_count = 0

    def normalize_landmark_key(self, text):
        """ランドマーク名照合用の正規化"""
        return normalize_instruction_text(str(text)).strip().replace(" ", "").replace("　", "").lower()

    def load_landmarks_for_current_map(self, force=False):
        """Nav2起動スクリプトが書いた現在map情報からランドマークを自動読み込みする"""
        try:
            state_path = os.path.expanduser(self.node.current_map_state_file)
            if not os.path.exists(state_path):
                return
            with open(state_path, "r") as f:
                state = yaml.safe_load(f) or {}

            landmark_path = state.get("current_landmarks") or ""
            if not landmark_path:
                if force:
                    self.node.get_logger().info("[Landmark] current map has no associated landmark file yet.")
                return
            landmark_path = os.path.expanduser(landmark_path)
            if not os.path.exists(landmark_path):
                self.node.get_logger().warning(f"[Landmark] landmark file does not exist: {landmark_path}")
                return

            mtime = os.path.getmtime(landmark_path)
            if not force and landmark_path == self.landmark_file_path and mtime == self.landmark_file_mtime:
                return

            with open(landmark_path, "r") as f:
                data = yaml.safe_load(f) or {}

            landmarks = {}
            aliases = {}
            for item in data.get("landmarks", []):
                name = str(item.get("name", "")).strip()
                if not name:
                    continue
                try:
                    pose = {
                        "name": name,
                        "x": float(item["x"]),
                        "y": float(item["y"]),
                        "yaw": float(item.get("yaw", item.get("angle_radians", 0.0))),
                    }
                except (KeyError, TypeError, ValueError) as e:
                    self.node.get_logger().warning(f"[Landmark] skipping invalid landmark '{name}': {e}")
                    continue

                key = self.normalize_landmark_key(name)
                landmarks[key] = pose
                aliases[key] = key
                for alias in item.get("aliases", []) or []:
                    alias_key = self.normalize_landmark_key(alias)
                    if alias_key:
                        aliases[alias_key] = key

            self.landmarks = landmarks
            self.landmark_aliases = aliases
            self.landmark_file_path = landmark_path
            self.landmark_file_mtime = mtime
            self.current_landmark_map_name = state.get("current_map_name") or data.get("map", {}).get("name")
            self.node.get_logger().info(
                f"[Landmark] loaded {len(self.landmarks)} landmarks from {os.path.basename(landmark_path)} "
                f"for map={self.current_landmark_map_name or 'unknown'}"
            )
            self.publish_landmark_markers()
            self.publish_landmark_status()
        except Exception as e:
            self.node.get_logger().warning(f"[Landmark] failed to load landmarks: {e}")

    def get_landmark_names(self):
        self.load_landmarks_for_current_map()
        return [item["name"] for item in sorted(self.landmarks.values(), key=lambda lm: lm["name"])]

    def publish_landmark_status(self):
        names = self.get_landmark_names() if self.landmarks else []
        msg = String()
        if names:
            map_name = self.current_landmark_map_name or "unknown"
            msg.data = json.dumps({
                "map": map_name,
                "count": len(names),
                "names": names,
                "file": self.landmark_file_path or "",
            }, ensure_ascii=False)
        else:
            msg.data = json.dumps({
                "map": self.current_landmark_map_name or "unknown",
                "count": 0,
                "names": [],
                "file": self.landmark_file_path or "",
            }, ensure_ascii=False)
        self.node.landmark_status_pub.publish(msg)

    def handle_landmark_list_question(self, instruction):
        """「どこに行ける？」に、読み込み済みランドマーク名で答える"""
        normalized = self.normalize_landmark_key(instruction)
        question_tokens = [
            "どこにいけ", "どこへいけ", "どこ行け", "行ける場所", "いける場所",
            "場所一覧", "ランドマーク", "目的地一覧", "どこまで", "wherecan",
        ]
        if not any(token in normalized for token in question_tokens):
            return False

        names = self.get_landmark_names()
        self.publish_landmark_markers()
        self.publish_landmark_status()
        if not names:
            speak = "[sad]今の地図に対応するランドマークがまだ読み込めていないのだ。"
            self.node.llm_client._append_dialogue_history(instruction, speak)
            self.node.send_sirius_speak(speak)
            return True

        joined = "、".join(names)
        speak = f"[happy]今行ける場所は、{joined}なのだ。"
        self.node.get_logger().info(f"[Landmark] Available destinations: {joined}")
        self.node.llm_client._append_dialogue_history(instruction, speak)
        self.node.send_sirius_speak(speak)
        return True

    def find_landmark_in_instruction(self, instruction):
        """指示文に含まれるランドマークを探す"""
        self.load_landmarks_for_current_map()
        normalized = self.normalize_landmark_key(instruction)
        for alias_key in sorted(self.landmark_aliases.keys(), key=len, reverse=True):
            if alias_key and alias_key in normalized:
                canonical_key = self.landmark_aliases[alias_key]
                return self.landmarks.get(canonical_key)
        return None

    def find_all_landmarks_in_instruction(self, instruction):
        """指示文に含まれるすべてのランドマークを出現順に取得する"""
        self.load_landmarks_for_current_map()
        normalized = self.normalize_landmark_key(instruction)
        
        matches = []
        for alias_key, canonical_key in self.landmark_aliases.items():
            if not alias_key:
                continue
            start = 0
            while True:
                idx = normalized.find(alias_key, start)
                if idx == -1:
                    break
                landmark = self.landmarks.get(canonical_key)
                if landmark:
                    matches.append((idx, landmark))
                start = idx + len(alias_key)
                
        # 出現インデックス順に並べ替え
        matches.sort(key=lambda x: x[0])
        
        # 重複するランドマークを除去
        seen_names = set()
        landmarks = []
        for _, lm in matches:
            if lm["name"] not in seen_names:
                seen_names.add(lm["name"])
                landmarks.append(lm)
        return landmarks

    def handle_landmark_navigation_instruction(self, instruction):
        """「リビングを経由して庭に行って」のようなランドマーク指定の経由移動を処理する"""
        landmarks = self.find_all_landmarks_in_instruction(instruction)
        if not landmarks:
            return False

        normalized = self.normalize_landmark_key(instruction)
        navigation_keywords = [
            "行", "いって", "向か", "移動", "案内", "連れて", "つれて",
            "go", "goto", "navigate", "move", "経由", "けいゆ", "通って", "とおって",
        ]
        if not any(keyword in normalized for keyword in navigation_keywords):
            return False

        self.node.set_target_following(False, speak_on_failure=False)
        self.node.publish_nav_control("pause_silent")
        self.node.cancel_navigation(clear_queue=True, preserve_current_goal=False)
        self.node.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})

        names = [lm["name"] for lm in landmarks]
        if len(landmarks) > 1:
            speak = f"[happy]" + "を経由して".join(names[:-1]) + f"を経由し、{names[-1]}に向かうのだ！"
            # 2番目以降の目標地を command_queue に追加
            sorted_commands = []
            for lm in landmarks[1:]:
                sorted_commands.append({
                    "type": "goto",
                    "value": [float(lm["x"]), float(lm["y"])],
                    "speak": f"[happy]次は{lm['name']}に向かうのだ！"
                })
            with self.node.lock:
                self.node.command_queue = sorted_commands
        else:
            speak = f"[happy]{landmarks[0]['name']}に向かうのだ！"

        self.node.get_logger().info(
            f"[Landmark] Navigation requested for sequence: {', '.join(names)}"
        )
        self.node.llm_client._append_dialogue_history(instruction, speak)
        self.node.send_sirius_speak(speak)
        
        # 最初の目標地に移動開始
        self.node.publish_direct_map_goal(landmarks[0]["x"], landmarks[0]["y"], landmarks[0]["yaw"])
        return True

    def publish_landmark_markers(self):
        """読み込み済みランドマークをRViz2向けMarkerArrayとして発行する"""
        marker_array = MarkerArray()
        now = self.node.get_clock().now().to_msg()

        landmarks = list(self.landmarks.values())
        sorted_landmarks = sorted(landmarks, key=lambda item: item["name"])
        for index, landmark in enumerate(sorted_landmarks):
            yaw = float(landmark.get("yaw", 0.0))
            x = float(landmark["x"])
            y = float(landmark["y"])

            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns = 'sirius_landmarks'
            arrow.id = index * 3
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x = 0.55
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color.r = 0.1
            arrow.color.g = 0.8
            arrow.color.b = 0.25
            arrow.color.a = 0.9
            arrow.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
            marker_array.markers.append(arrow)

            point = Marker()
            point.header.frame_id = 'map'
            point.header.stamp = now
            point.ns = 'sirius_landmarks'
            point.id = index * 3 + 1
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.08
            point.pose.orientation.w = 1.0
            point.scale.x = 0.24
            point.scale.y = 0.24
            point.scale.z = 0.12
            point.color.r = 0.0
            point.color.g = 0.45
            point.color.b = 1.0
            point.color.a = 0.85
            point.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
            marker_array.markers.append(point)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'sirius_landmark_labels'
            label.id = index * 3 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.15
            label.pose.orientation.x = 0.0
            label.pose.orientation.y = 0.0
            label.pose.orientation.z = 0.0
            label.pose.orientation.w = 1.0
            label.scale.x = 0.0
            label.scale.y = 0.0
            label.scale.z = 0.40
            label.color.r = 0.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            
            name_ja = landmark['name']
            name_en = JAPANESE_TO_ROMAJI.get(name_ja.strip(), "")
            if name_en:
                label.text = f"L{index + 1}: {name_en}"
            else:
                if any(ord(c) > 0x7F for c in name_ja):
                    label.text = f"L{index + 1}"
                else:
                    label.text = f"L{index + 1}: {name_ja}"
                    
            label.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
            marker_array.markers.append(label)

        current_count = len(sorted_landmarks)
        for index in range(current_count, self.last_landmark_marker_count):
            for ns, marker_id in (
                ('sirius_landmarks', index * 3),
                ('sirius_landmarks', index * 3 + 1),
                ('sirius_landmark_labels', index * 3 + 2),
            ):
                delete_marker = Marker()
                delete_marker.header.frame_id = 'map'
                delete_marker.header.stamp = now
                delete_marker.ns = ns
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

        self.last_landmark_marker_count = current_count
        self.node.landmark_marker_pub.publish(marker_array)
