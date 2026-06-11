# -*- coding: utf-8 -*-
import os
import json
import math
import re
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
        self.last_target_landmark = None

    def normalize_landmark_key(self, text):
        """ランドマーク名照合用の正規化"""
        return normalize_instruction_text(str(text)).strip().replace(" ", "").replace("　", "").lower()

    def _ensure_landmark_file_path(self):
        """現在の地図に紐づくランドマークファイルがなければ、保存先を自動生成する"""
        state_path = os.path.expanduser(self.node.current_map_state_file)
        current_map_name = None
        current_landmark_path = None

        try:
            if os.path.exists(state_path):
                with open(state_path, "r") as f:
                    state = yaml.safe_load(f) or {}
                current_map_name = state.get("current_map_name")
                current_landmark_path = state.get("current_landmarks") or None
        except Exception as e:
            self.node.get_logger().warning(f"[Landmark] failed to read current map state: {e}")

        if current_landmark_path:
            path = os.path.expanduser(current_landmark_path)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            return path

        home_dir = os.path.expanduser("~")
        base_dir = os.path.join(home_dir, "sirius_jazzy_ws", "maps_waypoints", "landmarks")
        os.makedirs(base_dir, exist_ok=True)
        map_name = current_map_name or "unknown_map"
        file_name = f"{map_name}.yaml"
        path = os.path.join(base_dir, file_name)
        if not os.path.exists(path):
            initial_data = {
                "format_version": "1.0",
                "map": {
                    "name": map_name,
                    "yaml": f"{map_name}.yaml",
                },
                "landmarks": [],
            }
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(initial_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

        try:
            if os.path.exists(state_path):
                with open(state_path, "r") as f:
                    state = yaml.safe_load(f) or {}
            else:
                state = {}
            state["current_landmarks"] = path
            if current_map_name:
                state["current_map_name"] = current_map_name
            with open(state_path, "w") as f:
                yaml.safe_dump(state, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        except Exception as e:
            self.node.get_logger().warning(f"[Landmark] failed to update current map state: {e}")

        return path

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
        near_tokens = ["近い", "ちかい", "最寄", "最も近", "いちばん近"]
        if any(token in normalized for token in near_tokens):
            return False
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

    def handle_nearest_landmark_question(self, instruction):
        """「今の座標に近いランドマークは？」に答える"""
        normalized = self.normalize_landmark_key(instruction)
        tokens = ["近い", "ちかい", "最寄", "最も近", "いちばん近", "nearest", "nearby"]
        if not any(token in normalized for token in tokens):
            return False

        self.load_landmarks_for_current_map()
        if not self.landmarks:
            speak = "[sad]ランドマークがまだ読み込めていないのだ。"
            self.node.llm_client._append_dialogue_history(instruction, speak)
            self.node.send_sirius_speak(speak)
            return True

        try:
            trans = self.node.tf_buffer.lookup_transform(
                "map",
                "sirius3/base_footprint",
                rclpy.time.Time(),
            )
            current_x = float(trans.transform.translation.x)
            current_y = float(trans.transform.translation.y)
        except Exception as e:
            self.node.get_logger().warning(f"[Landmark] failed to get current pose for nearest query: {e}")
            speak = "[sad]今の位置が取れないから、近いランドマークを判定できないのだ。"
            self.node.llm_client._append_dialogue_history(instruction, speak)
            self.node.send_sirius_speak(speak)
            return True

        nearest = None
        nearest_dist = None
        for landmark in self.landmarks.values():
            dist = math.hypot(float(landmark["x"]) - current_x, float(landmark["y"]) - current_y)
            if nearest is None or dist < nearest_dist:
                nearest = landmark
                nearest_dist = dist

        if nearest is None:
            speak = "[sad]近いランドマークが見つからないのだ。"
            self.node.llm_client._append_dialogue_history(instruction, speak)
            self.node.send_sirius_speak(speak)
            return True

        speak = f"[happy]いちばん近いのは、{nearest['name']}やで。だいたい{nearest_dist:.1f}メートルくらいなのだ。"
        self.node.get_logger().info(
            f"[Landmark] Nearest landmark: {nearest['name']} distance={nearest_dist:.2f}m from X={current_x:.2f}, Y={current_y:.2f}"
        )
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
        """指示文に含まれるすべてのランドマークを出現順に取得する（仮想の「原点」を含む）"""
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

        # 仮想の「原点/ホーム」の抽出
        origin_keywords = ["原点", "げんてん", "ホーム", "ほーむ", "home"]
        for kw in origin_keywords:
            start = 0
            while True:
                idx = normalized.find(kw, start)
                if idx == -1:
                    break
                virtual_landmark = {
                    "name": "原点",
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0
                }
                matches.append((idx, virtual_landmark))
                start = idx + len(kw)
                
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
        normalized = self.normalize_landmark_key(instruction)
        loop_match = re.search(r"(\d+)\s*(?:回|周|回繰り返して|回繰り返し)", normalized)
        loop_times = int(loop_match.group(1)) if loop_match else 1
        if loop_times < 1:
            loop_times = 1
        vague_reference_tokens = [
            "さっき目指してた場所", "さっきの場所", "前に目指してた場所", "前の場所",
            "さっき向かってた場所", "前回の場所", "さっき目指した場所", "直前の場所",
            "さっきの目的地", "前の目的地", "さっき向かってた先"
        ]
        if any(token in normalized for token in vague_reference_tokens):
            if self.last_target_landmark is not None:
                landmarks = [self.last_target_landmark]
                is_vague_reference = True
            else:
                return False
        else:
            landmarks = self.find_all_landmarks_in_instruction(instruction)
            is_vague_reference = False

        # 条件付き指示（例:「〜だったら」「〜のとき」）の場合は、直接のナビゲーション処理をスキップしてLLMに判断を委ねる
        conditional_tokens = ["たら", "なら", "のとき", "の時", "ときに", "時に", "もし", "if", "when"]
        if any(token in normalized for token in conditional_tokens):
            return False

        if not landmarks:
            # ランドマーク名だけが入っている場合は、ナビゲーション指示として扱う
            exact_landmark = self.find_landmark_in_instruction(instruction)
            if exact_landmark is not None:
                landmarks = [exact_landmark]
            else:
                return False

        if loop_times > 1:
            landmarks = landmarks * loop_times

        movement_keywords = [
            "行", "いって", "向か", "移動", "案内", "連れて", "つれて",
            "戻", "帰", "かえ",
            "go", "goto", "navigate", "move", "経由", "けいゆ", "通って", "とおって",
        ]
        if not is_vague_reference and len(landmarks) == 1:
            only_name = self.normalize_landmark_key(landmarks[0]["name"])
            input_is_just_landmark = normalized == only_name
            if not input_is_just_landmark and not any(keyword in normalized for keyword in movement_keywords):
                return False
        elif not is_vague_reference and not any(keyword in normalized for keyword in movement_keywords):
            # 複数ランドマークを含むときも、地名だけならナビゲーションとして扱う
            pass

        self.node.set_target_following(False, speak_on_failure=False)
        self.node.publish_nav_control("pause_silent")
        self.node.cancel_navigation(clear_queue=True, preserve_current_goal=False)
        self.node.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})

        names = [lm["name"] for lm in landmarks]
        if len(landmarks) > 1:
            if loop_times > 1:
                speak = f"[happy]{names[0]}から{names[-1]}までのルートを{loop_times}回くり返すのだ！"
            else:
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
        
        import time
        with self.node.lock:
            self.node.active_command = {
                "type": "goto",
                "value": [float(landmarks[0]["x"]), float(landmarks[0]["y"])],
                "name": f"{landmarks[0]['name']}に移動",
                "status": "executing",
                "time": time.strftime('%H:%M:%S')
            }
        self.node.publish_queue_status()

        # 最初の目標地に移動開始
        self.last_target_landmark = landmarks[0]
        self.node.publish_direct_map_goal(landmarks[0]["x"], landmarks[0]["y"], landmarks[0]["yaw"])
        return True

    def register_current_pose_as_landmark(self, name: str):
        """現在のロボット位置（TF）を取得し、sim.yaml にランドマークとして登録または更新する"""
        try:
            trans = self.node.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time()
            )
            x = float(trans.transform.translation.x)
            y = float(trans.transform.translation.y)
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = float(math.atan2(siny_cosp, cosy_cosp))
        except Exception as e:
            self.node.get_logger().error(f"Failed to lookup TF (map -> sirius3/base_footprint) for registration: {e}")
            return False, "[sad]現在位置の取得に失敗したのだ。"

        yaml_path = getattr(self, 'landmark_file_path', None)
        if not yaml_path or not os.path.exists(yaml_path):
            yaml_path = self._ensure_landmark_file_path()
        
        if not yaml_path or not os.path.exists(yaml_path):
            return False, "[sad]ランドマーク定義ファイルを作れなかったのだ。"

        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            return False, f"[sad]ファイルの読み込みに失敗したのだ。{e}"

        if "landmarks" not in data or data["landmarks"] is None:
            data["landmarks"] = []

        found = False
        for lm in data["landmarks"]:
            if lm.get("name") == name:
                lm["x"] = round(x, 3)
                lm["y"] = round(y, 3)
                lm["yaw"] = round(yaw, 3)
                found = True
                break
        
        if not found:
            data["landmarks"].append({
                "name": name,
                "x": round(x, 3),
                "y": round(y, 3),
                "yaw": round(yaw, 3),
                "aliases": []
            })

        try:
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)
        except Exception as e:
            return False, f"[sad]ファイルの保存に失敗したのだ。{e}"

        self.load_landmarks_for_current_map(force=True)
        self.publish_landmark_markers()
        self.publish_landmark_status()

        action_word = "更新" if found else "新しく登録"
        return True, f"[happy]現在位置を『{name}』として{action_word}したで！"

    def handle_landmark_registration_instruction(self, instruction):
        """「ここをリビングとして登録して」や「ここを玄関にして」のような登録・更新指令をルールベースでパースする"""
        normalized = self.normalize_landmark_key(instruction)
        
        patterns = [
            r"ここを(.+?)(?:として|と|に)?(?:登録|保存|記憶|記録|おぼえて|覚えて|設定)",
            r"ここを(.+?)(?:に|と)(?:して|決定|覚える|記憶)",
        ]
        
        target_name = None
        for pat in patterns:
            m = re.search(pat, normalized)
            if m:
                target_name = m.group(1).strip()
                break
                
        if target_name:
            # 助詞などの余分な語尾をトリミングする安全策
            for suffix in ["として", "と", "に"]:
                if target_name.endswith(suffix):
                    target_name = target_name[:-len(suffix)]
                    break

        if not target_name:
            return False

        self.node.get_logger().info(f"[Landmark] Voice registration requested for landmark name: {target_name}")
        
        self.node.set_target_following(False, speak_on_failure=False)
        self.node.publish_nav_control("pause_silent")
        self.node.cancel_navigation(clear_queue=True, preserve_current_goal=False)
        
        ok, speak = self.register_current_pose_as_landmark(target_name)
        self.node.llm_client._append_dialogue_history(instruction, speak)
        self.node.send_sirius_speak(speak)
        return True

    def remove_landmark_by_name(self, name: str):
        """指定された名前のランドマークを sim.yaml から削除する"""
        yaml_path = getattr(self, 'landmark_file_path', None)
        if not yaml_path or not os.path.exists(yaml_path):
            yaml_path = self._ensure_landmark_file_path()
        
        if not yaml_path or not os.path.exists(yaml_path):
            return False, "[sad]ランドマーク定義ファイルを作れなかったのだ。"

        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            return False, f"[sad]ファイルの読み込みに失敗したのだ。{e}"

        if "landmarks" not in data or not data["landmarks"]:
            return False, f"[sad]『{name}』というランドマークは登録されていないのだ。"

        # 対象ランドマークをリストから除去
        original_len = len(data["landmarks"])
        data["landmarks"] = [lm for lm in data["landmarks"] if lm.get("name") != name]
        
        if len(data["landmarks"]) == original_len:
            return False, f"[sad]『{name}』というランドマークは登録されていないのだ。"

        try:
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)
        except Exception as e:
            return False, f"[sad]ファイルの保存に失敗したのだ。{e}"

        # メモリ内のランドマーク情報をリロード
        self.load_landmarks_for_current_map(force=True)
        # RVizマーカー更新 (不要になったマーカーは自動的にDELETEされる)
        self.publish_landmark_markers()
        self.publish_landmark_status()

        return True, f"[happy]『{name}』の登録を消したで！"

    def handle_landmark_deletion_instruction(self, instruction):
        """「リビングの登録を消して」のような削除指令をルールベースでパースする"""
        normalized = self.normalize_landmark_key(instruction)
        
        patterns = [
            r"(.+?)(?:の登録|のランドマーク)?を(?:消して|削除|取り消して|クリア|消去)",
            r"(.+?)(?:の登録|のランドマーク)?(?:取り消し|削除|消去)",
        ]
        
        target_name = None
        for pat in patterns:
            m = re.search(pat, normalized)
            if m:
                target_name = m.group(1).strip()
                break
                
        # 登録指令や他の指令に誤爆しないようフィルタ
        if not target_name or any(w in normalized for w in ["ここを", "行って", "向かって", "経由"]):
            return False

        # 存在するランドマーク名か判定
        canonical_key = self.normalize_landmark_key(target_name)
        if canonical_key not in self.landmarks and canonical_key not in self.landmark_aliases:
            return False

        # エイリアスの解決
        resolved_key = self.landmark_aliases.get(canonical_key, canonical_key)
        landmark = self.landmarks.get(resolved_key)
        if not landmark:
            return False
        
        target_name = landmark["name"]
        self.node.get_logger().info(f"[Landmark] Voice deletion requested for landmark name: {target_name}")
        
        self.node.set_target_following(False, speak_on_failure=False)
        self.node.publish_nav_control("pause_silent")
        self.node.cancel_navigation(clear_queue=True, preserve_current_goal=False)
        
        ok, speak = self.remove_landmark_by_name(target_name)
        self.node.llm_client._append_dialogue_history(instruction, speak)
        self.node.send_sirius_speak(speak)
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
