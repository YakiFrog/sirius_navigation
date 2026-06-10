# -*- coding: utf-8 -*-
import json
import math
import time
import urllib.request
import urllib.error
import re
import rclpy

try:
    from ..local_parser import parse_local_rules, normalize_instruction_text, DIALOGUE_TEMPLATES, DEFAULT_HUMOR_LEVEL
except ImportError:
    from local_parser import parse_local_rules, normalize_instruction_text, DIALOGUE_TEMPLATES, DEFAULT_HUMOR_LEVEL

class LlmClient:
    def __init__(self, node):
        self.node = node
        self.chat_history = []
        self.history_max_turns = 10  # 最大5往復分

    def _append_dialogue_history(self, user_text, assistant_text):
        """会話履歴に1往復を追加し、上限を超えたら古い履歴を削除する"""
        with self.node.lock:
            self.chat_history.append({"role": "user", "content": user_text})
            self.chat_history.append({"role": "assistant", "content": assistant_text})
            if len(self.chat_history) > self.history_max_turns:
                self.chat_history = self.chat_history[-self.history_max_turns:]

    def get_robot_state_context_string(self):
        """現在のロボットの物理的な実行状況を自然言語のテキストとして構築する"""
        with self.node.lock:
            goal_x = self.node.active_goal_x
            goal_y = self.node.active_goal_y
            goal_yaw = self.node.active_goal_yaw
            stuck = self.node.is_stuck
            vel_x = self.node.current_vel_x
            vel_theta = self.node.current_vel_theta
            queue_len = len(self.node.command_queue)
            executing = self.node.executing_command
            obs_dists = getattr(self.node, 'obstacle_distances', {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0})
            
        with self.node.lock:
            last_status = self.node.last_action_status
            last_type = self.node.last_action_type
            last_target = self.node.last_target_value
            last_dist_err = self.node.last_final_distance_error
            last_yaw_err = self.node.last_final_yaw_error
            humor_level = self.node.current_humor_level
            last_start_x = self.node.last_action_start_x
            last_start_y = self.node.last_action_start_y
            last_start_yaw = self.node.last_action_start_yaw

        # 現在位置とヘディング（角度）をTFから取得
        current_x, current_y, current_yaw_deg = 0.0, 0.0, 0.0
        current_yaw = 0.0
        try:
            trans = self.node.tf_buffer.lookup_transform('map', 'sirius3/base_footprint', rclpy.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            current_yaw_deg = math.degrees(current_yaw)
        except Exception:
            pass

        # 直前のアクションにおける実際の移動距離・回転量を計算
        last_move_str = "N/A"
        if last_type in ["forward", "backward"]:
            dist_moved = math.sqrt((current_x - last_start_x)**2 + (current_y - last_start_y)**2)
            last_move_str = f"Moved linear distance of {dist_moved:.2f} meters (Start: X={last_start_x:.2f}, Y={last_start_y:.2f} -> End: X={current_x:.2f}, Y={current_y:.2f})"
        elif last_type in ["turn", "spin", "face"]:
            diff_rad = current_yaw - last_start_yaw
            diff_rad = (diff_rad + math.pi) % (2 * math.pi) - math.pi
            diff_deg = math.degrees(diff_rad)
            last_move_str = f"Rotated angle of {diff_deg:+.1f} degrees (Start: {math.degrees(last_start_yaw):+.1f}deg -> End: {current_yaw_deg:+.1f}deg)"

        if goal_x is None:
            state_str = (
                "【Robot Current Hardware State Feedback】: Status=Idle. No active navigation goal. Robot is stationary.\n"
                f"- Current Robot Pose: X={current_x:.2f}, Y={current_y:.2f}, Yaw={current_yaw_deg:+.1f}deg\n"
                f"- Obstacle distances (meters): front={obs_dists.get('front', 999.0):.2f}, back={obs_dists.get('back', 999.0):.2f}, left={obs_dists.get('left', 999.0):.2f}, right={obs_dists.get('right', 999.0):.2f}\n"
                f"- Last Action Status: {last_status} (type: {last_type}, target: {last_target}, final distance error: {last_dist_err:.2f}m, final yaw error: {last_yaw_err:.1f}deg)\n"
                f"- Last Action Actual Execution Result: {last_move_str}"
            )
            return state_str
            
        distance = -1.0
        try:
            distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        except Exception:
            pass
            
        stuck_str = "BLOCKED / STUCK (Robot is command-active but linear velocity is zero. An obstacle likely blocks the way.)" if stuck else "Moving normally towards target"
        vel_str = f"linear={vel_x:.2f}m/s, angular={vel_theta:.2f}rad/s"
        
        state_str = (
            f"【Robot Current Hardware State Feedback】\n"
            f"- Status: {'Executing Sequence' if executing else 'Idle/Ready'}\n"
            f"- Current Robot Pose: X={current_x:.2f}, Y={current_y:.2f}, Yaw={current_yaw_deg:+.1f}deg\n"
            f"- Target Waypoint: X={goal_x:.2f}, Y={goal_y:.2f}, Yaw={math.degrees(goal_yaw):+.1f}deg\n"
            f"- Distance remaining to target: {distance:.2f} meters\n"
            f"- Current velocities: {vel_str}\n"
            f"- Current Sirius humor level: {humor_level:.2f}\n"
            f"- Physical obstacles / blockage state: {stuck_str}\n"
            f"- Obstacle distances (meters): front={obs_dists.get('front', 999.0):.2f}, back={obs_dists.get('back', 999.0):.2f}, left={obs_dists.get('left', 999.0):.2f}, right={obs_dists.get('right', 999.0):.2f}\n"
            f"- Actions left in sequence queue: {queue_len} commands\n"
            f"- Last Action Status: {last_status} (type: {last_type}, target: {last_target}, final distance error: {last_dist_err:.2f}m, final yaw error: {last_yaw_err:.1f}deg)\n"
            f"- Last Action Actual Execution Result: {last_move_str}"
        )
        return state_str

    def _build_state_info(self):
        """parse_local_rules へ渡す現在の状態情報をまとめた辞書を構築する"""
        current_x, current_y = 0.0, 0.0
        try:
            trans = self.node.tf_buffer.lookup_transform('map', 'sirius3/base_footprint', rclpy.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
        except Exception:
            pass

        with self.node.lock:
            executing = self.node.executing_command
            queue_len = len(self.node.command_queue)
            stuck = self.node.is_stuck
            last_action_status = self.node.last_action_status
            last_action_type = self.node.last_action_type
            last_target_value = self.node.last_target_value
            last_action_start_x = self.node.last_action_start_x
            last_action_start_y = self.node.last_action_start_y
            last_action_start_yaw = self.node.last_action_start_yaw
            current_humor_level = self.node.current_humor_level
            
            last_cmd_was_backward = False
            last_cmd_was_forward = False
            for msg in reversed(self.chat_history):
                if msg.get("role") == "assistant":
                    try:
                        import json as _json
                        hist_cmds = _json.loads(msg.get("content", "{}")).get("commands", [])
                        if hist_cmds:
                            last_t = hist_cmds[-1].get("type")
                            last_cmd_was_backward = (last_t == "backward")
                            last_cmd_was_forward = (last_t == "forward")
                            break
                    except Exception:
                        pass
            obs_dists = getattr(self.node, 'obstacle_distances', {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0})

        return {
            "current_x": current_x,
            "current_y": current_y,
            "executing": executing,
            "queue_len": queue_len,
            "stuck": stuck,
            "people_count": self.node.surrounding_people_count,
            "face_active": self.node.face_client.face_server_active,
            "current_expression": self.node.current_expression,
            "current_humor_level": current_humor_level,
            "current_speed_setting": self.node.current_speed_setting,
            "current_vel_x": self.node.current_vel_x,
            "current_vel_theta": self.node.current_vel_theta,
            "last_action_status": last_action_status,
            "last_action_type": last_action_type,
            "last_target_value": last_target_value,
            "last_action_start_x": last_action_start_x,
            "last_action_start_y": last_action_start_y,
            "last_action_start_yaw": last_action_start_yaw,
            "last_cmd_was_backward": last_cmd_was_backward,
            "last_cmd_was_forward": last_cmd_was_forward,
            "obstacle_distances": obs_dists,
        }

    def _local_parse_needs_llm_check(self, instruction, local_result, norm_inst):
        """ローカルパースが少し不安なときだけ LLM verifier に回す。"""
        if any(token in norm_inst for token in ["1周", "一周", "周"]):
            return False
        if local_result.get("cancel", False):
            return False

        commands = local_result.get("commands", [])
        if not commands:
            return False

        cmd_types = [cmd.get("type") for cmd in commands if isinstance(cmd, dict)]
        if not cmd_types:
            return False

        uncertain_words = [
            "そのまま", "ちょっと", "少し", "すこし", "もう少し", "もうちょっと",
            "もっと", "さらに", "いきすぎ", "行き過ぎ", "進みすぎ", "下がりすぎ",
            "回りすぎ", "まわりすぎ", "逆", "反対", "違う", "ちがう", "そっちじゃ",
            "足りない", "たりない", "戻って", "もどって", "戻し", "やり直",
            "さっき", "前回", "同じ", "その方向", "そっち", "こっち",
        ]
        if any(word in norm_inst for word in uncertain_words):
            return True

        if len(commands) > 1:
            return True

        if any(cmd_type == "goto" for cmd_type in cmd_types):
            explicit_coordinate_words = ["座標", "目標座標", "goto", "go to", "原点", "ホーム", "home"]
            if not any(word in norm_inst for word in explicit_coordinate_words):
                return True

        if any(cmd_type in ["turn", "spin", "backward"] for cmd_type in cmd_types):
            directional_words = ["右", "左", "後ろ", "うしろ", "裏", "みぎ", "ひだり"]
            if any(word in norm_inst for word in directional_words) and len(norm_inst) <= 12:
                return True

        return False

    def _parse_llm_json_content(self, content):
        content = content.strip()
        if content.startswith("```"):
            lines = content.splitlines()
            if len(lines) >= 3:
                content = "\n".join(lines[1:-1]).strip()
        return json.loads(content), content

    def _validate_verifier_result(self, verifier_result, local_result, instruction=""):
        if not isinstance(verifier_result, dict):
            return None

        normalized_instruction = normalize_instruction_text(instruction).strip().replace(" ", "").replace("　", "").lower()
        if any(token in normalized_instruction for token in ["1周", "一周", "周"]):
            return local_result

        decision = verifier_result.get("decision", "accept")
        if decision == "accept":
            return local_result

        if decision == "ask":
            speak = verifier_result.get("speak") or "[normal]念のため確認したいのだ。どう動けばいいか、もう少し具体的に言ってほしいのだ。"
            return {"commands": [], "cancel": False, "fast_path": True, "speak": speak}

        if decision != "revise":
            return None

        allowed_types = {"forward", "backward", "turn", "spin", "face", "goto", "speed", "expression", "parameter", "reset", "effect", "look", "humor"}
        revised_commands = verifier_result.get("commands", [])
        if not isinstance(revised_commands, list):
            return None

        safe_commands = []
        for cmd in revised_commands:
            if not isinstance(cmd, dict):
                continue
            cmd_type = cmd.get("type")
            if cmd_type not in allowed_types:
                continue
            safe_commands.append(cmd)

        if not safe_commands and not verifier_result.get("speak"):
            return None

        result = {
            "commands": safe_commands,
            "cancel": bool(verifier_result.get("cancel", False)),
            "fast_path": True,
        }
        if verifier_result.get("speak"):
            result["speak"] = verifier_result.get("speak")
        return result

    def _verify_local_parse_with_llm(self, instruction, local_result, state_info):
        """ローカルパース結果を、短いLLM verifierで検算する。失敗時はNoneを返す。"""
        system_prompt = (
            "You are a safety verifier for a Japanese robot navigation parser. "
            "Check whether rule_parse correctly captures the user's intent. "
            "Reply raw JSON only, no markdown. "
            "Schema: {\"decision\":\"accept|revise|ask\", \"commands\":[], \"cancel\":false, \"speak\":\"optional Japanese\"}. "
            "Use accept if rule_parse is good enough. "
            "Use revise only when the rule_parse is clearly wrong. "
            "Use ask only when the instruction is genuinely ambiguous. "
            "If executing the command requires guessing missing direction, distance, destination, face expression, or effect, use ask and keep commands empty. "
            "Allowed command types: forward, backward, turn, spin, face, goto, speed, expression, parameter, reset, effect, look, humor. "
            "Important: backward means move backward while keeping the current yaw; do not convert it to a turn. "
            "Keep distances and angles conservative. Do not invent extra commands."
        )

        verifier_input = {
            "user_text": instruction,
            "rule_parse": local_result,
            "state": {
                "executing": state_info.get("executing", False),
                "last_action_status": state_info.get("last_action_status", ""),
                "last_action_type": state_info.get("last_action_type", ""),
                "last_target_value": state_info.get("last_target_value", 0.0),
                "current_humor_level": state_info.get("current_humor_level", DEFAULT_HUMOR_LEVEL),
                "obstacle_distances": state_info.get("obstacle_distances", {}),
            },
        }
        payload = {
            "model": self.node.model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": json.dumps(verifier_input, ensure_ascii=False)},
            ],
            "temperature": 0.0,
            "max_tokens": 384,
            "top_p": 1.0,
        }

        try:
            req = urllib.request.Request(
                self.node.lm_studio_url,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=5.0) as response:
                res_body = response.read().decode("utf-8")
                res_json = json.loads(res_body)
                content = res_json["choices"][0]["message"]["content"]
                verifier_json, cleaned_content = self._parse_llm_json_content(content)

            checked = self._validate_verifier_result(verifier_json, local_result, instruction)
            if checked is None:
                self.node.get_logger().warning(f"LLM verifier returned unusable result: {verifier_json}")
                return None

            self.node.get_logger().info(
                f"LLM verifier checked uncertain local parse: decision={verifier_json.get('decision', 'accept')} raw='{cleaned_content}'"
            )
            return checked
        except Exception as e:
            self.node.get_logger().warning(f"LLM verifier failed; using local parse as-is: {e}")
            return None

    def query_lm_studio(self, instruction):
        """ローカルルールを先に試み、マッチしなければLM StudioへLLMクエリを送る"""
        norm_inst = normalize_instruction_text(instruction).strip().replace(" ", "").replace("　", "").lower()

        def _extract_jsonish_speak(text):
            if not text:
                return None
            m = re.search(r'"speak"\s*:\s*"([^"]{1,400})', text)
            if m:
                return m.group(1)
            m = re.search(r'"response"\s*:\s*"([^"]{1,400})', text)
            if m:
                return m.group(1)
            return None

        # --- ローカルルールベース判定 ---
        state_info = self._build_state_info()
        local_result = parse_local_rules(
            instruction,
            state_info,
            battery_callback=self.node.get_battery_report_string
        )
        if local_result is not None:
            if self._local_parse_needs_llm_check(instruction, local_result, norm_inst):
                self.node.get_logger().info(f"Local parse is uncertain; asking LLM verifier. parse={local_result}")
                checked_result = self._verify_local_parse_with_llm(instruction, local_result, state_info)
                if checked_result is not None:
                    return checked_result
            return local_result
        
        # --- LLM 問い合わせ ---
        system_prompt = (
            "You are Sirius, a small outdoor navigation robot with a warm, playful Kansai-ben personality.\n"
            "Keep the Sirius character in every reply. Sound friendly, a little cute, and confident.\n"
            "Output raw JSON only, no markdown, no reasoning, no extra text.\n"
            "Schema: {\"commands\": [ {\"type\": one_of(forward, backward, turn, spin, face, goto, speed, expression, parameter, reset, effect, look, humor, register_landmark), \"value\": any } ], \"cancel\": boolean, \"speak\": string?}\n"
            "If the user asks a question, explain briefly in Japanese in speak and keep commands empty.\n"
            "If the user requests movement, output the exact command sequence.\n"
            "If the user asks to save, remember, or register the current position as a landmark under a name, output a register_landmark command with the name as the value.\n"
            "If the user asks to change humor level, output a humor command with value 0.0 to 1.0.\n"
            "Use a short expression tag and speak like Sirius. Humor 0.0 means polite Japanese. Humor >=0.5 means lively Kansai-ben with playful confidence.\n"
            "Always prefix speak with exactly one expression tag from [normal], [happy], [angry], [sad], [surprised], [cat], [wink], [pien], [sleeping]. "
            "Use [normal] for clarification questions, [sad] for apologies/failures, [happy] for success/helpful answers, [wink] for jokes.\n"
            "If the request is missing required details or would require guessing intent, ask one concise clarification question in Japanese in speak and keep commands empty.\n"
            "Do not invent a direction, distance, destination, expression, or face effect that the user did not specify.\n"
            "Use the robot state context for current status, remaining distance, blockage, people count, battery, and last action.\n"
            "Conversation history may contain earlier user requests and Sirius replies; use it to preserve context and avoid asking redundant clarification.\n"
            "Always prefer a valid minimal JSON object. Do not wrap in code fences."
        )
        
        headers = {
            'Content-Type': 'application/json',
        }
        
        state_context = self.get_robot_state_context_string()
        
        with self.node.lock:
            if len(self.chat_history) > 4:
                self.chat_history = self.chat_history[-4:]
            
            messages = [{"role": "system", "content": system_prompt}]
            messages.extend(self.chat_history)
            messages.append({"role": "system", "content": state_context})
            messages.append({"role": "user", "content": instruction})
        
        payload = {
            "model": self.node.model_name,
            "messages": messages,
            "temperature": 0.0,
            "max_tokens": 2048,
            "top_p": 1.0
        }

        def _fallback_short_query(reason_hint=""):
            fallback_prompt = (
                "You are a robotic navigator assistant. Reply with raw JSON only. "
                "If the user asks a general question, answer briefly in Japanese in speak. "
                "If details are missing or intent must be guessed, ask one concise clarification question in Japanese in speak and keep commands empty. "
                "Do not invent movement or face-control parameters. "
                "Do not reason. Do not use markdown."
            )
            fallback_messages = [
                {"role": "system", "content": fallback_prompt},
                {"role": "system", "content": state_context},
                {"role": "user", "content": instruction},
            ]
            fallback_payload = {
                "model": self.node.model_name,
                "messages": fallback_messages,
                "temperature": 0.0,
                "max_tokens": 256,
                "top_p": 1.0
            }
            try:
                req2 = urllib.request.Request(
                    self.node.lm_studio_url,
                    data=json.dumps(fallback_payload).encode('utf-8'),
                    headers=headers,
                    method='POST'
                )
                with urllib.request.urlopen(req2, timeout=15.0) as response2:
                    res_body2 = response2.read().decode('utf-8')
                    res_json2 = json.loads(res_body2)
                    content2 = res_json2['choices'][0]['message']['content'].strip()
                    if content2.startswith("```"):
                        lines2 = content2.splitlines()
                        if len(lines2) >= 3:
                            content2 = "\n".join(lines2[1:-1])
                    try:
                        parsed_json2 = json.loads(content2)
                    except Exception:
                        speak_hint2 = _extract_jsonish_speak(content2)
                        if speak_hint2:
                            self.node.get_logger().warning("Fallback JSON was broken, but speak text was recoverable.")
                            parsed_json2 = {"commands": [], "cancel": False, "speak": speak_hint2}
                        elif content2:
                            self.node.get_logger().warning("Fallback returned plain text; wrapping it as speak.")
                            parsed_json2 = {"commands": [], "cancel": False, "speak": content2[:400]}
                        else:
                            raise
                    self.node.get_logger().info(f"LLM Fallback Raw Output: '{content2}'")
                    self._append_dialogue_history(instruction, content2)
                    return parsed_json2
            except Exception as e2:
                self.node.get_logger().error(f"Fallback LLM call also failed ({reason_hint}): {e2}")
            return None
        
        try:
            req = urllib.request.Request(
                self.node.lm_studio_url,
                data=json.dumps(payload).encode('utf-8'),
                headers=headers,
                method='POST'
            )
            with urllib.request.urlopen(req, timeout=15.0) as response:
                res_body = response.read().decode('utf-8')
                res_json = json.loads(res_body)
                finish_reason = res_json['choices'][0].get('finish_reason')
                if finish_reason == "length":
                    self.node.get_logger().warning("LLM response was truncated by token limit; JSON may be incomplete.")
                content = res_json['choices'][0]['message']['content'].strip()
                if not content:
                    self.node.get_logger().warning("LLM returned empty content; retrying with a shorter prompt.")
                    return _fallback_short_query("empty_content")
                
                if content.startswith("```"):
                    lines = content.splitlines()
                    if len(lines) >= 3:
                        content = "\n".join(lines[1:-1])
                try:
                    parsed_json = json.loads(content)
                except Exception:
                    speak_hint = _extract_jsonish_speak(content)
                    if speak_hint:
                        self.node.get_logger().warning("LLM JSON was broken, but speak text was recoverable.")
                        parsed_json = {"commands": [], "cancel": False, "speak": speak_hint}
                    elif content:
                        self.node.get_logger().warning("LLM returned plain text; wrapping it as speak.")
                        parsed_json = {"commands": [], "cancel": False, "speak": content[:400]}
                    else:
                        raise
                self.node.get_logger().info(f"LLM Raw Output: '{content}'")
                
                self._append_dialogue_history(instruction, content)
                return parsed_json
        except urllib.error.URLError as e:
            self.node.get_logger().error(f"LM Studio API connection failed: {e}")
            return _fallback_short_query("primary_url_error")
        except Exception as e:
            self.node.get_logger().error(f"Error calling LLM: {e}")
            return _fallback_short_query("primary_parse_failed")
