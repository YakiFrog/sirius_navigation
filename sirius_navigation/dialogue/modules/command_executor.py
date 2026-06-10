# -*- coding: utf-8 -*-
import math
import rclpy
try:
    from ..local_parser import DIALOGUE_TEMPLATES, clamp_humor_level
except ImportError:
    from local_parser import DIALOGUE_TEMPLATES, clamp_humor_level

class CommandExecutor:
    def __init__(self, node):
        self.node = node

    def _normalize_expression_value(self, value):
        valid = ["normal", "happy", "angry", "sad", "surprised", "cat", "wink", "pien", "sleeping"]
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in valid:
                return lowered
            if lowered in ["笑顔", "にこにこ", "ニコニコ"]:
                return "happy"
            if lowered in ["ウインク", "ウィンク", "wink"]:
                return "wink"
        try:
            fval = float(value)
            if fval >= 0.75:
                return "happy"
            elif fval <= 0.25:
                return "sad"
            return "normal"
        except (TypeError, ValueError):
            return "normal"

    def execute_next_command(self):
        """キューから次のコマンドを取り出して実行する"""
        import time
        t_str = time.strftime('%H:%M:%S')
        with self.node.lock:
            # Archive previous active command if exists
            if self.node.active_command:
                status = self.node.last_action_status
                if status == "none":
                    status = "completed"
                self.node.active_command["status"] = status
                self.node.active_command["time"] = t_str
                self.node.command_history.append(self.node.active_command)
                self.node.active_command = None
                self.node.last_action_status = "none"

            if not self.node.command_queue:
                self.node.executing_command = False
                self.node.current_xy_tolerance = 0.50
                self.node.suppress_step_speech = False
                self.node.get_logger().info("All commands in sequence finished.")
                self.node.nav_ctrl.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})
                self.node.publish_queue_status()
                return
            cmd = self.node.command_queue.pop(0)
            self.node.executing_command = True
            self.node.command_start_time = self.node.get_clock().now()
            
            should_speak = not self.node.suppress_step_speech
            if self.node.command_queue:
                self.node.suppress_step_speech = True
            
        cmd_type = cmd.get("type", "forward")
        value = cmd.get("value", 0.0)
        speak_override = cmd.get("speak")
        
        # Formulate human-readable command name
        cmd_name = ""
        if cmd_type == "goto":
            cmd_name = cmd.get("name")
            if not cmd_name:
                if isinstance(value, list) and len(value) == 2:
                    tx, ty = value[0], value[1]
                    matched_name = None
                    for lm_key, lm_val in self.node.landmarks.items():
                        if abs(lm_val["x"] - tx) < 0.1 and abs(lm_val["y"] - ty) < 0.1:
                            matched_name = lm_val["name"]
                            break
                    if matched_name:
                        cmd_name = f"{matched_name}に移動"
                    else:
                        cmd_name = f"座標({tx:.2f}, {ty:.2f})に移動"
                else:
                    cmd_name = "目的地に移動"
            else:
                cmd_name = f"{cmd_name}に移動"
        elif cmd_type == "forward":
            cmd_name = f"前進 ({value}m)"
        elif cmd_type == "backward":
            cmd_name = f"後退 ({value}m)"
        elif cmd_type == "turn":
            deg = round(math.degrees(value))
            cmd_name = f"{'右' if value < 0 else '左'}に{abs(deg)}度旋回"
        elif cmd_type == "spin":
            cmd_name = f"その場旋回 ({value}度)"
        elif cmd_type == "face":
            cmd_name = f"方位 {value}度を向く"
        elif cmd_type == "register_landmark":
            cmd_name = f"ランドマーク登録 '{value}'"
        elif cmd_type == "remove_landmark":
            cmd_name = f"ランドマーク削除 '{value}'"
        elif cmd_type == "expression":
            cmd_name = f"表情変更 '{value}'"
        elif cmd_type == "speed":
            cmd_name = f"速度係数変更 '{value}'"
        else:
            cmd_name = f"コマンド '{cmd_type}'"
        
        with self.node.lock:
            self.node.active_command = {
                "type": cmd_type,
                "value": value,
                "name": cmd_name,
                "status": "executing",
                "time": t_str
            }
            
            self.node.last_action_type = cmd_type
            if isinstance(value, list):
                self.node.last_target_value = value
            elif isinstance(value, str):
                self.node.last_target_value = value
            else:
                try:
                    self.node.last_target_value = float(value)
                except (TypeError, ValueError):
                    self.node.last_target_value = value
        
        self.node.publish_queue_status()
        self.node.get_logger().info(f"Executing next sequence command -> type: {cmd_type}, value: {value}")
        
        if cmd_type == "speed":
            try:
                speed_factor = float(value)
                self.node.nav_ctrl.set_controller_speed(speed_factor)
                with self.node.lock:
                    if self.node.assisted_drive_active:
                        base_distance = max(self.node.assisted_drive_distance, 0.3)
                        base_speed = min(max(base_distance * 0.35, 0.12), 0.22)
                        adjusted_speed = max(0.08, min(base_speed * speed_factor, 0.35))
                        self.node.assisted_drive_speed = adjusted_speed
                        self.node.get_logger().info(
                            f"[AssistDrive] active speed updated -> {adjusted_speed:.2f} m/s "
                            f"(factor={speed_factor:.2f})"
                        )
                print(f"⚡ 速度パラメータを変更しました (factor: {speed_factor:.2f})")
                if should_speak:
                    self.node.send_sirius_speak(DIALOGUE_TEMPLATES["speed_change"].format(speed=speed_factor))
            except Exception as e:
                self.node.get_logger().error(f"Failed to set speed: {e}")
            self.execute_next_command()
            return

        if cmd_type == "expression":
            exp_state = self._normalize_expression_value(value)
            ok = self.node.face_client.set_expression(exp_state)
            if ok:
                with self.node.lock:
                    self.node.current_expression = exp_state
                self.node.get_logger().info(f"[Expression] Set to '{exp_state}'")
                if should_speak:
                    self.node.send_sirius_speak(f"[{exp_state}]表情を変えたのだ！")
            else:
                self.node.get_logger().warning(f"[Expression] Failed to set '{exp_state}' (face server offline)")
            self.execute_next_command()
            return

        if cmd_type == "parameter":
            try:
                if isinstance(value, dict):
                    param_name = value.get("name")
                    param_amount = float(value.get("amount", 1.0))
                else:
                    param_name = None
                    param_amount = 1.0

                if param_name not in ["blushAmount", "sparkleAmount"]:
                    self.node.get_logger().warning(f"[Parameter] Unsupported parameter: {param_name}")
                else:
                    ok = self.node.face_client.update_parameters({param_name: param_amount})
                    if ok:
                        self.node.get_logger().info(f"[Parameter] Set {param_name}={param_amount}")
                        if should_speak:
                            if param_name == "blushAmount":
                                self.node.send_sirius_speak("[happy]照れを足したのだ！")
                            elif param_name == "sparkleAmount":
                                self.node.send_sirius_speak("[happy]キラキラを足したのだ！")
                    else:
                        self.node.get_logger().warning(f"[Parameter] Failed to set {param_name} (face server offline)")
            except Exception as e:
                self.node.get_logger().error(f"[Parameter] Failed to apply effect: {e}")
            self.execute_next_command()
            return

        if cmd_type == "humor":
            level = clamp_humor_level(value)
            with self.node.lock:
                self.node.current_humor_level = level
            self.node.get_logger().info(f"[Humor] Set humor level to {level:.2f}")
            if should_speak:
                if level <= 0.0:
                    self.node.send_sirius_speak("[normal]ユーモアレベルを0.0に設定しました。真面目な敬語モードで対応します。")
                elif level < 0.5:
                    self.node.send_sirius_speak(f"[normal]ユーモアレベルを{level:.1f}にしたのだ。落ち着いて対応するのだ。")
                else:
                    self.node.send_sirius_speak(f"[cat]ユーモアレベルを{level:.1f}にしたのだ！ちょっと生意気にいくのだ。")
            self.execute_next_command()
            return

        if cmd_type == "reset":
            ok = self.node.face_client.reset_face()
            if ok:
                with self.node.lock:
                    self.node.current_expression = "normal"
                self.node.get_logger().info("[Face] Reset face state")
                if should_speak:
                    self.node.send_sirius_speak("[normal]表情を戻したのだ！")
            else:
                self.node.get_logger().warning("[Face] Failed to reset face state (face server offline)")
            self.execute_next_command()
            return

        if cmd_type == "effect":
            effect_type = str(value or "shake")
            ok = self.node.face_client.trigger_effect(effect_type)
            if ok:
                self.node.get_logger().info(f"[Effect] Triggered '{effect_type}'")
                if should_speak:
                    self.node.send_sirius_speak("[happy]演出を出したのだ！")
            else:
                self.node.get_logger().warning(f"[Effect] Failed to trigger '{effect_type}' (face server offline)")
            self.execute_next_command()
            return

        if cmd_type == "look":
            look_map = {
                "center": (960.0, 540.0),
                "left": (0.0, 540.0),
                "right": (1920.0, 540.0),
                "up": (960.0, 0.0),
                "down": (960.0, 1080.0),
            }
            try:
                if isinstance(value, dict):
                    look_x = float(value.get("x", 960.0))
                    look_y = float(value.get("y", 540.0))
                    look_name = "custom"
                elif isinstance(value, (list, tuple)) and len(value) >= 2:
                    look_x = float(value[0])
                    look_y = float(value[1])
                    look_name = "custom"
                else:
                    look_name = str(value or "center").lower()
                    look_x, look_y = look_map.get(look_name, look_map["center"])

                ok = self.node.face_client.look_at(look_x, look_y)
                if ok:
                    self.node.get_logger().info(f"[Look] Set gaze '{look_name}' -> x={look_x:.1f}, y={look_y:.1f}")
                    if should_speak:
                        self.node.send_sirius_speak("[happy]目線を動かしたのだ！")
                else:
                    self.node.get_logger().warning(f"[Look] Failed to set gaze '{look_name}' (face server offline)")
            except Exception as e:
                self.node.get_logger().error(f"[Look] Failed to parse look command: {e}")
            self.execute_next_command()
            return

        if cmd_type == "register_landmark":
            try:
                landmark_name = str(value).strip()
                ok, speak = self.node.landmark_mgr.register_current_pose_as_landmark(landmark_name)
                self.node.send_sirius_speak(speak)
            except Exception as e:
                self.node.get_logger().error(f"Failed to register landmark via executor: {e}")
            self.execute_next_command()
            return

        if cmd_type == "remove_landmark":
            try:
                landmark_name = str(value).strip()
                ok, speak = self.node.landmark_mgr.remove_landmark_by_name(landmark_name)
                self.node.send_sirius_speak(speak)
            except Exception as e:
                self.node.get_logger().error(f"Failed to remove landmark via executor: {e}")
            self.execute_next_command()
            return

        try:
            trans = self.node.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw_robot = math.atan2(siny_cosp, cosy_cosp)
            self.node.get_logger().info(f"📍 [Start Pose] X={tx:.3f}, Y={ty:.3f}, Yaw={math.degrees(yaw_robot):+.1f}deg")
            with self.node.lock:
                self.node.last_action_start_x = tx
                self.node.last_action_start_y = ty
                self.node.last_action_start_yaw = yaw_robot
        except Exception as e:
            self.node.get_logger().warning(f"Failed to lookup TF (map -> sirius3/base_footprint) during execution: {e}")
            self.execute_next_command()
            return

        if cmd_type in ["forward", "backward"]:
            with self.node.lock:
                active_x = self.node.active_goal_x
                active_y = self.node.active_goal_y
                active_yaw = self.node.active_goal_yaw
                logged = self.node.goal_reached_logged
                last_type = self.node.last_active_cmd_type
                
            if active_x is not None and not logged and last_type == cmd_type:
                base_x = active_x
                base_y = active_y
                base_yaw = active_yaw
                self.node.get_logger().info(f"Extending active {cmd_type} goal: X={base_x:.3f}, Y={base_y:.3f}, Yaw={math.degrees(base_yaw):+.1f}deg")
            else:
                base_x = tx
                base_y = ty
                base_yaw = yaw_robot
                
            with self.node.lock:
                self.node.last_active_cmd_type = cmd_type
        else:
            with self.node.lock:
                self.node.last_active_cmd_type = None
            base_x = tx
            base_y = ty
            base_yaw = yaw_robot
        
        if cmd_type not in ["forward", "backward"]:
            self.node.teleop_ctrl.stop_assisted_drive()

        if cmd_type not in ["turn", "spin", "face"]:
            with self.node.lock:
                self.node.turn_remaining_angle = None
                self.node.turn_target_yaw = None
                self.node.turn_arrival_triggered = False

        if cmd_type == "forward":
            self.node.teleop_ctrl.start_assisted_drive(1.0, value, should_speak=should_speak)
            
        elif cmd_type == "backward":
            self.node.teleop_ctrl.start_assisted_drive(-1.0, value, should_speak=should_speak)
            
        elif cmd_type == "turn":
            with self.node.lock:
                self.node.turn_remaining_angle = value
                self.node.last_yaw_robot = yaw_robot
                self.node.turn_target_yaw = None
                self.node.turn_arrival_triggered = False
            
            if should_speak:
                if speak_override:
                    turn_msg = speak_override
                else:
                    deg_val = round(math.degrees(abs(value)))
                    if abs(deg_val - 180) < 15:
                        turn_msg = "[wink]後ろを向くのだ！"
                    elif abs(deg_val - 90) < 15:
                        turn_msg = f"[wink]{'右' if value < 0 else '左'}に90度回るのだ！"
                    elif deg_val >= 300:
                        turn_msg = f"[wink]{'右' if value < 0 else '左'}にその場旋回するのだ！"
                    else:
                        turn_msg = f"[wink]{'右' if value < 0 else '左'}を向くのだ！"
                self.node.send_sirius_speak(turn_msg)
            self.node.nav_ctrl.send_spin_goal(value)
            
        elif cmd_type == "spin":
            total_deg = float(value) if value != 0.0 else 360.0
            rad_value = math.radians(total_deg)
            self.node.get_logger().info(f"[Spin] Converting {total_deg:+.0f}deg to turn command ({rad_value:+.3f} rad).")
            direction = "右" if rad_value < 0 else "左"
            spin_speak = f"[wink]{direction}にその場旋回するのだ！"
            with self.node.lock:
                self.node.command_queue.insert(0, {"type": "turn", "value": rad_value, "speak": spin_speak})
            self.execute_next_command()
            return
            
        elif cmd_type == "face":
            try:
                target_abs_deg = float(value)
                target_abs_yaw = math.radians(target_abs_deg)
                diff_yaw = target_abs_yaw - yaw_robot
                diff_yaw = (diff_yaw + math.pi) % (2 * math.pi) - math.pi
                
                if abs(diff_yaw) < math.radians(5.0):
                    self.node.get_logger().info("[Face] Already facing target direction. Skipping.")
                    self.execute_next_command()
                    return
                
                self.node.get_logger().info(
                    f"[Face] Current yaw={math.degrees(yaw_robot):+.1f}deg → "
                    f"Target={target_abs_deg:+.1f}deg, Δ={math.degrees(diff_yaw):+.1f}deg"
                )
                with self.node.lock:
                    self.node.command_queue.insert(0, {"type": "turn", "value": diff_yaw})
                self.execute_next_command()
                return
            except Exception as e:
                self.node.get_logger().error(f"[Face] Failed to parse angle: {e}")
                self.execute_next_command()
                return
                
        elif cmd_type == "goto":
            try:
                target_x = float(value[0])
                target_y = float(value[1])
                
                dx = target_x - tx
                dy = target_y - ty
                target_yaw = math.atan2(dy, dx)
                
                r = math.sqrt(dx**2 + dy**2)
                tolerance = max(r * 0.3, 0.15) if r < 1.0 else 0.50
                with self.node.lock:
                    self.node.current_xy_tolerance = tolerance
                self.node.nav_ctrl.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': tolerance})
                
                if speak_override:
                    self.node.send_sirius_speak(speak_override)
                elif abs(target_x) < 0.001 and abs(target_y) < 0.001:
                    self.node.send_sirius_speak("[happy]原点、Xゼロ、Yゼロに向かうのだ！")
                else:
                    self.node.send_sirius_speak(DIALOGUE_TEMPLATES["goto_start"].format(x=target_x, y=target_y))
                
                self.node.nav_ctrl.publish_direct_map_goal(target_x, target_y, target_yaw)
            except Exception as e:
                self.node.get_logger().error(f"Failed to parse coordinates for goto: {e}")
                self.execute_next_command()
                return
        else:
            self.node.get_logger().error(f"Unknown command type: {cmd_type}")
            self.execute_next_command()
            return
