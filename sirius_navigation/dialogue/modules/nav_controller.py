# -*- coding: utf-8 -*-
import math
import threading
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin, AssistedTeleop
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from visualization_msgs.msg import Marker

try:
    from ..local_parser import DIALOGUE_TEMPLATES
except ImportError:
    from local_parser import DIALOGUE_TEMPLATES
from .landmark_manager import JAPANESE_TO_ROMAJI

class NavController:
    def __init__(self, node):
        self.node = node

    def publish_goal_pose(self, tx, ty, yaw_robot, r, theta):
        """相対目標値(r, theta)をmap絶対座標に変換してパブリッシュする内部メソッド"""
        target_map_x = tx + (r * math.cos(theta) * math.cos(yaw_robot) - r * math.sin(theta) * math.sin(yaw_robot))
        target_map_y = ty + (r * math.cos(theta) * math.sin(yaw_robot) + r * math.sin(theta) * math.cos(yaw_robot))
        target_map_yaw = yaw_robot + theta
        self.publish_direct_map_goal(target_map_x, target_map_y, target_map_yaw)

    def publish_direct_map_goal(self, target_x, target_y, target_yaw):
        """map座標系に直接目標ゴールをパブリッシュする"""
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = target_x
        pose_msg.pose.position.y = target_y
        pose_msg.pose.position.z = 0.0
        
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(target_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(target_yaw / 2.0)
            
        self.node.goal_pub.publish(pose_msg)
        self.publish_marker(pose_msg)
        
        with self.node.lock:
            self.node.active_goal_x = target_x
            self.node.active_goal_y = target_y
            self.node.active_goal_yaw = target_yaw
            self.node.goal_reached_logged = False
            self.node.distance_remaining_history = []
            self.node.yaw_diff_history = []
            self.node.is_stuck = False
            
        self.node.get_logger().info(f"Published goal pose to /goal_pose: X={target_x:.2f}, Y={target_y:.2f}, Yaw={math.degrees(target_yaw):+.1f}deg")

    def publish_marker(self, pose_msg):
        """目標位置にRViz可視化用のマーカーをパブリッシュする"""
        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = 'llm_goal'
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        
        marker.scale.x = 0.6
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.9
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
        self.node.marker_pub.publish(marker)

    def delete_marker(self):
        """RViz上のマーカーを削除する"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = 'llm_goal'
        marker.id = 1
        marker.action = Marker.DELETE
        self.node.marker_pub.publish(marker)

    def cancel_navigation(self, clear_queue=True, preserve_current_goal=False):
        """実行中のナビゲーションをキャンセル"""
        self.delete_marker()
        
        # 前進・後退のアシスト走行（タイマーパブリッシャー）を停止する
        self.node.teleop_ctrl.stop_assisted_drive()
        
        stuck_msg_bool = Bool()
        stuck_msg_bool.data = False
        self.node.stuck_pub.publish(stuck_msg_bool)
        
        with self.node.lock:
            spin_handle = self.node.spin_goal_handle
            self.node.spin_goal_handle = None
            assisted_handle = self.node.assisted_teleop_goal_handle
            self.node.assisted_teleop_goal_handle = None
            
        if spin_handle is not None:
            self.node.get_logger().info("Canceling active Spin action...")
            spin_handle.cancel_goal_async()

        if assisted_handle is not None:
            self.node.get_logger().info("Canceling active AssistedTeleop action...")
            assisted_handle.cancel_goal_async()
            
        with self.node.lock:
            if preserve_current_goal and self.node.active_goal_x is not None and self.node.active_goal_y is not None:
                self.node.paused_goal_snapshot = {
                    "active_goal_x": self.node.active_goal_x,
                    "active_goal_y": self.node.active_goal_y,
                    "active_goal_yaw": self.node.active_goal_yaw,
                    "last_action_type": self.node.last_action_type,
                    "last_target_value": self.node.last_target_value,
                    "turn_remaining_angle": self.node.turn_remaining_angle,
                    "turn_target_yaw": self.node.turn_target_yaw,
                    "current_xy_tolerance": self.node.current_xy_tolerance,
                }
            elif not preserve_current_goal:
                self.node.paused_goal_snapshot = None
            self.node.active_goal_x = None
            self.node.active_goal_y = None
            self.node.active_goal_yaw = None
            self.node.goal_reached_logged = True
            self.node.distance_remaining_history = []
            self.node.yaw_diff_history = []
            self.node.is_stuck = False
            self.node.turn_target_yaw = None
            if not self.node.turn_arrival_triggered:
                self.node.turn_arrival_triggered = False
            if self.node.last_action_status not in ["success", "failed_stuck"]:
                self.node.last_action_status = "failed_cancelled"
            
            if clear_queue:
                import time
                t_str = time.strftime('%H:%M:%S')
                if self.node.active_command:
                    self.node.active_command["status"] = "cancelled"
                    self.node.active_command["time"] = t_str
                    self.node.command_history.append(self.node.active_command)
                    self.node.active_command = None
                
                for cmd in self.node.command_queue:
                    cmd_copy = dict(cmd)
                    cmd_copy["status"] = "cancelled"
                    cmd_copy["time"] = t_str
                    self.node.command_history.append(cmd_copy)
                
                self.node.command_queue = []
                self.node.executing_command = False
                self.node.current_xy_tolerance = 0.50
        
        self.node.publish_queue_status()
        self.node.get_logger().info(
            "Navigation state updated: "
            f"{'pause' if preserve_current_goal else 'cancel'} "
            f"(clear_queue={clear_queue})"
        )
            
        if clear_queue:
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})

        if not self.node.cancel_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warning("Navigation cancel service not available.")
            return
        
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        try:
            self.node.get_logger().info("Sending asynchronous NavigateToPose cancel request...")
            future = self.node.cancel_client.call_async(req)
            
            def done_callback(f):
                try:
                    f.result()
                    self.node.get_logger().info("NavigateToPose cancel request completed.")
                except Exception as e:
                    self.node.get_logger().error(f"NavigateToPose cancel request failed: {e}")
            future.add_done_callback(done_callback)
        except Exception as e:
            self.node.get_logger().error(f"Failed to call NavigateToPose cancel service: {e}")

        # コントローラーが完全に減速停止し、Nav2状態が落ち着くまで少し待機する
        import time
        time.sleep(0.4)
        
        stop_msg = Bool()
        stop_msg.data = True
        self.node.stop_pub.publish(stop_msg)
        
        def reset_stop():
            msg = Bool()
            msg.data = False
            self.node.stop_pub.publish(msg)
        threading.Timer(1.0, reset_stop).start()

    def resume_navigation(self):
        """停止前の目標を再開する"""
        with self.node.lock:
            snapshot = self.node.paused_goal_snapshot

        if not snapshot:
            return False

        if snapshot.get("active_goal_x") is not None and snapshot.get("active_goal_y") is not None:
            self.publish_direct_map_goal(
                snapshot["active_goal_x"],
                snapshot["active_goal_y"],
                snapshot["active_goal_yaw"] if snapshot.get("active_goal_yaw") is not None else 0.0,
            )
            with self.node.lock:
                self.node.executing_command = True
                self.node.command_start_time = self.node.get_clock().now()
                self.node.last_active_cmd_type = snapshot.get("last_action_type", "goto")
                self.node.current_xy_tolerance = snapshot.get("current_xy_tolerance", 0.50)
                self.node.last_action_status = "none"
                self.node.distance_remaining_history = []
                self.node.yaw_diff_history = []
                self.node.is_stuck = False
                self.node.paused_goal_snapshot = None
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': self.node.current_xy_tolerance})
            return True

        return False

    def publish_nav_control(self, command: str):
        """move_goal.py へ明示的な制御コマンドを送る"""
        msg = String()
        msg.data = command
        self.node.nav_control_pub.publish(msg)

    def ensure_assisted_teleop_goal(self):
        """Nav2 AssistedTeleop action を開始して、cmd_vel_teleop を安全経路に通す。"""
        if self.node.assisted_teleop_goal_handle or self.node.assisted_teleop_goal_pending:
            return

        if not self.node.assisted_teleop_client.server_is_ready():
            self.node.get_logger().warning(
                "[AssistedTeleop] action server is not ready. "
                "cmd_vel_teleop may not be obstacle-filtered yet."
            )
            return

        goal_msg = AssistedTeleop.Goal()
        goal_msg.time_allowance = Duration(sec=600)
        self.node.assisted_teleop_goal_pending = True
        future = self.node.assisted_teleop_client.send_goal_async(goal_msg)
        future.add_done_callback(self._assisted_teleop_goal_response_callback)

    def _assisted_teleop_goal_response_callback(self, future):
        self.node.assisted_teleop_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.get_logger().error(f"[AssistedTeleop] failed to start action: {exc}")
            return

        if not goal_handle.accepted:
            self.node.get_logger().error("[AssistedTeleop] action goal was rejected")
            return

        self.node.assisted_teleop_goal_handle = goal_handle
        self.node.get_logger().info("[AssistedTeleop] action goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._assisted_teleop_result_callback)

    def _assisted_teleop_result_callback(self, future):
        self.node.assisted_teleop_goal_handle = None
        try:
            result = future.result()
            self.node.get_logger().info(f"[AssistedTeleop] action finished status={result.status}")
        except Exception as exc:
            self.node.get_logger().warning(f"[AssistedTeleop] action result error: {exc}")

    def set_controller_speed(self, speed_setting):
        """Configure controller_server and velocity_smoother based on navigation mode configs"""
        nav_modes = {
            'slow': {
                '/controller_server': {
                    'FollowPath.vx_max': 0.20,
                    'FollowPath.vx_min': -0.10,
                    'FollowPath.wz_max': 0.20,
                    'FollowPath.vx_std': 0.20,
                    'FollowPath.wz_std': 0.20,
                    'FollowPath.ax_max': 0.20,
                    'FollowPath.ax_min': -0.20,
                    'FollowPath.az_max': 0.50,
                },
                '/velocity_smoother': {
                    'max_velocity': [0.20, 0.0, 0.20],
                    'min_velocity': [-0.10, 0.0, -0.20],
                    'max_accel': [0.20, 0.0, 0.50],
                    'max_decel': [-0.20, 0.0, -0.50]
                }
            },
            'safe': {
                '/controller_server': {
                    'FollowPath.vx_max': 0.40,
                    'FollowPath.vx_min': -0.20,
                    'FollowPath.wz_max': 0.40,
                    'FollowPath.vx_std': 0.20,
                    'FollowPath.wz_std': 0.20,
                    'FollowPath.ax_max': 0.40,
                    'FollowPath.ax_min': -0.40,
                    'FollowPath.az_max': 1.00,
                },
                '/velocity_smoother': {
                    'max_velocity': [0.40, 0.0, 0.40],
                    'min_velocity': [-0.20, 0.0, -0.40],
                    'max_accel': [0.40, 0.0, 1.00],
                    'max_decel': [-0.40, 0.0, -1.00]
                }
            },
            'normal': {
                '/controller_server': {
                    'FollowPath.vx_max': 0.90,
                    'FollowPath.vx_min': -0.60,
                    'FollowPath.wz_max': 0.90,
                    'FollowPath.vx_std': 0.25,
                    'FollowPath.wz_std': 0.30,
                    'FollowPath.ax_max': 0.90,
                    'FollowPath.ax_min': -0.90,
                    'FollowPath.az_max': 1.50,
                },
                '/velocity_smoother': {
                    'max_velocity': [0.90, 0.0, 0.90],
                    'min_velocity': [-0.90, 0.0, -0.90],
                    'max_accel': [0.90, 0.0, 1.50],
                    'max_decel': [-0.90, 0.0, -1.50]
                }
            },
            'fast': {
                '/controller_server': {
                    'FollowPath.vx_max': 1.00,
                    'FollowPath.vx_min': -0.60,
                    'FollowPath.wz_max': 1.00,
                    'FollowPath.vx_std': 0.40,
                    'FollowPath.wz_std': 0.48,
                    'FollowPath.ax_max': 1.50,
                    'FollowPath.ax_min': -1.50,
                    'FollowPath.az_max': 2.20,
                },
                '/velocity_smoother': {
                    'max_velocity': [1.00, 0.0, 1.00],
                    'min_velocity': [-0.60, 0.0, -1.00],
                    'max_accel': [1.50, 0.0, 2.20],
                    'max_decel': [-1.50, 0.0, -2.20]
                }
            }
        }

        if isinstance(speed_setting, str):
            mode = speed_setting.lower()
        else:
            try:
                val = float(speed_setting)
                if val <= 0.25:
                    mode = 'slow'
                elif val <= 0.55:
                    mode = 'safe'
                elif val <= 0.95:
                    mode = 'normal'
                else:
                    mode = 'fast'
            except Exception:
                mode = 'normal'
                
        if mode not in nav_modes:
            mode = 'normal'
            
        self.node.get_logger().info(f"Applying navigation mode config: '{mode}'")
        with self.node.lock:
            self.node.current_speed_setting = {
                "slow": 0.20,
                "safe": 0.40,
                "normal": 0.90,
                "fast": 1.00,
            }.get(mode, 0.90)
        
        cfg = nav_modes[mode]
        for node_name, params in cfg.items():
            self.set_node_parameters(node_name, params)

    def set_node_parameters(self, node_name, params_dict):
        """Helper to call SetParameters service asynchronously on target node"""
        srv_name = f'{node_name}/set_parameters'
        client = self.node.create_client(SetParameters, srv_name)
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warning(f"Service {srv_name} not available!")
            return
            
        req = SetParameters.Request()
        for name, val in params_dict.items():
            param = Parameter()
            param.name = name
            
            p_val = ParameterValue()
            if isinstance(val, bool):
                p_val.type = ParameterType.PARAMETER_BOOL
                p_val.bool_value = val
            elif isinstance(val, int):
                p_val.type = ParameterType.PARAMETER_INTEGER
                p_val.integer_value = val
            elif isinstance(val, float):
                p_val.type = ParameterType.PARAMETER_DOUBLE
                p_val.double_value = val
            elif isinstance(val, list):
                if len(val) > 0 and isinstance(val[0], float):
                    p_val.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                    p_val.double_array_value = [float(v) for v in val]
            param.value = p_val
            req.parameters.append(param)
            
        client.call_async(req)

    def send_spin_goal(self, relative_yaw):
        if not self.node.spin_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("Spin action server not available!")
            self.node.cmd_executor.execute_next_command()
            return
            
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = float(relative_yaw)
        
        self.node.get_logger().info(f"Sending Spin action goal: target_yaw={math.degrees(relative_yaw):+.1f}deg")
        
        with self.node.lock:
            self.node.spin_goal_handle = None
            
        self.node.spin_send_goal_future = self.node.spin_client.send_goal_async(goal_msg)
        self.node.spin_send_goal_future.add_done_callback(self.spin_goal_response_callback)
        
    def spin_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error("Spin goal rejected by server")
                self.node.cmd_executor.execute_next_command()
                return
                
            self.node.get_logger().info("Spin goal accepted by server")
            with self.node.lock:
                self.node.spin_goal_handle = goal_handle
                
            self.node.spin_result_future = goal_handle.get_result_async()
            self.node.spin_result_future.add_done_callback(self.spin_goal_result_callback)
        except Exception as e:
            self.node.get_logger().error(f"Error in spin goal response: {e}")
            self.node.cmd_executor.execute_next_command()

    def spin_goal_result_callback(self, future):
        self.node.get_logger().info("Spin goal finished execution")
        
        success = False
        status = None
        status_name = "UNKNOWN"
        result_error = None
        try:
            result_msg = future.result()
            status = result_msg.status
            status_name = {
                0: "不明",
                1: "受理済み",
                2: "実行中",
                3: "キャンセル中",
                4: "成功",
                5: "キャンセル完了",
                6: "中止",
            }.get(status, f"STATUS_{status}")
            if status == 4:
                success = True
        except Exception as e:
            result_error = str(e)

        with self.node.lock:
            if self.node.turn_arrival_triggered:
                success = True
                self.node.turn_arrival_triggered = False

        yaw_robot = None
        end_x = None
        end_y = None
        try:
            trans = self.node.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time()
            )
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw_robot = math.atan2(siny_cosp, cosy_cosp)
            end_x = trans.transform.translation.x
            end_y = trans.transform.translation.y
            self.node.get_logger().info(
                f"📍 [End Pose] X={end_x:.3f}, Y={end_y:.3f}, Yaw={math.degrees(yaw_robot):+.1f}deg"
            )
        except Exception:
            pass

        with self.node.lock:
            self.node.spin_goal_handle = None
            self.node.executing_command = False
            self.node.current_xy_tolerance = 0.50
            has_more = bool(self.node.command_queue)
            obs_dists = getattr(self.node, 'obstacle_distances', {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0})
            nearest_obstacle = min(obs_dists.values()) if obs_dists else 999.0
            requested_turn = self.node.turn_remaining_angle
            start_yaw = self.node.last_action_start_yaw
            achieved_turn = None
            yaw_error_deg = None
            if yaw_robot is not None and requested_turn is not None:
                achieved_turn = (yaw_robot - start_yaw + math.pi) % (2 * math.pi) - math.pi
                yaw_error_deg = abs(math.degrees(requested_turn - achieved_turn))
                yaw_error_deg = min(yaw_error_deg, 360.0 - yaw_error_deg)
            if success:
                self.node.last_action_status = "success"
                self.node.last_final_distance_error = 0.0
                self.node.last_final_yaw_error = 0.0
                self.node.is_stuck = False
            else:
                self.node.last_action_status = "failed_stuck" if nearest_obstacle < 0.8 else "failed"
                self.node.last_final_distance_error = 0.0
                self.node.last_final_yaw_error = yaw_error_deg if yaw_error_deg is not None else (
                    abs(math.degrees(self.node.turn_remaining_angle)) if self.node.turn_remaining_angle is not None else 0.0
                )
                self.node.is_stuck = nearest_obstacle < 0.8
                self.node.llm_client.chat_history.append({
                    "role": "assistant",
                    "content": "【System Feedback】旋回に失敗しました。近くの障害物で安全に回れなかった可能性があります。"
                })

        achieved_str = "unknown" if achieved_turn is None else f"{math.degrees(achieved_turn):+.1f}deg"
        requested_str = "unknown" if requested_turn is None else f"{math.degrees(requested_turn):+.1f}deg"
        yaw_error_str = "unknown" if yaw_error_deg is None else f"{yaw_error_deg:.1f}deg"
        end_pose_str = (
            f"({end_x:.3f}, {end_y:.3f})" if end_x is not None and end_y is not None else "不明"
        )
        spin_result_msg = (
            "[旋回結果] "
            f"成功={success} 状態={status_name}({status}) "
            f"要求={requested_str} 実績={achieved_str} yaw誤差={yaw_error_str} "
            f"最寄り障害物={nearest_obstacle:.2f}m 障害物={dict(obs_dists)} "
            f"終了位置={end_pose_str}"
        )
        if success:
            self.node.get_logger().info(spin_result_msg)
        else:
            self.node.get_logger().warning(spin_result_msg)
            
        if has_more:
            self.node.cmd_executor.execute_next_command()
        else:
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})
            if success:
                self.node.send_sirius_speak(DIALOGUE_TEMPLATES["turn_success"])
            else:
                self.node.send_sirius_speak(DIALOGUE_TEMPLATES["turn_failure"])
            print("\n✅ 旋回完了！次の指示をどうぞ。")
            self.node.print_prompt()
