#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
import math
import signal

class TargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower')
        
        # parameters
        self.declare_parameter('enable_following', True)
        self.declare_parameter('follow_distance', 1.2)
        self.declare_parameter('min_update_distance', 0.2)
        self.declare_parameter('control_rate', 1.0) # Hz
        self.declare_parameter('deadband', 0.15) # Deadband to prevent oscillation (meters)
        
        self.enable_following = self.get_parameter('enable_following').value
        self.follow_distance = self.get_parameter('follow_distance').value
        self.min_update_distance = self.get_parameter('min_update_distance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.deadband = self.get_parameter('deadband').value
        
        # Parameter update callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Action Client for Nav2 NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Service Client for canceling goals directly (robust backup)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriber to target ground truth pose (published by Unity NPC)
        self.target_sub = self.create_subscription(
            Odometry,
            '/npc/odom',
            self.target_callback,
            10
        )
        
        self.target_pose = None
        self.last_sent_target_pose = None
        self.goal_handle = None
        self.goal_sending_in_progress = False
        
        # Timer for control loop
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        self.get_logger().info(
            f"Target Follower Node Initialized.\n"
            f"  Subscribing to: /npc/odom\n"
            f"  Follow Distance: {self.follow_distance}m\n"
            f"  Min Update Distance: {self.min_update_distance}m"
        )

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'enable_following':
                self.enable_following = param.value
                self.get_logger().info(f"Parameter 'enable_following' updated to: {self.enable_following}")
                if not self.enable_following:
                    self.cancel_current_goal()
            elif param.name == 'follow_distance':
                self.follow_distance = param.value
                self.get_logger().info(f"Parameter 'follow_distance' updated to: {self.follow_distance}m")
            elif param.name == 'min_update_distance':
                self.min_update_distance = param.value
                self.get_logger().info(f"Parameter 'min_update_distance' updated to: {self.min_update_distance}m")
            elif param.name == 'deadband':
                self.deadband = param.value
                self.get_logger().info(f"Parameter 'deadband' updated to: {self.deadband}m")
            elif param.name == 'control_rate':
                self.control_rate = param.value
                self.get_logger().info(f"Parameter 'control_rate' updated to: {self.control_rate}Hz")
                # Re-create timer with new rate
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        return SetParametersResult(successful=True)

    def target_callback(self, msg: Odometry):
        # Store target pose (from Unity map frame)
        self.target_pose = msg.pose.pose

    def cancel_current_goal(self):
        self.get_logger().info("Canceling all active NavigateToPose goals...")
        self.last_sent_target_pose = None
        self.goal_handle = None
        
        # Wait for service briefly
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Cancel goal service not available. Trying action client cancel...")
            # Fallback to action client goal handle if available
            if self.goal_handle is not None:
                try:
                    self.goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().error(f"Failed fallback cancel: {e}")
            return
            
        # Send empty goal info request (cancels all goals)
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        
        # Send async call
        self.cancel_client.call_async(req)

    def control_loop(self):
        if not self.enable_following:
            return
            
        if self.target_pose is None:
            self.get_logger().info("Waiting for target pose on /npc/odom...", throttle_duration_sec=5.0)
            return

        # 1. Lookup robot pose using TF (map -> base_footprint)
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except LookupException as e:
            self.get_logger().warning(f"Could not look up robot transform: {e}", throttle_duration_sec=3.0)
            return
        except Exception as e:
            self.get_logger().warning(f"TF error: {e}", throttle_duration_sec=3.0)
            return

        # Target position
        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y

        # Calculate distance between robot and target
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.sqrt(dx**2 + dy**2)

        # Calculate deviation from target follow distance
        dist_diff = dist - self.follow_distance

        # 2. If within deadband (i.e. close enough), stop and cancel goal to avoid jitter
        if abs(dist_diff) <= self.deadband:
            self.get_logger().info(
                f"Target is within deadband ({dist:.2f}m is close to {self.follow_distance:.2f}m). Stopping.", 
                throttle_duration_sec=3.0
            )
            self.cancel_current_goal()
            return

        # 3. Check if target has moved enough since last sent goal
        if self.last_sent_target_pose is not None:
            tdx = target_x - self.last_sent_target_pose.position.x
            tdy = target_y - self.last_sent_target_pose.position.y
            moved_dist = math.sqrt(tdx**2 + tdy**2)
            if moved_dist < self.min_update_distance:
                # Target has not moved enough, skip updating goal to prevent spamming
                return

        # 4. Wait for action server to be ready
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning("NavigateToPose action server is not available.", throttle_duration_sec=5.0)
            return

        # 5. Prevent sending concurrent goal requests
        if self.goal_sending_in_progress:
            return

        # Calculate goal position (target offset by follow distance along line-of-sight)
        theta = math.atan2(dy, dx)
        goal_x = target_x - self.follow_distance * math.cos(theta)
        goal_y = target_y - self.follow_distance * math.sin(theta)

        # Formulate Goal Message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        
        # Face the target (quaternion yaw)
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.get_logger().info(
            f"Sending target tracking goal: Dist={dist:.2f}m -> Goal: ({goal_x:.2f}, {goal_y:.2f}) Yaw={theta:.2f}rad"
        )
        
        self.last_sent_target_pose = self.target_pose
        self.goal_sending_in_progress = True
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_sending_in_progress = False
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning("Follow goal was REJECTED by Nav2.")
                return
            
            # Store active goal handle for cancellation fallback
            self.goal_handle = goal_handle
        except Exception as e:
            self.get_logger().error(f"Error during goal response callback: {e}")

def sigterm_handler(signum, frame):
    raise KeyboardInterrupt

def main(args=None):
    # Setup signal handler for clean shutdown on pkill (SIGTERM)
    signal.signal(signal.SIGTERM, sigterm_handler)
    
    rclpy.init(args=args)
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_current_goal()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
