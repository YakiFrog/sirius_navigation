#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__('odom_path_publisher')
        
        # Declare parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'sirius3/base_footprint')
        self.declare_parameter('path_topic', '/robot_path')
        self.declare_parameter('min_distance', 0.1)  # meters
        self.declare_parameter('max_poses', 2000)    # Maximum number of poses to keep
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        path_topic = self.get_parameter('path_topic').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_poses = self.get_parameter('max_poses').value
        update_rate = self.get_parameter('update_rate').value
        
        # TF Listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize variables
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        self.last_pose = None
        
        # Publisher
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        
        # Timer for TF lookup
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)
        
        self.get_logger().info(
            f"Initialized OdomPathPublisher. Listening to TF: '{self.map_frame}' -> '{self.robot_frame}' "
            f"and publishing path to '{path_topic}' (min_dist: {self.min_distance}m, max_poses: {self.max_poses})"
        )

    def timer_callback(self):
        try:
            # Lookup latest transform from map to base_link
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                now
            )
        except TransformException as ex:
            # Silent warning to avoid spamming console during startup
            return

        # Extract current pose from transform
        current_pose = trans.transform
        
        # Check if we moved enough from the last recorded pose
        if self.last_pose is not None:
            dx = current_pose.translation.x - self.last_pose.translation.x
            dy = current_pose.translation.y - self.last_pose.translation.y
            dz = current_pose.translation.z - self.last_pose.translation.z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < self.min_distance:
                return
        
        # Create PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = trans.header.stamp
        pose_stamped.header.frame_id = self.map_frame
        
        pose_stamped.pose.position.x = current_pose.translation.x
        pose_stamped.pose.position.y = current_pose.translation.y
        pose_stamped.pose.position.z = current_pose.translation.z
        pose_stamped.pose.orientation = current_pose.rotation
        
        # Append to path
        self.path_msg.header.stamp = trans.header.stamp
        self.path_msg.poses.append(pose_stamped)
        
        # Enforce max poses limit (FIFO)
        if self.max_poses > 0 and len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)
            
        # Update last pose
        self.last_pose = current_pose
        
        # Publish path
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
