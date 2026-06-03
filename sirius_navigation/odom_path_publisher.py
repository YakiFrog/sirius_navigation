#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
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
        self.last_time = None
        self.pose_speeds = []  # list of tuples: (translation, speed)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.marker_pub = self.create_publisher(Marker, path_topic + '_marker', 10)
        
        # Timer for TF lookup
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)
        
        self.get_logger().info(
            f"Initialized OdomPathPublisher. Listening to TF: '{self.map_frame}' -> '{self.robot_frame}' "
            f"and publishing path to '{path_topic}' and marker path to '{path_topic}_marker' "
            f"(min_dist: {self.min_distance}m, max_poses: {self.max_poses})"
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
        current_time = rclpy.time.Time.from_msg(trans.header.stamp)
        
        # Check if we moved enough from the last recorded pose and compute speed
        speed = 0.0
        if self.last_pose is not None:
            dx = current_pose.translation.x - self.last_pose.translation.x
            dy = current_pose.translation.y - self.last_pose.translation.y
            dz = current_pose.translation.z - self.last_pose.translation.z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < self.min_distance:
                return
            
            if self.last_time is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt > 0:
                    speed = distance / dt
        
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
        
        # Track speed along with translation
        self.pose_speeds.append((current_pose.translation, speed))
        
        # Enforce max poses limit (FIFO)
        if self.max_poses > 0 and len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)
            self.pose_speeds.pop(0)
            
        # Update last pose
        self.last_pose = current_pose
        self.last_time = current_time
        
        # Publish path
        self.path_pub.publish(self.path_msg)
        
        # Publish marker
        self.publish_marker(trans.header.stamp)

    def publish_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "robot_path_velocity"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        
        # Line width (5cm)
        marker.scale.x = 0.05
        
        # We use a Red-to-Yellow gradient:
        # - Slow (0.0 m/s): Pure Red (R=1.0, G=0.0, B=0.0)
        # - Threshold (0.5 m/s): Orange (R=1.0, G=0.5, B=0.0)
        # - Fast (>= 1.0 m/s): Pure Yellow (R=1.0, G=1.0, B=0.0)
        for translation, speed in self.pose_speeds:
            p = Point()
            p.x = translation.x
            p.y = translation.y
            p.z = translation.z
            marker.points.append(p)
            
            # Interpolate Green component to transition Yellow -> Red
            if speed < 0.5:
                # 0.0 to 0.5 m/s maps to G: 1.0 down to 0.5 (Yellow to Orange)
                g_val = 1.0 - (speed / 0.5) * 0.5
            else:
                # 0.5 to 1.0 m/s maps to G: 0.5 down to 0.0 (Orange to Red)
                g_val = 0.5 - ((speed - 0.5) / 0.5) * 0.5
                g_val = max(0.0, g_val)
            
            c = ColorRGBA()
            c.r = 1.0
            c.g = float(g_val)
            c.b = 0.0
            c.a = 1.0  # Keep fully opaque for clear visibility
            marker.colors.append(c)
            
        self.marker_pub.publish(marker)

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
