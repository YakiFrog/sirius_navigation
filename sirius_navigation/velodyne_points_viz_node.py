import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class VelodynePointsVizNode(Node):
    def __init__(self):
        super().__init__('velodyne_points_viz')
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_viz')
        self.declare_parameter('rate_hz', 10.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            rate_hz = 10.0

        self._latest = None
        self._updated = False
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, 5)
        publish_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._publisher = self.create_publisher(PointCloud2, output_topic, publish_qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_cloud(self, msg):
        self._latest = msg
        self._updated = True

    def _on_timer(self):
        if not self._updated or self._latest is None:
            return
        self._publisher.publish(self._latest)
        self._updated = False


def main(args=None):
    rclpy.init(args=args)
    node = VelodynePointsVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
