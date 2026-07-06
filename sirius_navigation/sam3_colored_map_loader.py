import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from std_msgs.msg import Header
import json
import os
import struct

class SAM3ColoredMapLoader(Node):
    def __init__(self):
        super().__init__('sam3_colored_map_loader')
        
        # Parameters
        self.declare_parameter('map_path', '')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('z_offset', -0.2)
        # コスト閾値: default_cost >= この値のクラスのみ LETHAL(100) として出力する
        # 例: 50 に設定すれば grass(120) と tactile paving(50) のみが対象、sidewalk(10) は除外
        self.declare_parameter('lethal_cost_threshold', 50)
        self.declare_parameter('soft_semantic_max_cost', 70)
        self.declare_parameter('soft_semantic_min_cost', 5)
        self.declare_parameter('semantic_cost_classes', ['grass', 'tactile paving', 'roadway'])
        self.declare_parameter('global_lethal_classes', ['grass', 'roadway'])
        self.declare_parameter('soft_semantic_inflation_radius', 0.45)
        self.declare_parameter('soft_semantic_inflation_cost', 45)
        
        self.map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value
        self.lethal_cost_threshold = self.get_parameter('lethal_cost_threshold').get_parameter_value().integer_value
        self.soft_semantic_max_cost = self.get_parameter('soft_semantic_max_cost').get_parameter_value().integer_value
        self.soft_semantic_min_cost = self.get_parameter('soft_semantic_min_cost').get_parameter_value().integer_value
        self.semantic_cost_classes = {
            str(name).lower()
            for name in self.get_parameter('semantic_cost_classes').get_parameter_value().string_array_value
        }
        self.global_lethal_classes = {
            str(name).lower()
            for name in self.get_parameter('global_lethal_classes').get_parameter_value().string_array_value
        }
        self.soft_semantic_inflation_radius = (
            self.get_parameter('soft_semantic_inflation_radius').get_parameter_value().double_value
        )
        self.soft_semantic_inflation_cost = (
            self.get_parameter('soft_semantic_inflation_cost').get_parameter_value().integer_value
        )
        
        # Publishers
        self.pub_cloud = self.create_publisher(PointCloud2, '/sam3/static_colored_map_cloud', 10)
        
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_grid = self.create_publisher(OccupancyGrid, '/sam3/static_colored_map_grid', qos_latched)
        self.pub_soft_grid = self.create_publisher(OccupancyGrid, '/sam3/static_semantic_cost_grid', qos_latched)
        
        self.cloud_msg = None
        self.grid_msg = None
        self.soft_grid_msg = None
        
        if self.map_path:
            self.load_map(self.map_path)
        else:
            self.get_logger().warn('No map_path provided. Use: ros2 run sirius_navigation sam3_colored_map_loader --ros-args -p map_path:=/path/to/my_map')

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def load_map(self, path):
        # Handle extension-less path
        if path.endswith('.pgm') or path.endswith('.json'):
            base_path = os.path.splitext(path)[0]
            if base_path.endswith('.colored'):
                base_path = base_path[:-8]
        else:
            base_path = path

        # Try to load .colored.pgm / .colored.json first (preferred)
        pgm_file = base_path + ".colored.pgm"
        json_file = base_path + ".colored.json"

        # Fallback to standard .pgm / .json if .colored variants do not exist
        if not os.path.exists(pgm_file) or not os.path.exists(json_file):
            pgm_file = base_path + ".pgm"
            json_file = base_path + ".json"

        if not os.path.exists(pgm_file) or not os.path.exists(json_file):
            self.get_logger().error(f'File not found: {base_path}.colored.pgm or {base_path}.pgm')
            return


        self.get_logger().info(f'Loading map from {base_path}...')
        
        # Load PGM (indices)
        grid = cv2.imread(pgm_file, cv2.IMREAD_UNCHANGED)
        if grid is None:
            self.get_logger().error(f'Failed to read PGM: {pgm_file}')
            return
        
        # RTAB-Map convention / our saver convention: flipped vertically?
        # Our saver did: cv2.imwrite(path + ".pgm", self.grid[::-1, :])
        # So we flip it back
        grid = grid[::-1, :]
        
        # Load JSON (metadata)
        try:
            with open(json_file, 'r') as f:
                meta = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to read JSON: {e}')
            return

        res = meta.get('resolution', 0.05)
        origin = meta.get('origin', [0.0, 0.0])
        palette = np.array(meta.get('palette', []), dtype=np.uint8)
        
        if len(palette) == 0:
            self.get_logger().error('Palette is empty in JSON')
            return

        self.get_logger().info(f'Map Loaded: {grid.shape[1]}x{grid.shape[0]} at {res}m/pix')

        # Convert to PointCloud2
        rows, cols = np.where(grid > 0) # Skip unknown (0)
        if len(rows) == 0:
            self.get_logger().warn('No known cells (indices > 0) found in map.')
            return

        indices = grid[rows, cols]
        
        # Map indices to RGB
        rgb_list = palette[indices]
        
        # Map indices to coordinates
        x_coords = cols * res + origin[0]
        y_coords = rows * res + origin[1]
        
        points = []
        for i in range(len(rows)):
            r, g, b = rgb_list[i]
            # Pack RGB into a single float (standard ROS pattern)
            rgb_packed = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
            points.append([float(x_coords[i]), float(y_coords[i]), self.z_offset, rgb_packed])

        self.cloud_msg = pc2.create_cloud(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame),
            fields=[
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            points=points
        )
        # --- Generate OccupancyGrid for Costmap (Method A: Binary LETHAL mapping) ---
        # Nav2のStaticLayerはtrinary_costmap=falseの場合、OccupancyGridの中間値を
        # costmap costへ線形変換できる。グローバル用は従来どおり二値LETHAL、
        # ローカル用は復帰可能な非LETHAL高コストとして別トピックへ出す。
        # static_colored_map_grid: グローバル計画用。対象セマンティック領域をLETHAL化する。
        # static_semantic_cost_grid: ローカル制御用。芝生・点字ブロック等を非LETHAL高コストにする。
        h, w = grid.shape
        cost_grid = np.zeros((h, w), dtype=np.int8)
        soft_cost_grid = np.zeros((h, w), dtype=np.int8)
        
        labels = meta.get('labels', {})
        
        lethal_classes = []
        soft_classes = []
        # 登録されたラベルで default_cost >= lethal_cost_threshold のもの → LETHAL(100) として出力
        for idx_str, info in labels.items():
            try:
                idx = int(idx_str)
                cost = info.get('default_cost', 0)
                name = str(info.get('name', idx_str)).lower()
                if name in self.global_lethal_classes and cost >= self.lethal_cost_threshold:
                    cost_grid[grid == idx] = 100  # LETHAL
                    if info.get('name') not in lethal_classes:
                        lethal_classes.append(info.get('name', idx_str))

                if name in self.semantic_cost_classes and cost > 0:
                    if cost >= 254 or name == 'roadway':
                        soft_cost = 100
                    else:
                        soft_cost = int(np.clip(cost, self.soft_semantic_min_cost, self.soft_semantic_max_cost))
                    soft_cost_grid[grid == idx] = soft_cost
                    if soft_cost > 0 and info.get('name') not in soft_classes:
                        soft_classes.append(info.get('name', idx_str))
            except Exception:
                pass
        
        # 壁は通常のLiDAR/SLAM由来の /map に任せ、semantic gridでは扱わない。
        # RTAB-Mapの床ノイズが通常PGMで黒になった場合でも、semantic側で壁コスト化しない。

        soft_inflation_cells = int(round(self.soft_semantic_inflation_radius / res))
        if soft_inflation_cells > 0 and self.soft_semantic_inflation_cost > 0:
            soft_obstacle_mask = ((soft_cost_grid > 0) & (soft_cost_grid < 100)).astype(np.uint8)
            if np.any(soft_obstacle_mask):
                kernel_size = soft_inflation_cells * 2 + 1
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
                inflated_mask = cv2.dilate(soft_obstacle_mask, kernel, iterations=1).astype(bool)
                inflated_cost = int(
                    np.clip(
                        self.soft_semantic_inflation_cost,
                        self.soft_semantic_min_cost,
                        self.soft_semantic_max_cost
                    )
                )
                soft_cost_grid[inflated_mask] = np.maximum(soft_cost_grid[inflated_mask], inflated_cost)
        
        # ログ出力
        semantic_cells = int(np.sum(cost_grid == 100))
        self.get_logger().info(
            f'Semantic costmap: {semantic_cells} LETHAL cells '
            f'(threshold>={self.lethal_cost_threshold}, targets: {list(set(lethal_classes))})'
        )
        soft_cells = int(np.sum(soft_cost_grid > 0))
        soft_lethal_cells = int(np.sum(soft_cost_grid == 100))
        self.get_logger().info(
            f'Soft semantic costmap: {soft_cells} costed cells, '
            f'{soft_lethal_cells} LETHAL cells '
            f'(soft range {self.soft_semantic_min_cost}-{self.soft_semantic_max_cost}, '
            f'targets: {list(set(soft_classes))})'
        )
        
        self.grid_msg = OccupancyGrid()
        self.grid_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame)
        self.grid_msg.info = MapMetaData(
            resolution=res,
            width=w,
            height=h
        )
        self.grid_msg.info.origin.position.x = origin[0]
        self.grid_msg.info.origin.position.y = origin[1]
        self.grid_msg.info.origin.orientation.w = 1.0
        self.grid_msg.data = cost_grid.flatten().tolist()

        self.soft_grid_msg = OccupancyGrid()
        self.soft_grid_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame)
        self.soft_grid_msg.info = MapMetaData(
            resolution=res,
            width=w,
            height=h
        )
        self.soft_grid_msg.info.origin.position.x = origin[0]
        self.soft_grid_msg.info.origin.position.y = origin[1]
        self.soft_grid_msg.info.origin.orientation.w = 1.0
        self.soft_grid_msg.data = soft_cost_grid.flatten().tolist()


    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()
        if self.cloud_msg is not None:
            self.cloud_msg.header.stamp = stamp
            self.pub_cloud.publish(self.cloud_msg)
        if self.grid_msg is not None:
            self.grid_msg.header.stamp = stamp
            self.pub_grid.publish(self.grid_msg)
        if self.soft_grid_msg is not None:
            self.soft_grid_msg.header.stamp = stamp
            self.pub_soft_grid.publish(self.soft_grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SAM3ColoredMapLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
