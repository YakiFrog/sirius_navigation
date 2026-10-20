#!/usr/bin/env python3
"""3D地図への局所自己位置推定ノード（small_gicp）。

- 事前3D地図（.npy/.ply/.pcd）を読み、LiDARスキャンを GICP/VGICP で位置合わせする。
- FAST-LIO2 などのオドメトリ（odom->base TF）を初期値として使う。
- 大域探索は行わない（勝手に飛ばない）。補正量をクランプし、過大な補正は棄却する。
- 初期姿勢は パラメータ / ファイル / RVizの /initialpose で指定できる。
- 出力は map->odom のTFのみ（AMCLとは排他で使用する）。
"""

import math
import os
import threading

import numpy as np
import rclpy
import small_gicp
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def _matrix_from_transform(tf):
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
    return T


def _yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return T


def _pose_matrix(x, y, z, yaw):
    T = _yaw_matrix(yaw)
    T[:3, 3] = [x, y, z]
    return T


def _quat_from_matrix(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _rotation_angle_deg(R):
    cos_a = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_a))))


def _pc2_to_xyz(msg: PointCloud2):
    offsets = {f.name: f.offset for f in msg.fields}
    if not all(k in offsets for k in ("x", "y", "z")):
        return None
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [(">" if msg.is_bigendian else "<") + "f4"] * 3,
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": msg.point_step,
    })
    arr = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step))
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).reshape(-1, 3).astype(np.float64)


def _read_pcd_ascii(path):
    with open(path, "r", errors="ignore") as f:
        fields = None
        data_start = False
        rows = []
        for line in f:
            s = line.strip()
            if not s:
                continue
            if not data_start:
                if s.startswith("FIELDS"):
                    fields = s.split()[1:]
                elif s.startswith("DATA"):
                    if "ascii" not in s:
                        raise ValueError("binary PCD is not supported; use ascii or .npy")
                    data_start = True
                continue
            rows.append([float(v) for v in s.split()])
    if fields is None or not rows:
        raise ValueError("invalid PCD")
    idx = [fields.index(k) for k in ("x", "y", "z")]
    arr = np.asarray(rows, dtype=np.float64)
    return arr[:, idx]


_PLY_TYPES = {
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
}


def _read_ply_xyz(path):
    """PLY（ascii / binary_little_endian）から x,y,z を読む（rgb等の追加属性は無視）。"""
    with open(path, "rb") as f:
        header_lines = []
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError("invalid PLY header")
            line = raw.decode("ascii", errors="ignore").strip()
            header_lines.append(line)
            if line == "end_header":
                break

        fmt = None
        count = 0
        props = []
        in_vertex = False
        for line in header_lines:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "format":
                fmt = parts[1]
            elif parts[0] == "element":
                in_vertex = len(parts) > 1 and parts[1] == "vertex"
                if in_vertex:
                    count = int(parts[2])
            elif parts[0] == "property" and in_vertex:
                if parts[1] == "list":
                    raise ValueError("PLY list property is not supported")
                if parts[1] not in _PLY_TYPES:
                    raise ValueError(f"unsupported PLY type: {parts[1]}")
                props.append((parts[2], _PLY_TYPES[parts[1]]))

        names = [n for n, _ in props]
        if not all(k in names for k in ("x", "y", "z")):
            raise ValueError("PLY has no x/y/z properties")

        if fmt == "ascii":
            arr = np.loadtxt(f, dtype=np.float64, ndmin=2, max_rows=count)
            idx = [names.index(k) for k in ("x", "y", "z")]
            return arr[:, idx]
        if fmt != "binary_little_endian":
            raise ValueError(f"unsupported PLY format: {fmt}")
        data = np.fromfile(f, dtype=np.dtype(props), count=count)
        return np.c_[data["x"], data["y"], data["z"]].astype(np.float64)


def load_map_points(path):
    p = os.path.expanduser(path)
    if not os.path.exists(p):
        raise FileNotFoundError(p)
    if p.endswith(".npy"):
        arr = np.load(p)
    elif p.endswith(".ply"):
        arr = _read_ply_xyz(p)
    elif p.endswith(".pcd"):
        arr = _read_pcd_ascii(p)
    else:
        raise ValueError(f"unsupported map format: {p}")
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError("map points must be Nx3 or Nx4")
    arr = arr[:, :3]
    arr = arr[np.isfinite(arr).all(axis=1)]
    return arr


class MapLocalizer(Node):
    def __init__(self):
        super().__init__("map_localizer")
        self.declare_parameter("cloud_topic", "/velodyne_points")
        self.declare_parameter("map_path", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "sirius3/odom")
        self.declare_parameter("base_frame", "sirius3/base_footprint")
        self.declare_parameter("update_period", 0.5)
        self.declare_parameter("map_voxel", 0.2)
        self.declare_parameter("scan_voxel", 0.2)
        self.declare_parameter("registration_type", "VGICP")
        self.declare_parameter("voxel_resolution", 0.5)
        self.declare_parameter("max_correspondence_distance", 1.0)
        self.declare_parameter("max_iterations", 30)
        self.declare_parameter("num_threads", 4)
        self.declare_parameter("use_3dof", True)
        self.declare_parameter("max_translation_correction", 0.5)
        self.declare_parameter("max_rotation_correction_deg", 10.0)
        self.declare_parameter("min_inlier_ratio", 0.3)
        self.declare_parameter("min_range", 1.0)
        self.declare_parameter("max_range", 60.0)
        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_z", 0.0)
        self.declare_parameter("initial_pose_yaw", 0.0)
        self.declare_parameter("initial_pose_file", "")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_diagnostics", True)

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self._lock = threading.Lock()
        self._cloud = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.T_map_odom = None
        self._init_pose = self._load_initial_pose()
        self._last_status = ""

        self._map_pc = None
        self._map_tree = None
        self._voxel_map = None
        self._load_map()

        self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value,
                                 self._on_cloud, 1)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 self._on_initial_pose, 1)
        self._pose_pub = self.create_publisher(PoseStamped, "/map_localizer/pose", 1)
        self._status_pub = self.create_publisher(String, "/map_localizer/status", 1)
        self.create_timer(float(self.get_parameter("update_period").value), self._update)
        # Keep the last accepted map correction available between registrations,
        # including when a scan is rejected or temporarily unavailable.
        self.create_timer(0.1, self._publish_tf)
        self.get_logger().info(
            f"map_localizer started: map={self.get_parameter('map_path').value} "
            f"({0 if self._map_pc is None else int(self._map_pc.size())} pts), "
            f"reg={self.get_parameter('registration_type').value}, "
            f"3dof={self.get_parameter('use_3dof').value}")

    def _load_initial_pose(self):
        path = str(self.get_parameter("initial_pose_file").value).strip()
        if path and yaml is not None:
            p = os.path.expanduser(path)
            if os.path.exists(p):
                try:
                    with open(p, "r") as f:
                        data = yaml.safe_load(f) or {}
                    return (
                        float(data.get("x", 0.0)),
                        float(data.get("y", 0.0)),
                        float(data.get("z", 0.0)),
                        float(data.get("yaw", 0.0)),
                    )
                except Exception as error:  # noqa: BLE001
                    self.get_logger().warn(f"初期姿勢ファイル読み込み失敗: {error}")
        return (
            float(self.get_parameter("initial_pose_x").value),
            float(self.get_parameter("initial_pose_y").value),
            float(self.get_parameter("initial_pose_z").value),
            float(self.get_parameter("initial_pose_yaw").value),
        )

    def _load_map(self):
        path = str(self.get_parameter("map_path").value).strip()
        if not path:
            self.get_logger().warn("map_path が未指定です。地図は /initialpose で初期化できません。")
            return
        try:
            points = load_map_points(path)
            threads = int(self.get_parameter("num_threads").value)
            voxel = float(self.get_parameter("map_voxel").value)
            self._map_pc, self._map_tree = small_gicp.preprocess_points(
                points, voxel, 20, threads)
            if str(self.get_parameter("registration_type").value).upper() == "VGICP":
                self._voxel_map = small_gicp.GaussianVoxelMap(
                    float(self.get_parameter("voxel_resolution").value))
                self._voxel_map.insert(self._map_pc)
            self.get_logger().info(f"地図読み込み完了: {len(points)} 点 ({path})")
        except Exception as error:  # noqa: BLE001
            self._map_pc = None
            self._voxel_map = None
            self.get_logger().error(f"地図読み込み失敗: {error}")

    def _on_cloud(self, msg):
        with self._lock:
            self._cloud = msg

    def _lookup(self, target, source, stamp=None):
        try:
            when = rclpy.time.Time() if stamp is None else rclpy.time.Time.from_msg(stamp)
            tf = self._tf_buffer.lookup_transform(
                target, source, when, Duration(seconds=0.1))
            return _matrix_from_transform(tf)
        except Exception:  # noqa: BLE001
            return None

    def _on_initial_pose(self, msg):
        T_odom_base = self._lookup(self.odom_frame, self.base_frame)
        if T_odom_base is None:
            self.get_logger().warn("/initialpose 受信したが odom->base が未取得のため無視")
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        T_map_base = _pose_matrix(p.x, p.y, p.z, yaw)
        with self._lock:
            self.T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
        self.get_logger().info(
            f"/initialpose 適用: x={p.x:.2f} y={p.y:.2f} z={p.z:.2f} yaw={math.degrees(yaw):.1f}deg")
        self._publish_tf()

    def _ensure_init(self):
        if self.T_map_odom is not None:
            return True
        T_odom_base = self._lookup(self.odom_frame, self.base_frame)
        if T_odom_base is None:
            return False
        x, y, z, yaw = self._init_pose
        T_map_base = _pose_matrix(x, y, z, yaw)
        with self._lock:
            self.T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
        self.get_logger().info(
            f"初期姿勢を適用: x={x:.2f} y={y:.2f} z={z:.2f} yaw={math.degrees(yaw):.1f}deg")
        self._publish_tf()
        return True

    def _publish_tf(self):
        if not self.get_parameter("publish_tf").value:
            return
        with self._lock:
            T = None if self.T_map_odom is None else self.T_map_odom.copy()
        if T is None:
            return
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = float(T[0, 3])
        msg.transform.translation.y = float(T[1, 3])
        msg.transform.translation.z = float(T[2, 3])
        x, y, z, w = _quat_from_matrix(T[:3, :3])
        msg.transform.rotation.x = x
        msg.transform.rotation.y = y
        msg.transform.rotation.z = z
        msg.transform.rotation.w = w
        self._tf_broadcaster.sendTransform(msg)

    def _update(self):
        if self._map_pc is None:
            return
        if str(self.get_parameter("registration_type").value).upper() == "VGICP" \
                and self._voxel_map is None:
            return
        with self._lock:
            cloud = self._cloud
        if cloud is None:
            return
        if not self._ensure_init():
            return

        T_odom_lidar = self._lookup(self.odom_frame, cloud.header.frame_id,
                                  cloud.header.stamp)
        if T_odom_lidar is None:
            return
        with self._lock:
            T_map_odom = self.T_map_odom.copy()
        T_map_lidar = T_map_odom @ T_odom_lidar

        xyz = _pc2_to_xyz(cloud)
        if xyz is None or len(xyz) < 50:
            return
        rng = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(rng >= float(self.get_parameter("min_range").value)) &
                  (rng <= float(self.get_parameter("max_range").value))]
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz) < 50:
            return
        xyz_map = (T_map_lidar[:3, :3] @ xyz.T).T + T_map_lidar[:3, 3]

        threads = int(self.get_parameter("num_threads").value)
        scan_voxel = float(self.get_parameter("scan_voxel").value)
        max_dist = float(self.get_parameter("max_correspondence_distance").value)
        max_iter = int(self.get_parameter("max_iterations").value)
        reg = str(self.get_parameter("registration_type").value).upper()
        try:
            src_pc, _ = small_gicp.preprocess_points(xyz_map, scan_voxel, 20, threads)
            if reg == "VGICP":
                result = small_gicp.align(self._voxel_map, src_pc, np.eye(4),
                                          max_correspondence_distance=max_dist,
                                          num_threads=threads, max_iterations=max_iter)
            else:
                result = small_gicp.align(self._map_pc, src_pc, self._map_tree, np.eye(4),
                                          registration_type=reg,
                                          max_correspondence_distance=max_dist,
                                          num_threads=threads, max_iterations=max_iter)
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f"位置合わせ失敗: {error}")
            return

        n_src = max(1, int(src_pc.size()))
        inlier_ratio = float(result.num_inliers) / n_src
        delta = np.asarray(result.T_target_source, dtype=np.float64).copy()
        dt = float(np.linalg.norm(delta[:3, 3]))
        dr = _rotation_angle_deg(delta[:3, :3])
        max_t = float(self.get_parameter("max_translation_correction").value)
        max_r = float(self.get_parameter("max_rotation_correction_deg").value)
        min_inlier = float(self.get_parameter("min_inlier_ratio").value)

        status = (f"conv={bool(result.converged)} inliers={result.num_inliers}/{n_src} "
                  f"({inlier_ratio:.2f}) d_trans={dt:.3f}m d_rot={dr:.2f}deg")
        accepted = bool(result.converged) and inlier_ratio >= min_inlier
        if accepted and (dt > max_t or dr > max_r):
            accepted = False
            status += " REJECTED(over-limit)"
        if not accepted:
            self._publish_status("reject " + status)
            return

        if bool(self.get_parameter("use_3dof").value):
            yaw = math.atan2(delta[1, 0], delta[0, 0])
            delta3 = _yaw_matrix(yaw)
            delta3[:3, 3] = [delta[0, 3], delta[1, 3], 0.0]
            delta = delta3

        T_map_lidar_new = delta @ T_map_lidar
        T_map_odom_new = T_map_lidar_new @ np.linalg.inv(T_odom_lidar)
        with self._lock:
            self.T_map_odom = T_map_odom_new
        self._publish_tf()
        self._publish_pose(T_map_lidar_new, cloud)
        self._publish_status("ok " + status)

    def _publish_pose(self, T_map_lidar, cloud):
        T_lidar_base = self._lookup(cloud.header.frame_id, self.base_frame)
        T_map_base = T_map_lidar @ T_lidar_base if T_lidar_base is not None else T_map_lidar
        msg = PoseStamped()
        msg.header.stamp = cloud.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(T_map_base[0, 3])
        msg.pose.position.y = float(T_map_base[1, 3])
        msg.pose.position.z = float(T_map_base[2, 3])
        x, y, z, w = _quat_from_matrix(T_map_base[:3, :3])
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w
        self._pose_pub.publish(msg)

    def _publish_status(self, text):
        if not self.get_parameter("publish_diagnostics").value:
            return
        if text != self._last_status:
            self._last_status = text
            self.get_logger().info(text)
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
