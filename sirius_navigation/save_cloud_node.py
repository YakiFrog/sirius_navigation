#!/usr/bin/env python3
"""PointCloud2を1回受信して 3D地図（.npy/.pcd）として保存し、2D占有格子へ投影する。

3D地図（自己位置推定用）と2D地図（Nav2用）を同じ点群から生成するための保存ノード。
map_saver_cli の代替ではなく、3D→2D圧縮を一括で行う。
"""

import os

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def _pc2_to_xyz(msg: PointCloud2):
    offsets = {f.name: f.offset for f in msg.fields}
    if not all(k in offsets for k in ("x", "y", "z")):
        return None
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": msg.point_step,
    })
    arr = np.frombuffer(msg.data, dtype=dtype)
    if msg.height > 1:
        arr = arr.reshape(msg.height, msg.width)
    return np.c_[arr["x"], arr["y"], arr["z"]].astype(np.float64)


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


def write_pcd_ascii(path, points):
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def project_2d(points, resolution, z_min, z_max, padding, occupied_min_points):
    xs, ys = points[:, 0], points[:, 1]
    x0 = float(np.floor((xs.min() - padding) / resolution) * resolution)
    y0 = float(np.floor((ys.min() - padding) / resolution) * resolution)
    x1 = float(np.ceil((xs.max() + padding) / resolution) * resolution)
    y1 = float(np.ceil((ys.max() + padding) / resolution) * resolution)
    w = int(round((x1 - x0) / resolution))
    h = int(round((y1 - y0) / resolution))
    ix = ((xs - x0) / resolution).astype(np.int64)
    iy = ((ys - y0) / resolution).astype(np.int64)
    valid = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    ix, iy, z = ix[valid], iy[valid], points[valid, 2]
    band = (z >= z_min) & (z <= z_max)
    counts_all = np.zeros((h, w), dtype=np.int64)
    counts_band = np.zeros((h, w), dtype=np.int64)
    np.add.at(counts_all, (iy, ix), 1)
    np.add.at(counts_band, (iy[band], ix[band]), 1)
    img = np.full((h, w), 205, dtype=np.uint8)
    img[counts_all > 0] = 254
    img[counts_band >= occupied_min_points] = 0
    return img, x0, y0


def write_pgm(path, img):
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(np.flipud(img).tobytes())


def write_map_yaml(path, image_name, resolution, origin):
    with open(path, "w") as f:
        f.write(f"image: {image_name}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.000]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")


class SaveCloudNode(Node):
    def __init__(self):
        super().__init__("save_cloud_node")
        self.declare_parameter("topic", "/cloud_map")
        self.declare_parameter("output_base", "")
        self.declare_parameter("target_frame", "")
        self.declare_parameter("project_2d", True)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("z_min", 0.2)
        self.declare_parameter("z_max", 2.0)
        self.declare_parameter("padding", 1.0)
        self.declare_parameter("occupied_min_points", 1)
        self.declare_parameter("save_npy", True)
        self.declare_parameter("save_pcd", True)
        self.declare_parameter("crop_above_floor", 0.0)
        self.declare_parameter("crop_below_floor", 0.0)
        self.declare_parameter("crop_floor_percentile", 2.0)
        self.declare_parameter("normalize_floor_z", True)

        self._done = False
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        topic = self.get_parameter("topic").value
        # ラッチ配信(/cloud_map等)と通常配信(/Laser_map等)の両方を受けるため複数QoSで購読
        qos_volatile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
        qos_latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos_volatile)
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos_latched)
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos_best_effort)
        self.get_logger().info(f"待機中: {topic} の点群を1回受信して保存します")

    def _on_cloud(self, msg):
        if self._done:
            return
        self._done = True
        xyz = _pc2_to_xyz(msg)
        if xyz is None:
            self.get_logger().error("x,y,zフィールドがありません")
            rclpy.shutdown()
            return
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        target = self.get_parameter("target_frame").value
        if target and msg.header.frame_id and target != msg.header.frame_id:
            try:
                tf = self._buffer.lookup_transform(
                    target, msg.header.frame_id, rclpy.time.Time(), Duration(seconds=1.0))
                T = _matrix_from_transform(tf)
                xyz = (T[:3, :3] @ xyz.T).T + T[:3, 3]
            except Exception as error:  # noqa: BLE001
                self.get_logger().warn(
                    f"{msg.header.frame_id} -> {target} のTFが無いため元フレームで保存: {error}")

        # 床を推定し、天井・床下をカット（0で無効）。normalize_floor_z で床を z=0 に正規化
        above = float(self.get_parameter("crop_above_floor").value)
        below = float(self.get_parameter("crop_below_floor").value)
        normalize = bool(self.get_parameter("normalize_floor_z").value)
        if (above > 0.0 or below > 0.0 or normalize) and len(xyz) > 0:
            z = xyz[:, 2]
            floor = float(np.percentile(z, float(self.get_parameter("crop_floor_percentile").value)))
            keep = np.ones(len(xyz), dtype=bool)
            if below > 0.0:
                keep &= z >= (floor - below)
            if above > 0.0:
                keep &= z <= (floor + above)
            removed = int((~keep).sum())
            xyz = xyz[keep]
            self.get_logger().info(
                f"床推定 z={floor:.2f}m / 天井カット上限 z={floor + above:.2f}m: "
                f"除去 {removed} 点, 残り {len(xyz)} 点")
            if normalize:
                xyz = xyz.copy()
                xyz[:, 2] -= floor
                self.get_logger().info(f"床を z=0 に正規化しました (offset {-floor:+.3f}m)")

        base = os.path.expanduser(str(self.get_parameter("output_base").value))
        if not base:
            self.get_logger().error("output_base が未指定です")
            rclpy.shutdown()
            return
        os.makedirs(os.path.dirname(base), exist_ok=True)
        n = len(xyz)
        self.get_logger().info(f"点群 {n} 点を保存します ({base})")
        if self.get_parameter("save_npy").value:
            np.save(base + ".npy", xyz.astype(np.float32))
        if self.get_parameter("save_pcd").value:
            write_pcd_ascii(base + ".pcd", xyz)
        if self.get_parameter("project_2d").value:
            img, x0, y0 = project_2d(
                xyz,
                float(self.get_parameter("resolution").value),
                float(self.get_parameter("z_min").value),
                float(self.get_parameter("z_max").value),
                float(self.get_parameter("padding").value),
                int(self.get_parameter("occupied_min_points").value))
            name = os.path.basename(base)
            write_pgm(base + ".pgm", img)
            write_map_yaml(base + ".yaml", name + ".pgm",
                           float(self.get_parameter("resolution").value), (x0, y0))
            self.get_logger().info(
                f"2D投影を保存: {base}.pgm / {base}.yaml ({img.shape[1]}x{img.shape[0]})")
        self.get_logger().info("保存完了")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SaveCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
