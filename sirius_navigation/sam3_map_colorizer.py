import os
import sys
import yaml
import cv2
import numpy as np
import struct
import json
from sklearn.cluster import KMeans

# Predefined semantic categories mapping to colors and default costmap costs
PREDEFINED_CATEGORIES = {
    "wall": {"color": [0, 0, 0], "default_cost": 254},
    "floor": {"color": [255, 255, 255], "default_cost": 0},
    "grass": {"color": [0, 255, 0], "default_cost": 120},
    "tactile paving": {"color": [255, 255, 0], "default_cost": 50},
    "roadway": {"color": [0, 0, 255], "default_cost": 254},
    "sidewalk": {"color": [128, 128, 128], "default_cost": 10}
}


# Helper class to dynamically load dynamic class colors over ROS 2 topic
class ClassColorsListener:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy

        self.node = Node('sam3_class_colors_listener')
        self.received_config = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.node.create_subscription(String, '/sam3/class_colors', self.cb, qos)

    def cb(self, msg):
        try:
            self.received_config = json.loads(msg.data)
        except Exception as e:
            print(f"Error parsing class colors from topic: {e}")

def getColor(packed):
    # Depending on how it was packed in CloudToPly
    # pr = (packed >> 16) & 0xFF, pg = (packed >> 8) & 0xFF, pb = packed & 0xFF
    return ( (packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF )


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 sam3_map_colorizer.py <map_base_path>")
        sys.exit(1)

    base = sys.argv[1]
    pgm_file = base + ".pgm"
    yaml_file = base + ".yaml"
    ply_file = base + ".ply"

    if not (os.path.exists(pgm_file) and os.path.exists(yaml_file) and os.path.exists(ply_file)):
        print(f"Error: Missing one of .pgm, .yaml, or .ply for {base}")
        sys.exit(1)

    # Try to load dynamic class colors configuration
    dynamic_colors = None
    import time
    
    # Attempt 1: Load from ROS 2 Topic (Latched)
    try:
        import rclpy
        print("Waiting for dynamic class colors config from ROS 2 topic /sam3/class_colors...")
        rclpy.init()
        listener = ClassColorsListener()
        
        # Spin with a short timeout to catch the latched topic message
        start_time = time.time()
        while time.time() - start_time < 2.0 and listener.received_config is None:
            rclpy.spin_once(listener.node, timeout_sec=0.1)
            
        if listener.received_config:
            dynamic_colors = listener.received_config
            print(f"SUCCESS: Loaded class colors from ROS 2 topic /sam3/class_colors: {dynamic_colors}")
        else:
            print("Timeout: No class colors config received on topic. Checking local fallback file.")
        listener.node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"Notice: ROS 2 context not available or topic listener failed ({e}). Checking local fallback file.")

    # Attempt 2: Fallback to local class_colors.json file
    if dynamic_colors is None:
        config_path = "/home/kotantu-desktop/DA3_SAM3_Project/sam3_da3_3d/sam3_server_export/class_colors.json"
        if os.path.exists(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    dynamic_colors = json.load(f)
                print(f"SUCCESS: Loaded class colors configuration from local file: {config_path}")
            except Exception as e:
                print(f"Warning: Failed to load dynamic class colors config from file: {e}")

    # Overwrite PREDEFINED_CATEGORIES with dynamically loaded colors
    if dynamic_colors:
        for name, rgb in dynamic_colors.items():
            cost = 100
            if name in PREDEFINED_CATEGORIES:
                cost = PREDEFINED_CATEGORIES[name]["default_cost"]
            elif "grass" in name:
                cost = 120
            elif "tactile" in name:
                cost = 50
            elif "roadway" in name:
                cost = 254
            elif "sidewalk" in name:
                cost = 10
            PREDEFINED_CATEGORIES[name] = {"color": rgb, "default_cost": cost}


    print(f"Loading structural map: {pgm_file}")

    grid = cv2.imread(pgm_file, cv2.IMREAD_UNCHANGED)
    with open(yaml_file, 'r') as f:
        meta = yaml.safe_load(f)

    resolution = meta['resolution']
    origin = meta['origin'] # [x, y, z]
    h, w = grid.shape

    # Initialize colored grid
    # index 0: unknown, 1: wall, 2: floor
    colored_grid = np.zeros_like(grid, dtype=np.uint8)
    # RTAB-Map PGM convention: 0 (Occupied/Wall), 255 (Free/Floor), 205 (Unknown)
    # Our indexed convention: 0 (Unknown), 1 (Wall), 2 (Floor)
    colored_grid[grid == 0] = 1   # Wall
    colored_grid[grid == 255] = 2 # Floor
    
    # We'll store (R, G, B) sums and counts per pixel to average colors
    # For memory efficiency, we use a dictionary for pixels that get colors
    pixel_colors = {} # (gx, gy) -> [r_sum, g_sum, b_sum, count]

    print(f"Processing color from: {ply_file}")
    with open(ply_file, 'rb') as f:
        # Simple binary PLY parser for our specific format
        header = ""
        while "end_header" not in header:
            line = f.readline().decode('ascii')
            header += line
            if "element vertex" in line:
                num_points = int(line.split()[-1])
        
        # Format: float x, y, z (12 bytes), uchar r, g, b (3 bytes) = 15 bytes per point
        # struct.pack('<fffBBB')
        point_struct = struct.Struct('<fffBBB')
        
        for _ in range(num_points):
            data = f.read(15)
            if not data: break
            x, y, z, r, g, b = point_struct.unpack(data)
            
            # Project to grid
            gx = int(round((x - origin[0]) / resolution))
            gy = int(round((y - origin[1]) / resolution))
            
            # In PGM, (0,0) is TOP-LEFT, but ROS origin is BOTTOM-LEFT
            # RTAB-Map's map_saver might have flipped it.
            # Standard map_saver PGM: Y=0 is top.
            # Origin Y maps to row H-1.
            py = h - 1 - gy
            px = gx
            
            if 0 <= px < w and 0 <= py < h:
                key = (px, py)
                if key not in pixel_colors:
                    pixel_colors[key] = [0, 0, 0, 0]
                pixel_colors[key][0] += r
                pixel_colors[key][1] += g
                pixel_colors[key][2] += b
                pixel_colors[key][3] += 1

    if not pixel_colors:
        print("Warning: No points projected onto the grid.")
    else:
        print(f"Projected color onto {len(pixel_colors)} pixels.")
        
        # Collect all unique averaged colors for K-Means
        unique_colors = []
        for key in pixel_colors:
            c = pixel_colors[key]
            avg_color = (c[0]//c[3], c[1]//c[3], c[2]//c[3])
            unique_colors.append(avg_color)
            pixel_colors[key] = avg_color # Replace with average for next step
        
        # Color Quantization (K-Means)
        print("Performing color quantization (K-Means)...")
        data = np.array(unique_colors, dtype=np.float32)
        reserved = np.array([[127, 127, 127], [0, 0, 0], [255, 255, 255]], dtype=np.uint8)
        
        if len(data) > 10:
            k = min(253, len(data))
            kmeans = KMeans(n_clusters=k, n_init=1, random_state=42).fit(data)
            semantic = sorted(kmeans.cluster_centers_.astype(np.uint8).tolist(), 
                             key=lambda c: 0.299*c[0]+0.587*c[1]+0.114*c[2])
            palette = np.vstack([reserved, semantic])
        else:
            palette = reserved # Fallback if too few colors

        # Assign indices to grid
        # index 0: Unknown, 1: Wall, 2: Floor, 3+: Semantic
        visual_color_grid = np.zeros((h, w, 3), dtype=np.uint8)
        
        # Fill visual map with defaults
        visual_color_grid[grid == 205] = [127, 127, 127] # Unknown (Gray)
        visual_color_grid[grid == 0] = [0, 0, 0]         # Wall (Black)
        visual_color_grid[grid == 255] = [255, 255, 255] # Floor (White)

        for (px, py), color in pixel_colors.items():
            if colored_grid[py, px] == 1: continue # Walls (priority)
            
            # Find nearest index in semantic palette
            dist = np.sum((palette[3:] - color)**2, axis=1)
            idx = np.argmin(dist) + 3
            colored_grid[py, px] = idx
            
            # Set color in visual map (BGR for OpenCV)
            visual_color_grid[py, px] = [color[2], color[1], color[0]]

        # Save result
        out_pgm = base + ".colored.pgm"
        out_json = base + ".colored.json"
        out_visual = base + ".color.png"
        
        # We save PGM flipped as we read it? 
        # map_saver wrote it, cv2.imread read it. We just write it back.
        cv2.imwrite(out_pgm, colored_grid)
        cv2.imwrite(out_visual, visual_color_grid)
        
        print(f"SUCCESS: Saved semantic map to {out_pgm}")
        print(f"SUCCESS: Saved visual map to {out_visual}")
        
        # Map each palette entry to semantic labels using minimum distance in RGB
        labels_out = {}
        for idx, color_rgb in enumerate(palette.tolist()):
            best_cat = "unknown"
            best_dist = float('inf')
            for cat_name, cat_info in PREDEFINED_CATEGORIES.items():
                d = sum((c1 - c2)**2 for c1, c2 in zip(color_rgb, cat_info["color"]))
                if d < best_dist:
                    best_dist = d
                    best_cat = cat_name
            
            # If color matches close enough to a predefined class
            if best_dist < 2500:
                labels_out[str(idx)] = {
                    "name": best_cat,
                    "color": color_rgb,
                    "default_cost": PREDEFINED_CATEGORIES[best_cat]["default_cost"]
                }


        
        meta_out = {
            "resolution": resolution,
            "origin": [origin[0], origin[1]],
            "width": w,
            "height": h,
            "palette": palette.tolist(),
            "labels": labels_out
        }
        with open(out_json, 'w') as f:
            json.dump(meta_out, f, indent=4)
        
        print(f"SUCCESS: Saved colored map to {out_pgm}")


if __name__ == "__main__":
    main()
