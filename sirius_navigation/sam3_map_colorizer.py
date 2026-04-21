import os
import sys
import yaml
import cv2
import numpy as np
import struct
import json
from sklearn.cluster import KMeans

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
        for (px, py), color in pixel_colors.items():
            if colored_grid[py, px] == 1: continue # Walls (priority)
            
            # Find nearest index in semantic palette
            dist = np.sum((palette[3:] - color)**2, axis=1)
            idx = np.argmin(dist) + 3
            colored_grid[py, px] = idx

        # Save result
        out_pgm = base + ".colored.pgm"
        out_json = base + ".colored.json"
        
        # We save PGM flipped as we read it? 
        # map_saver wrote it, cv2.imread read it. We just write it back.
        cv2.imwrite(out_pgm, colored_grid)
        
        meta_out = {
            "resolution": resolution,
            "origin": [origin[0], origin[1]],
            "width": w,
            "height": h,
            "palette": palette.tolist()
        }
        with open(out_json, 'w') as f:
            json.dump(meta_out, f, indent=4)
        
        print(f"SUCCESS: Saved colored map to {out_pgm}")

if __name__ == "__main__":
    main()
