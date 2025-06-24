import numpy as np
import open3d as o3d
import cv2
import yaml

# Load map metadata from YAML
def load_map_metadata(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    return data['resolution'], data['origin']

# Convert PGM to XYZ point cloud
def pgm_to_pointcloud(pgm_path, yaml_path, output_pcd):
    resolution, origin = load_map_metadata(yaml_path)
    origin_x, origin_y, _ = origin

    # Load PGM image (as grayscale)
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)

    # Threshold to find occupied cells (usually < 50 is occupied in ROS maps)
    occupied_indices = np.argwhere(img < 50)

    # Convert image coords to world coords
    points = []
    height, width = img.shape
    for y, x in occupied_indices:
        wx = x * resolution + origin_x
        wy = (height - y) * resolution + origin_y  # y-axis flip
        points.append([wx, wy, 0.0])  # Z = 0 for 2D map

    # Convert to Open3D PointCloud
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.array(points))
    o3d.io.write_point_cloud(output_pcd, cloud)
    print(f"Saved point cloud to {output_pcd}")

# Example usage
pgm_to_pointcloud("/home/wildan/Desktop/map_baru/map_baru_bagus.pgm", "/home/wildan/Desktop/map_baru/map_baru_bagus.yaml", "jalan_baru.pcd")
