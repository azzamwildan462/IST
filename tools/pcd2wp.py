import open3d as o3d
import numpy as np
import csv

# Load the voxelized PCD file
pcd = o3d.io.read_point_cloud("jalan_baru.pcd")
points = np.asarray(pcd.points)

# Save to CSV
with open("waypoint22.csv", mode="w", newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["x", "y"])  # Header

    for point in points:
        x, y = point[0], point[1]
        writer.writerow([x, y])

print("✅ waypoint22.csv created successfully.")
