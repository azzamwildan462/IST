import open3d as o3d

# Load and visualize the PCD file
pcd = o3d.io.read_point_cloud("jalan_baru.pcd")
print(pcd)  # Prints basic info about the point cloud

# Simple interactive viewer
o3d.visualization.draw_geometries([pcd],
    zoom=0.8,
    front=[0.0, -1.0, 0.0],
    lookat=[-35.0, 47.5, 0.0],
    up=[0.0, 0.0, 1.0]
)
