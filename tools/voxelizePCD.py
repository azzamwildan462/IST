import open3d as o3d

# Load original point cloud
pcd = o3d.io.read_point_cloud("jalan2.pcd")

# Apply voxel downsampling
voxel_size = 1  # meters
voxelized_pcd = pcd.voxel_down_sample(voxel_size)

# Save voxelized result
o3d.io.write_point_cloud("jalan_voxelized2.pcd", voxelized_pcd)
print("Saved voxelized point cloud with resolution:", voxel_size)
