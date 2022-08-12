import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("/home/yc/catkin_ws/src/realsense-ros/realsense2_camera/dataset/ply/sense8/sense8_view7_37.ply")
# Visualize
# o3d.visualization.draw_geometries([pcd])

# Surface normal
downpcd = pcd.voxel_down_sample(0.5)
# Visualize
o3d.visualization.draw_geometries([downpcd])

downpcd.estimate_normals()

#normals: Points normals.
#print the first 1000 normals
print(np.asarray(downpcd.normals)[:1000, :])
