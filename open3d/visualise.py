import open3d as o3d
import numpy as np
pcd = o3d.io.read_point_cloud("/home/yc/catkin_ws/src/realsense-ros/realsense2_camera/dataset/ply/sense1/sense1_view1_16.ply")

# Visual point cloud
#o3d.visualization.draw_geometries([pcd])

# Surface normal
downpcd = pcd.voxel_down_sample(0.01)
o3d.visualization.draw_geometries([downpcd])
downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=50))
#radius: cm of search radius
#max_nn: how many neighbors to save computation time
print(np.asarray(downpcd.normals)[:1000, :])
