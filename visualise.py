import open3d as o3d
import numpy as np
pcd = o3d.io.read_point_cloud("/home/yc/catkin_ws/src/realsense-ros/realsense2_camera/dataset/ply/sense1/sense1_view1_17.ply")
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])
