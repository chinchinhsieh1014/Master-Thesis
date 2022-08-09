import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/yc/catkin_ws/src/realsense-ros/realsense2_camera/dataset/raw/try.ply")
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])
