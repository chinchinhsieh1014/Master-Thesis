#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo
import numpy as np
from realsense2_camera.msg import selected_area, color_data
import cv2
import os
path = "/home/yc/catkin_ws/src/realsense-ros/realsense2_camera/dataset/ply/"
import open3d as o3d

class point():
    def __init__(self):
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depth_info)
        self.cx = None
        self.focalx = None
        self.cy = None
        self.focaly = None
        rospy.Subscriber('color_data', color_data, self.data)
        self.id = []
        self.v = None
        self.s = None
        rospy.Subscriber("selected_area", selected_area, self.callback)

    def depth_info(self, cameraInfo):
        self.cx = cameraInfo.K[2]
        self.focalx = cameraInfo.K[0]
        self.cy = cameraInfo.K[5]
        self.focaly = cameraInfo.K[4]

    def data(self, data):
        self.id = list(data.detected_ids)
        self.v = data.view
        self.s = data.sense

    def callback(self, data):
        w = data.size[0]
        h = data.size[1]
        a = np.array(data.area)
        a = np.squeeze(a)
        a_np = a.reshape((h,w))
        if len(self.id)!=0:
            i = 0
            while i<len(self.id):
                p = []
                for u in range(w):
                    for v in range(h):
                        d = a_np[v, u]
                        if d!=0:
                            x_over_z = (self.cx - u) / self.focalx
                            y_over_z = (self.cy - v) / self.focaly
                            z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
                            x = x_over_z * z
                            y = y_over_z * z
                            p.append([x,y,z])
                xyz = np.array(p)
                xyz = xyz.reshape(-1,3)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(xyz)
                file = "sense"+str(self.s)+"/sense"+str(self.s)+"_view"+str(self.v)+"_"+str(self.id[i])+".ply"
                o3d.io.write_point_cloud(os.path.join(path,file), pcd)
                i = i+1

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('point',anonymous=True)
    # Subscribe
    point = point()
    # Keep the node existing
    rospy.spin()
