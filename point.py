#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import numpy as np
from realsense2_camera.msg import selected_area
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import cv2
import os
'''
collect
1. depth image
2.
'''
class point():
    def __init__(self):
        rospy.Subscriber("selected_area", selected_area, self.callback)
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depth_info)
        self.cx = None
        self.focalx = None
        self.cy = None
        self.focaly = None

    def depth_info(self, cameraInfo):
        self.cx = cameraInfo.K[2]
        self.focalx = cameraInfo.K[0]
        self.cy = cameraInfo.K[5]
        self.focaly = cameraInfo.K[4]

    def callback(self, data):
        w = data.size[0]
        h = data.size[1]
        a = np.array(data.area)
        a = np.squeeze(a)
        a_np = a.reshape((h,w))
        # 2D Depth image to 3D point cloud
        # [x] 1. get depth image from message
        # [x] 2. calculate point cloud object(pcl) from depth image
        # [ ] 3. assign point cloud object to ROS PointCloud2 message
        # [ ] 4. Publish ROS message
        pc_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
        #header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
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
        pointcloud = pc2.create_cloud_xyz32(header, p)
        pc_pub.publish(pointcloud)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('point',anonymous=True)
    # Initialize the CvBridge class
    bridge = CvBridge()
    # Subscribe
    point = point()
    # Keep the node existing
    rospy.spin()



###
    #
    # #header
    # header = Header()
    # header.stamp = rospy.Time.now()
    # header.frame_id = 'map'
    # x = np.array(list(pc2.read_points(data, skip_nans=True, field_names="x")))
    # y = np.array(list(pc2.read_points(data, skip_nans=True, field_names="y")))
    # z = np.array(list(pc2.read_points(data, skip_nans=True, field_names="z")))
    # print("point:")
    # print(z)
    # print(z.shape)
    # # points_array = np.zeros(x.shape[0] + y.shape[0] + z.shape[0], dtype=np.float32)
    # # points_array[::3] = np.squeeze(x)
    # # points_array[1::3] = np.squeeze(y)
    # # points_array[2::3] = np.squeeze(z)
    # # pc_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
    # # ointcloud = pc2.create_cloud_xyz32(header, points_array)
    # # pc_pub.publish(pointcloud)
    #
    #     # 2D Depth image to 3D point cloud
    #     # [x] 1. get depth image from message
    #     # [x] 2. calculate point cloud object(pcl) from depth image
    #     # [x] 3. assign point cloud object to ROS PointCloud2 message
    #     # [x] 4. Publish ROS message
    #     # point cloud
    #     # if len(box)!=0:
    #     #     pc_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
    #     #     #header
    #     #     header = Header()
    #     #     header.stamp = rospy.Time.now()
    #     #     header.frame_id = 'map'
    #         #cloud
    #         # points = []
    #         # for i in box: #[[x0,y0],[x1,y1]]
    #         #     p = []
    #         #     # for u in range(i[0][0],i[1][0]):
    #         #     #     for v in range(i[0][1],i[1][1]):
    #         #             depth = image_cv[v, u]
    #         #             result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)
    #         #             p.append([result[0],result[1],result[2]])
    #         #     points.append(p)
    #         #pointcloud = pc2.create_cloud_xyz32(header, points[0])
    #         #pc_pub.publish(pointcloud)
    #     #header
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = 'map'
    #     pc_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
    #     p = []
    #     for u in range(640):
    #         for v in range(480):
    #             d = image_cv[v, u]
    #             if d!=0:
    #                 x_over_z = (self.cx - u) / self.focalx
    #                 y_over_z = (self.cy - v) / self.focaly
    #                 z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
    #                 x = x_over_z * z
    #                 y = y_over_z * z
    #                 #result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)
    #                 p.append([x,y,z])
    #     pointcloud = pc2.create_cloud_xyz32(header, p)
    #     pc_pub.publish(pointcloud)
###
