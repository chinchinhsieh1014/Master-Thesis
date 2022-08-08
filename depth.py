#!/usr/bin/env python
import os
import sys
import cv2
import open3d as o3d
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2
from realsense2_camera.msg import color_data
import open3d as o3d

class depth():
    def __init__(self):
        rospy.Subscriber('color_data', color_data, self.getcorner)
        self.corner = []
        # Depth
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depth_info)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.intrinsics = None

    def getcorner(self, data):
        c = data.detected_corners
        c = list(c)
        np_array=np.asarray(c)
        self.corner = np_array.reshape((len(c)//8,4,2)).tolist()

    def depth_info(self, cameraInfo):
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.K[2]
        self.intrinsics.ppy = cameraInfo.K[5]
        self.intrinsics.fx = cameraInfo.K[0]
        self.intrinsics.fy = cameraInfo.K[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in cameraInfo.D]

    def depth_callback(self, data):
        # Convert the ROS Image message to a CV2 Image
        try:
            image_cv = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # numpy.ndarray (720, 1280)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(image_cv, alpha=0.05), cv2.COLORMAP_JET)
        # (720, 1280, 3)
        box = []
        if len(self.corner)!=0:
            for i in self.corner:
                x0 = min(i[0][0],i[1][0],i[2][0],i[3][0])
                y0 = min(i[0][1],i[1][1],i[2][1],i[3][1])
                x1 = max(i[0][0],i[1][0],i[2][0],i[3][0])
                y1 = max(i[0][1],i[1][1],i[2][1],i[3][1])
                cv2.rectangle(depth_colormap, (x0,y0), (x1,y1), (0,255,255), 3)
                box.append([[x0,y0],[x1,y1]])
        # 2D Depth image to 3D point cloud
        # [x] 1. get depth image from message
        # [x] 2. calculate point cloud object(pcl) from depth image
        # [x] 3. assign point cloud object to ROS PointCloud2 message
        # [x] 4. Publish ROS message
        # point cloud
        if len(box)!=0:
            pc_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
            #header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            #cloud
            points = []
            for i in box: #[[x0,y0],[x1,y1]]
                p = []
                for u in range(i[0][0],i[1][0]):
                    for v in range(i[0][1],i[1][1]):
                        depth = image_cv[v, u]
                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)
                        p.append([result[0],result[1],result[2]])
            pointcloud = pc2.create_cloud_xyz32(header, points[0])
            pc_pub.publish(pointcloud)

        # frame
        pub = rospy.Publisher('image_depth_detected', Image, queue_size=10)
        msg_frame = CvBridge().cv2_to_imgmsg(depth_colormap, "bgr8")
        pub.publish(msg_frame)
        self.corner = []

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('depth_frame',anonymous=True)
    # Initialize the CvBridge class
    bridge = CvBridge()
    # Subscribe
    depth = depth()
    # Keep the node existing
    rospy.spin()
