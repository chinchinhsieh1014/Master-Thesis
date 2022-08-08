#!/usr/bin/env python
import os
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2
from realsense2_camera.msg import color_data, selected_area

class depth():
    def __init__(self):
        rospy.Subscriber('color_data', color_data, self.getcorner)
        self.corner = []
        # Depth
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depth_info)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

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
        if len(self.corner)!=0:
            for i in self.corner:
                x0 = min(i[0][0],i[1][0],i[2][0],i[3][0])
                y0 = min(i[0][1],i[1][1],i[2][1],i[3][1])
                x1 = max(i[0][0],i[1][0],i[2][0],i[3][0])
                y1 = max(i[0][1],i[1][1],i[2][1],i[3][1])
                # Publish
                area_pub = rospy.Publisher('selected_area', selected_area, queue_size=10)
                data = selected_area()
                image_np = np.asarray(image_cv[y0:y1,x0:x1], dtype=int) #(y,x)
                data.area = image_np.reshape(-1).tolist()
                data.size = [x1-x0,y1-y0]
                area_pub.publish(data)
                # collect data
                path = "/home/yc/catkin_ws/src/realsense-ros/realsense2_camera/dataset/raw"
                cv2.imwrite(os.path.join(path,"depth.png"),depth_colormap[y0:y1,x0:x1])
                # Mark
                cv2.rectangle(depth_colormap, (x0,y0), (x1,y1), (0,255,255), 3)
                self.corner
                #wait - publish the next msg
                #rospy.sleep(5)
            # frame
            pub = rospy.Publisher('image_depth_detected', Image, queue_size=10)
            msg_frame = CvBridge().cv2_to_imgmsg(depth_colormap, "bgr8")
            pub.publish(msg_frame)
            #wait - change another frame
            rospy.sleep(1)
        else:
            # frame
            pub = rospy.Publisher('image_depth_detected', Image, queue_size=10)
            msg_frame = CvBridge().cv2_to_imgmsg(depth_colormap, "bgr8")
            pub.publish(msg_frame)
if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('depth_frame',anonymous=True)
    # Initialize the CvBridge class
    bridge = CvBridge()
    # Subscribe
    depth = depth()
    # Keep the node existing
    rospy.spin()
