#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from realsense2_camera.msg import image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs2

class depth():
    def __init__(self):
        self.mtx = np.empty([3, 3])
        self.dist = np.empty([1, 5])
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.info)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback)

    def info(self, data):
        self.mtx = np.array(data.K).reshape(3, 3)
        self.dist  = np.array(data.D).reshape(5,1)

    def callback(self, data):

if __name__ == '__main__':
    # Aruco marker dictionaries
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    # Detector parameters
    parameters = cv2.aruco.DetectorParameters_create()

    # Initialize the node
    rospy.init_node('depth_detected',anonymous=True)
    # Initialize the CvBridge class
    bridge = CvBridge()
    # Subscribe
    depth_image = depth()

    # Keep the node existing
    rospy.spin()
