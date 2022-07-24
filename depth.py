#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs2
import numpy as np
import sys

class depth():
    def __init__(self):
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.info)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

    def info(self, cameraInfo):
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

    def callback(self, data):
        # Convert the ROS Image message to a CV2 Image
        try:
            image_cv = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # numpy.ndarray (720, 1280)
            # pick one pixel among all the pixels with the closest range:
            indices = np.array(np.where(image_cv == image_cv[image_cv > 0].min()))[:,0]
            pix = (indices[1], indices[0])
            self.pix = pix
            line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], image_cv[pix[1], pix[0]])
            if self.intrinsics:
                depth = image_cv[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            sys.stdout.write(line)
            sys.stdout.flush()

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(image_cv, alpha=0.05), cv2.COLORMAP_JET)
        # (720, 1280, 3)

        # Publish
        pub = rospy.Publisher('image_depth_detected', Image, queue_size=10)
        msg_frame = CvBridge().cv2_to_imgmsg(depth_colormap, "bgr8")
        pub.publish(msg_frame)


if __name__ == '__main__':
    # Aruco marker dictionaries
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    # Detector parameters
    parameters = cv2.aruco.DetectorParameters_create()

    # Initialize the node
    rospy.init_node('depth_detected',anonymous=True)
    # Initialize the CvBridge class
    bridge = CvBridge()
    # # Subscribe
    depth_image = depth()
    # Keep the node existing
    rospy.spin()
