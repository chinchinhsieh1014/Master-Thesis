#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs2
import numpy as np
import sys

class image():
    def __init__(self):
        # Color
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.color_info)
        rospy.Subscriber("/camera/color/image_rect_color", Image, self.color_callback)
        self.mtx = np.empty([3, 3])
        self.dist = np.empty([1, 5])
        # Depth
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depth_info)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.intrinsics = None
        self.pix = None

    def color_info(self, data):
        self.mtx = np.array(data.K).reshape(3, 3)
        self.dist  = np.array(data.D).reshape(5,1)

    def color_callback(self, data):
        # Convert the ROS Image message to a CV2 Image
        try:
            image_cv = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # numpy.ndarray (720, 1280, 3)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # BRG to RGB
        image_rgb = cv2.cvtColor(image_cv, cv2.COLOR_BGR2RGB)
        # RGB to Gray
        image_gray = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(image_gray,
                                                         dictionary,
                                                         parameters=parameters)

        Max = 100
        M = []
        if ids is not None: # detect something
            # Reshape ids and corners
            ids = ids.reshape(-1)
            corner = np.asarray(corners)
            (number, one, points, xy) = corner.shape
            corner = np.reshape(corner, (number,points,xy))
            corner_selected = corners
            count = 0
            # Remove patches which are smaller than Max
            for i,c in enumerate(corner):
                dist1 = np.linalg.norm(c[0]-c[1])
                dist2 = np.linalg.norm(c[1]-c[2])
                dist3 = np.linalg.norm(c[2]-c[3])
                dist4 = np.linalg.norm(c[3]-c[0])
                if max(dist1,dist2,dist3,dist4)<Max:
                    corner_selected = np.delete(corner_selected, count, 0)
                    ids = np.delete(ids, count, 0)
                else:
                    M.append(max(dist1,dist2,dist3,dist4))
                    count = count+1

        centers = []
        if ids is not None: # still have patches > M
            # Bounding boxs and ids
            frame = cv2.aruco.drawDetectedMarkers(image=image_rgb,
                                                  corners=corner_selected,
                                                  ids=ids,
                                                  borderColor=(0, 255, 0))
            corner_selected = np.asarray(corner_selected)
            (number, one, points, xy) = corner_selected.shape
            corner_selected = np.reshape(corner_selected, (number,points,xy))
            # Center
            for c in corner_selected:
                c_x = [c[0][0],c[1][0],c[2][0],c[3][0]]
                c_y = [c[0][1],c[1][1],c[2][1],c[3][1]]
                # center
                x = int(max(c_x)/2 + min(c_x)/2)
                y = int(max(c_y)/2 + min(c_y)/2)
                centers.append((x,y))
                cv2.circle(frame, (x,y), 5, (0,0,255), -1)
            # Axis
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corner_selected, 1, self.mtx, self.dist)
            if rvecs is not None and tvecs is not None:
                for rvec, tvec in zip(rvecs, tvecs):
                    cv2.drawFrameAxes(frame, self.mtx, self.dist, rvec, tvec, 1)
                rvecs_np = np.reshape(rvecs, (ids.size,3))
                tvecs_np = np.reshape(tvecs, (ids.size,3))
                # RGB data
                for i in range(ids.size):
                    print("id:",ids[i])
                    print("corner:",corner_selected[i])
                    print("center:",centers[i])
                    print("M:",M[i])
                    print("rvec",rvecs_np[i])
                    print("tvec",tvecs_np[i])
                    print("-----------------------------")
        else:
            frame = image_rgb

        # Publish
        pub_frame = rospy.Publisher("image_color_detected", Image, queue_size=10)
        msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
        pub_frame.publish(msg_frame)

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
    rospy.init_node('image',anonymous=True)
    # Initialize the CvBridge class
    bridge = CvBridge()
    # Subscribe
    image = image()
    # Keep the node existing
    rospy.spin()
