#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs2
import numpy as np
import sys
import os
path = "/home/yc/catkin_ws/src/realsense-ros/realsense2_camera/dataset/RGB/"
import pandas as pd
from realsense2_camera.msg import color_data

class image():
    def __init__(self):
        # Color
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.color_info)
        rospy.Subscriber("/camera/color/image_rect_color", Image, self.color_callback)
        self.mtx = np.empty([3, 3])
        self.dist = np.empty([1, 5])
        self.sense = 1
        self.view = 1

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
        image_gray = cv2.medianBlur(image_gray,5)
        # Adaptive threshold
        th3 = cv2.adaptiveThreshold(image_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
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
                if len(corner_selected)==2:
                    print("Detected")
                    print("Collected RGB data(csv and image)")
                    rospy.sleep(3)
                    #publish the topic
                    pub = rospy.Publisher('color_data', color_data, queue_size=10, latch=True)
                    data = color_data()
                    corner_selected_np = np.asarray(corner_selected, dtype=int)
                    data.detected_corners = corner_selected_np.reshape(-1).tolist()
                    data.sense = self.sense
                    data.view = self.view
                    ids_np = np.asarray(ids, dtype=int)
                    data.detected_ids = ids_np.tolist()
                    # publish the message to the topic
                    pub.publish(data)
                    print("RGB")
                    print(data)
                    # write to csv
                    for i in range(ids.size):
                        data = {"surface ID":"sense"+str(self.sense)+"_"+str(ids[i]),
                                "view":self.view,
                                "rvec":[rvecs_np[i]],
                                "tvec":[tvecs_np[i]],
                                "feasibility":0}
                        df = pd.DataFrame(data)
                        df.to_csv(os.path.join(path,"data.csv"), mode='a', header=False)
                    file = "sense"+str(self.sense)+"_"+str(self.view)+".png"
                    cv2.imwrite(os.path.join(path,file),frame)
                    self.view = self.view+1
            # Publish
            pub_frame = rospy.Publisher("image_color_detected", Image, queue_size=10)
            msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
            pub_frame.publish(msg_frame)
        else:
            frame = image_rgb
            # Publish
            pub_frame = rospy.Publisher("image_color_detected", Image, queue_size=10)
            msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
            pub_frame.publish(msg_frame)

if __name__ == '__main__':
    # Aruco marker dictionaries
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    # Detector parameters
    parameters = cv2.aruco.DetectorParameters_create()
    # Initialize the node
    rospy.init_node('rgb_frame',anonymous=True)
    rospy.sleep(2)
    print("Capture Sense1_View1")
    # Initialize the CvBridge class
    bridge = CvBridge()
    # Subscribe
    image = image()
    # Keep the node existing
    rospy.spin()
