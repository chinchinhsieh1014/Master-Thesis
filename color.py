#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def callback(data):
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

    M = 100
    if ids is not None:
        # Reshape ids and corners
        ids = ids.reshape(-1)
        corner = np.asarray(corners)
        (number, one, points, xy) = corner.shape
        corner = np.reshape(corner, (number,points,xy))
        corner_selected = corners
        count = 0
        # Remove patches which are smaller than M
        for i,c in enumerate(corner):
            dist1 = np.linalg.norm(c[0]-c[1])
            dist2 = np.linalg.norm(c[1]-c[2])
            dist3 = np.linalg.norm(c[2]-c[3])
            dist4 = np.linalg.norm(c[3]-c[0])
            if max(dist1,dist2,dist3,dist4)<M:
                print("Remove:",ids[count])
                print("Distance:",max(dist1,dist2,dist3,dist4))
                corner_selected = np.delete(corner_selected, count, 0)
                ids = np.delete(ids, count, 0)
            else:
                count = count+1
    if ids is not None:
        # Center
        
        # Draw bounding boxs and ids
        frame = cv2.aruco.drawDetectedMarkers(image=image_rgb,
                                              corners=corner_selected,
                                              ids=ids,
                                              borderColor=(0, 255, 0))
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
    rospy.init_node('color_detected',anonymous=True)
    # Initialize the CvBridge class
    bridge = CvBridge()

    # Subscribe
    rospy.Subscriber("/camera/color/image_raw", Image, callback)

    # Keep the node existing
    rospy.spin()
