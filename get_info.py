#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo

def callback(data):
    rospy.loginfo(data)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('camera', anonymous=True)
    # Subscribe - depth
    rospy.Subscriber("/camera/depth/camera_info", CameraInfo, callback)
    # Subscribe - color
    # rospy.Subscriber("/camera/color/camera_info", CameraInfo, callback)
    # keep the node exiting
    rospy.spin()
    
''' Example
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)
#   The projection matrix P projects 3D points into the rectified image.
[INFO] [1658305510.300339]: header:
seq: 2891
stamp:
    secs: 1658305510
    nsecs: 295875549
    frame_id: "camera_depth_optical_frame"
height: 720
width: 1280
distortion_model: "plumb_bob"

# The distortion parameters
D: [0.0, 0.0, 0.0, 0.0, 0.0]

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# the focal lengths (fx, fy) and principal point (cx, cy).
# Depth
K: [893.9522094726562, 0.0, 625.6613159179688, 0.0, 893.9522094726562, 354.7122802734375, 0.0, 0.0, 1.0]
# Color
K: [906.0023193359375, 0.0, 639.5233764648438, 0.0, 904.431884765625, 352.720947265625, 0.0, 0.0, 1.0]

# Rectification matrix (stereo cameras only)
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# For monocular cameras, Tx = Ty = 0.
P: [893.9522094726562, 0.0, 625.6613159179688, 0.0, 0.0, 893.9522094726562, 354.7122802734375, 0.0, 0.0, 0.0, 1.0, 0.0]

binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
'''
