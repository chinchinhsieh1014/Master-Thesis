#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
#import open3d as o3d
import numpy as np

class point():
    def __init__(self):
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)
        
    def callback(self, data):
        x = np.array(list(pc2.read_points(data, skip_nans=True, field_names="x")))
        y = np.array(list(pc2.read_points(data, skip_nans=True, field_names="y")))
        z = np.array(list(pc2.read_points(data, skip_nans=True, field_names="z")))
        p = np.array(list(pc2.read_points(data, skip_nans=True, field_names="intensity")))[:, 0]
        points_array = np.zeros(x.shape[0] + y.shape[0] + z.shape[0], dtype=np.float32)
        points_array[::3] = np.squeeze(x)
        points_array[1::3] = np.squeeze(y)
        points_array[2::3] = np.squeeze(z)
        print(points_array.size)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('point',anonymous=True)
    # Subscribe
    point = point()
    # Keep the node existing
    rospy.spin()
