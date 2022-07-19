import cv2
import numpy as np

# Load the dictionary
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

# Background
image = np.zeros((600, 600), dtype=np.uint8)
image[:] = 255

# Generate the marker
m1 = 33
marker1 = np.zeros((200, 200), dtype=np.uint8)
marker1 = cv2.aruco.drawMarker(dictionary, m1, 200, marker1, 1)
m2 = 50
marker2 = np.zeros((200, 200), dtype=np.uint8)
marker2 = cv2.aruco.drawMarker(dictionary, m2, 200, marker2, 1)

# Merge two markers
image[10:210,10:210] = marker1
image[220:420,220:420] = marker2

cv2.imwrite("image_{}_{}.png".format(m1,m2), image)
