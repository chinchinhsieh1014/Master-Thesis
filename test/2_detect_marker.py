import cv2
import numpy as np
import cv2.aruco as aruco

# Load the dictionary
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
# Set parameters
parameters = cv2.aruco.DetectorParameters_create()

# Input image
file = "image_33_50.png"
image = cv2.imread(file)

# Detect markers
corners, ids, rejected = cv2.aruco.detectMarkers(image,
                                                 dictionary,
                                                 parameters=parameters)

# Draw the bounding box
if ids is not None:
    for i in range(len(ids)):
        # Draw the bounding box
        (topLeft,topRight,bottomRight,bottomLeft) = corners[i][0]
        topRight = (int(topRight[0]),int(topRight[1]))
        bottomRight = (int(bottomRight[0]),int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]),int(bottomLeft[1]))
        topLeft = (int(topLeft[0]),int(topLeft[1]))
        cv2.line(image,topLeft,topRight,(0,255,0),2)
        cv2.line(image,topRight,bottomRight,(0,255,0),2)
        cv2.line(image,bottomRight,bottomLeft,(0,255,0),2)
        cv2.line(image,bottomLeft,topLeft,(0,255,0),2)
        # Spot the center
        c = corners[i][0]
        M = cv2.moments(c)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        cv2.circle(image,(cx,cy),1,(0,0,255),8)

# Output the result
file = "detect_result.png"
cv2.imwrite(file, image)
