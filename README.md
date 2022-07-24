# Title
> detail

## Requirement
### Hardware
- [Intel RealSense Depth Camera D415](https://dev.intelrealsense.com/docs/docs-get-started)
### Software
- Environment: Ubuntu 20.04
- ROS Noetic Ninjemys

## Step
1. collect data

## Dataset
### Data Collection
- Markers are on different surfaces and orientations
- Take images with RealSnese from different camera positoins
-> several hundred Aruco identifications
- Recoreded information
  -  Surface ID 
     -  SceneNumber+ArUcoID
  -  Viewpoint ID
     -  ImageNumber
  -  Surface normal
     -  [u, v, w] (pointing inward) (from Aruco rotation matrix)
  -  Depth data
     - NxN bounding box [x0, y0, x1, y1]
  -  RGB data
     - MxM block
     - corner [x_i, y_i] with i=0...3,
     - w = max(x_0, x_1, x_2, x_3) - min(x_0, x_1, x_2, x_3)
     - h = max(y_0, y_1, y_2, y_3) - min(y_0, y_1, y_2, y_3)
     - M = max(w,h)
  -  Drone landing feasibility
     - 0: impossible
     - 1: difficult
     - 2: easy
