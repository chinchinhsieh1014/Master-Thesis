# Safe Landing Decision Based on Logistic regression For Visual-based UAV

> This project aims to take information collected from a depth camera on a drone and to decide if the position is safe to land or not based on a trained logistic regression classifier.

### Robotics(ROS and Rviz) + Computer vision(Depth camera, OpenCV and Point cloud)
The image captured by the depth camera was processed in a ROS package, and it transferred the 2D world to 3D world by adding depth information. Firstly, detect ArUco markers from the RGB image in the RGB node. It will provide the location of the marker.(At this step, if the marker is too small, it will not be detected. This helps for reducing uncessary detection.) Then, get the depth of the marker from the depth image in the depth node. The distance of the marker will be published through the topic. Lastly, the point node subscribe to the topic and transfer it into the 3D world. It could be presented in cloud point to simulate the real-world situation.

### Machine learning (classifier)
Each case was manually labelled to safe/unsafe. Because it is a binary classification problem and the number of parameters is 3, a logistic regression classifier was trained by gradient descent. After trying several learning rates and iterations, the ideal parameters were found. Then, analyzed the model with the confused matrix and the decision boundary.

https://user-images.githubusercontent.com/79919595/190517989-98f16aae-07e1-4108-9053-38329ca081bd.mp4

---
## Requirement
### Hardware
- [Intel RealSense Depth Camera D415](https://dev.intelrealsense.com/docs/docs-get-started)
### Software
- Environment: Ubuntu 20.04
- ROS Noetic Ninjemys
- OpenCV
- Open3d
- Python
---
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
---
## Classifier

---
## Result
The result shows it has 0.85 accuracy and 0.911 precision. Lastly, I picked 4 special cases - labelled 1 but predicted 0(incorrect), labelled 0 but predicted 1(incorrect), labelled 1 and predicted 1(correct), labelled 0 and predicted 0(correct), to evaluate the performance of the model. In this project, I learned how to deal with massive data, apply machine learning method, and also analyze the model with different tools. 
