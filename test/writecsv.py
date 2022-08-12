import pandas as pd
import numpy as np
import cv2

file = "/home/yc/catkin_ws/src/realsense-ros/realsense2_camera/dataset/RGB/data.csv"
df = pd.read_csv(file)
rvec = np.zeros((500,3))
normal = np.zeros((500,3))
r = df['rvec'].tolist()

for i,v in enumerate(r):
    v = v[1:-1]
    v_list =  v.split()
    rvec[i] = [float(v_list[0]),float(v_list[1]),float(v_list[2])]


for i,v in enumerate(rvec):
    r, _  = cv2.Rodrigues(v)
    normal[i] = r[:, 2]

for v in normal:
    data = {"x":v[0],"y":v[1],"z":v[2]}
    df = pd.DataFrame(data,dtype=float, index=[0])
    df.to_csv("normal.csv", mode="a", header=False)
