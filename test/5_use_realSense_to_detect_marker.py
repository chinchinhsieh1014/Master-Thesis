# Insert usb
import pyrealsense2 as rs
import numpy as np
import cv2

def detect(frame,mtx,dist):
    '''
    - Input -

    - Output -
    '''
    # Detect markers
    corners, ids, rejected = cv2.aruco.detectMarkers(frame,
                                                     dictionary,
                                                     parameters=parameters)
    # the bounding box and the id
    frame = cv2.aruco.drawDetectedMarkers(image=frame,
                                          corners=corners,
                                          ids=ids,
                                          borderColor=(0, 255, 0))
    # the axis
    # rvecs, tvecs分别是角点中每个标记的旋转和平移向量
    if ids is not None:
         rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                               1,
                                                               mtx,
                                                               dist)
         for rvec, tvec in zip(rvecs, tvecs):
             cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 1)
    return frame

try:
    # Initialize
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # Aruco marker dictionaries
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    # Detector parameters
    parameters = cv2.aruco.DetectorParameters_create()

    while True:
        # Frame
        frames = pipeline.wait_for_frames()
        frame_rgb = frames.get_color_frame()
        frame_depth = frames.get_depth_frame()
        if not frame_rgb or not frame_depth:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(frame_rgb.get_data())
        depth_image = np.asanyarray(frame_depth.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Merge RGB and Depth
        images = np.hstack((color_image, depth_colormap))

        #Intel Realsense camera Matrix
        mtx = [[613.8421020507812, 0, 321.9654541015625],
                [0, 614.451416015625,249.003173828125],
                [ 0,0,1]]
        mtx= np.array(mtx).reshape(3,3)

        # distortion coefficient of the camera
        dist = [-1.71057285e-02, -7.15934343e-05, -1.00114895e-03,  1.15475573e-02, 5.46266277e-07]
        dist = np.array(dist).reshape(5,1)

        # Detect
        output = detect(images,mtx,dist)

        # Window
        window = 'RealSense'
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow(window, 0,0)
        cv2.imshow(window, output)

        # Shut down
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(e)
    pass

finally:
    # Stop streaming
    pipeline.stop()
    # Close the window
    cv2.destroyAllWindows()
