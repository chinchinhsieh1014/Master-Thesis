# Insert usb
import pyrealsense2 as rs
import numpy as np
import cv2

try:
    # Initialize
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

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

        # Window
        window = 'RealSense'
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow(window, 0,0)
        cv2.imshow(window, images)

        # Capture
        if cv2.waitKey(1) & 0xFF == ord('c'):
            cv2.imwrite('Dataset/rgb1.png',color_image)
            #cv2.imwrite('Dataset/depth1.png',depth_colormap)
            break

except Exception as e:
    print(e)
    pass

finally:
    # Stop streaming
    pipeline.stop()
    # Close the window
    cv2.destroyAllWindows()
