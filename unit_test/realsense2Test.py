import pyrealsense2 as rs
import numpy as np
import cv2
#import matplotlib.pyplot as plt
import random
#print(rs.__version__)
# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640,480, rs.format.rgb8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)


pf = pipeline.start(config)

print(f"device: {pf.get_device()}")
print(f"depth_sensor: {pf.get_device().first_depth_sensor()}")
print(f"depth_scale: {pf.get_device().first_depth_sensor().get_depth_scale()}")
print(f"streams: {pf.get_streams()}")

try:
    while True:
        # Create a pipeline object. This object configures the streaming camera and owns it's handle
        frames = pipeline.wait_for_frames()

        depth = frames.get_depth_frame()
        data_rgb = frames.get_data_size()
        color_rs = frames.get_color_frame()
        rgbimg = np.asanyarray(color_rs.get_data())
        dpimg=np.asanyarray(depth.get_data())
        b, g, r = cv2.split(rgbimg)
        rgbimg = cv2.merge([r, g, b])
        cv2.imshow("RGB", rgbimg)
        cv2.imshow("Depth",dpimg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
