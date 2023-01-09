import pyrealsense2 as rs
import numpy as np
import cv2
class mycamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
    def start(self):
        config = rs.config()

        config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        pf = self.pipeline.start( )
    def stop(self):
        self.pipeline.stop()
    def getRGBFrame(self):
        frames = self.pipeline.wait_for_frames()
        color_rs = frames.get_color_frame()
        rgbimg = np.asanyarray(color_rs.get_data())
        b, g, r = cv2.split(rgbimg)
        rgbimg = cv2.merge([r, g, b])
        return rgbimg
    def getDepthFrame(self):
        frames = self.pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        dpimg = np.asanyarray(depth.get_data())
        return dpimg

cmr=mycamera()
cmr.start()
try:
    while True:

        cv2.imshow("RGB", cmr.getRGBFrame())
        cv2.imshow("Depth",cmr.getDepthFrame())
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cmr.stop()
