import pyrealsense2 as rs
import numpy as np
import cv2
import math

class webcamera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
    def start(self):
        a=None
    def getRGBFrame(self):
        success, frame = self.cap.read()

        return frame


class realsensecamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        print('RealSense L515 INIT:', self.pipeline)

    def start(self):
        config = rs.config()
        # config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.gyro)

        print('RealSense L515 START')

        pf = self.pipeline.start(config)
        profile = pf.get_stream(rs.stream.color)
        intr = profile.as_video_stream_profile().get_intrinsics()
        print(
            intr)  # 获取内参 width: 640, height: 480, ppx: 319.115, ppy: 234.382, fx: 597.267, fy: 597.267, model: Brown Conrady, coeffs: [0, 0, 0, 0, 0]

    def stop(self):
        self.pipeline.stop()
        print('RealSense L515 STOP')

    def getFrame(self):
        self.frames = self.pipeline.wait_for_frames()

    def getRGBFrame(self):

        frames=self.frames
        color_rs = frames.get_color_frame()
        rgbimg = np.asanyarray(color_rs.get_data())
        # b, g, r = cv2.split(rgbimg)
        # rgbimg = cv2.merge([r, g, b])
        return rgbimg

    def getDepthFrame(self):
        frames=self.frames
        depth = frames.get_depth_frame()
        dpimg = np.asanyarray(depth.get_data())
        return dpimg

    def getGyroFrame(self):
        frames = self.frames
        gyro = frames.first_or_default(rs.stream.gyro)
        return [gyro.as_motion_frame().get_motion_data().x,gyro.as_motion_frame().get_motion_data().y,gyro.as_motion_frame().get_motion_data().z]



if __name__ == '__main__':
    cmr = realsensecamera()
    cmr.start()
    try:
        while True:
            cv2.imshow("RGB", cmr.getRGBFrame())
            # cv2.imshow("Depth",cmr.getDepthFrame())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cmr.stop()
