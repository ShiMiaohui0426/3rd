import pyvirtualcam
import cv2


class vcam:
    def __init__(self):
        self.cam = pyvirtualcam.Camera(width=640, height=480, fps=20, device='/dev/video8')
        print(f'Using virtual camera: {self.cam.device}')

    def disp_image(self, img):
        self.cam.send(img)
        self.cam.sleep_until_next_frame()

if __name__ == '__main__':
    cam=vcam()
    cap = cv2.VideoCapture(0)  # cv2.VideoCapture(0)代表调取摄像头资源，其中0代表电脑摄像头，1代表外接摄像头(usb摄像头)
    while True:
        success, frame = cap.read()
        center = (int(frame.shape[1] / 2), int(frame.shape[0] / 2))
        cv2.circle(frame, center, 50, (255, 0, 0), -1)
        cv2.imshow("Video", frame)
        cam.disp_image(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
