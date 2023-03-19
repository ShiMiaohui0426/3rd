import cv2
import numpy as np
#import pyfakewebcam
import pyvirtualcam
class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

def GetCross(p1, p2, p):
    return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y)

def IsPointInMatrix(p1, p2, p3, p4, p):
    isPointIn = GetCross(p1, p2, p) * GetCross(p3, p4, p) >= 0 and GetCross(p2, p3, p) * GetCross(p4, p1, p) >= 0
    return isPointIn

print('OpenCV Version:' + cv2.__version__)
fource = cv2.VideoWriter_fourcc(*'DIVX')
cap = cv2.VideoCapture(0)  # cv2.VideoCapture(0)代表调取摄像头资源，其中0代表电脑摄像头，1代表外接摄像头(usb摄像头)

time = int(1)

resulte = cv2.VideoWriter('demo.mp4', fource, 30.0, (640, 480))

vcam=pyvirtualcam.Camera(width=640, height=480, fps=20, device='/dev/video2')
print(f'Using virtual camera: {vcam.device}')


while True:
    success, frame = cap.read()
    flipCode = 1
    center = (int(frame.shape[1]/2), int(frame.shape[0]/2))
    cv2.circle(frame, center, 50, (255, 0, 0), -1)
    LeastOneCatch = False
    cv2.imshow("Video", frame)

    vcam.send(frame)
    vcam.sleep_until_next_frame()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
resulte.release()
cv2.destroyAllWindows()
