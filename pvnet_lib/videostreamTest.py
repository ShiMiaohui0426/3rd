from object_detector import object_detector
from MyCamera import realsensecamera
import cv2
import numpy as np

detector = object_detector('cat')

cam = realsensecamera()
cam.start()
fource = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('demo.mp4', fource, 30.0, (640, 480))
cam.getFrame()
img = cam.getRGBFrame()
detector.find_obj(img)
center = detector.get_center_2d()
p_center = center
l_center = center
import math

x = 0
y = 0
rate = 3
while True:
    cam.getFrame()
    img = cam.getRGBFrame()
    rpy = cam.getGyroFrame()
    detector.find_obj(img)
    center = detector.get_center_2d()
    corner_2d_pred = detector.get_corner_2d()
    temp = center[0] - corner_2d_pred[[2]]
    R = round(0.6 * math.hypot(temp[0][0], temp[0][1]))
    if (0 < center[0][0] < 640) & (0 < center[0][1] < 480):
        cv2.polylines(img, [np.array(corner_2d_pred[[0, 1, 3, 2, 0, 4, 6, 2]], np.int)], True, (98, 9, 11), 1)
        cv2.polylines(img, [np.array(corner_2d_pred[[5, 4, 6, 7, 5, 1, 3, 7]], np.int)], True, (98, 9, 11), 1)
        cv2.circle(img, (round(center[0][0]), round(center[0][1])), R, (255, 0, 0), 3)
    else:
        center = l_center + (l_center - p_center) * 0.5
        cv2.circle(img, (round(center[0][0]), round(center[0][1])), 20, (255, 0, 0), -1)
    p_center = l_center
    l_center = center
    x = x + rpy[0]
    y = y + rpy[1]
    imgcenter = (int(img.shape[1] / 2 + rate * y), int(img.shape[0] / 2 - rate * x))
    cv2.circle(img, imgcenter, 3, (255, 0, 0), -1)

    out.write(img)
    cv2.imshow("RGB", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
out.release()
