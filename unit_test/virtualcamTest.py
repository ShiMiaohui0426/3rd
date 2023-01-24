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
'''
detector = cv2.wechat_qrcode_WeChatQRCode()
depro = 'QRModel/detect.prototxt'
decaf = 'QRModel/detect.caffemodel'
srpro = 'QRModel/sr.prototxt'
srcaf = 'QRModel/sr.caffemodel'
detector = cv2.wechat_qrcode_WeChatQRCode(depro, decaf, srpro, srcaf)
'''
time = int(1)

resulte = cv2.VideoWriter('demo.mp4', fource, 30.0, (640, 480))

#camera = pyfakewebcam.FakeWebcam('/dev/video2', 640, 480)


cam=pyvirtualcam.Camera(width=640, height=480, fps=20, device='/dev/video2')
print(f'Using virtual camera: {cam.device}')


while True:


    #framev = np.zeros((cam.height, cam.width, 3), np.uint8)  # RGB
    #framev[:] = cam.frames_sent % 255  # grayscale animation

    success, frame = cap.read()
    flipCode = 1
    #frame = cv2.flip(frame, flipCode)
    center = (int(frame.shape[1]/2), int(frame.shape[0]/2))
    cv2.circle(frame, center, 50, (255, 0, 0), -1)
    #res, points = detector.detectAndDecode(frame)
    #qrcodes, points = detector.detectAndDecode(frame)
    LeastOneCatch = False
    '''
    i = 0
    for pos in points:
        color = (0, 0, 255)
        thick = 3
        corners = []

        for p in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            start = int(pos[p[0]][0]), int(pos[p[0]][1])
            end = int(pos[p[1]][0]), int(pos[p[1]][1])
            corners.append(Point(start[0], start[1]))
            cv2.line(frame, start, end, (255, 0, 0), thick)
            str_context = str(res[i])
            font = cv2.FONT_HERSHEY_COMPLEX  # 字体类型
            color = (0, 255, 0)  # 颜色选择，单通道只有黑白
            start_point = (int(pos[0][0]), int(pos[0][1] - 10))  # 文字起始点
            print_size = 1  # 字体大小
            thickness = 1  # 文字粗细
            cv2.putText(frame, str_context, start_point, font, print_size, color, thickness)
        isInMat = IsPointInMatrix(corners[0], corners[1], corners[2], corners[3], Point(center[0], center[1]))
        if (isInMat):
            LeastOneCatch = isInMat
            cv2.circle(frame, center, 20, (0, 0, 255), 0)
            cv2.circle(frame, center, int(round(time / 2)), (0, 0, 255), -1)
            print(time)
            if (time > 40):
                time = int(0)
            for p in [(0, 1), (1, 2), (2, 3), (3, 0)]:
                start = int(pos[p[0]][0]), int(pos[p[0]][1])
                end = int(pos[p[1]][0]), int(pos[p[1]][1])
                cv2.line(frame, start, end, color, thick)
        i = i + 1
    if (LeastOneCatch):
        time = time + 1
    else:
        time = 0

    # print('qrcodes: ', qrcodes)
'''
    cv2.imshow("Video", frame)
    #print(type(frame))
    #resulte.write(frame)
    #camera.schedule_frame(frame)
    cam.send(frame)
    cam.sleep_until_next_frame()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
resulte.release()
cv2.destroyAllWindows()
