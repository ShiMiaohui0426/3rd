import json

import MyVirtualcam
from object_detector import object_detector
from MyCamera import realsensecamera, webcamera

from MyVirtualcam import vcam
from kinetic_tcp_client import kinetic_tcp_client


def load_estimators(model_names):
    detectors = []
    for model_name in model_names:
        detector = object_detector(model_name)
        detectors.append(detector)
    return detectors


from numpy import integer
import cv2
import numpy as np




# m_detectors = load_estimators(['cat', 'can', 'duck'])

m_rcam = realsensecamera()
# m_rcam = webcamera()

m_vcam = vcam()
# qrcoder = cv2.QRCodeDetector()
from ultralytics import YOLO

yolomodel = YOLO("/home/iwata/yolov8/runs/detect/train9/weights/last.pt")
#detector = object_detector('cat')
cat_detector = object_detector('cat')
ape_detector = object_detector('ape')
duck_detector = object_detector('duck')
detectors = {
            'cat': cat_detector,
            'ape': ape_detector,
            'duck': duck_detector,
        }
obj_classes=['ape', 'cat', 'duck']
# note duck rpy's y plus -90

import time
import math

rate = 7
k = 50




fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 视频编解码器

out = cv2.VideoWriter('demo.avi', fourcc, 30, (640, 480))  # 写入视频
class state_machine:
    def __init__(self):
        self.obj_pose = None
        self.rs_y = None
        self.rs_x = None
        self.jdata = None
        self.data = None
        self.case = {
            'generate_fake_cmr': self.generate_fake_cmr,
            'connect_server': self.connect_server,
            'wait_operation': self.wait_operation,
            'confirm': self.confirm,
            'wait_grasp_done': self.wait_grasp_done
        }
        self.cnt = kinetic_tcp_client()
        self.state = 'generate_fake_cmr'
        self.counter = 0
        self.last_select_flag = False
        self.last_select_obj=None

    def switch_auto(self):
        method = self.case.get(self.state)
        if method:
            method()

    def generate_fake_cmr(self):
        m_rcam.start()
        m_rcam.getFrame()
        m_vcam.disp_image(m_rcam.getRGBFrame())
        self.state = 'connect_server'

    def connect_server(self):
        self.cnt.try_connect_srv()
        self.cnt.start_rec()
        self.state = 'wait_operation'
        img = m_rcam.getRGBFrame()
    

    def wait_operation(self):
        m_rcam.getFrame()
        img = m_rcam.getRGBFrame()
        #
        imgcenter = (int(img.shape[1] / 2), int(img.shape[0] / 2))
        cv2.circle(img, imgcenter, round(self.counter * 20 / k), (255, 0, 0), -1)
        cv2.circle(img, imgcenter, 20, (255, 0, 0), 0)
        R=40
        # yoloresults = yoloresults
        if not self.last_select_flag:
            yoloresults = yolomodel(img, stream=True)
            select_flag = False
            for result in yoloresults:
                boxes = result.boxes
                for box in boxes:
                    yolo_center_x=(box.xyxy[0][0]+box.xyxy[0][2])/2
                    yolo_center_y=(box.xyxy[0][1]+box.xyxy[0][3])/2
                    
                    L = round(math.hypot(imgcenter[0] - yolo_center_x, imgcenter[1] - yolo_center_y))
                    if L < R:
                        self.last_select_obj=obj_classes[int(box.cls.item())]
                        detector=detectors[self.last_select_obj]
                        detector.find_obj(img)
                        #center = detector.get_center_2d()
                        #corner_2d_pred = detector.get_corner_2d()
                        
                        o_pose = detector.get_center_3d()
                        o_quaternion = detector.get_quaternion()
                        cv2.circle(img, (round(int(yolo_center_x)), round(int(yolo_center_y))), 40, (0, 255, 255), 3)
                        detector.draw3Dbox(img)
                        self.obj_pose = [o_pose[0][0], o_pose[0][1], o_pose[0][2], o_quaternion[0], o_quaternion[1],o_quaternion[2], o_quaternion[3]]
                        select_flag = True
                    else:
                        cv2.circle(img, (round(int(yolo_center_x)), round(int(yolo_center_y))), 40, (0, 0, 255), 3)
                        cv2.rectangle(img, (round(int(box.xyxy[0][0])), round(int(box.xyxy[0][1]))), (round(int(box.xyxy[0][2])), round(int(box.xyxy[0][3]))), (0, 255, 0), 2)
        else:
            detector=detectors[self.last_select_obj]
            detector.find_obj(img)
            center = detector.get_center_2d()
            L = round(math.hypot(imgcenter[0] - center[0][0], imgcenter[1] - center[0][1]))
            # print('x:', o_pose[0][0], 'y:', o_pose[0][1], 'z:', o_pose[0][2])
            detector.draw3Dbox(img)
            cv2.circle(img, (round(center[0][0]), round(center[0][1])), R, (0, 0, 255), 3)
            if L < R:
                self.counter = self.counter + 1
                o_pose = detector.get_center_3d()
                o_quaternion = detector.get_quaternion()
                cv2.circle(img, (round(center[0][0]), round(center[0][1])), R, (0, 255, 0), 3)

                self.obj_pose = [o_pose[0][0], o_pose[0][1], o_pose[0][2], o_quaternion[0], o_quaternion[1],o_quaternion[2], o_quaternion[3]]
                select_flag = True
            else:
                select_flag = False

        if select_flag == True:
            self.counter = self.counter + 1
            self.send_pose(self.obj_pose)
        else:
            if self.counter > 1:
                    data = {'command': 'clear_grasp_target'}
                    jdata = json.dumps(data)
                    self.cnt.send_data(jdata)
                    self.counter = 0
        self.last_select_flag = select_flag
        out.write(img)  # 写入帧
        m_vcam.disp_image(img)
        
        if self.counter > k:
            self.counter = 0
            self.state = 'confirm'
            self.counter = time.time()
            self.rs_x = 0
            self.rs_y = 0
            self.last_select_flag = False

    def send_pose(self, pose):
        data = {'position': {'x': pose[0], 'y': pose[1], 'z': pose[2]},
                'orientation': {'x': pose[3], 'y': pose[4], 'z': pose[5], 'w': pose[6]},
                'command': 'set_grasp_target'}
        jdata = json.dumps(data)
        self.cnt.send_data(jdata)

    def confirm(self):

        while not self.cnt.inputData.empty():
            jdata = self.cnt.inputData.get()
        m_rcam.getFrame()
        img = m_rcam.getRGBFrame()
        rpy_accel = m_rcam.getGyroFrame()
        self.rs_x = self.rs_x + rpy_accel[0]
        self.rs_y = self.rs_y + rpy_accel[1]
        imgZero = np.zeros(img.shape, np.uint8)
        confirm = (int(2 * imgZero.shape[1] / 3), int(imgZero.shape[0] / 2))
        cancel = (int(imgZero.shape[1] / 3), int(imgZero.shape[0] / 2))
        imgcenter = (int(img.shape[1] / 2 + rate * self.rs_y), int(img.shape[0] / 2 - rate * self.rs_x))
        cv2.circle(imgZero, confirm, 50, (0, 255, 0), 3)

        cv2.circle(imgZero, cancel, 50, (255, 0, 0), 3)

        
        combine = cv2.addWeighted(img, 0.5, imgZero, 0.5, 0)
        cv2.circle(combine, imgcenter, 5, (255, 0, 0), -1)

        text = 'Yes'
        cv2.putText(combine, text, confirm, cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
        text = 'No'
        cv2.putText(combine, text, cancel, cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
        out.write(combine)  # 写入帧
        m_vcam.disp_image(combine)

        L_cancel = round(math.hypot(imgcenter[0] - cancel[0], imgcenter[1] - cancel[1]))
        L_confirm = round(math.hypot(imgcenter[0] - confirm[0], imgcenter[1] - confirm[1]))
        if L_cancel < 50:
            self.counter = 1
            self.state = 'wait_operation'
            data = {'command': 'clear_grasp_target'}
            jdata = json.dumps(data)
            self.cnt.send_data(jdata)
        if L_confirm < 50:
            data = {'command': 'start_grasp'}
            jdata = json.dumps(data)
            self.cnt.send_data(jdata)
            self.counter = 0
            self.state = 'wait_grasp_done'
        # print('confirm')

    #旋转矩阵转四元数
    def rotationMatrixToQuaternion(R):
        q = np.zeros((4, 1))
        q[0] = math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        q[1] = (R[2, 1] - R[1, 2]) / (4 * q[0])
        q[2] = (R[0, 2] - R[2, 0]) / (4 * q[0])
        q[3] = (R[1, 0] - R[0, 1]) / (4 * q[0])
        return q

    def wait_grasp_done(self):
        print('wait_grasp_done')
        data = {'command': 'query_grasp_done'}
        jdata = json.dumps(data)
        while True:
            m_rcam.getFrame()
            img = m_rcam.getRGBFrame()
            imgZero = np.zeros(img.shape, np.uint8)
            center = (int(imgZero.shape[1] / 2), int(imgZero.shape[0] / 2))
            cv2.circle(imgZero, center, 50, (255, 255, 0), -1)

            combine = cv2.addWeighted(img, 0.5, imgZero, 0.5, 0)
            text = 'Wait for Grasp'
            cv2.putText(combine, text, center, cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            out.write(combine)  # 写入帧
            m_vcam.disp_image(combine)

            if self.counter > 50:
                self.counter = 0
                self.cnt.send_data(jdata)
            self.counter = self.counter + 1
            if not self.cnt.inputData.empty():
                jdata = self.cnt.inputData.get()
                if jdata['command']:
                    self.state = 'wait_operation'
                    self.counter = 1
                    break


if __name__ == '__main__':
    m_state_machine = state_machine()
    try:

        while True:
            m_state_machine.switch_auto()
    finally:
        out.release()
        a=1