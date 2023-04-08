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
detector = object_detector('cat')

import time
import math

rate = 7


class state_machine:
    def __init__(self):
        self.rs_y = None
        self.rs_x = None
        self.l_center = None
        self.p_center = None
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
        detector.find_obj(img)
        center = detector.get_center_2d()
        self.p_center = center
        self.l_center = center
        self.counter = 1

    def wait_operation(self):
        #        center = (int(frame.shape[1] / 2), int(frame.shape[0] / 2))
        #        cv2.circle(frame, center, 50, (255, 0, 0), -1)

        '''
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        m_vcam.disp_image(img)
        # 解码
        
        codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(gray)
        '''
        m_rcam.getFrame()
        img = m_rcam.getRGBFrame()
        detector.find_obj(img)
        center = detector.get_center_2d()
        corner_2d_pred = detector.get_corner_2d()
        imgcenter = (int(img.shape[1] / 2), int(img.shape[0] / 2))
        cv2.circle(img, imgcenter, self.counter, (255, 0, 0), -1)
        cv2.circle(img, imgcenter, 20, (255, 0, 0), 0)
        if (0 < center[0][0] < 640) & (0 < center[0][1] < 480):
            temp = center[0] - corner_2d_pred[[2]]
            R = round(0.6 * math.hypot(temp[0][0], temp[0][1]))
            cv2.polylines(img, [np.array(corner_2d_pred[[0, 1, 3, 2, 0, 4, 6, 2]], np.int)], True, (98, 9, 11), 1)
            cv2.polylines(img, [np.array(corner_2d_pred[[5, 4, 6, 7, 5, 1, 3, 7]], np.int)], True, (98, 9, 11), 1)

            L = round(math.hypot(imgcenter[0] - center[0][0], imgcenter[1] - center[0][1]))
            if L < R:
                self.counter = self.counter + 1
                cv2.circle(img, (round(center[0][0]), round(center[0][1])), R, (0, 255, 0), 3)
            else:
                self.counter = 1
                cv2.circle(img, (round(center[0][0]), round(center[0][1])), R, (255, 0, 0), 3)
        else:
            center = self.l_center + (self.l_center - self.p_center) * 0.5
            cv2.circle(img, (round(center[0][0]), round(center[0][1])), 20, (255, 0, 0), 3)

        self.p_center = self.l_center
        self.l_center = center
        m_vcam.disp_image(img)

        if self.counter > 30:
            self.counter = 0
            self.state = 'confirm'
            self.counter = time.time()
            self.rs_x = 0
            self.rs_y = 0
            o_pose = detector.get_center_3d()
            self.obj_pose = [o_pose[0][0], o_pose[0][1], o_pose[0][2]]
            self.send_pose(self.obj_pose)

    def send_pose(self,pose):
        data = {'position': {'x': pose[0], 'y': pose[1], 'z': pose[2]},
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

        cv2.circle(img, imgcenter, 5, (255, 0, 0), -1)
        combine = cv2.addWeighted(img, 0.5, imgZero, 0.5, 0)

        text = 'Yes'
        cv2.putText(combine, text, confirm, cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
        text = 'No'
        cv2.putText(combine, text, cancel, cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
        m_vcam.disp_image(combine)

        L_cancel = round(math.hypot(imgcenter[0] - cancel[0], imgcenter[1] - cancel[1]))
        L_confirm = round(math.hypot(imgcenter[0] - confirm[0], imgcenter[1] - confirm[1]))
        if L_cancel < 50:
            self.counter = 1
            self.state = 'wait_operation'
        if L_confirm < 50:
            data = {'command': 'start_grasp'}
            jdata = json.dumps(data)
            self.cnt.send_data(jdata)
            self.counter = 0
            self.state = 'wait_grasp_done'
        # print('confirm')

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
            m_vcam.disp_image(combine)

            if self.counter > 10:
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
    while True:
        m_state_machine.switch_auto()
