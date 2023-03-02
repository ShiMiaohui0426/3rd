import numpy as np
import os
from lib.config.config import make_cfg
from lib.networks import make_network
from lib.utils.net_utils import load_network
from lib.utils.pvnet import pvnet_pose_utils
from plyfile import PlyData
from PIL import Image
import torchvision.transforms as transforms
from lib.utils.linemod.opengl_renderer import OpenGLRenderer
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import argparse
import io
import cv2

import threading
from queue import Queue


def get_model_corners(model):
    min_x, max_x = np.min(model[:, 0]), np.max(model[:, 0])
    min_y, max_y = np.min(model[:, 1]), np.max(model[:, 1])
    min_z, max_z = np.min(model[:, 2]), np.max(model[:, 2])
    corners_3d = np.array([
        [min_x, min_y, min_z],
        [min_x, min_y, max_z],
        [min_x, max_y, min_z],
        [min_x, max_y, max_z],
        [max_x, min_y, min_z],
        [max_x, min_y, max_z],
        [max_x, max_y, min_z],
        [max_x, max_y, max_z],
    ])
    return corners_3d


def read_ply_points(ply_path):
    ply = PlyData.read(ply_path)
    data = ply.elements[0].data
    points = np.stack([data['x'], data['y'], data['z']], axis=1)
    return points


transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
]
)


class object_detector:
    def __init__(self, model_name):
        self.kpt_2d = None
        self.pose_pred = None
        self.model_name = model_name
        self.myparser = argparse.ArgumentParser()
        self.myparser.add_argument("--cfg_file", default="configs/linemod.yaml", type=str)
        self.myparser.add_argument('--test', action='store_true', dest='test', default=False)
        self.myparser.add_argument("--type", type=str, default="")
        self.myparser.add_argument('--det', type=str, default='')
        self.myopts = ['model', model_name, 'cls_type', model_name]
        self.myparser.add_argument("--opts", default=self.myopts, nargs=argparse.REMAINDER)
        self.myargs = self.myparser.parse_args()
        self.cfg = make_cfg(self.myargs)

        self.network = make_network(self.cfg).cuda()
        load_network(self.network, self.cfg.model_dir, resume=self.cfg.resume, epoch=self.cfg.test.epoch)
        self.network.eval()
        self.set_para()

        self.find_thread = threading.Thread(target=self.find_obj_thread)
        self.find_queue = Queue(1024)

    def start_thread(self):
        self.find_thread.start()

    def find_obj_thread(self):
        while True:
            if not self.find_queue.empty():
                img = self.find_queue.get_nowait()
                if img:
                    print(img)
                    self.find_obj(img)

    def is_busy(self):
        return not self.find_queue.empty()

    def add_img_thread(self, img):
        self.find_queue.put_nowait(img)

    def set_para(self):
        model_name = self.model_name
        # prediction of 2d point

        # 3d point (measured)
        data_root = '/home/ros/clean-pvnet/data/linemod/' + model_name

        model_path = os.path.join(data_root, model_name + '.ply')

        renderer = OpenGLRenderer(model_path)
        model = renderer.model['pts'] / 1000

        farthest20 = np.loadtxt('/home/ros/clean-pvnet/data/linemod/' + model_name + '/farthest20.txt')
        kpt_3d = farthest20[0:8]
        mean_3d = np.mean(kpt_3d, axis=0)
        kpt_3d = np.r_[kpt_3d, [mean_3d]]

        # camera K
        '''
        K = np.array([[572.4114, 0, 325.2611],
                      [0, 573.57043, 242.04899],
                      [0, 0, 1, ]])
        '''
        K = np.array([[602.51, 0,         332.199],
                      [0,       602.569, 236.664],
                      [0,       0,          1, ]])

        # draw box

        self.corner_3d = get_model_corners(model)
        # print(corner_3d)
        self.center_3d = np.mean(self.corner_3d, axis=0)
        self.kpt_3d = kpt_3d
        self.K = K

    def find_obj(self, img):
        img = np.array(img)
        inp = transform(img).unsqueeze(0).to("cuda")
        output = self.network(inp)

        # prediction of 6d-pose
        self.kpt_2d = output['kpt_2d'][0].detach().cpu().numpy()
        self.pose_pred = pvnet_pose_utils.pnp(self.kpt_3d, self.kpt_2d, self.K)
        '''
        RT = self.pose_pred
        xyz = np.dot(self.corner_3d, RT[:, :3].T) + RT[:, 3:].T
        center_3d_pred = np.mean(xyz, axis=0)
        return center_3d_pred.tolist()
        '''

    def visualize(self, img, isShow=False):
        corner_2d_pred = self.get_corner_2d()

        fig, ax = plt.subplots(1)
        plt.axis('off')
        ax.imshow(img)
        m_patches = self.get_patches()
        for m_patche in m_patches:
            ax.add_patch(m_patche)
        canvas = fig.canvas
        buffer = io.BytesIO()
        canvas.print_png(buffer)
        data = buffer.getvalue()
        buffer.write(data)
        img = Image.open(buffer)
        img = np.asarray(img)
        while isShow:
            cv2.imshow('image', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # plt.imshow(img)
        # plt.show()
        return img

    def get_corner_2d(self):
        corner_2d_pred = pvnet_pose_utils.project(self.corner_3d, self.K, self.pose_pred)
        return corner_2d_pred

    def get_center_2d(self):
        center_2d_pred = pvnet_pose_utils.project(self.center_3d, self.K, self.pose_pred)
        return center_2d_pred

    def get_center_3d(self):
        center_3d_pred = np.dot(self.center_3d, self.pose_pred[:, :3].T) + self.pose_pred[:, 3:].T
        return center_3d_pred

    def get_patches(self):
        corner_2d_pred = self.get_corner_2d()
        center_2d_pred = pvnet_pose_utils.project(self.center_3d, self.K, self.pose_pred)
        if (0 < center_2d_pred[0][0] < 640) & (0 < center_2d_pred[0][1] < 480):
            m_patches = [Polygon(xy=corner_2d_pred[[0, 1, 3, 2, 0, 4, 6, 2]], fill=False, linewidth=1, edgecolor='b'),
                         Polygon(xy=corner_2d_pred[[5, 4, 6, 7, 5, 1, 3, 7]], fill=False, linewidth=1, edgecolor='b'),
                         Circle(xy=(center_2d_pred[0][0], center_2d_pred[0][1]), radius=20, alpha=1)]
        else:
            m_patches=[]
        return m_patches


def single_test():
    model_name = 'cat'
    cat_detector = object_detector(model_name)
    imgs = []
    i = 40
    while i < 41:
        img = Image.open('/home/ros//clean-pvnet/data/linemod/' + model_name + '/JPEGImages/' + "%06d" % i + '.jpg')
        imgs.append(img)
        i = i + 1
    import time

    time_start = time.time()  # 记录开始时间
    # function()   执行的程序

    for img in imgs:
        cat_detector.find_obj(img)
    time_end = time.time()  # 记录结束时间
    time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s
    print('use time ' + '%f' % (time_sum * 1000 / len(imgs)) + 'ms/img')
    imgs = []

    cat_detector.visualize(img)


def multitest(model_names):
    detectors = []
    for model_name in model_names:
        detector = object_detector(model_name)
        detectors.append(detector)
    model_name = 'duck'
    i = 0
    imgs = []
    while i < 100:
        img = Image.open('/home/ros//clean-pvnet/data/linemod/' + model_name + '/JPEGImages/' + "%06d" % i + '.jpg')
        imgs.append(img)
        i = i + 1
    import time

    time_start = time.time()  # 记录开始时间
    # function()   执行的程序

    for img in imgs:
        for detector in detectors:
            detector.find_obj(img)

    time_end = time.time()  # 记录结束时间
    time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s
    print('use time ' + '%f' % (time_sum * 1000 / len(imgs)) + 'ms/img')
    i = i - 1
    img = Image.open('/home/ros//clean-pvnet/data/linemod/' + model_name + '/JPEGImages/' + "%06d" % i + '.jpg')
    patches = []
    for detector in detectors:
        v_patches = detector.get_patches()
        for v_patche in v_patches:
            patches.append(v_patche)
    fig, ax = plt.subplots(1)
    plt.axis('off')
    ax.imshow(img)
    for m_patche in patches:
        ax.add_patch(m_patche)
    canvas = fig.canvas
    buffer = io.BytesIO()
    canvas.print_png(buffer)
    data = buffer.getvalue()
    buffer.write(data)
    img = Image.open(buffer)
    img = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
    while True:
        cv2.imshow('image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def multitest_thread(model_names):
    detectors = []
    for model_name in model_names:
        detector = object_detector(model_name)
        detectors.append(detector)
    model_name = 'duck'
    i = 0
    imgs = []
    while i < 100:
        img = Image.open('/home/ros//clean-pvnet/data/linemod/' + model_name + '/JPEGImages/' + "%06d" % i + '.jpg')
        imgs.append(img)
        i = i + 1
    import time

    time_start = time.time()  # 记录开始时间
    # function()   执行的程序
    for detector in detectors:
        detector.start_thread()
    for img in imgs:
        for detector in detectors:
            detector.add_img_thread(img)
    status = True
    while status:
        for detector in detectors:
            status = status & detector.is_busy()
    time_end = time.time()  # 记录结束时间
    time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s
    print('use time ' + '%f' % (time_sum * 1000 / len(imgs)) + 'ms/img')
    i = 100
    img = Image.open('/home/ros//clean-pvnet/data/linemod/' + model_name + '/JPEGImages/' + "%06d" % i + '.jpg')
    patches = []
    for detector in detectors:
        v_patches = detector.get_patches()
        for v_patche in v_patches:
            patches.append(v_patche)
    fig, ax = plt.subplots(1)
    plt.axis('off')
    ax.imshow(img)
    for m_patche in patches:
        ax.add_patch(m_patche)
    canvas = fig.canvas
    buffer = io.BytesIO()
    canvas.print_png(buffer)
    data = buffer.getvalue()
    buffer.write(data)
    img = Image.open(buffer)
    img = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
    while True:
        cv2.imshow('image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


'''def savetest():
    model_name = 'cat'
    cat_detector = object_detector(model_name)
    cat_detector.save_onnx()'''

if __name__ == '__main__':
    # single_test()
    multitest(['cat', 'can', 'duck'])
# multitest_thread(['cat', 'can', 'duck'])
