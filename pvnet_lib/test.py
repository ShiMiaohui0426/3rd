#from lib.config import cfg
# import lib.config
import numpy as np
import os
from lib.config.config import make_cfg
from lib.csrc.fps import fps_utils
from lib.networks import make_network
from lib.utils.net_utils import load_network
from lib.utils.pvnet import pvnet_pose_utils
from plyfile import PlyData
from PIL import Image
import torchvision.transforms as transforms
from lib.utils.linemod.opengl_renderer import OpenGLRenderer
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle
import argparse



model_name = 'cat'
img = Image.open(model_name + '.jpg')
img = np.array(img)
transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
]
)

inp = transform(img).unsqueeze(0).to("cuda")
myparser = argparse.ArgumentParser()
myparser.add_argument("--cfg_file", default="configs/linemod.yaml", type=str)
myparser.add_argument('--test', action='store_true', dest='test', default=False)
myparser.add_argument("--type", type=str, default="")
myparser.add_argument('--det', type=str, default='')
myopts=['model', model_name, 'cls_type', model_name]
myparser.add_argument("--opts", default=myopts, nargs=argparse.REMAINDER)
myargs = myparser.parse_args()

cfg=make_cfg(myargs)
network = make_network(cfg).cuda()
load_network(network, cfg.model_dir, resume=cfg.resume, epoch=cfg.test.epoch)
network.eval()


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


def visualize(img, output):
    # prediction of 2d point
    kpt_2d = output['kpt_2d'][0].detach().cpu().numpy()

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
    K = np.array([[572.4114, 0, 325.2611],
                  [0, 573.57043, 242.04899],
                  [0, 0, 1, ]])
    # prediction of 6d-pose
    pose_pred = pvnet_pose_utils.pnp(kpt_3d, kpt_2d, K)

    # draw box

    corner_3d = get_model_corners(model)
    # print(corner_3d)
    center_3d = np.mean(corner_3d, axis=0)
    corner_2d_pred = pvnet_pose_utils.project(corner_3d, K, pose_pred)
    RT = pose_pred
    xyz = np.dot(corner_3d, RT[:, :3].T) + RT[:, 3:].T
    center_3d_pred = np.mean(xyz, axis=0)
    center_2d_pred = pvnet_pose_utils.project(center_3d, K, pose_pred)
    _, ax = plt.subplots(1)
    ax.imshow(img)
    ax.add_patch(patches.Polygon(xy=corner_2d_pred[[0, 1, 3, 2, 0, 4, 6, 2]], fill=False, linewidth=1, edgecolor='b'))
    ax.add_patch(patches.Polygon(xy=corner_2d_pred[[5, 4, 6, 7, 5, 1, 3, 7]], fill=False, linewidth=1, edgecolor='b'))

    # print(center_2d_pred[0])
    # print(np.mean(corner_2d_pred, axis=0))
    cir1 = Circle(xy=(center_2d_pred[0][0], center_2d_pred[0][1]), radius=20, alpha=1)
    ax.add_patch(cir1)
    plt.show()


if __name__ == '__main__':
    output = network(inp)
    # print(output)
    visualize(img, output)
