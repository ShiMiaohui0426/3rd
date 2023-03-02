from lib.config import cfg
import numpy as np
import os

from lib.networks import make_network
from lib.utils.net_utils import load_network
from lib.utils.pvnet import pvnet_pose_utils

from PIL import Image
import torch
import torchvision.transforms as transforms

import matplotlib.pyplot as plt
import matplotlib.patches as patches

transform =  transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ]
)

#visualize for can (in linemod dataset),it can't work for other object's visualization
def visualize(img,output):
    #prediction of 2d point
    kpt_2d = output['kpt_2d'][0].detach().cpu().numpy()
    #3d point (measured)
    kpt_3d = np.array([[-0.00917353, -0.05002207, -0.0756572 ],\
        [ 0.00759332,  0.04584576, -0.06784071],\
        [ 0.00451049, -0.00901732,  0.11692607],\
        [-0.00830873, -0.08648306,  0.00880504],\
        [ 0.00969079,  0.09486647,  0.01997318],\
        [ 0.04785323, -0.01317644, -0.03835452],\
        [-0.04797452,  0.00296431, -0.03814207],\
        [-0.04599569, -0.00109471,  0.07093264],\
        [ 0.0002785,   0.004249,    0.020324  ]])
    #camera K
    K = np.array([[572.4114,    0,      325.2611 ],\
        [  0,      573.57043, 242.04899],\
        [  0,        0,        1,     ]])
    #prediction of 6d-pose
    pose_pred = pvnet_pose_utils.pnp(kpt_3d, kpt_2d, K)
    #draw box
    corner_3d = np.array([[-0.050117, -0.086649, -0.076543],\
        [-0.050117, -0.086649,  0.117191],\
        [-0.050117,  0.095147, -0.076543],\
        [-0.050117,  0.095147,  0.117191],\
        [ 0.050674, -0.086649, -0.076543],\
        [ 0.050674, -0.086649,  0.117191],\
        [ 0.050674,  0.095147, -0.076543],\
        [ 0.050674,  0.095147,  0.117191]])
    corner_2d_pred = pvnet_pose_utils.project(corner_3d, K, pose_pred)
    _, ax = plt.subplots(1)
    ax.imshow(img)
    ax.add_patch(patches.Polygon(xy=corner_2d_pred[[0, 1, 3, 2, 0, 4, 6, 2]], fill=False, linewidth=1, edgecolor='b'))
    ax.add_patch(patches.Polygon(xy=corner_2d_pred[[5, 4, 6, 7, 5, 1, 3, 7]], fill=False, linewidth=1, edgecolor='b'))
    plt.show()

def test():
    #load model
    network = make_network(cfg).cuda()
    load_network(network, cfg.model_dir, resume=cfg.resume, epoch=cfg.test.epoch)
    network.eval()
    #test image  (480, 640, 3)
    path="can.jpg"
    img = Image.open(path)
    img = np.array(img)
    #inp torch.Size([1, 3, 480, 640])
    inp = transform(img).unsqueeze(0).to("cuda")
    with torch.no_grad():
        output = network(inp)
    visualize(img,output)

if __name__ == '__main__':
    test()