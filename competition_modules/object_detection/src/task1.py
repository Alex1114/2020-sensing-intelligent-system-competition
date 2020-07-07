#! /usr/bin/env python
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
from torch.optim import lr_scheduler
from torch.autograd import Variable
from torchvision import models
from torchvision.models.vgg import VGG

import pandas as pd
import scipy.misc
import random
import sys
import numpy as np
import time
import os
import cv2
import rospy
import rospkg
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image , PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, Float32MultiArray, MultiArrayDimension, String
from std_srvs.srv import *
from object_detection.srv import get_mask, get_maskResponse
import ros_numpy


global_rgb_data = Image()
global_depth_data = Image()
global_pc_data = PointCloud2()

r = rospkg.RosPack()
path = r.get_path('object_detection')
global_model_path =  path + '/src/model/FCNs_mini_epoch190_epoch190.pkl'

flag_do_prediction = False
counter_prediction = 0
cv_bridge = CvBridge() 
mask_publisher = rospy.Publisher('/prediction/mask', Image, queue_size=1)
mask_one_publisher = rospy.Publisher('/prediction/mask_one', Image, queue_size=1)
pub_pc = rospy.Publisher("/prediction/pc", PointCloud2, queue_size=1)

class FCN16s(nn.Module):

    def __init__(self, pretrained_net, n_class):
        super(FCN16s, self).__init__()
        self.n_class = n_class
        self.pretrained_net = pretrained_net
        self.relu    = nn.ReLU(inplace = True)
        self.deconv1 = nn.ConvTranspose2d(512, 512, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn1     = nn.BatchNorm2d(512)
        self.deconv2 = nn.ConvTranspose2d(512, 256, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn2     = nn.BatchNorm2d(256)
        self.deconv3 = nn.ConvTranspose2d(256, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn3     = nn.BatchNorm2d(128)
        self.deconv4 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn4     = nn.BatchNorm2d(64)
        self.deconv5 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn5     = nn.BatchNorm2d(32)
        self.classifier = nn.Conv2d(32, n_class, kernel_size=1)

    def forward(self, x):
        output = self.pretrained_net(x)
        x5 = output['x5']  # size=(N, 512, x.H/32, x.W/32)
        x4 = output['x4']  # size=(N, 512, x.H/16, x.W/16)

        score = self.relu(self.deconv1(x5))               # size=(N, 512, x.H/16, x.W/16)
        score = self.bn1(score + x4)                      # element-wise add, size=(N, 512, x.H/16, x.W/16)
        score = self.bn2(self.relu(self.deconv2(score)))  # size=(N, 256, x.H/8, x.W/8)
        score = self.bn3(self.relu(self.deconv3(score)))  # size=(N, 128, x.H/4, x.W/4)
        score = self.bn4(self.relu(self.deconv4(score)))  # size=(N, 64, x.H/2, x.W/2)
        score = self.bn5(self.relu(self.deconv5(score)))  # size=(N, 32, x.H, x.W)
        score = self.classifier(score)                    # size=(N, n_class, x.H/1, x.W/1)
        
        return score

class VGGNet(VGG):
    def __init__(self, pretrained=True, model='vgg16', requires_grad=True, remove_fc=True, show_params=False):
        super(VGGNet, self).__init__(make_layers(cfg[model]))
        self.ranges = ranges[model]

        if pretrained:
            exec("self.load_state_dict(models.%s(pretrained=True).state_dict())" % model)

        if not requires_grad:
            for param in super().parameters():
                param.requires_grad = False

        if remove_fc:  # delete redundant fully-connected layer params, can save memory
            del self.classifier

        if show_params:
            for name, param in self.named_parameters():
                print(name, param.size())

    def forward(self, x):
        output = {}

        # get the output of each maxpooling layer (5 maxpool in VGG net)
        for idx in range(len(self.ranges)):
            for layer in range(self.ranges[idx][0], self.ranges[idx][1]):      
                x = self.features[layer](x)
            output["x%d"%(idx+1)] = x
        return output

ranges = {
    'vgg11': ((0, 3), (3, 6),  (6, 11),  (11, 16), (16, 21)),
    'vgg13': ((0, 5), (5, 10), (10, 15), (15, 20), (20, 25)),
    'vgg16': ((0, 5), (5, 10), (10, 17), (17, 24), (24, 31)),
    'vgg19': ((0, 5), (5, 10), (10, 19), (19, 28), (28, 37))
}

# cropped version from https://github.com/pytorch/vision/blob/master/torchvision/models/vgg.py
cfg = {
    'vgg11': [64, 'M', 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg13': [64, 64, 'M', 128, 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg16': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 'M', 512, 512, 512, 'M', 512, 512, 512, 'M'],
    'vgg19': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 256, 'M', 512, 512, 512, 512, 'M', 512, 512, 512, 512, 'M'],
}

def make_layers(cfg, batch_norm=False):
    layers = []
    in_channels = 3
    for v in cfg:
        if v == 'M':
            layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
        else:
            conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
            if batch_norm:
                layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
            else:
                layers += [conv2d, nn.ReLU(inplace=True)]
            in_channels = v
    return nn.Sequential(*layers)

n_class = 6
use_gpu = torch.cuda.is_available()
num_gpu = list(range(torch.cuda.device_count()))

vgg_model = VGGNet(requires_grad=True, remove_fc=True)
fcn_model = FCN16s(pretrained_net=vgg_model, n_class=n_class)

if use_gpu:
    ts = time.time()
    vgg_model = vgg_model.cuda()
    fcn_model = fcn_model.cuda()
    fcn_model = nn.DataParallel(fcn_model, device_ids=num_gpu)
    print("Finish cuda loading, time elapsed {}".format(time.time() - ts))

def prediction_publish(img):
    means = np.array([103.939, 116.779, 123.68]) / 255.
    img = img[:, :, ::-1]  # switch to BGR
        
    img = np.transpose(img, (2, 0, 1)) / 255.
    img[0] -= means[0]
    img[1] -= means[1]
    img[2] -= means[2]

    # convert to tensor
    img = torch.from_numpy(img.copy()).float()
    img = img[np.newaxis,:,:,:]

    state_dict = torch.load(global_model_path)
    fcn_model.load_state_dict(state_dict)

    if use_gpu:
        inputs = Variable(img.cuda())
    else:
        inputs = Variable(img)
    
    output = fcn_model(inputs)
    output = output.data.cpu().numpy()

    N, _, h, w = output.shape
    pred = output.transpose(0, 2, 3, 1).reshape(-1, n_class).argmax(axis = 1).reshape(N, h, w)
    pred_mask = pred[0]
    area = np.asarray(pred_mask)*30
    gray_img = np.uint8(area)
    backtorgb = cv2.applyColorMap(gray_img, cv2.COLORMAP_JET)

    # cv_mask = cv_bridge.cv2_to_imgmsg(np.uint8(area), encoding="mono8")   #grays scale
    cv_rgbmask = cv_bridge.cv2_to_imgmsg(backtorgb)
    mask_publisher.publish(cv_rgbmask)

    return pred_mask

def prediction_save(img):
    means = np.array([103.939, 116.779, 123.68]) / 255.
    img = img[:, :, ::-1]  # switch to BGR
        
    img = np.transpose(img, (2, 0, 1)) / 255.
    img[0] -= means[0]
    img[1] -= means[1]
    img[2] -= means[2]

    # convert to tensor
    img = torch.from_numpy(img.copy()).float()
    img = img[np.newaxis,:,:,:]

    state_dict = torch.load(global_model_path)
    fcn_model.load_state_dict(state_dict)

    if use_gpu:
        inputs = Variable(img.cuda())
    else:
        inputs = Variable(img)
    
    output = fcn_model(inputs)
    output = output.data.cpu().numpy()

    N, _, h, w = output.shape
    pred = output.transpose(0, 2, 3, 1).reshape(-1, n_class).argmax(axis = 1).reshape(N, h, w)
    pred_mask = pred[0]
    area = np.asarray(pred_mask)*30

    gray_img = np.uint8(area)
    backtorgb = cv2.applyColorMap(gray_img, cv2.COLORMAP_JET)
    
    print("saved image")
    cv2.imwrite(path + '/src/FCN_mask.jpg', backtorgb)

    cv_mask = cv_bridge.cv2_to_imgmsg(np.uint8(area), encoding="mono8")   #grays scale
    cv_rgbmask = cv_bridge.cv2_to_imgmsg(backtorgb)
    mask_one_publisher.publish(cv_rgbmask)

    return pred_mask

def prediction_handler(req):
    global flag_do_prediction
    print "%s"%req.data

    if req.data == False:
        flag_do_prediction = False
        return [True, "Stop FCN prediction"]

    if req.data == True:
        flag_do_prediction = True
        return [True, "Start FCN prediction"]

def task1_prediction(req):
    rgb_image = cv_bridge.imgmsg_to_cv2(global_rgb_data, "bgr8")
    mask = prediction_save(rgb_image)
    return [True, "Start task1 prediction"]


def getimage_cb(rgb_data):
    global global_rgb_data
    global counter_prediction
    global_rgb_data  = rgb_data
    counter_prediction += 1
    
    if flag_do_prediction == True :
        print 'Start Callback'
        if(counter_prediction % 10 == 0):
            rgb_image = cv_bridge.imgmsg_to_cv2(global_rgb_data, "bgr8")
            mask = prediction_publish(rgb_image)

def getdepthimage_cb(depth_data):
    global global_depth_data
    global_depth_data  = depth_data



def getcloud_cb(pc_data):
    global global_pc_data
    global_pc_data  = pc_data
        

def main_cb(rgb_pt_msg):
    global flag_do_prediction
    if flag_do_prediction == True :
        print 'Start Callback'
        rgb_image = cv_bridge.imgmsg_to_cv2(global_rgb_data, "bgr8")
        mask = prediction_publish(rgb_image)
        

   
def main():
    rospy.init_node('fcn_prediction', anonymous=True)
    # 4*4 extrinsic matrix
    # depth2rgb_ext = make_4by4_extrinsic(rgb_int, depth_int, depth2rgb_ext_rot, depth2rgb_ext_trans)
    rospy.Subscriber("/camera/color/image_raw", Image, getimage_cb)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, getdepthimage_cb)
    # rospy.Subscriber("/camera/depth_registered/points", PointCloud2, getcloud_cb)
    rospy.Service('fcn_prediction', SetBool, prediction_handler)
    rospy.Service('task1_prediction', SetBool, task1_prediction)
    rospy.Service('get_maskcloud', get_mask, get_maskcloud)
    rospy.spin()


def get_maskcloud(req):
    res = get_maskResponse()
    points = []
    rgb_image = cv_bridge.imgmsg_to_cv2(global_rgb_data, "bgr8")
    depth = cv_bridge.imgmsg_to_cv2(global_depth_data)
    mask = prediction_save(rgb_image)

    cx = 323.6322
    cy = 240.377166
    fx = 607.2167
    fy = 607.34753
    
    for v in range(rgb_image.shape[1]):
        for u in range(rgb_image.shape[0]):
            color = rgb_image[u,v]
            Z = depth[u,v] / 1000.0
            if Z==0: continue
            X = (v - cx) * Z / fx
            Y = (u - cy) * Z / fy
            if(mask[u,v] != 0):
                points.append([X,Y,Z,color[0],color[1],color[2]])
    points = np.array(points)
    
    data = np.zeros(points.shape[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('r', np.float32),
        ('g', np.float32),
        ('b', np.float32)
    ])
    data['x'] = points[:,0]
    data['y'] = points[:,1]
    data['z'] = points[:,2]
    data['r'] = points[:,3]
    data['g'] = points[:,4]
    data['b'] = points[:,5]

    # res.mask_cloud = array_to_pointcloud2(np.array(points))

    res.mask_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(data)
    res.mask_cloud.header.frame_id = "camera_color_optical_frame"
    res.mask_cloud.header.stamp = rospy.Time.now()
    pub_pc.publish(res.mask_cloud) 
    return res

if __name__ == '__main__':
    main()
    
