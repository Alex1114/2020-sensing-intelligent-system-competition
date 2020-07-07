#! /usr/bin/env python
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

global_rgb_data = Image()
global_depth_data = Image()
global_pc_data = PointCloud2()


flag_do_prediction = False
counter_prediction = 0
cv_bridge = CvBridge() 
mask_publisher = rospy.Publisher('/prediction/mask', Image, queue_size=1)
mask_one_publisher = rospy.Publisher('/prediction/mask_one', Image, queue_size=1)


def prediction_publish(img):
    # define range of red color in HSV
    lower_red = np.array([0,70,80])
    upper_red = np.array([10,255,255])
    lower_green = np.array([35,80,80])
    upper_green = np.array([77,255,255])
    lower_blue = np.array([100,80,80])
    upper_blue = np.array([124,255,255])

    # Threshold the HSV image to get only red colors
    mask_r = cv2.inRange(img, lower_red, upper_red)
    mask_g = cv2.inRange(img, lower_green, upper_green)
    mask_b = cv2.inRange(img, lower_blue, upper_blue)

    mask_eroded = cv2.erode(mask_r, None, iterations = 3)
    mask_eroded_dilated_r = cv2.dilate(mask_eroded, None, iterations = 3)
    mask_eroded = cv2.erode(mask_g, None, iterations = 3)
    mask_eroded_dilated_g = cv2.dilate(mask_eroded, None, iterations = 3)
    mask_eroded = cv2.erode(mask_b, None, iterations = 3)
    mask_eroded_dilated_b = cv2.dilate(mask_eroded, None, iterations = 3)

    area_r = np.asarray(mask_eroded_dilated_r)*1.
    area_g = np.asarray(mask_eroded_dilated_g)*2.
    area_b = np.asarray(mask_eroded_dilated_b)*3.

    # area = (area_r + area_g + area_b)/3.
    area = mask_eroded_dilated_r
    gray_img = np.uint8(area)
    backtorgb = cv2.applyColorMap(gray_img, cv2.COLORMAP_JET)

    # cv_mask = cv_bridge.cv2_to_imgmsg(np.uint8(area), encoding="mono8")   #grays scale
    cv_rgbmask = cv_bridge.cv2_to_imgmsg(backtorgb)
    mask_publisher.publish(cv_rgbmask)

    return gray_img

def prediction_save(img):
    # define range of red color in HSV
    img=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([160,40,40])
    upper_red = np.array([180,255,255])
    lower_green = np.array([65,50,50])
    upper_green = np.array([85,255,255])
    lower_blue = np.array([100,80,80])
    upper_blue = np.array([124,255,255])

    # Threshold the HSV image to get only red colors
    mask_r = cv2.inRange(img, lower_red, upper_red)
    mask_g = cv2.inRange(img, lower_green, upper_green)
    mask_b = cv2.inRange(img, lower_blue, upper_blue)

    mask_eroded = cv2.erode(mask_r, None, iterations = 3)
    mask_eroded_dilated_r = cv2.dilate(mask_eroded, None, iterations = 3)
    mask_eroded = cv2.erode(mask_g, None, iterations = 3)
    mask_eroded_dilated_g = cv2.dilate(mask_eroded, None, iterations = 3)
    mask_eroded = cv2.erode(mask_b, None, iterations = 3)
    mask_eroded_dilated_b = cv2.dilate(mask_eroded, None, iterations = 3)

    area_r = np.asarray(mask_eroded_dilated_r)*1.
    area_g = np.asarray(mask_eroded_dilated_g)*2.
    area_b = np.asarray(mask_eroded_dilated_b)*3.

    area = (area_r + area_g + area_b)/3.
    # area = mask_eroded_dilated_b
    gray_img = np.uint8(area)
    backtorgb = cv2.applyColorMap(gray_img, cv2.COLORMAP_JET)
    
    print("saved image")
    r = rospkg.RosPack()
    path = r.get_path('object_detection')
    cv2.imwrite(path + '/src/opencv.jpg', backtorgb)

    cv_mask = cv_bridge.cv2_to_imgmsg(np.uint8(area), encoding="mono8")   #grays scale
    cv_rgbmask = cv_bridge.cv2_to_imgmsg(backtorgb)
    mask_one_publisher.publish(cv_rgbmask)

    return gray_img

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

def xyzrgb_array_to_pointcloud2(points, stamp=None, frame_id=None, seq=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()

    buf = []

    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if seq: 
        msg.header.seq = seq

    N = len(points)
    msg.height = 1
    msg.width = N

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1)
    ]
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * N
    msg.is_dense = True; 
    msg.data = points.tostring()

    return msg 

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
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            if(mask[u,v] != 0):
                points.append([X,Y,Z,color[0],color[1],color[2]])

    res.mask_cloud = xyzrgb_array_to_pointcloud2(np.array(points))
    return res

if __name__ == '__main__':
    main()
    
