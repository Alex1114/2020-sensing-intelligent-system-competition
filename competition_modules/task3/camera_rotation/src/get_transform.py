#!/usr/bin/env python

import math
import time
import numpy as np
import roslib
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
# from scipy.spatial.transform import Rotation as R
from camera_rotation.srv import find_plate
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Twist


class Get_transform(object):
    def __init__(self):
        self.pan = 0
        self.state = 0
        self._trans = []
        self._rot = []
        self.trans = []
        self.rot = []
        self.listener = tf.TransformListener()
        self.pan_pub = rospy.Publisher("/teleop/pan", Float64, queue_size=1)
        self.cmd_vel = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=1) 
        rospy.Service("get_pose", find_plate, self.get_pose)      
    
    def get_pose(self, req):

        plate = req.plate
        time_out = 20
        now = rospy.Time.now()
        while not rospy.is_shutdown():
            if rospy.Time.now() - now > rospy.Duration(time_out):
                return ["tf timeout", None]
            try:
                (trans,rot) = self.listener.lookupTransform('/base_link', "/plate_%d" % plate, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    if self.state == 0:
                        self.pan -= 0.25
                        self.pan_pub.publish(self.pan)
                     
                    else:
                        self.pan += 0.25
                        self.pan_pub.publish(self.pan)
                      
                    if self.pan < -0.8:
                        self.state = 1 

                    if self.pan > 0.8:
                        self.state = 0

                    rospy.sleep(1)
                    continue


            pose = PoseStamped()
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]

            ## get yaw and spin
            yaw = math.atan2(trans[1], trans[0])
            move_cmd = Twist()
            move_cmd.angular.z = yaw * 2
            self.cmd_vel.publish(move_cmd)
            ## get yaw and spin

            yaw = 0
            self.pan = 0
            self.state = 0
            self._trans = []
            self._rot = []
            self.trans = []
            self.rot = []
            self.pan_pub.publish(self.pan)

            rospy.sleep(3)
            break

        time_out = 5
        now = rospy.Time.now()
        while not rospy.is_shutdown():
            if rospy.Time.now() - now > rospy.Duration(time_out):
                return ["tf timeout", None]
            try:
                (trans,rot) = self.listener.lookupTransform('/base_link', "/plate_%d" % plate, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
                    
            pose = PoseStamped()
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            return ["successful", pose]


if __name__ == "__main__":
    rospy.init_node("get_transform")
    transform = Get_transform()
    rospy.spin()
