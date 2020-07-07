#!/usr/bin/env python 

import rospy
from pyrobot import Robot
import numpy as np
import time
import tf
from place.srv import data, dataResponse

def placing(req):
        br = tf.TransformBroadcaster()
        # receive data  
        pos_x = req.pose.position.x + 0.03
        pos_y = req.pose.position.y + 0.02
        pos_z = req.pose.position.z + 0.14
        ori_x = req.pose.orientation.x
        ori_y = req.pose.orientation.y
        ori_z = req.pose.orientation.z
        ori_w = req.pose.orientation.w
        obj = req.id

        # setting placing position and orientation
        target_poses= [{"position":np.array([pos_x, pos_y, pos_z]),
                        "orientation":np.array([ori_x,ori_y,ori_z,ori_w])},]

        br.sendTransform((pos_x, pos_y, pos_z),(ori_x, ori_y, ori_z, ori_w),rospy.Time.now(),"place_pose","base_link")

        # placing
        robot.arm.set_ee_pose(**target_poses[0])
        time.sleep(1)
        robot.gripper.open()
        time.sleep(1)

        target_joint = [0, -1.2, 1.57, 0.6 , 0.7]
        robot.arm.set_joint_positions(target_joint,plan=False)

        return dataResponse(obj)


if __name__ == "__main__":

	rospy.init_node("place_object")
	robot = Robot('locobot')
	place = rospy.Service("place", data, placing)
	print("place object")
	rospy.spin()
