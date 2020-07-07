#!/usr/bin/env python 

import rospy
from pyrobot import Robot
import numpy as np
import time
from grasp.srv import data, dataResponse

def grasping(req):

	# receive data	
	pos_x = req.pose.position.x# + 0.03
	pos_y = req.pose.position.y# + 0.02
	pos_z = req.pose.position.z# + 0.145
	ori_x = req.pose.orientation.x
	ori_y = req.pose.orientation.y
	ori_z = req.pose.orientation.z
	ori_w = req.pose.orientation.w
	obj = req.id
	print(pos_x,ori_x)

	# setting grasping position and orientation
	target_poses= [{"position":np.array([pos_x, pos_y, pos_z + 0.03]),
			"orientation":np.array([ori_x,ori_y,ori_z,ori_w])},
			{"position":np.array([pos_x,pos_y,pos_z]),
			"orientation":np.array([ori_x,ori_y,ori_z,ori_w])}]
	
	# go to wait pose
	robot.arm.go_home()
	robot.gripper.open()
	time.sleep(1)

	# go to grasping pose
	for pose in target_poses:
		robot.arm.set_ee_pose(**pose)
		time.sleep(1)

	# grasping
	robot.gripper.close()
	time.sleep(1)	
	print('Hi')
	robot.arm.set_ee_pose(**target_poses[0])
	time.sleep(0.3)

	target_joint = [0, -1.2, 1.57, 0.6 , 0.7]
	robot.arm.set_joint_positions(target_joint,plan=False)
	
	return dataResponse(1) #(obj)

if __name__ == "__main__":

	rospy.init_node("grasp_object")
	robot = Robot('locobot')
	pick = rospy.Service("grasp", data, grasping)
	print("grasp object")
	rospy.spin()
