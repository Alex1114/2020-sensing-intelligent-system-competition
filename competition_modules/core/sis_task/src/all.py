#! /usr/bin/env python
import math
import rospy
import sys
# from place.srv import data as place_data
from grasp.srv import data, dataRequest
from place.srv import place_data
from camera_rotation.srv import find_plate
from object_detection.srv import get_object_pose
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Pose
import tf
from tf import TransformListener,TransformerROS
from tf.transformations import quaternion_matrix, quaternion_from_euler, euler_from_quaternion
import numpy as np


# def get_plate_pose(to_pos_idx):
# 	rospy.wait_for_service('get_pose') 
# 	try:
# 		get_plate_pose = rospy.ServiceProxy('get_pose', find_plate)
# 		resp1 = get_plate_pose(plate_id) 
# 		return resp1.pose
# 	except rospy.ServiceException, e: 
# 		print "Service call failed: %s"%e

# def usage():
# 	return "%s [plate_id]"%sys.argv[0]





if __name__ == "__main__":
	rospy.init_node("all")
	pub_tilt = rospy.Publisher("/teleop/tilt",Float64, queue_size=1)

	listener = tf.TransformListener()
	transformer = TransformerROS()

	rospy.wait_for_service('get_object_pose') 
	get_object_pose = rospy.ServiceProxy('get_object_pose', get_object_pose)
	object_pose = Pose()
	try:
		response = get_object_pose()
		object_pose = response.pose
		print(object_pose)
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e
		exit()
	
	try:
		(trans,rot) = listener.lookupTransform('/base_link', "/camera_color_optical_frame", rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print("tf error %s"%e)
	

	transpose_matrix = transformer.fromTranslationRotation(trans, rot)
	matrix = quaternion_matrix([object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w])
	position = np.array([object_pose.position.x, object_pose.position.y, object_pose.position.z, 1])

	matrix[:,3] = position
	base_position = np.dot(transpose_matrix, matrix)

	print(base_position)

	#-----------------------------------grasp-------------------------------------------
	grasp_request = dataRequest()

	pub_tilt.publish(0)

	rospy.wait_for_service('grasp', 3)
	
	position = base_position[:-1,3]
	next_tf = np.array([[1,0,0,1],
						[0,1,0,0],
						[0,0,1,0],
						[0,0,0,1]])
	# next_tf[:3,:3] = base_position[:3,:3]
	next_pos = np.dot(base_position, next_tf)

	theta = math.atan2((next_pos[1,3]-position[1]),(next_pos[0,3]-position[0])) 
	
	theta = theta-3.14159265 if theta >= 1.5707 else theta
	theta = theta+3.14159265 if theta <= -1.5707 else theta


	orien = quaternion_from_euler(0, 1.5707, theta)
	print(theta)


	grasp_request.pose.position.x = position[0] - 0.02
	grasp_request.pose.position.y = position[1] 
	grasp_request.pose.position.z = position[2] + 0.07
	grasp_request.pose.orientation.x = orien[0]
	grasp_request.pose.orientation.y = orien[1]
	grasp_request.pose.orientation.z = orien[2]
	grasp_request.pose.orientation.w = orien[3]
	grasp_request.id = 0
	
	try:
		do_grasping = rospy.ServiceProxy('grasp', data) # call service
		resp = do_grasping(grasp_request)
		print(resp.obj_id)

	except rospy.ServiceException as exc:
	   print("service did not process request: " + str(exc))


	#-----------------------------------to pos-------------------------------------------
	to_pos_idx = 2
	topos_request = GoToPosRequest()
	topos_request.pos = to_pos_idx
	gotopos = rospy.ServiceProxy('to_position', GoToPos)

	try:
		resp = gotopos(topos_request)
		print(resp)

	except rospy.ServiceException as exc:
	   print("service did not process request: " + str(exc))

	rospy.sleep(5)
	try:
		get_plate_pose = rospy.ServiceProxy('get_pose', find_plate)
		resp1 = get_plate_pose(to_pos_idx) 
		print("plate_pose: ", resp1)
		
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e

	plate_pose = PoseStamped()
	plate_pose = resp1

	place_pose = Pose()
	place_pose.position.x = plate_pose.pose.pose.position.x - 0.05
	place_pose.position.y = plate_pose.pose.pose.position.y - 0.05
	place_pose.position.z = plate_pose.pose.pose.position.z + 0.1
	place_pose.orientation.x = 0
	place_pose.orientation.y = 0.707
	place_pose.orientation.z = 0
	place_pose.orientation.w = 0.707

	rospy.sleep(5)
	try:
		do_placing = rospy.ServiceProxy("place", place_data) # call service
		resp = do_placing(place_pose, int(0))
		print("successful placing")

	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e	

	# # place_pose.position.x = 0.40983
	# # place_pose.position.y = 0.1010
	# # place_pose.position.z = 0.1003
	# # place_pose.orientation.x = 0
	# # place_pose.orientation.y = 0.707
	# # place_pose.orientation.z = 0
	# # place_pose.orientation.w = 0.707

	# state = 1
	# rospy.sleep(5)

	# if state == 1:
	# 	try:
	# 		do_placing = rospy.ServiceProxy("place", data) # call service
	# 		resp = do_placing(place_pose, 0)
	# 		print("successful placing")

	# 	except rospy.ServiceException, e: 
	# 		print "Service call failed: %s"%e
