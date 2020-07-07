#! /usr/bin/env python

import rospy
import sys
from place.srv import place_data
from camera_rotation.srv import find_plate
from geometry_msgs.msg import PoseStamped, Pose


def get_plate_pose(plate_id):
	rospy.wait_for_service('get_pose') 
	try:
		get_plate_pose = rospy.ServiceProxy('get_pose', find_plate)
		resp1 = get_plate_pose(plate_id) 
		print(resp1)
		return resp1.pose
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e

def usage():
	return "%s [plate_id]"%sys.argv[0]





if __name__ == "__main__":
	rospy.init_node("place_to_box")
	state = 0

	if len(sys.argv) == 2:
		plate_id = int(sys.argv[1])
		print "Requesting plate %s"%(plate_id)

	else:
		print usage()
		sys.exit(1)

	plate_pose = PoseStamped()
	plate_pose = get_plate_pose(plate_id)

	place_pose = Pose()
	place_pose.position.x = plate_pose.pose.position.x - 0.05
	place_pose.position.y = plate_pose.pose.position.y - 0.05
	place_pose.position.z = plate_pose.pose.position.z + 0.1
	place_pose.orientation.x = 0
	place_pose.orientation.y = 0.707
	place_pose.orientation.z = 0
	place_pose.orientation.w = 0.707
	
	# print(place_pose)

	# place_pose.position.x = 0.40983
	# place_pose.position.y = 0.1010
	# place_pose.position.z = 0.1003
	# place_pose.orientation.x = 0
	# place_pose.orientation.y = 0.707
	# place_pose.orientation.z = 0
	# place_pose.orientation.w = 0.707

	state = 1
	rospy.sleep(5)

	if state == 1:
		try:
			do_placing = rospy.ServiceProxy("place", place_data) # call service
			resp = do_placing(place_pose, int(0))
			print("successful placing")

		except rospy.ServiceException, e: 
			print "Service call failed: %s"%e
