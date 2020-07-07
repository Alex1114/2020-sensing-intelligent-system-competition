#! /usr/bin/env python
import rospy
import numpy as np
import tf
import tf2_ros
import sys
import math
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped, Vector3, Quaternion, Transform
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

MAX = 500


class apriltagsglobalframe():
	def __init__(self):
		self.listener = tf.TransformListener()
		self.broadcaster = tf.TransformBroadcaster()
		self.target = rospy.get_param("~target", 1)
		rospy.loginfo("target %d" % self.target)
		self.trans = np.zeros((MAX, 3))  # translation placeholder
		self.quat = np.zeros((MAX, 4))  # orientation placeholder
		self.count = 0  # time counter
		self.collected = False
		self.trans_mean = None
		self.quat_mean = None

		rospy.Timer(rospy.Duration(0.01), self.listen_tf)

	def listen_tf(self, event):
		if self.count < MAX:
			print "fetching"
			try:
				# self.listener.waitForTransform('slam_map', 'GATE'+str(x), rospy.Time(0), rospy.Duration(1.0))
				(trans_g, rot_g) = self.listener.lookupTransform('map', 'GATE'+str(self.target), rospy.Time(0))
				self.trans[self.count] = np.array(trans_g)
				self.quat[self.count]  = np.array(rot_g)
				self.count = self.count + 1
			except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				pass
		else:
			if not self.collected:
				# take mean values
				self.trans_mean = np.mean(self.trans, axis=0)
				self.quat_mean  = np.mean(self.quat, axis=0)
				self.quat_mean = self.quat_mean/np.linalg.norm(self.quat_mean) # normalization
				self.collected = True
			# build homogeneous transformation from `slam_map` to `GATE(num)` 
			homo_mat = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(self.trans_mean), tf.transformations.quaternion_matrix(self.quat_mean))
			rot_mat = tf.transformations.euler_matrix(-np.pi/2, np.pi/2, 0) # make X-axis into the turnel and Z-axis upward
			
			trans_map2global = np.matmul(homo_mat, rot_mat) # this is the final homogeneous transformation matrix from `slam_map` to `global`
			translation = (trans_map2global[0, 3], trans_map2global[1, 3], trans_map2global[2, 3]) # build translation tuple
			
			euler_ = tf.transformations.euler_from_matrix(trans_map2global) # convert quaternion to euler
			quaternion  = tf.transformations.quaternion_from_euler(0, 0, euler_[2]) # make R and P 0, i.e., they are in the same plane
			self.broadcaster.sendTransform(translation, quaternion, rospy.Time.now(), "global", "map") # broadcast transformation
			
			print "broadcast"

	def on_shutdown(self):
		pass

if __name__ == "__main__":
	rospy.init_node('Apriltag_Global_Frame')
	node = apriltagsglobalframe()
	rospy.on_shutdown(node.on_shutdown)
	rospy.spin()
