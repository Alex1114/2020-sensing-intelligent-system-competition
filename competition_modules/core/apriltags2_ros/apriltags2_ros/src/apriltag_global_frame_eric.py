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

MAX = 200


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
		self.publishing = False
		self.translation = None
		self.quaternion = None

		self.timer = None
		self.sub_pose = rospy.Subscriber("global_pose",PoseStamped,self.cb_pose,queue_size=1)
	
	def pub_tf(self,event):
		# broadcast transformation
		self.broadcaster.sendTransform(self.translation, self.quaternion, rospy.Time.now(), "global", "map") 
		rospy.loginfo("BROADCASTING")

	def cb_pose(self, msg):
		if self.count < MAX:
			# print "fetching"
			trans_g = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
			rot_g = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
			self.trans[self.count] = np.array(trans_g)
			self.quat[self.count]  = np.array(rot_g)
			self.count = self.count + 1
			rospy.loginfo(self.count*100.0/MAX)
		else:
			if not self.collected:
				# take mean values
				self.trans_mean = np.mean(self.trans, axis=0)
				self.quat_mean  = np.mean(self.quat, axis=0)
				self.quat_mean = self.quat_mean/np.linalg.norm(self.quat_mean) # normalization
				self.collected = True
			# build homogeneous transformation from `slam_map` to `GATE(num)` 
			homo_mat = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(self.trans_mean), tf.transformations.quaternion_matrix(self.quat_mean))
			# rot_mat = tf.transformations.euler_matrix(-np.pi/2, np.pi/2, 0) # make X-axis into the turnel and Z-axis upward

			global2camera = tf.transformations.inverse_matrix(homo_mat)

			try:
				# self.listener.waitForTransform('slam_map', 'GATE'+str(x), rospy.Time(0), rospy.Duration(1.0))
				(trans_c, rot_c) = self.listener.lookupTransform('map', 'camera_middle_link', rospy.Time(0))
				camera2map = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans_c), tf.transformations.quaternion_matrix(rot_c))
				global2map = np.matmul(camera2map,global2camera)
				
				self.translation = (global2map[0, 3], global2map[1, 3], global2map[2, 3]) # build translation tuple
				
				euler_ = tf.transformations.euler_from_matrix(global2map) # convert quaternion to euler
				self.quaternion  = tf.transformations.quaternion_from_euler(0, 0, euler_[2]) # manual overwrite roll, pitch to 0

				
				self.sub_pose.unregister()
				# start publish tf
				self.timer = rospy.Timer(rospy.Duration(0.05),self.pub_tf)
				
			except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				rospy.logerr("faile to catch tf camera 2 map")

			



	def on_shutdown(self):
		pass

if __name__ == "__main__":
	rospy.init_node('global_frame')
	node = apriltagsglobalframe()
	rospy.on_shutdown(node.on_shutdown)
	rospy.spin()
