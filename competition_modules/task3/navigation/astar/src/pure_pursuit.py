#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point
from nav_msgs.msg import Path, Odometry
import numpy as np
import math
import tf


class PurePursuit(object):
    def __init__(self):
        self.look_ahead_distance = 0.1
        self.path = None

    def set_look_ahead_distance(self, dist):
        self.look_ahead_distance = dist

    def set_path(self, path):
        self.path = np.zeros([len(path.poses), 2])
        for i in range(len(path.poses)):
            self.path[i] = [path.poses[i].pose.position.x,
                            path.poses[i].pose.position.y]

    def get_goal(self, current_pose):  # current_pose : PoseStamped
        if self.path is None:
            rospy.logwarn("purepursuit : no path")
            return
        pose = np.repeat(
            [[current_pose.pose.position.x, current_pose.pose.position.y]], self.path.shape[0], axis=0)
        dist = np.linalg.norm(pose-self.path, axis=1)
        idx = np.argmin(dist)
        unfinished = self.path[idx:]
        pose = pose[idx:]
        dist = np.linalg.norm(pose-unfinished, axis=1)
        unfinished = unfinished[np.where(dist > self.look_ahead_distance)]

        if len(unfinished) < 2:
            rospy.logwarn("purepursuit : no unfinished path")
            self.path = None
            return

        goal = PoseStamped()
        goal.pose.position.x = unfinished[0][0]
        goal.pose.position.y = unfinished[0][1]
        return goal
