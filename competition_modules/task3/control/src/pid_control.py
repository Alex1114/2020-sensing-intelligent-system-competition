#!/usr/bin/env python
import numpy as np
import rospy
import tf
import math
import time
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.server import Server
from control.cfg import pos_PIDConfig, ang_PIDConfig
from std_srvs.srv import SetBool, SetBoolResponse
from PID import PID_control


class Robot_PID():
    def __init__(self):
        self.dis4constV = 1.  # Distance for constant velocity
        self.pos_ctrl_max = 1
        self.pos_ctrl_min = 0.0
        self.cmd_ctrl_max = 1
        self.cmd_ctrl_min = -1
        self.goal = None
        self.robot_pos = None
        self.robot_yaw = None

        rospy.loginfo("[%s] Initializing " % rospy.get_name())

        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.pos_control = PID_control("Position")
        self.ang_control = PID_control("Angular")

        self.pos_srv = Server(pos_PIDConfig, self.pos_pid_cb, "Position")
        self.ang_srv = Server(ang_PIDConfig, self.ang_pid_cb, "Angular")

        self.initialize_PID()

        self.sub_goal = rospy.Subscriber(
            "pid_control/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_pose = rospy.Subscriber(
            'pose', PoseStamped, self.cb_pose, queue_size=1)

    def cb_pose(self, msg):
        self.robot_pos = [msg.pose.position.x, msg.pose.position.y]
        quat = (msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        self.robot_yaw = yaw

    def control(self, goal_distance, goal_angle):
        self.pos_control.update(goal_distance)
        self.ang_control.update(goal_angle)

        # pos_output will always be positive
        pos_output = self.pos_constrain(-self.pos_control.output /
                                        self.dis4constV)

        # -1 = -180/180 < output/180 < 180/180 = 1
        ang_output = self.ang_control.output/180.
        return pos_output, ang_output

    def cb_goal(self, p):
        self.goal = [p.pose.position.x, p.pose.position.y]

        if self.robot_pos is None:
            rospy.logwarn("%s : no robot pose" % rospy.get_name())
            return

        # self.robot_yaw = self.robot_yaw + np.pi/2
        goal_distance = self.get_distance(self.robot_pos, self.goal)
        goal_angle = self.get_goal_angle(
            self.robot_yaw, self.robot_pos, self.goal)

        pos_output, ang_output = self.control(goal_distance, goal_angle)

        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_constarin(pos_output)
        cmd_msg.angular.z = self.cmd_constarin(ang_output)
        self.pub_cmd.publish(cmd_msg)

    def cmd_constarin(self, input):
        if input > self.cmd_ctrl_max:
            return self.cmd_ctrl_max
        if input < self.cmd_ctrl_min:
            return self.cmd_ctrl_min
        return input

    def pos_constrain(self, input):
        if input > self.pos_ctrl_max:
            return self.pos_ctrl_max
        if input < self.pos_ctrl_min:
            return self.pos_ctrl_min
        return input

    def initialize_PID(self):
        self.pos_control.setSampleTime(1)
        self.ang_control.setSampleTime(1)

        self.pos_control.SetPoint = 0.0
        self.ang_control.SetPoint = 0.0

    def get_goal_angle(self, robot_yaw, robot, goal):
        robot_angle = np.degrees(robot_yaw)
        p1 = [robot[0], robot[1]]
        p2 = [robot[0], robot[1]+1.]
        p3 = goal
        angle = self.get_angle(p1, p2, p3)
        result = angle - robot_angle
        result = self.angle_range(-(result + 90.))
        return result

    def get_angle(self, p1, p2, p3):
        v0 = np.array(p2) - np.array(p1)
        v1 = np.array(p3) - np.array(p1)
        angle = np.math.atan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
        return np.degrees(angle)

    # limit the angle to the range of [-180, 180]
    def angle_range(self, angle):
        if angle > 180:
            angle = angle - 360
            angle = self.angle_range(angle)
        elif angle < -180:
            angle = angle + 360
            angle = self.angle_range(angle)
        return angle

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def pos_pid_cb(self, config, level):
        print(
            "Position: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.pos_control.setKp(Kp)
        self.pos_control.setKi(Ki)
        self.pos_control.setKd(Kd)
        return config

    def ang_pid_cb(self, config, level):
        print(
            "Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.ang_control.setKp(Kp)
        self.ang_control.setKi(Ki)
        self.ang_control.setKd(Kd)
        return config


if __name__ == '__main__':
    rospy.init_node('pid_control')
    foo = Robot_PID()
    rospy.spin()
