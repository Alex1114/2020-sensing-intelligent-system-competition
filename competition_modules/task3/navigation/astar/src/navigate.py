#!/usr/bin/env python

import rospy
from pure_pursuit import PurePursuit
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from visualization_msgs.msg import Marker
import numpy as np
import tf


class Navigation(object):
    def __init__(self):
        rospy.loginfo("Initializing %s" % rospy.get_name())
        self.end = np.array([[0.3, 2.2, 1.2, 2.2],
                    [4.1, 3.9, 3, 3.9],
                    [4.1, 2.2, 3, 2.2],
                    [4.1, 0.5, 3, 0.5]])
        self.pursuit = PurePursuit()
        self.pursuit.set_look_ahead_distance(0.2)
        self.pose = None

        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_goal_marker = rospy.Publisher(
            "goal_marker", Marker, queue_size=1)
        self.pub_pid_goal = rospy.Publisher(
            "pid_control/goal", PoseStamped, queue_size=1)
        self.req_path_srv = rospy.ServiceProxy("plan_service", GetPlan)
        self.sub_pose = rospy.Subscriber(
            "pose", PoseStamped, self.cb_pose, queue_size=1)
        self.srv_topos = rospy.Service("to_position", GoToPos, self.to_pos)

    def pub_marker(self, goal=Marker()):
        marker = Marker()
        marker.header.frame_id = "global"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "look ahead"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x = goal.pose.position.x
        marker.pose.position.y = goal.pose.position.y
        marker.id = 0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.pub_goal_marker.publish(marker)

    def to_pos(self, req):
        rospy.loginfo("%s : request pos %d" % (rospy.get_name(), req.pos))
        res = GoToPosResponse()

        if req.pos < 0 or req.pos > 3:
            rospy.logerr("%s : pos not exist" % rospy.get_name())
            res.result = False
            return res
        if self.pose is None:
            rospy.logerr("%s : no pose" % rospy.get_name())
            res.result = False
            return res
        
        my_pose = np.array([self.pose.pose.position.x, self.pose.pose.position.y])
        dis = np.linalg.norm(self.end[req.pos][:2]-my_pose)

        # turn arround --------------------------------------------------------
        if dis > 0.5:
            while not rospy.is_shutdown(): 
                quat = (self.pose.pose.orientation.x,
                    self.pose.pose.orientation.y,
                    self.pose.pose.orientation.z,
                    self.pose.pose.orientation.w)
                _, _, yaw = tf.transformations.euler_from_quaternion(quat)
                ang_err = self.get_goal_angle(yaw, my_pose, self.end[req.pos][:2])                    
                cmd_msg = Twist()
                if ang_err < -30:
                    cmd_msg.linear.x = -0.15
                    cmd_msg.angular.z = -0.8
                elif ang_err > 30:
                    cmd_msg.linear.x = -0.15
                    cmd_msg.angular.z = 0.8
                else:
                    rospy.loginfo("start plan")
                    break
                self.pub_cmd.publish(cmd_msg)
                rospy.sleep(0.1)
        rospy.sleep(1)

        # to pre pos --------------------------------------------------------

        end_p = PoseStamped()
        end_p.pose.position.x = self.end[req.pos][0]
        end_p.pose.position.y = self.end[req.pos][1]
        end_p.pose.position.z = 0
        
        while not rospy.is_shutdown():
            req_path = GetPlanRequest()
            req_path.start = self.pose
            req_path.goal = end_p
            try:
                res_path = self.req_path_srv(req_path)
            except:
                rospy.logerr("%s : path request fail" % rospy.get_name())
                res.result = False
                return res

            self.pursuit.set_path(res_path.plan)
            goal = self.pursuit.get_goal(self.pose)
            if goal is None:
                break
            self.pub_marker(goal)
            self.pub_pid_goal.publish(goal)
            rospy.sleep(0.1)

        # to target pos --------------------------------------------------------
        # end_p = PoseStamped()
        # end_p.pose.position.x = self.end[req.pos][0]
        # end_p.pose.position.y = self.end[req.pos][1]
        # end_p.pose.position.z = 0

        # # pre_p = PoseStamped()
        # # pre_p.pose.position.x = self.end[req.pos][2]
        # # pre_p.pose.position.y = self.end[req.pos][3]
        # # pre_p.pose.position.z = 0
        
        # req_path = GetPlanRequest()
        # req_path.start = self.pose
        # req_path.goal = end_p
        # try:
        #     res_path = self.req_path_srv(req_path)
        # except:
        #     rospy.logerr("%s : path request fail" % rospy.get_name())
        #     res.result = False
        #     return res

        # self.pursuit.set_path(res_path.plan)

        # while not rospy.is_shutdown():
        #     goal = self.pursuit.get_goal(self.pose)
        #     if goal is None:
        #         break
        #     self.pub_marker(goal)
        #     self.pub_pid_goal.publish(goal)
        #     rospy.sleep(0.1)


        res.result = True
        return res

    def cb_pose(self, msg):
        self.pose = msg
    
    def get_goal_angle(self, robot_yaw, robot, goal):
        robot_angle = np.degrees(robot_yaw)
        p1 = [robot[0], robot[1]]
        p2 = [robot[0], robot[1]+1.]
        p3 = goal
        angle = self.get_angle(p1, p2, p3)
        result = angle - robot_angle
        result += 90
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


if __name__ == "__main__":
    rospy.init_node("navigation")
    nav = Navigation()
    rospy.spin()
