#! /usr/bin/env python
import rospy
import os
from std_msgs.msg import Int16, Float64
from sensor_msgs.msg import Image, CompressedImage, Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import argparse
from tf2_msgs.msg import TFMessage


# support forwad type
class_type = {'Int16': Int16, 'Image': Image, 'Joy': Joy,
        'CompressedImage': CompressedImage, 'Twist': Twist, 'Float64': Float64, 'Odometry': Odometry, 'TFMessage': TFMessage}


class Bridge(object):
    def __init__(self, ip):
        self.bridge_type = rospy.get_param('~bridge_type', 'Twist')
        self.from_msg = rospy.get_param('~bridge_from', '/topic_name_from')
        self.to_msg = rospy.get_param('~bridge_to', '/topic_name_to')

        rospy.loginfo('ip in target network %s' % ip)
        rospy.loginfo('bridging from %s' % self.from_msg)
        rospy.loginfo('bridging to %s' % self.to_msg)
        rospy.loginfo('type %s' % self.bridge_type)

        self.pub_msg = rospy.Publisher(
            self.to_msg, class_type[self.bridge_type], queue_size=1)
        self.sub_msg = rospy.Subscriber(
            self.from_msg, class_type[self.bridge_type], self.cb_msg, queue_size=1)

    def cb_msg(self, msg):
        self.pub_msg.publish(msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='brige option')
    parser.add_argument('ip', help='ip in target network')
    parser.add_argument('dummy1', help='dummy')
    parser.add_argument('dummy2', help='dummy')

    args = parser.parse_args()
    os.environ['ROS_IP'] = args.ip

    rospy.init_node('bridge')
    bridge = Bridge(args.ip)
    rospy.spin()
