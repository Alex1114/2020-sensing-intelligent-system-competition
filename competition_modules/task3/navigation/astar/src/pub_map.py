#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid


class PubMap(object):
    def __init__(self):
        rospy.loginfo("init %s" % rospy.get_name())

        self.fix_map = OccupancyGrid()
        self.fix_map.header.frame_id = "global"
        self.fix_map.info.map_load_time = rospy.Time.now()
        self.fix_map.info.resolution = 0.1
        self.fix_map.info.height = 42
        self.fix_map.info.width = 42
        self.fix_map.info.origin.position.x = 0
        self.fix_map.info.origin.position.y = 0
        self.fix_map.info.origin.position.z = 0
        self.fix_map.info.origin.orientation.w = 1

        tmp_map = np.zeros(
            [self.fix_map.info.width, self.fix_map.info.height], dtype=np.int16)
        tmp_map[13:29, 13:29] = 40
        tmp_map[16:26, 16:26] = 90
        tmp_map[19:23, 0:3] = 20
        tmp_map[3:6, 39:42] = 20
        tmp_map[19:23, 39:42] = 20
        tmp_map[36:39, 39:42] = 20
        tmp_map = np.reshape(tmp_map, -1)

        self.fix_map.data = tmp_map.tolist()

        self.pub_map = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        timer = rospy.Timer(rospy.Duration(1), self.cb_pub)

    def cb_pub(self, event):
        self.fix_map.header.stamp = rospy.Time.now()
        self.pub_map.publish(self.fix_map)


if __name__ == "__main__":
    rospy.init_node("pub_map")
    pubmap = PubMap()
    rospy.spin()
