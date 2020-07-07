#! /usr/bin/env python

import rospy
from grasp.srv import data, dataRequest, dataResponse
import numpy as np




if __name__ =='__main__':
    
    rospy.init_node('object_pose')
    ObjectPose = dataRequest()

    rospy.wait_for_service('grasp', 3) # service: grsp
    
    ObjectPose.pose.position.x = 0.339-0.03
    ObjectPose.pose.position.y = 0.0116-0.02
    ObjectPose.pose.position.z = 0.255-0.145
    ObjectPose.pose.orientation.x = 0.245
    ObjectPose.pose.orientation.y = 0.613 
    ObjectPose.pose.orientation.z = -0.202
    ObjectPose.pose.orientation.w = 0.723    
    ObjectPose.id = 0
    
    try:
        do_grasping = rospy.ServiceProxy('grasp', data) # call service
        resp = do_grasping(ObjectPose)
        print(resp.obj_id)

    except rospy.ServiceException as exc:
       print("service did not process request: " + str(exc))

    #rospy.spin()
