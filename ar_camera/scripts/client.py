#!/usr/bin/env python

import sys
import rospy
from obj_msg.srv import CameraRequests

def pose_client():
    rospy.wait_for_service('/AR_Marker_Service')
    try:
        pose_esti = rospy.ServiceProxy('/AR_Marker_Service', CameraRequests)
        resp1 = pose_esti()
        print(resp1)
        return resp1.pose_array
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print "%s"%(pose_client())
