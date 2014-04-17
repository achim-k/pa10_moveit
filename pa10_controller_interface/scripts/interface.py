#!/usr/bin/env python
import roslib; roslib.load_manifest('pa10_controller_interface')

import sys

import rospy
from pa10controller.srv import *

def getJointConfig_client():
    rospy.wait_for_service('/pa10/getJointConfig')
    try:
        ey = rospy.ServiceProxy('/pa10/getJointConfig', getJointConfig)
        resp1 = ey()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('pa10_controller_interface', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print (getJointConfig_client().positions)
        r.sleep()