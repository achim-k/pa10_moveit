#!/usr/bin/env python
import roslib; roslib.load_manifest('pa10_controller_interface')

import sys
import rospy


from pa10controller.srv import *

def addItemToQueue_client(position, endOfQueue):
    rospy.wait_for_service('/pa10/addItemtoQueue')
    try:
        ey = rospy.ServiceProxy('/pa10/addItemtoQueue', addToQueue)
        resp1 = ey(False, endOfQueue, position, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('pa10_controller_interface')
    startPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    endPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    stepCount = 10
    multiplier = 5.0

    for i in range(0, stepCount):
        newPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        newPos.append(i*multiplier)
        
        print(tuple(newPos))
        addItemToQueue_client(tuple(newPos), False)

    addItemToQueue_client((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, stepCount*multiplier), True)