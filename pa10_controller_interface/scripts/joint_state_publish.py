#!/usr/bin/env python
import roslib; roslib.load_manifest('pa10_controller_interface')

import sys

import rospy
from pa10controller.srv import *
from sensor_msgs.msg import JointState


class PA10ControllerInterface():


    """
    Constructor of industrial robot simulator
    """
    def __init__(self):
        rospy.init_node('pa10_controller_interface', anonymous=True)
        self.joint_state_pub = rospy.Publisher('joint_states', JointState) #Publisher of current joint states

    def get_joints(self):
        rospy.wait_for_service('/pa10/getJointConfig')
        try:
            service = rospy.ServiceProxy('/pa10/getJointConfig', getJointConfig)
            return service()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def publish_joints(self, joint_positions):
        try:
            time = rospy.Time.now()
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = time
            joint_state_msg.name = ['S1', 'S2', 'S3', 'E1', 'E2', 'W1', 'W2']
            joint_state_msg.position = joint_positions
            self.joint_state_pub.publish(joint_state_msg)
        except Exception as e:
            rospy.logerr('Unexpected exception in joint state publisher: %s', e)


if __name__ == "__main__":
    rospy.loginfo('Executing pa10_controller_interface')
    pa10controller = PA10ControllerInterface()
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        getJointResponse = pa10controller.get_joints()
        joint_positions = getJointResponse.positions
        j0 = joint_positions[0] * 3.1416 /180
        j1 = joint_positions[1] * 3.1416 /180
        j2 = joint_positions[2] * 3.1416 /180
        j3 = joint_positions[3] * 3.1416 /180
        j4 = joint_positions[4] * 3.1416 /180
        j5 = joint_positions[5] * 3.1416 /180
        j6 = joint_positions[6] * 3.1416 /180
        joints = j0, j1, j2, j3, j4, j5, j6
        print(type(joints))

        pa10controller.publish_joints(joints)
        r.sleep()
