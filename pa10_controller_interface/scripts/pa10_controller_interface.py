#!/usr/bin/env python
import roslib; roslib.load_manifest('pa10_controller_interface')

import rospy
import threading
import utils

# Services from pa10_server
from pa10controller.srv import *

# Publish
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback 

# Subscribe
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class PA10ControllerInterface():

    """
    Constructor of PA10 controller interface
    """
    def __init__(self):
        rospy.init_node('pa10_controller_interface')

        # Class lock
        self.lock = threading.Lock()
        
        # Publish rate (hz)
        self.pub_rate = rospy.get_param('pub_rate', 10.0)
        rospy.loginfo("Setting publish rate (hz) based on parameter: %f", self.pub_rate)
        
        # Joint names
        def_joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"] 
        self.joint_names = rospy.get_param('controller_joint_names', def_joint_names)
        if len(self.joint_names) == 0:
            rospy.logwarn("Joint list is empty, did you set controller_joint_name?")
        rospy.loginfo("Setting joint names based on parameter: %s", str(self.joint_names))

        # Published to joint states
        rospy.loginfo("Creating joint state publisher")
        self.joint_state_pub = rospy.Publisher('joint_states', JointState)
        
        # Published to joint feedback
        rospy.loginfo("Creating joint feedback publisher")
        self.joint_feedback_pub = rospy.Publisher('feedback_states', FollowJointTrajectoryFeedback)

        # Subscribe to a joint trajectory
        rospy.loginfo("Creating joint trajectory subscriber")
        self.joint_path_sub = rospy.Subscriber('joint_path_command', JointTrajectory, self.trajectory_callback)

        # Timed task (started automatically)
        period = rospy.Duration(1.0/self.pub_rate)
        rospy.loginfo('Setting up publish worker with period (sec): %s', str(period.to_sec()))
        rospy.Timer(period, self.publish_worker)


    """
    The publish worker is executed at a fixed rate.  The publishes the various
    state and status information for the robot.
    """
    def publish_worker(self, event):
        self.joint_state_publisher()
            
       
    """
    The joint state publisher publishes the current joint state and the current
    feedback state (as these are closely releated)
    """       
    def joint_state_publisher(self):

        try:
            joint_state_msg = JointState()
            joint_fb_msg = FollowJointTrajectoryFeedback()
            time = rospy.Time.now()
           
            with self.lock:
                
                #Joint states 
                joint_state_msg.header.stamp = time
                joint_state_msg.name = self.joint_names
                getJointsResponse = self.get_joints()
                joint_state_msg.position = utils.tupleToRad(getJointsResponse.positions) # TODO check return value
                # print(getJointsResponse.positions)
                # print(getJointsResponse.commands)
                self.joint_state_pub.publish(joint_state_msg)
                
                #Joint feedback
                joint_fb_msg.header.stamp = time
                joint_fb_msg.joint_names = self.joint_names
                joint_fb_msg.actual.positions = utils.tupleToRad(getJointsResponse.positions) # TODO check return value
                
                self.joint_feedback_pub.publish(joint_fb_msg)
                         
        except Exception as e:
            rospy.logerr('Unexpected exception in joint state/feedback publisher: %s', e)


    """
    Trajectory subscription callback (gets called whenever a joint trajectory
    is received).

    @param msg_in: joint trajectory message
    @type  msg_in: JointTrajectory
    """
    def trajectory_callback(self, msg_in):
        try:
    
            rospy.loginfo('Received trajectory with %s points, executing callback', str(len(msg_in.points)))
            print('Received trajectory with %s points, executing callback')%str(len(msg_in.points))
            # TODO check wether robot is in motion
            # if self.motion_ctrl.is_in_motion():
            #     if len(msg_in.points) > 0:
            #         rospy.logerr('Received trajectory while still in motion, trajectory splicing not supported')
            #     else:
            #         rospy.loginfo('Received empty trajectory while still in motion, stopping current trajectory')
            #     self.motion_ctrl.stop()
                
            # else:
            if(len(msg_in.points) <= 0):
                return


            joints = self.get_joints().positions
            print("joints: " + str(joints))
            print("start:" + str(utils.tupleToDeg(msg_in.points[0].positions)))
            print("stop:" + str(utils.tupleToDeg(msg_in.points[-1].positions)))
            print("continue? [y/n]")
            if 'y' != raw_input():
                print "trajectory aborted"
                return
        
            for point in msg_in.points:
                
                endOfQueue = False
                if point == msg_in.points[-1]: # if point is last point of trajectory
                    endOfQueue = True

                rospy.wait_for_service('/pa10/addItemtoQueue')
                try:
                    addItmToQueueService = rospy.ServiceProxy('/pa10/addItemtoQueue', addToQueue)
                    resp1 = addItmToQueueService(False, endOfQueue, utils.tupleToDeg(point.positions), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
                # point = self._to_controller_order(msg_in.joint_names, point)
                # self.motion_ctrl.add_motion_waypoint(point)
                # TODO call pa10_server service addQueue 
                
                    
        except Exception as e:
            rospy.logerr('Unexpected exception: %s', e)

        
        rospy.loginfo('Exiting trajectory callback')

    """
    Calls the pa10_server service to get the joint values
    """
    def get_joints(self):
        rospy.wait_for_service('/pa10/getJointConfig')
        try:
            service = rospy.ServiceProxy('/pa10/getJointConfig', getJointConfig)
            return service()
        except rospy.ServiceException, e:
            rospy.logerr("Service call /pa10/getJointConfig failed: %s", e)

        return []



if __name__ == "__main__":
    try:
        rospy.loginfo('Executing pa10_controller_interface')
        controller = PA10ControllerInterface()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    

        