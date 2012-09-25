#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')

import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

class init_robot(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        
    def execute(self, userdata):
        
        # init arm
        arm_to_init = sss.move("arm", "initposition")
        
        #init gripper
        gripper_open = sss.move("gripper", "open")
                       
        rospy.loginfo("robot initialized")
        
        return 'succeeded'
