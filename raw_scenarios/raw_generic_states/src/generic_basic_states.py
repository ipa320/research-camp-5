#!/usr/bin/python
PKG = 'raw_generic_states'
import roslib
roslib.load_manifest(PKG)

import rospy
import smach
import smach_ros

import action_cmdr
action_cmdr.init(PKG)

class init_robot(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        
    def execute(self, userdata):
        
        # init arm
        arm_to_init = action_cmdr.move_arm("initposition")
        
        #init gripper
        #gripper_open = action_cmdr.move("gripper", "open")
                       
        rospy.loginfo("robot initialized")
        
        return 'succeeded'
