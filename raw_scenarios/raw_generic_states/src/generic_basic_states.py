#!/usr/bin/python
PKG = 'raw_generic_states'
import roslib
roslib.load_manifest(PKG)

import rospy
import smach
import smach_ros

from os import getenv
robot_platform = getenv("ROBOT")
# yes, this is a hack, a better solution is known and has been discussed,
# cf. the Lua-based Behavior Engine, but is pending implementation after RC5
if robot_platform == "cob3-3":
    robot_platform = "cob"

import action_cmdr
action_cmdr.load(["generic_actions", robot_platform + "_actions"])

class init_robot(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        
        # init arm
        arm_to_init = action_cmdr.move_arm("look_at_table")
        
        #init gripper
        # gripper_open = action_cmdr.move_gripper("open")
                       
        rospy.loginfo("robot initialized")
        
        return 'succeeded'
