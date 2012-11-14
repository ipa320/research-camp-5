#!/usr/bin/python
PKG = 'raw_generic_states'
import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import actionlib 
import raw_base_placement.msg

from os import getenv
robot_platform = getenv("ROBOT")
# yes, this is a hack, a better solution is known and has been discussed,
# cf. the Lua-based Behavior Engine, but is pending implementation after RC5
if robot_platform == "cob3-3":
    robot_platform = "cob"

import action_cmdr
action_cmdr.load(["generic_actions", robot_platform + "_actions"])

class approach_pose(smach.State):

    def __init__(self, pose = ""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['base_pose_to_approach'])

        self.pose = pose;    

    def execute(self, userdata):
        
        if(self.pose == ""):
            self.pose2 = userdata.base_pose_to_approach
        else:
	    	self.pose2 = self.pose 
        
        handle_base = action_cmdr.move_base(self.pose2)

        while True:                
            rospy.sleep(0.1)
            base_state = handle_base.get_state()
            if (base_state == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
                return "succeeded"
            elif (base_state == actionlib.simple_action_client.GoalStatus.ACTIVE):
                continue
            else:
                print 'last state: ',base_state
                return "failed"
            

class adjust_pose_wrt_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.ac_base_adj_name = '/raw_base_placement/adjust_to_workspace'
        self.ac_base_adj = actionlib.SimpleActionClient(self.ac_base_adj_name, raw_base_placement.msg.OrientToBaseAction)

    def execute(self, userdata):
        
            
        rospy.loginfo("Waiting for action server <<%s>> to start ...", self.ac_base_adj_name);
        self.ac_base_adj.wait_for_server()
        rospy.loginfo("action server <<%s>> is ready ...", self.ac_base_adj_name);
        action_goal = raw_base_placement.msg.OrientToBaseActionGoal()
            
        action_goal.goal.distance = 0.4;
        rospy.loginfo("send action");
        self.ac_base_adj.send_goal(action_goal.goal);
        
        rospy.loginfo("wait for base to adjust");
        finished_before_timeout = self.ac_base_adj.wait_for_result()
    
        if finished_before_timeout:
            rospy.loginfo("Action finished: %s", self.ac_base_adj.get_state())
            return 'succeeded'    
        else:
            rospy.logerr("Action did not finish before the time out!")
            return 'failed'         

