#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_navigation')

import rospy
import sensor_msgs.msg
import actionlib
import brics_actuator.msg
import raw_arm_navigation.msg
import arm_navigation_msgs.msg
import tf
from raw_srvs.srv import *


class GripperActionServer:
	def __init__(self):
		self.received_state = False
		
		#read joint names
		if (not rospy.has_param("joints")):
			rospy.logerr("No gripper joints given.")
			exit(0)
		else:
			self.joint_names = sorted(rospy.get_param("joints"))
			rospy.loginfo("gripper joints: %s", self.joint_names)
		
		# read joint limits
		self.joint_limits = []
		for joint in self.joint_names:
			if ((not rospy.has_param("limits/" + joint + "/min")) or (not rospy.has_param("limits/" + joint + "/min"))):
				rospy.logerr("No gripper limits given.")
				exit(0)
			else:
				limit = arm_navigation_msgs.msg.JointLimits()
				limit.joint_name = joint 
				limit.min_position = rospy.get_param("limits/" + joint + "/min")
				limit.max_position = rospy.get_param("limits/" + joint + "/max")
				self.joint_limits.append(limit)	
		
		self.current_joint_configuration = [0 for i in range(len(self.joint_names))]
		self.unit = "m"
		
		
		# subscriptions
		rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)
		
		# publications
		self.pub_joint_positions = rospy.Publisher("position_command", brics_actuator.msg.JointPositions)
		
		# action server
		self.as_move_joint_direct = actionlib.SimpleActionServer("MoveToJointConfigurationDirect", raw_arm_navigation.msg.MoveToJointConfigurationAction, execute_cb = self.execute_cb_move_joint_config_direct)
	
		# service server
		self.srv_gripper_closed = rospy.Service('is_gripper_closed',  ReturnBool , self.is_gripper_closed)
  
  
	def joint_states_callback(self, msg):
		for k in range(len(self.joint_names)):
			for i in range(len(msg.name)):
				if (msg.name[i] == self.joint_names[k]):
					#rospy.loginfo("%s: %f", msg.name[i], msg.position[i])
					self.current_joint_configuration[k] = msg.position[i]
					
		#print 'joint states received'
		self.received_state = True
			
		return True
	
	def execute_cb_move_joint_config_direct(self, action_msgs):
		rospy.loginfo("move gripper to joint configuration")
		
		if not self.is_joint_configuration_not_in_limits(action_msgs.goal):
			result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
			result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
			self.as_move_joint_direct.set_aborted(result)
			return
		
		timeout = rospy.Duration(3)
		movement_start = rospy.get_rostime()
		self.pub_joint_positions.publish(action_msgs.goal)
		
		#wait to reach the goal position
		while (not rospy.is_shutdown):
			if (self.is_goal_reached(action_msgs.goal) or (rospy.get_rostime() > (movement_start + timeout))):
				break
					
		result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
		result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
		
		self.as_move_joint_direct.set_succeeded(result)
		
	def is_joint_configuration_not_in_limits(self, goal_configuration):
		for goal_joint in goal_configuration.positions:
			for joint_limit in self.joint_limits:
				if ((goal_joint.joint_uri == joint_limit.joint_name) and ((goal_joint.value < joint_limit.min_position) or (goal_joint.value > joint_limit.max_position))):
					rospy.logerr("goal configuration has <<%s>> in joint limit: %lf", goal_joint.joint_uri, goal_joint.value)
					return False
		
		return True				
		
	def is_goal_reached(self, goal_pose):
		for i in range(len(self.joint_names)):
			#rospy.loginfo("joint: %d -> curr_val: %f --- goal_val: %f", i, goal_pose.positions[i].value, self.current_joint_configuration[i])
			#rospy.loginfo(self.current_joint_configuration[i])
			if (abs(goal_pose.positions[i].value - self.current_joint_configuration[i]) > 0.002):   #ToDo: threshold via parameter
				return False		
					
		rospy.loginfo("gripper goal pose reached")
		return True
	

	def is_gripper_closed(self, req):
		for gripper_joint_value in self.current_joint_configuration:
			if(gripper_joint_value > 0.003):
				return False
		return True
		            

if __name__ == "__main__":
	rospy.init_node("gripper_action_server")	
	action = GripperActionServer()		
	rospy.loginfo("gripper action server started")
	rospy.spin()
