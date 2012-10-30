#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_command_tools
# \note
#   ROS package name: cob_script_server
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Implements script server functionalities.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import time
import os
import sys
import types
import thread
import commands
import math

# ROS imports
import roslib
roslib.load_manifest('script_server')
import rospy
import actionlib

from action_handle import *

# msg imports
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
#from pr2_controllers_msgs.msg import *	#replaced by control_msgs.msg (FollowJointTrajectoryAction)
from move_base_msgs.msg import *
from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from tf.transformations import *
from std_msgs.msg import String,ColorRGBA
from kinematics_msgs.srv import *
from control_msgs.msg import *

# care-o-bot includes
from cob_sound.msg import *

## Simple script server class.
#
# Implements the python interface for the script server.
class simple_script_server:
	## Initializes simple_script_server class.
	#
	# \param parse Defines wether to run script in simulation for graph generation or not
	def __init__(self):
		self.ns_global_prefix = "/script_server" ### TODO: make this configurable
		self.wav_path = "" ### TODO: make this configurable
		
		# init publishers
		self.pub_light = rospy.Publisher('/light_controller/command', ColorRGBA)
		
		rospy.sleep(1) # we have to wait here until publishers are ready, don't ask why

#------------------- Move section -------------------#
	## Deals with movements of the base.
	#
	### TODO_ sort out different modes of base movements
	#
	# A target will be sent to the actionlib interface of the move_base node.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_base(self,component_name,parameter_name,blocking=True, mode=""):
		ah = action_handle("move", component_name, parameter_name, blocking)
		
		if(mode == None or mode == ""):
			rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		else:
			rospy.loginfo("Move <<%s>> to <<%s>> using <<%s>> mode",component_name,parameter_name,mode)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah
		else:
			#print i,"type1 = ", type(i)
			DOF = 3
			if not len(param) == DOF: # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,DOF,len(param))
				print "parameter is:",param
				ah.set_failed(3)
				return ah
			else:
				for i in param:
					#print i,"type2 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						#print type(i)
						rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
						print "parameter is:",param
						ah.set_failed(3)
						return ah
					else:
						rospy.logdebug("accepted parameter %f for %s",i,component_name)

		# convert to pose message
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "/map"
		pose.pose.position.x = param[0]
		pose.pose.position.y = param[1]
		pose.pose.position.z = 0.0
		q = quaternion_from_euler(0, 0, param[2])
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		
		# call action server
		if(mode == None or mode == ""):
			action_server_name = "/move_base"
		elif(mode == "omni"):
			action_server_name = "/move_base"
		elif(mode == "diff"):
			action_server_name = "/move_base_diff"
		elif(mode == "linear"):
			action_server_name = "/move_base_linear"
		else:
			rospy.logerr("no valid navigation mode given for %s, aborting...",component_name)
			print "navigation mode is:",mode
			ah.set_failed(33)
			return ah
		
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		client_goal = MoveBaseGoal()
		client_goal.target_pose = pose
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()

		return ah

	## Deals with all kind of trajectory movements for different components.
	#
	### TODO: make component beeing namespace of follow_joint_trajectory_action
	#
	# A trajectory will be sent to the actionlib interface of the corresponding component.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_joint_trajectory(self,component_name,parameter_name,blocking):
		ah = action_handle("move", component_name, parameter_name, blocking)
		
		rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		
		# get joint_names from parameter server
		param_string = self.ns_global_prefix + "/" + component_name + "/joint_names"
		if not rospy.has_param(param_string):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
				ah.set_failed(2)
				return ah
		joint_names = rospy.get_param(param_string)
		
		# check joint_names parameter
		if not type(joint_names) is list: # check list
				rospy.logerr("no valid joint_names for %s: not a list, aborting...",component_name)
				print "joint_names are:",joint_names
				ah.set_failed(3)
				return ah
		else:
			for i in joint_names:
				#print i,"type1 = ", type(i)
				if not type(i) is str: # check string
					rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",component_name)
					print "joint_names are:",param
					ah.set_failed(3)
					return ah
				else:
					rospy.logdebug("accepted joint_names for component %s",component_name)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name

		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

		traj = []

		for point in param:
			#print point,"type1 = ", type(point)
			if type(point) is str:
				if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + point):
					rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + point)
					ah.set_failed(2)
					return ah
				point = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + point)
				point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
				#print point
			elif type(point) is list:
				rospy.logdebug("point is a list")
			else:
				rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

			# here: point should be list of floats/ints
			#print point
			if not len(point) == len(joint_names): # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(point))
				print "parameter is:",param
				ah.set_failed(3)
				return ah

			for value in point:
				#print value,"type2 = ", type(value)
				if not ((type(value) is float) or (type(value) is int)): # check type
					#print type(value)
					rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
					print "parameter is:",param
					ah.set_failed(3)
					return ah
			
				rospy.logdebug("accepted value %f for %s",value,component_name)
			traj.append(point)

		rospy.logdebug("accepted trajectory for %s",component_name)
		
		# convert to ROS trajectory message
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now()+rospy.Duration(0.5)
		traj_msg.joint_names = joint_names
		point_nr = 0
		for point in traj:
			point_nr = point_nr + 1
			point_msg = JointTrajectoryPoint()
			point_msg.positions = point
			point_msg.velocities = [0]*len(joint_names)
			point_msg.time_from_start=rospy.Duration(3*point_nr) # this value is set to 3 sec per point. \todo TODO: read from parameter
			traj_msg.points.append(point_msg)

		# call action server
		action_server_name = "/" + component_name + '_controller/follow_joint_trajectory'
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, FollowJointTrajectoryAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)
		
		# set operation mode to position
		#if not component_name == "arm":
		#	self.set_operation_mode(component_name,"position")
		#self.set_operation_mode(component_name,"position")		

		# sending goal
		client_goal = FollowJointTrajectoryGoal()
		client_goal.trajectory = traj_msg
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()
		return ah
		
	def move_planned(self, component_name, parameter_name, blocking=True): # for backward compatibility
		return self.move_joint_goal_planned(component_name, parameter_name, blocking)
	
	def move_joint_goal_planned(self, component_name, parameter_name, blocking=True):
		ah = action_handle("move_joint_goal_planned", component_name, parameter_name, blocking)
		
		if component_name != "arm":
			rospy.logerr("Only arm component is supported in move_joint_goal_planned.")
			ah.set_failed(4)
			return ah
			
		rospy.loginfo("Move planned <<%s>> to <<%s>>",component_name,parameter_name)
		
		# get joint_names from parameter server
		param_string = self.ns_global_prefix + "/" + component_name + "/joint_names"
		if not rospy.has_param(param_string):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
				ah.set_failed(2)
				return ah
		joint_names = rospy.get_param(param_string)
		
		# check joint_names parameter
		if not type(joint_names) is list: # check list
				rospy.logerr("no valid joint_names for %s: not a list, aborting...",component_name)
				print "joint_names are:",joint_names
				ah.set_failed(3)
				return ah
		else:
			for i in joint_names:
				#print i,"type1 = ", type(i)
				if not type(i) is str: # check string
					rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",component_name)
					print "joint_names are:",param
					ah.set_failed(3)
					return ah
				else:
					rospy.logdebug("accepted joint_names for component %s",component_name)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name

		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

		traj = []

		for point in param:
			#print point,"type1 = ", type(point)
			if type(point) is str:
				if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + point):
					rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + point)
					ah.set_failed(2)
					return ah
				point = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + point)
				point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
				#print point
			elif type(point) is list:
				rospy.logdebug("point is a list")
			else:
				rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

			# here: point should be list of floats/ints
			#print point
			if not len(point) == len(joint_names): # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(point))
				print "parameter is:",param
				ah.set_failed(3)
				return ah

			for value in point:
				#print value,"type2 = ", type(value)
				if not ((type(value) is float) or (type(value) is int)): # check type
					#print type(value)
					rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
					print "parameter is:",param
					ah.set_failed(3)
					return ah
			
				rospy.logdebug("accepted value %f for %s",value,component_name)
			traj.append(point)

		rospy.logdebug("accepted trajectory for %s",component_name)

		goal_constraints = Constraints() #arm_navigation_msgs/Constraints
		goal_constraints.joint_constraints=[]
		
		for i in range(len(joint_names)):
			new_constraint = JointConstraint()
			new_constraint.joint_name = joint_names[i]
			new_constraint.position = 0.0
			new_constraint.tolerance_below = 0.4
			new_constraint.tolerance_above = 0.4
			goal_constraints.joint_constraints.append(new_constraint)
		#no need for trajectories anymore, since planning (will) guarantee collision-free motion!
		traj_endpoint = traj[len(traj)-1]
		for k in range(len(traj_endpoint)):
			#print "traj_endpoint[%d]: %f", k, traj_endpoint[k]
			goal_constraints.joint_constraints[k].position = traj_endpoint[k]

		return self.move_constrained_planned(component_name, goal_constraints, blocking, ah)


	### TODO: handle different version of "move_arm", e.g. "move_arm_left"
	def move_constrained_planned(self, component_name, parameter_name, blocking=True, ah=None):
		ah = action_handle("move_constrained_planned", component_name, "constraint_goal", blocking)
			
		if component_name != "arm":
			rospy.logerr("Only arm component is supported in move_constrained_planned.")
			ah.set_failed(4)
			return ah
		
		# convert to ROS Move arm message
		motion_plan = MotionPlanRequest()
		motion_plan.group_name = component_name
		motion_plan.num_planning_attempts = 1
		motion_plan.allowed_planning_time = rospy.Duration(50.0)

		motion_plan.planner_id= ""
		motion_plan.goal_constraints = parameter_name

		# call action server
		action_server_name = "/move_"+component_name
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, MoveArmAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)
		
		# set operation mode to position
		#self.set_operation_mode(component_name,"position")
		
		# sending goal
		client_goal = MoveArmGoal()
		client_goal.planner_service_name = "ompl_planning/plan_kinematic_path"		#choose planner
		#client_goal.planner_service_name = "cob_prmce_planner/plan_kinematic_path"
		client_goal.motion_plan_request = motion_plan
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()
		return ah


	## Relative movement of the base
	#
	# \param component_name Name of component; here always "base".
	# \param parameter_name List of length 3: (item 1 & 2) relative x and y translation [m]; (item 3) relative rotation about z axis [rad].
	# \param blocking Bool value to specify blocking behaviour.
	# 
	# # throws error code 3 in case of invalid parameter_name vector 
	def move_base_rel(self, component_name, parameter_name=[0,0,0], blocking=True):	
		ah = action_handle("move_base_rel", component_name, parameter_name, blocking)

		rospy.loginfo("Move base relatively by <<%s>>", parameter_name)

		# step 0: check validity of parameters:
		if not len(parameter_name) == 3 or not isinstance(parameter_name[0], (int, float)) or not isinstance(parameter_name[1], (int, float)) or not isinstance(parameter_name[2], (int, float)):
			rospy.logerr("Non-numeric parameter_name list, aborting move_base_rel")
			print("parameter_name must be numeric list of length 3; (relative x and y transl [m], relative rotation [rad])")
			ah.set_failed(3)
			return ah
		if math.sqrt(parameter_name[0]**2 + parameter_name[1]**2) >= 0.15:
			rospy.logerr("Maximal relative translation step exceeded, aborting move_base_rel")
			print("Maximal relative translation step is 0.1 m")
			ah.set_failed(3)
			return(ah)
		if abs(parameter_name[2]) >= math.pi/2:
			rospy.logerr("Maximal relative rotation step exceeded, aborting move_base_rel")
			print("Maximal relative rotation step is pi/2")
			ah.set_failed(3)
			return(ah)

		# step 1: determine duration of motion so that upper thresholds for both translational as well as rotational velocity are not exceeded
		max_trans_vel = 0.05 # [m/s]
		max_rot_vel = 0.2 # [rad/s]
		duration_trans_sec = math.sqrt(parameter_name[0]**2 + parameter_name[1]**2) / max_trans_vel
		duration_rot_sec = abs(parameter_name[2] / max_rot_vel)
		duration_sec = max(duration_trans_sec, duration_rot_sec)
		duration_ros = rospy.Duration.from_sec(duration_sec) # duration of motion in ROS time

		# step 2: determine actual velocities based on calculated duration
		x_vel = parameter_name[0] / duration_sec
		y_vel = parameter_name[1] / duration_sec
		rot_vel = parameter_name[2] / duration_sec

		# step 3: send constant velocity command to base_controller for the calculated duration of motion
		pub = rospy.Publisher('/base_controller/command_safe', Twist)  # todo: use Matthias G.'s safe_command
		twist = Twist()
		twist.linear.x = x_vel
		twist.linear.y = y_vel
		twist.angular.z = rot_vel
		r = rospy.Rate(10) # send velocity commands at 10 Hz
		end_time = rospy.Time.now() + duration_ros
		while not rospy.is_shutdown() and rospy.Time.now() < end_time:
			pub.publish(twist)
			r.sleep()

		ah.set_succeeded()
		return ah

	

		
#------------------- LED section -------------------#
	## Set the color of the cob_light component.
	#
	# The color is given by a parameter on the parameter server.
	#
	# \param parameter_name Name of the parameter on the parameter server which holds the rgb values.
	def set_light(self,parameter_name,blocking=False):
		ah = action_handle("set", "light", parameter_name, blocking)
		ah.set_active(mode="topic")

		rospy.loginfo("Set light to <<%s>>",parameter_name)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/light/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/light/" + parameter_name)
				return 2
			param = rospy.get_param(self.ns_global_prefix + "/light/" + parameter_name)
		else:
			param = parameter_name
			
		# check color parameters
		if not type(param) is list: # check outer list
			rospy.logerr("no valid parameter for light: not a list, aborting...")
			print "parameter is:",param
			ah.error_code = 3
			return ah
		else:
			if not len(param) == 3: # check dimension
				rospy.logerr("no valid parameter for light: dimension should be 3 (r,g,b) and is %d, aborting...",len(param))
				print "parameter is:",param
				ah.error_code = 3
				return ah
			else:
				for i in param:
					#print i,"type1 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						#print type(i)
						rospy.logerr("no valid parameter for light: not a list of float or int, aborting...")
						print "parameter is:",param
						ah.error_code = 3
						return ah
					else:
						rospy.logdebug("accepted parameter %f for light",i)
		
		# convert to ColorRGBA message
		color = ColorRGBA()
		color.r = param[0]
		color.g = param[1]
		color.b = param[2]
		color.a = 1 # Transparency

		# publish color		
		self.pub_light.publish(color)
		
		ah.set_succeeded()
		ah.error_code = 0
		return ah

#-------------------- Sound section --------------------#
	## Say some text.
	#
	### TODO: check namespace (sound, sound_controller)
	#
	# The text to say may be given by a list of strings or a single string which points to a parameter on the ROS parameter server.
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use for the TTS system
	def say(self,parameter_name,blocking=True):
		component_name = "sound"
		ah = action_handle("say", component_name, parameter_name, blocking)
			
		text = ""
		
		# get values from parameter server
		language = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/language","en")
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check parameters
		if not type(param) is list: # check list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah
		else:
			for i in param:
				#print i,"type1 = ", type(i)
				if not type(i) is str:
					rospy.logerr("no valid parameter for %s: not a list of strings, aborting...",component_name)
					print "parameter is:",param
					ah.set_failed(3)
					return ah
				else:
					text = text + i + " "
					rospy.logdebug("accepted parameter <<%s>> for <<%s>>",i,component_name)

		rospy.loginfo("Saying <<%s>>",text)
		
		# call action server
		action_server_name = "/sound_controller/say"
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, SayAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		client_goal = SayGoal()
		client_goal.text.data = text
		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()
		return ah

	## Play a sound file.
	#
	### TODO: this is not generic enough (hard package name, namespace, ...)
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use
	def play(self,parameter_name,blocking=True):
		component_name = "sound"
		ah = action_handle("play", component_name, parameter_name, False)
		
		language = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/language","en")
		if self.wav_path == "":
			wav_path = commands.getoutput("rospack find cob_script_server")
		else:
			wav_path = self.wav_path
		filename = wav_path + "/common/files/" + language + "/" + parameter_name + ".wav"
		
		rospy.loginfo("Playing <<%s>>",filename)
		#self.soundhandle.playWave(filename)
		
		#\todo TODO: check if file exists
		# if filename exists:
		#	do ...
		# else 
		#	ah.set_fail(3)
		#	return ah
		
		if blocking:
			ret = os.system("aplay -q " + filename)
			if ret != 0:
				ah.set_failed(99)
				return ah
		else:
			os.system("aplay -q " + filename + "&") # TODO how to check if execution failed (e.g. file could be found)?
		ah.set_succeeded()
		return ah
		
	def set_wav_path(self,parameter_name,blocking=True):
		if type(parameter_name) is str:
			self.wav_path = parameter_name
		else:
			rospy.logerr("invalid wav_path parameter specified, aborting...")
			print "parameter is:", parameter_name
			ah.set_failed(2)
			return ah		

#------------------- General section -------------------#
	## Sleep for a certain time.
	#
	# \param duration Duration in seconds to sleep.
	#
	def sleep(self,duration):
		ah = action_handle("sleep", "", str(duration), True, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		rospy.loginfo("Wait for %f sec",duration)
		rospy.sleep(duration)
		
		ah.set_succeeded()

	## Waits for user input.
	#
	# Waits either for a user input or until timeout is reached.
	#
	# \param duration Duration in seconds for timeout.
	# 
	# \todo TODO: implement waiting for timeout
	def wait_for_input(self,duration=0):
		ah = action_handle("wait", "input", str(duration), True)

		if (duration != 0):
			rospy.logerr("Wait with duration not implemented yet") # \todo TODO: implement waiting with duration
		
		rospy.loginfo("Wait for user input...")
		retVal = raw_input()
		rospy.loginfo("...got string <<%s>>",retVal)
		ah.set_succeeded()
		return retVal
