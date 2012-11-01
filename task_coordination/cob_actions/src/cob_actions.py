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
#   ROS stack name: task_coordination
# \note
#   ROS package name: cob_actions
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Implements action functionalities.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#	 - Redistributions of source code must retain the above copyright
#	   notice, this list of conditions and the following disclaimer. \n
#	 - Redistributions in binary form must reproduce the above copyright
#	   notice, this list of conditions and the following disclaimer in the
#	   documentation and/or other materials provided with the distribution. \n
#	 - Neither the name of the Fraunhofer Institute for Manufacturing
#	   Engineering and Automation (IPA) nor the names of its
#	   contributors may be used to endorse or promote products derived from
#	   this software without specific prior written permission. \n
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

from action_cmdr.abstract_action import AbstractAction
from action_cmdr.action_handle import ActionHandle
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from cob_arm_navigation_python.MotionPlan import MotionPlan, CallFunction
from cob_arm_navigation_python.MoveArm import MoveArm
from pr2_python.world_interface import WorldInterface
from pr2_python import conversions

from copy import deepcopy

#only temporary
from simple_script_server import *  # import script
sss = simple_script_server()


import roslib
roslib.load_manifest('cob_actions')
import action_cmdr
action_cmdr.init("generic_actions")
import rospy
import actionlib
import tf

class PrintAction(AbstractAction):
	action_name = 'printit'
	
	def execute(self, what_to_print):
		print("YES! %s" % what_to_print)

class TestAction(AbstractAction):
	action_name = "test"
	
	def __init__(self, actions):
		self.actions = actions
	
	def execute(self, what_to_print):
		self.actions.printit(what_to_print)



#------------------- Move section -------------------#
## Deals with all kind of movements for different components.
#
# Based on the component, the corresponding move functions will be called.
#
# \param component_name Name of the component.
# \param parameter_name Name of the parameter on the ROS parameter server.
# \param blocking Bool value to specify blocking behaviour.

class CObMoveGripperJointAction(AbstractAction):
	action_name = "move_gripper"
	action_server_name_prefix = "/script_server"

	def __init__(self, actions):
		self.actions = actions

	def execute(self, target="", blocking=True):
		return self.actions.move_joint_trajectory("sdh", "/sdh_controller/follow_joint_trajectory", target, blocking)


class CObMoveTrayJointAction(AbstractAction):
	action_name = "move_tray"
	action_server_name_prefix = "/script_server"

	def __init__(self, actions):
		self.actions = actions

	def execute(self, target="", blocking=True):
		return self.actions.move_joint_trajectory("tray", "/tray_controller/follow_joint_trajectory", target, blocking)


class CObMoveTorsoJointAction(AbstractAction):
	action_name = "move_torso"
	action_server_name_prefix = "/script_server"

	def __init__(self, actions):
		self.actions = actions

	def execute(self, target="", blocking=True):
		return self.actions.move_joint_trajectory("torso", "/torso_controller/follow_joint_trajectory", target, blocking)


class CObMoveHeadJointAction(AbstractAction):
	action_name = "move_head"
	action_server_name_prefix = "/script_server"

	def __init__(self, actions):
		self.actions = actions

	def execute(self, target="", blocking=True):
		return self.actions.move_joint_trajectory("head", "/head_controller/follow_joint_trajectory", target, blocking)

class CObPreparePerception(AbstractAction):
    action_name = "prepare_perception"
 
    def __init__(self, actions):
        self.actions = actions
        
    def execute(self, blocking=True):
        ah = action_cmdr.move_tray(target="up", blocking=True)
        if ah.get_error_code() != 0:
            return ah
        ah = action_cmdr.move_arm("look_at_table") 
        if ah.get_error_code() != 0:
            return ah
        ah = action_cmdr.move_head(target="back", blocking=True)
        if ah.get_error_code() != 0:
            return ah
        return action_cmdr.move_torso(target="home", blocking=True)


class CObMoveArmAction(AbstractAction):
	action_name = "move_arm"
	DOF = 7

	def __init__(self, actions):
		self.actions = actions

	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.	
	# \param blocking Bool value to specify blocking behaviour.
	
	def execute(self, target="", blocking=True):
		if type(target) is PoseStamped:
			rospy.logerr("Not implemented yet!")
		else:
			return self.actions.move_joint_trajectory("arm", "/arm_controller/follow_joint_trajectory", target, blocking)


class CObMoveArmPlannedAction(AbstractAction):
	action_name = "move_arm_planned"
	DOF = 7

	def __init__(self, actions):
		self.actions = actions

	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.	
	# \param blocking Bool value to specify blocking behaviour.
	
	def execute(self, target="", blocking=True, planned=False):
		if type(target) is PoseStamped:
			return self.actions.move_arm_cartesian_planned("arm", target, blocking)
		else:
			return self.actions.move_arm_joint_planned("arm", target, blocking)


class CObMoveBaseAction(AbstractAction):
	action_name = 'move_base'
	ns_global_prefix = "/script_server"

	def __init__(self, actions):
		self.actions = actions

	def execute(self, parameter_name, blocking=True, mode=''):
		component_name = 'base'

		ah = ActionHandle("move", component_name, parameter_name, blocking)
		
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
		q = tf.transformations.quaternion_from_euler(0, 0, param[2])
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



class CObMoveArmJointGoalPlannedAction(AbstractAction):
	# this class appears to be just a wrapper
	action_name = 'move_arm_joint_planned'
	ns_global_prefix = "/script_server"
	DOF = 7
	
	def __init__(self, actions):
		self.actions = actions

	def execute(self, component_name, target="", blocking=True):
		ah = ActionHandle("move_arm_joint_planned", component_name, target, blocking)

		rospy.loginfo("Move planned <<%s>> to <<%s>>",component_name,target)

		# get joint values from parameter server
		if type(target) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + target):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + target)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + target)
		else:
			param = target

		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

		last_point = param[-1]
		
		if type(last_point) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + last_point):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + last_point)
				ah.set_failed(2)
				return ah
			last_point = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + last_point)
			last_point = last_point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
			#print last_point
		elif type(last_point) is list:
			rospy.logdebug("last_point is a list")
		else:
			rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",component_name)
			print "parameter is:",param
			ah.set_failed(3)
			return ah

		# here: last_point should be list of floats/ints
		#print last_point
		if not len(last_point) == self.DOF: # check dimension
			rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,self.DOF,len(last_point))
			print "parameter is:",param
			ah.set_failed(3)
			return ah

		for value in last_point:
			#print value,"type2 = ", type(value)
			if not ((type(value) is float) or (type(value) is int)): # check type
				#print type(value)
				rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah
		
			rospy.logdebug("accepted value %f for %s",value,component_name)


		mp = MotionPlan()
		mp += MoveArm("arm", [last_point])
		
		planning_res = mp.plan(2)
		print planning_res

		if planning_res.success:
			for e in mp.execute():
				exec_res = e.wait()
				print exec_res
				if not exec_res.success:
					rospy.logerr("Execution of MotionExecutable %s failed", e.name)
					ah.set_failed(3)
					break
		else:
			rospy.logerr("Planning failed")
			ah.set_failed(3)

		return ah



class CObMoveArmCartesianPlannedAction(AbstractAction):
	# this class appears to be just a wrapper
	action_name = 'move_arm_cartesian_planned'
	
	def __init__(self, actions):
		self.actions = actions

	def execute(self, component_name, target=PoseStamped(), blocking=True):
		ah = ActionHandle("move_arm_cartesian_planned", component_name, target, blocking)

		rospy.loginfo("Move <<%s>> CARTESIAN PLANNED", component_name)

		mp = MotionPlan()
		mp += MoveArm("arm",[target,['sdh_grasp_link']])
		
		planning_res = mp.plan(2)
		print planning_res

		if planning_res.success:
			for e in mp.execute():
				exec_res = e.wait()
				print exec_res
				if not exec_res.success:
					rospy.logerr("Execution of MotionExecutable %s failed", e.name)
					ah.set_failed(4)
					break
		else:
			rospy.logerr("Planning failed")
			ah.set_failed(4)

		return ah


## Relative movement of the base
#
# \param component_name Name of component; here always "base".
# \param parameter_name List of length 3: (item 1 & 2) relative x and y translation [m]; (item 3) relative rotation about z axis [rad].
# \param blocking Bool value to specify blocking behaviour.
# 
# # throws error code 3 in case of invalid parameter_name vector 
class CObMoveBaseRelAction(AbstractAction):
	action_name = "move_base_rel"
	
	def execute(self, component_name, parameter_name=[0,0,0], blocking=True):	
		ah = ActionHandle("move_base_rel", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active(mode="topic")

		rospy.loginfo("Move base relatively by <<%s>>", parameter_name)

		# step 0: check validity of parameters:
		if not len(parameter_name) == 3 or not isinstance(parameter_name[0], (int, float)) or not isinstance(parameter_name[1], (int, float)) or not isinstance(parameter_name[2], (int, float)):
			rospy.logerr("Non-numeric parameter_name list, aborting move_base_rel")
			print("parameter_name must be numeric list of length 3; (relative x and y transl [m], relative rotation [rad])")
			ah.set_failed(4)
			return ah
		if math.sqrt(parameter_name[0]**2 + parameter_name[1]**2) >= 0.15:
			rospy.logerr("Maximal relative translation step exceeded, aborting move_base_rel")
			print("Maximal relative translation step is 0.1 m")
			ah.set_failed(4)
			return(ah)
		if abs(parameter_name[2]) >= math.pi/2:
			rospy.logerr("Maximal relative rotation step exceeded, aborting move_base_rel")
			print("Maximal relative rotation step is pi/2")
			ah.set_failed(4)
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


## Set the operation mode for different components.
#
# Based on the component, the corresponding set_operation_mode service will be called.
#
# \param component_name Name of the component.
# \param mode Name of the operation mode to set.
# \param blocking Service calls are always blocking. The parameter is only provided for compatibility with other functions.
class CObSetOperationModeAction(AbstractAction):
	action_name = 'set_operation_mode'
	
	def __init__(self, actions):
		self.actions = actions
		
	def execute(self, component_name, mode, blocking=True, planning=False):
		#rospy.loginfo("setting <<%s>> to operation mode <<%s>>",component_name, mode)
		rospy.set_param("/" + component_name + "_controller/OperationMode",mode) # \todo TODO: remove and only use service call
		#rospy.wait_for_service("/" + component_name + "_controller/set_operation_mode")
		try:
			set_operation_mode = rospy.ServiceProxy("/" + component_name + "_controller/set_operation_mode", SetOperationMode)
			req = SetOperationModeRequest()
			req.operation_mode.data = mode
			#print req
			resp = set_operation_mode(req)
			resp = set_operation_mode(req)
			#print resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
#------------------- LED section -------------------#
## Set the color of the cob_light component.
#
# The color is given by a parameter on the parameter server.
#
# \param parameter_name Name of the parameter on the parameter server which holds the rgb values.
class CObSetLightsAction(AbstractAction):
	action_name = 'set_lights'
	
	def __init__(self, actions):
		self.actions = actions
	
	def execute(self,parameter_name,blocking=False):
		ah = ActionHandle("set", "light", parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
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





class CObPickUpAction(AbstractAction):
	action_name = "pick_up"

	def __init__(self, actions):
		self.actions = actions

	def execute(self, target=PoseStamped(), blocking=True):
		ah = ActionHandle("pick_up", "dummy", "", blocking)
		rospy.loginfo("Picking up object...")
		
		#ah = self.actions.grasp_object(target, blocking)
		wi = WorldInterface()
		wi.reset_attached_objects()
		wi.reset_collision_objects()
		
		# add table
		table_extent = (2.0, 2.0, 1.0)
		table_pose = conversions.create_pose_stamped([ -0.5 - table_extent[0]/2.0, 0 ,table_extent[2]/2.0 ,0,0,0,1], 'base_link')
		wi.add_collision_box(table_pose, table_extent, "table")
		
		mp = MotionPlan()
		mp += CallFunction(sss.move, 'torso','front')
		#mp += MoveArm('arm',['pregrasp'])
		mp += CallFunction(sss.move, 'sdh','cylopen', False)
		
		# OpenIssues:
		# - where to place the sdh_grasp_link to grasp the object located at target?
		# - there is no orientation in the target? -> hardcoded taken from pregrasp
		grasp_pose = PoseStamped()
		grasp_pose = deepcopy(target)
		grasp_pose.header.stamp = rospy.Time.now()
		grasp_pose.pose.orientation.x = 0.220
		grasp_pose.pose.orientation.y = 0.670
		grasp_pose.pose.orientation.z = -0.663
		grasp_pose.pose.orientation.w = -0.253
		mp += MoveArm('arm',[grasp_pose,['sdh_grasp_link']], seed = 'pregrasp')
		
		mp += CallFunction(sss.move, 'sdh','cylclosed', True)
		
		
		#ah = self.actions.lift_object(target, blocking)
		lift_pose = PoseStamped()
		lift_pose = deepcopy(target)
		lift_pose.header.stamp = rospy.Time.now()
		lift_pose.pose.position.z += 0.08
		lift_pose.pose.orientation.x = 0.220
		lift_pose.pose.orientation.y = 0.670
		lift_pose.pose.orientation.z = -0.663
		lift_pose.pose.orientation.w = -0.253
		mp += MoveArm('arm',[lift_pose,['sdh_grasp_link']])
		
		
		
		#ah = self.actions.retrieve_object(target, blocking)
		#mp += MoveArm('arm',['pregrasp'])
		mp += MoveArm('arm',['wavein'])
		
		
		
		
		planning_res = mp.plan(2)
		print planning_res

		if planning_res.success:
			for e in mp.execute():
				exec_res = e.wait()
				print exec_res
				if not exec_res.success:
					#rospy.logerr("Execution of MotionExecutable %s failed", e.name)
					ah.set_failed(4)
					break
			ah.set_succeeded()
		else:
			rospy.logerr("Planning failed")
			ah.set_failed(4)
		
		return ah


#class CObGraspObjectAction(AbstractAction):
	#action_name = "grasp_object"

	#def __init__(self, actions):
		#self.actions = actions

	#def execute(self, target=PoseStamped(), blocking=True):
		#ah = ActionHandle("grasp_object", "dummy", "", blocking)
		#rospy.loginfo("Grasping the object...")
		
		#wi = WorldInterface()
		#wi.reset_attached_objects()
		#wi.reset_collision_objects()
		
		## add table
		#table_extent = (2.0, 2.0, 0.8)
		#table_pose = conversions.create_pose_stamped([ -0.5 - table_extent[0]/2.0, 0 ,table_extent[2]/2.0 ,0,0,0,1], 'base_link')
		#wi.add_collision_box(table_pose, table_extent, "table")
		
		#mp = MotionPlan()
		#mp += CallFunction(sss.move, 'torso','front')
		#mp += MoveArm('arm',['pregrasp'])
		#mp += CallFunction(sss.move, 'sdh','cylopen', False)
		
		## OpenIssues:
		## - where to place the sdh_grasp_link to grasp the object located at target?
		## - there is no orientation in the target? -> hardcoded taken from pregrasp
		#grasp_pose = PoseStamped()
		#grasp_pose = target
		#grasp_pose.pose.orientation.x = 0.220
		#grasp_pose.pose.orientation.y = 0.670
		#grasp_pose.pose.orientation.z = -0.663
		#grasp_pose.pose.orientation.w = -0.253
		
		#mp += MoveArm('arm',[grasp_pose,['sdh_grasp_link']], seed = 'pregrasp')
		
		#mp += CallFunction(sss.move, 'sdh','cylclosed', False)
		
		
		#planning_res = mp.plan(2)
		#print planning_res

		#if planning_res.success:
			#for e in mp.execute():
				#exec_res = e.wait()
				#print exec_res
				#if not exec_res.success:
					#rospy.logerr("Execution of MotionExecutable %s failed", e.name)
					#ah.set_failed(4)
					#break
		#else:
			#rospy.logerr("Planning failed")
			#ah.set_failed(4)
		
		#return ah


#class CObLiftObjectAction(AbstractAction):
	#action_name = "lift_object"

	#def __init__(self, actions):
		#self.actions = actions

	#def execute(self, target=PoseStamped(), blocking=True):
		#ah = ActionHandle("lift_object", "dummy", "", blocking)
		#rospy.loginfo("Lifting the object...")
		
		#wi = WorldInterface()
		#wi.reset_attached_objects()
		#wi.reset_collision_objects()
		
		## add table
		#table_extent = (2.0, 2.0, 0.8)
		#table_pose = conversions.create_pose_stamped([ -0.5 - table_extent[0]/2.0, 0 ,table_extent[2]/2.0 ,0,0,0,1], 'base_link')
		#wi.add_collision_box(table_pose, table_extent, "table")
		
		#mp = MotionPlan()
		
		## OpenIssues:
		## - where to place the sdh_grasp_link to grasp the object located at target?
		## - there is no orientation in the target?
		#lift_pose = PoseStamped()
		#lift_pose = target
		#lift_pose.pose.position.z += 0.1
		#lift_pose.pose.orientation.x = 0.220
		#lift_pose.pose.orientation.y = 0.670
		#lift_pose.pose.orientation.z = -0.663
		#lift_pose.pose.orientation.w = -0.253
		#mp += MoveArm('arm',[lift_pose,['sdh_grasp_link']])
		
		
		#for ex in mp.execute():
			#if not ex.wait(80.0).success:
				##sss.set_light('red')
				#ah.set_failed(4)
				#break
			##sss.set_light('green')
			#ah.set_failed(3)
		
		#return ah


#class CObRetrieveObjectAction(AbstractAction):
	#action_name = "retrieve_object"

	#def __init__(self, actions):
		#self.actions = actions

	#def execute(self, target="", blocking=True):
		#ah = ActionHandle("retrieve_object", "dummy", "", blocking)
		#rospy.loginfo("Retrieving the object...")
		
		#wi = WorldInterface()
		#wi.reset_attached_objects()
		#wi.reset_collision_objects()
		
		## add table
		#table_extent = (2.0, 2.0, 0.8)
		#table_pose = conversions.create_pose_stamped([ -0.5 - table_extent[0]/2.0, 0 ,table_extent[2]/2.0 ,0,0,0,1], 'base_link')
		#wi.add_collision_box(table_pose, table_extent, "table")
		
		#mp = MotionPlan()
		#mp += MoveArm('arm',['hold'])
		
		
		#for ex in mp.execute():
			#if not ex.wait(80.0).success:
				##sss.set_light('red')
				#ah.set_failed(4)
				#break
			##sss.set_light('green')
			#ah.set_failed(3)
		
		#return ah


