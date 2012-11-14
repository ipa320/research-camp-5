#!/usr/bin/python
#################################################################
##\file
#
# \note
# Copyright (c) 2010 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: care-o-bot
# \note
# ROS stack name: cob_command_tools
# \note
# ROS package name: cob_script_server
#
# \author
# Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
# Implements script server functionalities.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
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
import random

# ROS imports
import roslib
roslib.load_manifest('raw_script_server')
import rospy
import actionlib
import tf

# msg imports
from raw_arm_navigation.msg import *
from raw_script_server.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *


# graph includes
import pygraphviz as pgv

graph = ""
graph_wait_list = []
function_counter = 0
ah_counter = 0
graph = pgv.AGraph()
graph.node_attr['shape'] = 'box'
last_node = "Start"

## Script class from which all script inherit.
#
# Implements basic functionalities for all scripts.
class script():
    def __init__(self):
        # use filename as nodename
        filename = os.path.basename(sys.argv[0])
        self.basename, extension = os.path.splitext(filename)
        rospy.init_node(self.basename)
        self.graph_pub = rospy.Publisher("/script_server/graph", String)

    ## Dummy function for initialization
    def Initialize(self):
        pass

    ## Dummy function for main run function
    def Run(self):
        pass

    ## Function to start the script
    #
    # First does a simulated turn and then calls Initialize() and Run().
    def Start(self):
        self.Parse()
        global ah_counter
        ah_counter = 0
        self.sss = simple_script_server()
        rospy.loginfo("Starting <<%s>> script...", self.basename)
        self.Initialize()
        self.Run()
        # wait until last threaded action finishes
        rospy.loginfo("Wait for script to finish...")
        while ah_counter != 0:
            rospy.sleep(1)
        rospy.loginfo("...script finished.")

    ## Function to generate graph view of script.
    #
    # Starts the script in simulation mode and calls Initialize() and Run().
    def Parse(self):
        rospy.loginfo("Start parsing...")
        global graph
        global function_counter
        function_counter = 0
        # run script in simulation mode
        self.sss = simple_script_server(parse=True)
        self.Initialize()
        self.Run()

        # save graph on parameter server for further processing
#        self.graph = graph
        rospy.set_param("/script_server/graph", graph.string())
        self.graph_pub.publish(graph.string())
        rospy.loginfo("...parsing finished")
        function_counter = 0
        return graph.string()

## Simple script server class.
#
# Implements the python interface for the script server.
class simple_script_server:
    ## Initializes simple_script_server class.
    #
    # \param parse Defines wether to run script in simulation for graph generation or not
    def __init__(self, parse=False):
        global graph
        self.ns_global_prefix = "/script_server"
        self.wav_path = ""
        self.parse = parse
        if (not rospy.has_param("/arm_1/arm_controller/joints")):
            rospy.logerr("No joints given.")
        else:
            self.arm1_joint_names = sorted(rospy.get_param("/arm_1/arm_controller/joints"))
            self.gripper1_joint_names = sorted(rospy.get_param("/arm_1/gripper_controller/joints"))

    #------------------- Move section -------------------#
    ## Deals with all kind of movements for different components.
    #
    # Based on the component, the corresponding move functions will be called.
    #
    # \param component_name Name of the component.
    # \param parameter_name Name of the parameter on the ROS parameter server.
    # \param blocking Bool value to specify blocking behaviour.
    def move(self, component_name, parameter_name, blocking=True, mode=""):
        if component_name == "base":
            return self.move_base(component_name, parameter_name, blocking)
        
        elif component_name == "arm":
            if mode == "planned":
                return self.move_arm_planned(component_name, parameter_name, blocking)
            else:
                return self.move_arm_direct(component_name, parameter_name, blocking)
            
        elif component_name == "arm_traj":
            return self.move_arm_traj(component_name, parameter_name, blocking)
        
        elif component_name == "gripper":
            return self.move_gripper_joint(component_name, parameter_name, blocking)


    ## Deals with movements of the base.
    #
    # A target will be sent to the actionlib interface of the move_base node.
    #
    # \param component_name Name of the component.
    # \param parameter_name Name of the parameter on the ROS parameter server.
    # \param blocking Bool value to specify blocking behaviour.

    def move_base(self, component_name, parameter_name, blocking):
        ah = action_handle("move_base", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        rospy.loginfo("Move <<%s>> to <<%s>>", component_name, parameter_name)

        # get pose from parameter server
        if type(parameter_name) is str:
            if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...", self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
                ah.set_failed(2)
                return ah
            param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
        else:
            param = parameter_name
        # check pose
        if not type(param) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 3
            if not len(param) == DOF: # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                    #print i,"type2 = ", type(i)
                    if not ((type(i) is float) or (type(i) is int)):
                        # check type
                        #print type(i)
                        rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...", component_name)
                        print "parameter is:", param
                        ah.set_failed(3)
                        return ah
                    else:
                        rospy.logdebug("accepted parameter %f for %s", i, component_name)
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

        action_server_name = "/move_base"

        rospy.logdebug("calling %s action server", action_server_name)
        client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)

        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", action_server_name)
        if not client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", action_server_name)

        # sending goal
        client_goal = MoveBaseGoal()
        client_goal.target_pose = pose
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
    def move_base_rel(self, component_name, parameter_name=[0, 0, 0], blocking=True):
        '''
        ah = action_handle("move_base_rel", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        rospy.loginfo("Move base relatively by <<%s>>", parameter_name)

        # step 0: check validity of parameters:
        if not len(parameter_name) == 3 or not isinstance(parameter_name[0], (int, float)) or not isinstance(parameter_name[1], (int, float)) or not isinstance(parameter_name[2], (int, float)):
            rospy.logerr("Non-numeric parameter_name list, aborting move_base_rel")
            print("parameter_name must be numeric list of length 3; (relative x and y transl [m], relative rotation [rad])")
            ah.set_failed(3)
            return ah
        if math.sqrt(parameter_name[0]**2 + parameter_name[1]**2) > 0.1:
            rospy.logerr("Maximal relative translation step exceeded, aborting move_base_rel")
            print("Maximal relative translation step is 0.1 m")
            ah.set_failed(3)
            return(ah)
        if abs(parameter_name[2]) > math.pi/2:
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
        pub = rospy.Publisher('/base_controller/command', Twist)  # todo: use Matthias G.'s safe_command
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
        '''
        return


    #########################################################
    ##### ARM MOVEMENTS WITHOUT PLANNING
    #########################################################
    
    def move_arm_direct(self, component_name, parameter_name=[0, 0, 0, 0, 0, 0, "/base_link"], blocking=True):
        if type(parameter_name) is str:
            return self.move_arm_joint_direct(component_name, parameter_name, blocking)
        elif type(parameter_name) is list:
            if len(parameter_name) == 7:
                return self.move_arm_cart_direct(component_name, parameter_name, blocking)
            elif len(parameter_name) == 4:
                return self.move_arm_cart_sample_rpy_direct(component_name, parameter_name, blocking)
            else:
                rospy.loginfo("parameter <<%s>> is not in the right format", parameter_name)

   
    def move_arm_joint_direct(self, component_name, parameter_name="", blocking=True):
        ah = action_handle("move_arm_joint_direct", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        rospy.loginfo("Move <<%s>> DIRECT to <<%s>>", component_name, parameter_name)

        # get pose from parameter server
        if type(parameter_name) is str:
            if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...", self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
                ah.set_failed(2)
                return ah
            param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
        else:
            param = parameter_name

        # check pose
        if not type(param) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 5
            if not len(param) == DOF: # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                    #print i,"type2 = ", type(i)
                    if not ((type(i) is float) or (type(i) is int)): # check type
                        #print type(i)
                        rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...", component_name)
                        print "parameter is:", param
                        ah.set_failed(3)
                        return ah
                    else:
                        rospy.logdebug("accepted parameter %f for %s", i, component_name)


        pose_goal = raw_arm_navigation.msg.MoveToJointConfigurationGoal()

        for i in range(DOF):
            jv = brics_actuator.msg.JointValue()
            jv.joint_uri = self.arm1_joint_names[i]
            jv.value = param[i]
            jv.unit = "rad"
            pose_goal.goal.positions.append(jv)

        action_server_name = "/arm_1/arm_controller/MoveToJointConfigurationDirect"

        rospy.logdebug("calling %s action server", action_server_name)
        client = actionlib.SimpleActionClient(action_server_name, MoveToJointConfigurationAction)
        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", action_server_name)
        if not client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", action_server_name)


        #print client_goal
        client.send_goal(pose_goal)
        ah.set_client(client)

        ah.wait_inside()

        return ah
        

    def move_arm_cart_direct(self, component_name, parameter_name=[0, 0, 0, 0, 0, 0, "/base_link"], blocking=True):
        ah = action_handle("move_arm_cart_direct", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()
        rospy.loginfo("Move <<%s>> DIRECT to <<%s>>", component_name, parameter_name)
        # get pose from parameter server
        if type(parameter_name) is str:
            rospy.logerr("parameter must be a 6DOF array")
            ah.set_failed(2)
            return ah
        else:
            param = parameter_name

        # check pose
        if not type(param) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 7
            if not len(param) == DOF:
                # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                    #print i,"type2 = ", type(i)
                    if i < (DOF - 1):
                        if not ((type(i) is float) or (type(i) is int)):
                            # check type
                            #print type(i)
                            rospy.logerr("no valid parameter for %s: not a list of float or int (1-6), aborting...", component_name)
                            print "parameter is:", param
                            ah.set_failed(3)
                            return ah
                        else:
                            rospy.logdebug("accepted parameter %f for %s", i, component_name)
                    elif i == DOF:
                        if not (type(i) is string):
                            # check type
                            #print type(i)
                            rospy.logerr("no valid parameter for %s: last parameter is not a string, aborting...", component_name)
                            print "parameter is:", param
                            ah.set_failed(3)
                            return ah
                        else:
                            rospy.logdebug("accepted parameter %f for %s", i, component_name)

        # convert to pose message
        pose = raw_arm_navigation.msg.MoveToCartesianPoseGoal()
        pose.goal.header.stamp = rospy.Time.now()
        pose.goal.header.frame_id = param[6]
        pose.goal.pose.position.x = param[0]
        pose.goal.pose.position.y = param[1]
        pose.goal.pose.position.z = param[2]

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(param[3], param[4], param[5])
        pose.goal.pose.orientation.x = qx
        pose.goal.pose.orientation.y = qy
        pose.goal.pose.orientation.z = qz
        pose.goal.pose.orientation.w = qw

        action_server_name = "/arm_1/arm_controller/MoveToCartesianPoseDirect"

        rospy.logdebug("calling %s action server", action_server_name)

        client = actionlib.SimpleActionClient(action_server_name, MoveToCartesianPoseAction)

        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", action_server_name)
        if not client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", action_server_name)

        client.send_goal(pose)
        ah.set_client(client)
        ah.wait_inside()

        return ah


    def move_arm_cart_sample_rpy_direct(self, component_name, parameter_name=[0, 0, 0, "/base_link"], blocking=True):
        ah = action_handle("move_arm_cart_sample_rpy_direct", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        rospy.loginfo("Move <<%s>> DIRECT to <<%s>>", component_name, parameter_name)

        # get pose from parameter server
        if type(parameter_name) is str:
            rospy.logerr("parameter must be a 4DOF array")
            ah.set_failed(2)
            return ah
        else:
            param = parameter_name

        # check pose
        if not type(param) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 4
            if not len(param) == DOF: # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                    #print i,"type2 = ", type(i)
                    if i < (DOF - 1):
                        if not ((type(i) is float) or (type(i) is int)): # check type
                            #print type(i)
                            rospy.logerr("no valid parameter for %s: not a list of float or int (1-6), aborting...", component_name)
                            print "parameter is:", param
                            ah.set_failed(3)
                            return ah
                        else:
                            rospy.logdebug("accepted parameter %f for %s", i, component_name)
                    elif i == DOF:
                        if not (type(i) is string): # check type
                            #print type(i)
                            rospy.logerr("no valid parameter for %s: last parameter is not a string, aborting...", component_name)
                            print "parameter is:", param
                            ah.set_failed(3)
                            return ah
                        else:
                            rospy.logdebug("accepted parameter %f for %s", i, component_name)

        # convert to pose message
        pose = raw_arm_navigation.msg.MoveToCartesianPoseGoal()
        pose.goal.header.stamp = rospy.Time.now()
        pose.goal.header.frame_id = param[3]

        pose.goal.pose.position.x = param[0]
        pose.goal.pose.position.y = param[1]
        pose.goal.pose.position.z = param[2]

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(0,0,0)
        pose.goal.pose.orientation.x = qx
        pose.goal.pose.orientation.y = qy
        pose.goal.pose.orientation.z = qz
        pose.goal.pose.orientation.w = qw

        action_server_name = "/arm_1/arm_controller/MoveToCartesianRPYSampledDirect"

        rospy.logdebug("calling %s action server", action_server_name)

        client = actionlib.SimpleActionClient(action_server_name, MoveToCartesianPoseAction)

        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", action_server_name)
        if not client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", action_server_name)

        client.send_goal(pose)
        ah.set_client(client)
        ah.wait_inside()

        return ah
    
    
    #########################################################
    ##### ARM MOVEMENTS WITHOUT PLANNING
    #########################################################
    
    def move_arm_planned(self, component_name, parameter_name, blocking=True):
        if type(parameter_name) is str:
            return self.move_arm_joint_planned(component_name, parameter_name, blocking)
        elif type(parameter_name) is list:
            if len(parameter_name) == 7:
                return self.move_arm_cart_planned(component_name, parameter_name, blocking)
            elif len(parameter_name) == 4:
                return self.move_arm_cart_sample_rpy_planned(component_name, parameter_name, blocking)
            else:
                rospy.loginfo("parameter <<%s>> is not in the right format", parameter_name)
        
        return
    
    
    def move_arm_joint_planned(self, component_name, parameter_name="", blocking=True):
        ah = action_handle("move_arm_joint_direct", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        rospy.loginfo("Move <<%s>> PLANNED to <<%s>>", component_name, parameter_name)

        # get pose from parameter server
        if type(parameter_name) is str:
            if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...", self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
                ah.set_failed(2)
                return ah
            param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
        else:
            param = parameter_name

        # check pose
        if not type(param) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 5
            if not len(param) == DOF: # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                    #print i,"type2 = ", type(i)
                    if not ((type(i) is float) or (type(i) is int)): # check type
                        #print type(i)
                        rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...", component_name)
                        print "parameter is:", param
                        ah.set_failed(3)
                        return ah
                    else:
                        rospy.logdebug("accepted parameter %f for %s", i, component_name)


        goal = arm_navigation_msgs.msg.MoveArmGoal()
        goal.motion_plan_request.group_name = "arm"
        goal.motion_plan_request.num_planning_attempts = 1
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "/ompl_planning/plan_kinematic_path"
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(15.0)
        
        for i in range(DOF):
            joint_constraint = arm_navigation_msgs.msg.JointConstraint()
            joint_constraint.joint_name = self.arm1_joint_names[i]
            joint_constraint.position = param[i]
            joint_constraint.tolerance_above = 0.1
            joint_constraint.tolerance_below = 0.1
            
            goal.motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)

        
        action_server_name = "/move_arm"

        rospy.logdebug("calling %s action server", action_server_name)
        client = actionlib.SimpleActionClient(action_server_name, arm_navigation_msgs.msg.MoveArmAction)
        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", action_server_name)
        if not client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", action_server_name)
     

        #print client_goal
        client.send_goal(goal)
        ah.set_client(client)

        ah.wait_inside()

        return ah
    
    
    def move_arm_cart_planned(self, component_name, parameter_name=[0, 0, 0, 0, 0, 0, "/base_link"], blocking=True):
        ah = action_handle("move_arm_cart_planned", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()
        rospy.loginfo("Move <<%s>> PLANNED to <<%s>>", component_name, parameter_name)
        # get pose from parameter server
        if type(parameter_name) is str:
            rospy.logerr("parameter must be a 6DOF array")
            ah.set_failed(2)
            return ah
        else:
            param = parameter_name

        # check pose
        if not type(param) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 7
            if not len(param) == DOF:
                # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                    #print i,"type2 = ", type(i)
                    if i < (DOF - 1):
                        if not ((type(i) is float) or (type(i) is int)):
                            # check type
                            #print type(i)
                            rospy.logerr("no valid parameter for %s: not a list of float or int (1-6), aborting...", component_name)
                            print "parameter is:", param
                            ah.set_failed(3)
                            return ah
                        else:
                            rospy.logdebug("accepted parameter %f for %s", i, component_name)
                    elif i == DOF:
                        if not (type(i) is string):
                            # check type
                            #print type(i)
                            rospy.logerr("no valid parameter for %s: last parameter is not a string, aborting...", component_name)
                            print "parameter is:", param
                            ah.set_failed(3)
                            return ah
                        else:
                            rospy.logdebug("accepted parameter %f for %s", i, component_name)

        
        goal = arm_navigation_msgs.msg.MoveArmGoal()
        goal.motion_plan_request.group_name = "arm"
        goal.motion_plan_request.num_planning_attempts = 1
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "/ompl_planning/plan_kinematic_path"
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(15.0)
        
        pc = arm_navigation_msgs.msg.PositionConstraint()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = param[6]
        pc.link_name = 'arm_link_5'
        pc.position.x = param[0]
        pc.position.y = param[1]
        pc.position.z = param[2]
        pc.constraint_region_shape.type = arm_navigation_msgs.msg.Shape.BOX
        pc.constraint_region_shape.dimensions = [0.02, 0.02, 0.02]
        pc.constraint_region_orientation.w = 1.0
        #pc.weight = 1.0
        goal.motion_plan_request.goal_constraints.position_constraints.append(pc)
         
        oc = arm_navigation_msgs.msg.OrientationConstraint()
        oc.header.stamp = rospy.Time.now()
        oc.header.frame_id = param[6]
        oc.link_name = 'arm_link_5'
        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(param[3], param[4], param[5])
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.orientation.w = qw
        oc.absolute_roll_tolerance = 0.28
        oc.absolute_pitch_tolerance = 0.28
        oc.absolute_yaw_tolerance = 0.28
        oc.weight = 1.0
        goal.motion_plan_request.goal_constraints.orientation_constraints.append(oc)
       
        
        
        action_server_name = "/move_arm"

        rospy.logdebug("calling %s action server", action_server_name)
        client = actionlib.SimpleActionClient(action_server_name, arm_navigation_msgs.msg.MoveArmAction)
        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", action_server_name)
        if not client.wait_for_server(rospy.Duration(10)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", action_server_name)
     

        #print client_goal
        client.send_goal(goal)
        ah.set_client(client)

        ah.wait_inside()

        return ah
    
        
    ## Deals with all kind of trajectory movements for different components.
    #
    # A trajectory will be sent to the actionlib interface of the corresponding component.
    #
    # \param component_name Name of the component.
    # \param parameter_name Name of the parameter on the ROS parameter server.
    # \param blocking Bool value to specify blocking behaviour.
    
    def move_arm_traj(self, component_name, parameter_name, blocking):
        '''
        ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        rospy.loginfo("Move <<%s>> to <<%s>>", component_name, parameter_name)

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
            point_msg.time_from_start=rospy.Duration(3*point_nr) # this value is set to 3 sec per point. \todo TODO: read from parameter
            traj_msg.points.append(point_msg)

        # call action server
        action_server_name = "/" + component_name + '_controller/joint_trajectory_action'
        rospy.logdebug("calling %s action server",action_server_name)
        client = actionlib.SimpleActionClient(action_server_name, JointTrajectoryAction)
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
        #    self.set_operation_mode(component_name,"position")
        #self.set_operation_mode(component_name,"position")

        # sending goal
        client_goal = JointTrajectoryGoal()
        client_goal.trajectory = traj_msg
        #print client_goal
        client.send_goal(client_goal)
        ah.set_client(client)

        ah.wait_inside()
        return ah
        '''
        return

    def move_gripper_joint(self, component_name, parameter_name="", blocking=True):
        ah = action_handle("move_gripper", component_name, parameter_name, blocking, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        rospy.loginfo("Move <<%s>> to <<%s>>", component_name, parameter_name)

        # get pose from parameter server
        if type(parameter_name) is str:
            if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...", self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
                ah.set_failed(2)
                return ah
            param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
        else:
            param = parameter_name

        # check pose
        if not type(param) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 2
            if not len(param) == DOF: # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                    #print i,"type2 = ", type(i)
                    if not ((type(i) is float) or (type(i) is int)): # check type
                        #print type(i)
                        rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...", component_name)
                        print "parameter is:", param
                        ah.set_failed(3)
                        return ah
                    else:
                        rospy.logdebug("accepted parameter %f for %s", i, component_name)


        pose_goal = raw_arm_navigation.msg.MoveToJointConfigurationGoal()

        for i in range(DOF):
            jv = brics_actuator.msg.JointValue()
            jv.joint_uri = self.gripper1_joint_names[i]
            jv.value = param[i]
            jv.unit = "m"
            pose_goal.goal.positions.append(jv)

        action_server_name = "/arm_1/gripper_controller/MoveToJointConfigurationDirect"

        rospy.logdebug("calling %s action server", action_server_name)
        client = actionlib.SimpleActionClient(action_server_name, MoveToJointConfigurationAction)
        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", action_server_name)
        if not client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", action_server_name)


        #print client_goal
        client.send_goal(pose_goal)
        ah.set_client(client)

        ah.wait_inside()

        return ah


    ## Play a sound file.
    #
    # \param parameter_name Name of the parameter
    # \param language Language to use
    def play(self, parameter_name, blocking=True):
        component_name = "sound"
        ah = action_handle("play", component_name, parameter_name, False, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        if self.wav_path == "":
            wav_path = commands.getoutput("rospack find raw_script_server")
        else:
            wav_path = self.wav_path
        filename = wav_path + "/common/files/" + parameter_name + ".wav"

        rospy.loginfo("Playing <<%s>>", filename)
        #self.soundhandle.playWave(filename)

        #\todo TODO: check if file exists
        # if filename exists:
        #    do ...
        # else
        #    ah.set_fail(3)
        #    return ah

        if blocking:
            os.system("aplay -q " + filename)
        else:
            os.system("aplay -q " + filename + "&")
        ah.set_succeeded()
        return ah
    def set_wav_path(self, parameter_name, blocking=True):
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
    def sleep(self, duration):
        ah = action_handle("sleep", "", str(duration), True, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()
        rospy.loginfo("Wait for %f sec", duration)
        rospy.sleep(duration)
        ah.set_succeeded()

    ## Waits for user input.
    #
    # Waits either for a user input or until timeout is reached.
    #
    # \param duration Duration in seconds for timeout.
    #
    # \todo TODO: implement waiting for timeout
    def wait_for_input(self, duration=0):
        ah = action_handle("wait", "input", str(duration), True, self.parse)
        if(self.parse):
            return ah
        else:
            ah.set_active()

        if (duration != 0):
            rospy.logerr("Wait with duration not implemented yet") # \todo TODO: implement waiting with duration

        rospy.loginfo("Wait for user input...")
        retVal = raw_input()
        rospy.loginfo("...got string <<%s>>", retVal)
        ah.set_succeeded()
        return retVal


#------------------- action_handle section -------------------#
## Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the script.
class action_handle:
    ## Initializes the action handle.
    def __init__(self, function_name, component_name, parameter_name, blocking, parse):
        global graph
        global function_counter
        self.parent_node = ""
        self.error_code = -1
        self.wait_log = False
        self.function_counter = function_counter
        self.function_name = function_name
        self.component_name = component_name
        self.parameter_name = parameter_name
        self.state = ScriptState.UNKNOWN
        self.blocking = blocking
        self.parse = parse
        self.level = int(rospy.get_param("/script_server/level", 100))
        self.state_pub = rospy.Publisher("/script_server/state", ScriptState)
        self.AppendNode(blocking)
        self.client = actionlib.SimpleActionClient("dummy", ScriptAction)

    ## Sets the actionlib client.
    def set_client(self, client):
        self.client = client

    ## Sets the execution state to active, if not paused
    def set_active(self):
        self.check_pause()
        self.state = ScriptState.ACTIVE
        self.error_code = -1
        self.PublishState()

        global ah_counter
        ah_counter += 1

    ## Checks for pause
    def check_pause(self):
        param_string = "/script_server/pause"
        while bool(rospy.get_param(param_string, False)):
            rospy.logwarn("Script is paused...")
            self.state = ScriptState.PAUSED
            self.PublishState()
            rospy.sleep(1)
        if self.state == ScriptState.PAUSED:
            rospy.loginfo("...continuing script")

    ## Sets the execution state to succeeded.
    def set_succeeded(self):
        self.state = ScriptState.SUCCEEDED
        self.error_code = 0
        self.PublishState()

        global ah_counter
        ah_counter -= 1

    ## Sets the execution state to failed.
    def set_failed(self, error_code):
        self.state = ScriptState.FAILED
        self.error_code = error_code
        self.PublishState()

        global ah_counter
        ah_counter -= 1

    ## Gets the state of an action handle.
    def get_state(self):
        return self.client.get_state()

    ## Gets the error code of an action handle.
    def get_error_code(self):
        return self.error_code

    ## Returns the graphstring.
    def GetGraphstring(self):
        if type(self.parameter_name) is types.StringType:
            graphstring = str(self.function_counter) + "_" + self.function_name + "_" + self.component_name + "_" + self.parameter_name
        else:
            graphstring = str(self.function_counter) + "_" + self.function_name + "_" + self.component_name
        return graphstring

    ## Gets level of function name.
    def GetLevel(self, function_name):
        if (function_name == "move"):
            level = 0
        elif (function_name == "init"):
            level = 1
        elif (function_name == "stop"):
            level = 1
        elif (function_name == "sleep"):
            level = 2
        else:
            level = 100
        return level

    ## Appends a registered function to the graph.
    def AppendNode(self, blocking=True):
        global graph
        global graph_wait_list
        global function_counter
        global last_node
        graphstring = self.GetGraphstring()
        if self.parse:
            if (self.level >= self.GetLevel(self.function_name)):
                #print "adding " + graphstring + " to graph"
                graph.add_edge(last_node, graphstring)
                for waiter in graph_wait_list:
                    graph.add_edge(waiter, graphstring)
                graph_wait_list = []
                if blocking:
                    last_node = graphstring
                else:
                    self.parent_node = graphstring
            #else:
                #print "not adding " + graphstring + " to graph"
        #else:
            #self.PublishState()
        function_counter += 1

    ## Publishs the state of the action handle
    def PublishState(self):
        script_state = ScriptState()
        script_state.header.stamp = rospy.Time.now()
        script_state.number = self.function_counter
        script_state.function_name = self.function_name
        script_state.component_name = self.component_name
        script_state.full_graph_name = self.GetGraphstring()
        if (type(self.parameter_name) is str):
            script_state.parameter_name = self.parameter_name
        else:
            script_state.parameter_name = ""
        script_state.state = self.state
        script_state.error_code = self.error_code
        self.state_pub.publish(script_state)

    ## Handles wait.
    #
    # This function is meant to be uses directly in the script.
    #
    # \param duration Duration for timeout.
    def wait(self, duration=None):
        global ah_counter
        ah_counter += 1
        self.blocking = True
        self.wait_for_finished(duration, True)

    ## Handles inside wait.
    #
    # This function is meant to be uses inside the simple_script_server.
    #
    # \param duration Duration for timeout.
    def wait_inside(self, duration=None):
        if self.blocking:
            self.wait_for_finished(duration, True)
        else:
            thread.start_new_thread(self.wait_for_finished, (duration, False,))
        return self.error_code

    ## Waits for the action to be finished.
    #
    # If duration is specified, waits until action is finished or timeout is reached.
    #
    # \param duration Duration for timeout.
    # \param logging Enables or disables logging for this wait.
    def wait_for_finished(self, duration, logging):
        global graph_wait_list
        if(self.parse):
            if(self.parent_node != ""):
                graph_wait_list.append(self.parent_node)
            return

        if self.error_code <= 0:
            if duration is None:
                if logging:
                    rospy.loginfo("Wait for <<%s>> reaching <<%s>>...", self.component_name, self.parameter_name)
                self.client.wait_for_result()
            else:
                if logging:
                    rospy.loginfo("Wait for <<%s>> reached <<%s>> (max %f secs)...", self.component_name, self.parameter_name, duration)
                if not self.client.wait_for_result(rospy.Duration(duration)):
                    if logging:
                        rospy.logerr("Timeout while waiting for <<%s>> to reach <<%s>>. Continuing...", self.component_name, self.parameter_name)
                    self.set_failed(10)
                    return
            # check state of action server
            #print self.client.get_state()
            if self.client.get_state() != 3:
                if logging:
                    rospy.logerr("...<<%s>> could not reach <<%s>>, aborting...", self.component_name, self.parameter_name)
                self.set_failed(11)
                return

            if logging:
                rospy.loginfo("...<<%s>> reached <<%s>>", self.component_name, self.parameter_name)
        else:
            rospy.logwarn("Execution of <<%s>> to <<%s>> was aborted, wait not possible. Continuing...", self.component_name, self.parameter_name)
            self.set_failed(self.error_code)
            return

        self.set_succeeded() # full success

    ## Cancel action
    #
    # Cancels action goal(s).
    def cancel(self):
        self.client.cancel_all_goals()
