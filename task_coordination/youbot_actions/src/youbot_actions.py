#!/usr/bin/python

#################################################################
# Copyright (c) 2010
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA)
# Copyright (c) 2012 Tim Niemueller [www.niemueller.de]
#################################################################

from action_cmdr.abstract_action import AbstractAction
from action_cmdr.action_handle import ActionHandle
from raw_arm_navigation.msg import MoveToJointConfigurationGoal, MoveToJointConfigurationAction, MoveToCartesianPoseGoal, MoveToCartesianPoseAction, MoveToJointConfigurationGoal
from brics_actuator.msg import JointValue
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from raw_base_placement.msg import OrientToBaseAction, OrientToBaseActionGoal

import roslib
roslib.load_manifest('youbot_actions')
import rospy
import actionlib
import tf

class PrintAction(AbstractAction):
    action_name = "printit"

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


class YouBotPickUpAction(AbstractAction):
    action_name = "pick_up"
    
    def __init__(self, actions):
        self.actions = actions

    def execute(self, target, blocking=True):
	self.actions.move_gripper(target="open", blocking=True)
	self.actions.move_arm(target=target, blocking=True)
	self.actions.move_gripper(target="close", blocking=True)
	lifted_target = target
	lifted_target.pose.position.z += 0.1
	return self.actions.move_arm(target=lifted_target, blocking=True)

class YouBotMoveArmAction(AbstractAction):
    action_name = "move_arm"
    DOF = 5
    
    def __init__(self, actions):
        self.actions = actions

    def execute(self, target, blocking=True):
        if type(target) is str:
            return self.actions.move_arm_joint_parameter("arm", target, blocking)
        elif type(target) is list:
            if len(target) == self.DOF:
                return self.actions.move_arm_joint_direct("arm", target, blocking)
            #if len(target) == 3:
                #return self.actions.move_arm_cart_direct("arm", target, blocking)
            elif len(target) == 4:
                return self.actions.move_arm_cart_sample_rpy_direct("arm", target, blocking)
            #else:
                #rospy.loginfo("parameter <<%s>> is not in the right format", target)        
        
        elif type(target) is PoseStamped:
            return self.actions.move_arm_cart_sample_rpy_direct("arm", [target.pose.position.x, target.pose.position.y, target.pose.position.z, target.header.frame_id], blocking)
            #rospy.logerr("not implemented yet")

        #return self.actions.move_arm_direct("arm", target, blocking)


        #WHATS MISSING:
        #- handle joint space trajectories
        #
        #- do full target parsing checks



#class YouBotMoveAction(AbstractAction):
    #action_name = "move"

    #def __init__(self, actions):
        #self.actions = actions

    ## \param component_name Name of the component.
    ## \param target Name of the parameter on the ROS parameter server.
    ## \param blocking Bool value to specify blocking behaviour.
    #def execute(self, component_name, target, blocking=True, mode=""):
        #if component_name == "base":
            #return self.actions.move_base(component_name, target, blocking)
    
        #elif component_name == "arm":
            #return self.actions.move_arm_direct(component_name, target, blocking)
    
        #elif component_name == "gripper":
            #return self.actions.move_gripper_joint(component_name, target, blocking)

class YouBotAlignWrtWorkspace(AbstractAction):
    action_name = "align_wrt_workspace"

    def __init__(self, actions):
	self.actions = actions

        self.ac_base_adj_name = '/raw_base_placement/adjust_to_workspace'
        self.ac_base_adj = actionlib.SimpleActionClient(self.ac_base_adj_name, OrientToBaseAction)

    def execute(self, blocking = True):
        rospy.loginfo("Waiting for action server <<%s>> to start ...", self.ac_base_adj_name);
        self.ac_base_adj.wait_for_server()
        rospy.loginfo("action server <<%s>> is ready ...", self.ac_base_adj_name);
        action_goal = OrientToBaseActionGoal()

        action_goal.goal.distance = 0.1;
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


class YouBotPreparePerception(AbstractAction):
    action_name = "prepare_perception"
 
    def __init__(self, actions):
        self.actions = actions
        
    def execute(self, blocking=True):
        self.actions.align_wrt_workspace(blocking=True)
        ah = self.actions.move_gripper("open")
        if ah.get_error_code() != 0:
            return ah
        return self.actions.move_arm("arm_out_of_view")

    

#########################################################
##### ARM MOVEMENTS WITHOUT PLANNING
#########################################################

#class YouBotMoveArmDirect(AbstractAction):
    #action_name = "move_arm_direct"

    #def __init__(self, actions):
        #self.actions = actions

    #def execute(self, component_name, target=[0, 0, 0, 0, 0, 0, "/base_link"], blocking=True):
        #if type(target) is str:
            #return self.actions.move_arm_joint_direct(component_name, target, blocking)
        #elif type(target) is list:
            #if len(target) == 7:
                #return self.actions.move_arm_cart_direct(component_name, target, blocking)
            #elif len(target) == 4:
                #return self.actions.move_arm_cart_sample_rpy_direct(component_name, target, blocking)
            #else:
                #rospy.loginfo("parameter <<%s>> is not in the right format", target)



class YouBotMoveArmJointParameter(AbstractAction):
    action_name = "move_arm_joint_parameter"
    ns_global_prefix = "/script_server"

    def __init__(self, actions):
        self.actions = actions
        
    def execute(self, component_name, target="", blocking=True):
        ah = ActionHandle("move_arm_joint_direct", component_name, target, blocking)

        rospy.loginfo("Move <<%s>> PARAMETER to <<%s>>", component_name, target)

        # get pose from parameter server
        if type(target) is str:
            if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + target):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...", self.ns_global_prefix + "/" + component_name + "/" + target)
                ah.set_failed(2)
                return ah
            param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + target)
        else:
            ah.set_failed(3)
            return ah

        return self.actions.move_arm_joint_direct(component_name, param, blocking)




class YouBotMoveArmJointDirect(AbstractAction):
    action_name = "move_arm_joint_direct"
    arm1_joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
    action_server_name_prefix = "/arm_1/arm_controller"

    def __init__(self, actions):
        self.actions = actions
        self.action_server_name = self.action_server_name_prefix + "/MoveToJointConfigurationDirect" #TODO: get this from somewhere else
        self.client = actionlib.SimpleActionClient(self.action_server_name, MoveToJointConfigurationAction)
        
    def execute(self, component_name, target="", blocking=True):
        ah = ActionHandle("move_arm_joint_direct", component_name, target, blocking)

        rospy.loginfo("Move <<%s>> DIRECT to <<%s>>", component_name, target)


        # check pose
        if not type(target) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", target
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 5
            if not len(target) == DOF: # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(target))
                print "parameter is:", target
                ah.set_failed(3)
                return ah
            else:
                for i in target:
                    #print i,"type2 = ", type(i)
                    if not ((type(i) is float) or (type(i) is int)): # check type
                        #print type(i)
                        rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...", component_name)
                        print "parameter is:", target
                        ah.set_failed(3)
                        return ah
                    else:
                        rospy.logdebug("accepted parameter %f for %s", i, component_name)


        pose_goal = MoveToJointConfigurationGoal()

        for i in range(DOF):
            jv = JointValue()
            jv.joint_uri = self.arm1_joint_names[i]
            jv.value = target[i]
            jv.unit = "rad"
            pose_goal.goal.positions.append(jv)

        

        rospy.logdebug("calling %s action server", self.action_server_name)
        
        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", self.action_server_name)
        if not self.client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", self.action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", self.action_server_name)


        #print client_goal
        self.client.send_goal(pose_goal)
        ah.set_client(self.client)

        ah.wait_inside()

        return ah    

#class YouBotMoveArmCartDirect(AbstractAction):
    #action_name = "move_arm_cart_direct"

    #def execute(self, component_name, target=[0,0,0, "/base_link"], blocking=True):
        #ah = ActionHandle("move_arm_cart_direct", component_name, target, blocking, self.parse)
        #if(self.parse):
            #return ah
        #else:
            #ah.set_active()
        #rospy.loginfo("Move <<%s>> DIRECT to <<%s>>", component_name, target)

        ### check pose
        ##if not type(target) is list: # check outer list
            ##rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            ##print "parameter is:", target
            ##ah.set_failed(3)
            ##return ah
        ##else:
            ##if ((not type(target[0]) is list) or (not type(target[1]) is list) or (not type(target[2]) is str)):
                ##rospy.logerr("no valid parameter for %s: not a cartesian, aborting...", component_name)
                ##print "parameter is:", target
                ##ah.set_failed(3)
                ##return ah


        ## convert to pose message
        #pose = raw_arm_navigation.msg.MoveToCartesianPoseGoal()
        #pose.goal.header.stamp = rospy.Time.now()
        #pose.goal.header.frame_id = param[6]
        #pose.goal.pose.position.x = param[0]
        #pose.goal.pose.position.y = param[1]
        #pose.goal.pose.position.z = param[2]

        #(qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(param[3], param[4], param[5])
        #pose.goal.pose.orientation.x = qx
        #pose.goal.pose.orientation.y = qy
        #pose.goal.pose.orientation.z = qz
        #pose.goal.pose.orientation.w = qw

        #action_server_name = "/arm_1/arm_controller/MoveToCartesianPoseDirect"

        #rospy.logdebug("calling %s action server", action_server_name)

        #client = actionlib.SimpleActionClient(action_server_name, MoveToCartesianPoseAction)

        ## trying to connect to server
        #rospy.logdebug("waiting for %s action server to start", action_server_name)
        #if not client.wait_for_server(rospy.Duration(5)):
            ## error: server did not respond
            #rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            #ah.set_failed(4)
            #return ah
        #else:
            #rospy.logdebug("%s action server ready", action_server_name)

        #client.send_goal(pose)
        #ah.set_client(client)
        #ah.wait_inside()

        #return ah

class YouBotMoveArmCartSampleRPYDirect(AbstractAction):
    action_name = "move_arm_cart_sample_rpy_direct"
    action_server_name_prefix = "/arm_1/arm_controller"

    def __init__(self, actions):
        self.actions = actions
        self.action_server_name = self.action_server_name_prefix + "/MoveToCartesianRPYSampledDirect" #TODO: Get from somewhere else
        self.client = actionlib.SimpleActionClient(self.action_server_name, MoveToCartesianPoseAction)
        
    def execute(self, component_name, target=[0, 0, 0, "/base_link"], blocking=True):
        ah = ActionHandle("move_arm_cart_sample_rpy_direct", component_name, target, blocking)

        rospy.loginfo("Move <<%s>> DIRECT to <<%s>>", component_name, target)

        # check pose
        if not type(target) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            if not len(target) == 4 or not type(target[0])==float or not type(target[1])==float or not type(target[2])==float or not type(target[3]) == str: # checking parameters
                rospy.logerr("no valid target parameters for %s: %s; aborting...", component_name, target)
                ah.set_failed(3)
                return ah

        # convert to pose message
        pose = MoveToCartesianPoseGoal()
        pose.goal.header.stamp = rospy.Time.now()
        pose.goal.header.frame_id = target[3]

        pose.goal.pose.position.x = target[0]
        pose.goal.pose.position.y = target[1]
        pose.goal.pose.position.z = target[2]

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(0,0,0)
        pose.goal.pose.orientation.x = qx
        pose.goal.pose.orientation.y = qy
        pose.goal.pose.orientation.z = qz
        pose.goal.pose.orientation.w = qw

        rospy.logdebug("calling %s action server", self.action_server_name)
        # trying to connect to server
        rospy.logdebug("waiting for %s action server to start", self.action_server_name)
        if not self.client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", self.action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", self.action_server_name)

        self.client.send_goal(pose)
        ah.set_client(self.client)
        ah.wait_inside()

        return ah
    

class YouBotMoveArmCartesian(YouBotMoveArmCartSampleRPYDirect):
	action_name = 'move_arm_cartesian'

class YouBotMoveGripperJoint(AbstractAction):
    action_name = "move_gripper"
    action_server_name_prefix = "/script_server"
    gripper1_joint_names = ["gripper_finger_joint_l", "gripper_finger_joint_r"]

    def execute(self, target="", blocking=True):
        component_name = "gripper"
        ah = ActionHandle("move_gripper", component_name, target, blocking)
		
        rospy.loginfo("Move <<%s>> to <<%s>>", component_name, target)

        # get pose from parameter server
        if type(target) is str:
            if not rospy.has_param(self.action_server_name_prefix + "/" + component_name + "/" + target):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...", self.action_server_name_prefix + "/" + component_name + "/" + target)
                ah.set_failed(2)
                return ah
            param = rospy.get_param(self.action_server_name_prefix + "/" + component_name + "/" + target)
        else:
            param = target

        # check pose
        """
	if not type(param) is list:  # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            print i,"type1 = ", type(i)
            DOF = 2
            if not len(param) == DOF:  # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                    print i,"type2 = ", type(i)
                    if not ((type(i) is float) or (type(i) is int)): # check type
                        print type(i)
                        rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...", component_name)
                        print "parameter is:", param
                        ah.set_failed(3)
                        return ah
                    else:
                        rospy.logdebug("accepted parameter %f for %s", i, component_name)
	"""

        pose_goal = MoveToJointConfigurationGoal()

        DOF = 2
        for i in range(DOF):
            jv = JointValue()
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


        print pose_goal 
        client.send_goal(pose_goal)
        ah.set_client(client)

        ah.wait_inside()
	# it should block but it does not, so we wait!
	rospy.sleep(3.0)

        return ah



### Deals with movements of the base.
##
## A target will be sent to the actionlib interface of the move_base node.
class YouBotMoveBaseAction(AbstractAction):
    action_name = "move_base"
    ns_global_prefix = "/script_server"

    
    # \param component_name Name of the component.
    # \param target Name of the parameter on the ROS parameter server.
    # \param blocking Bool value to specify blocking behaviour.

    def execute(self, target, blocking=True):
        component_name = "base"
        ah = ActionHandle("move_base", component_name, target, blocking)
            
        rospy.loginfo("Move <<%s>> to <<%s>>", component_name, target)

        #get pose from parameter server
        if type(target) is str:
            if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + target):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...", self.ns_global_prefix + "/" + component_name + "/" + target)
                ah.set_failed(2)
                return ah
            param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + target)
        else:
            param = target
        #check pose
        if not type(param) is list: # check outer list
            rospy.logerr("no valid parameter for %s: not a list, aborting...", component_name)
            #print "parameter is:", param
            ah.set_failed(3)
            return ah
        else:
            #print i,"type1 = ", type(i)
            DOF = 3
            if not len(param) == DOF: # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...", component_name, DOF, len(param))
                #print "parameter is:", param
                ah.set_failed(3)
                return ah
            else:
                for i in param:
                #print i,"type2 = ", type(i)
                    if not ((type(i) is float) or (type(i) is int)):
                    #check type
                    #print type(i)
                        rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...", component_name)
                        print "parameter is:", param
                        ah.set_failed(3)
                        return ah
                    else:
                        rospy.logdebug("accepted parameter %f for %s", i, component_name)

        #convert to pose message
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

        #trying to connect to server
        rospy.logdebug("waiting for %s action server to start", action_server_name)
        if not client.wait_for_server(rospy.Duration(5)):
            #error: server did not respond
            rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
            ah.set_failed(4)
            return ah
        else:
            rospy.logdebug("%s action server ready", action_server_name)

        # sending goal
        client_goal = MoveBaseGoal()
        client_goal.target_pose = pose
        print client_goal
        client.send_goal(client_goal)
        ah.set_client(client)
        ah.wait_inside()

        return ah
