#!/usr/bin/python

#################################################################
# Copyright (c) 2010
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA)
# Copyright (c) 2012 Tim Niemueller [www.niemueller.de]
#################################################################

from action_cmdr.abstract_action import AbstractAction
from action_cmdr.action_handle import ActionHandle

import roslib
roslib.load_manifest('generic_actions')
import rospy
import actionlib
import tf
from geometry_msgs.msg import PoseStamped
from brics_3d_msgs.srv import GetSceneObjects

from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal

#------------------- Move section -------------------#
## Deals with all kind of movements for different components.
#
# Based on the component, the corresponding move functions will be called.
#

class PercieveObject(AbstractAction):
    action_name = "perceive_object"

    def __init__(self, actions):
        self.actions = actions

    def execute(self, wait_time=1, retries=10, blocking=True):
        self.actions.prepare_perception(blocking=True)
        return self.actions.execute_perception(wait_time, retries, blocking)

class PercieveAction(AbstractAction):
    action_name = "execute_perception"
    perception_service = '/raw_perception/object_segmentation/get_scene_objects'

    def __init__(self, actions):
        self.actions = actions
        self.scene_object_finder_srv = rospy.ServiceProxy(self.perception_service, GetSceneObjects)

    def execute(self, wait_time=1, retries=10, blocking=True):
        rospy.logdebug("Waiting for perception service...")
        try:
            rospy.wait_for_service(self.perception_service, wait_time)
        except rospy.ROSException, e:
            rospy.logerr("Service did not respond: %s",e)
            return []

        for i in range(retries):
            rospy.logdebug("find object try: {0}".format(i))
            #resp = self.object_finder_srv()
            try:
               resp = self.scene_object_finder_srv()
            except rospy.ServiceException, e:
               rospy.logerr("Service did not process request: %s",e)
               return []
            
            #if (len(resp.objects) <= 0):
            if (len(resp.results.sceneObjects) <= 0):
                rospy.loginfo('found no objects')
                rospy.sleep(0.1);
            else:    
                rospy.loginfo('found {0} objects'.format(len(resp.results.sceneObjects)))
                ret_list = []
                for obj in resp.results.sceneObjects:
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "/base_link"
                    pose.pose.position  = obj.transform.transform.translation
                    pose.pose.orientation = obj.transform.transform.rotation
                    ret_list += [pose]
                return ret_list
        return []


class MoveTrajectoryAction(AbstractAction):
    action_name = "move_joint_trajectory"
    ns_global_prefix = "/script_server"

    def __init__(self, actions):
        self.actions = actions

    def execute(self, component_name, as_name, target="", blocking=True):
        ah = ActionHandle("move_joint_trajectory", component_name, target, blocking)

        rospy.loginfo("Move <<%s>> to <<%s>>",component_name,target)
        
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

        # sending goal
        client_goal = FollowJointTrajectoryGoal()
        client_goal.trajectory = traj_msg
        #print client_goal
        client.send_goal(client_goal)
        ah.set_client(client)

        ah.wait_inside()
        return ah
