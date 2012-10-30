#!/usr/bin/python

#################################################################
# Copyright (c) 2010
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA)
# Copyright (c) 2012 Tim Niemueller [www.niemueller.de]
#################################################################

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
        return self.move_arm_direct(component_name, parameter_name, blocking)
    
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
