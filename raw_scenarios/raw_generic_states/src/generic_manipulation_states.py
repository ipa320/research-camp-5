#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import math


from os import getenv
robot_platform = getenv("ROBOT")
# yes, this is a hack, a better solution is known and has been discussed,
# cf. the Lua-based Behavior Engine, but is pending implementation after RC5
if robot_platform == "cob3-3":
    robot_platform = "cob"

import action_cmdr
action_cmdr.load(["generic_actions", robot_platform + "_actions"])

class grasp_random_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_list'])
        
    def execute(self, userdata):
        action_cmdr.move_gripper("open")
        action_cmdr.move_arm("zeroposition")
        
        for object in userdata.object_list:         
                
            sss.move_arm("zeroposition")                             

            ##object.pose.pose.position.z = object.pose.pose.position.z + 0.02
            #object.pose.pose.position.x = object.pose.pose.position.x + 0.01
            #object.pose.pose.position.y = object.pose.pose.position.y - 0.005

            #object.transform.transform.translation.z = object.transform.transform.translation.z + 0.02
            #object.transform.transform.translation.x = object.transform.transform.translation.x + 0.01
            #object.transform.transform.translation.y = object.transform.transform.translation.y - 0.005

            #handle_arm = sss.move_arm([object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, "/base_link"])
            object_pose = PoseStamped()
            object_pose.pose.position.x = object.transform.transform.x
            object_pose.pose.position.y = object.transform.transform.y
            object_pose.pose.position.z = object.transform.transform.z
            object_pose.pose.orientation = object.transform.tranform.rotation
            object_pose.header.frame_id = object.transform.header.frame_id

            handle_arm = sss.move_arm(object_pose)
            
            if handle_arm.get_state() == 3:
                sss.move_gripper("close")
                rospy.sleep(3.0)
                sss.move_arm("zeroposition")        
                return 'succeeded'    
            else:
                rospy.logerr('could not find IK for current object')

        return 'failed'
 

class place_obj_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_free_poses'], input_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'], 
								output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'])

    def execute(self, userdata):   
        
        sss.move_arm("zeroposition")
        sss.move_arm("platform_intermediate")

        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on platform")
            return 'no_more_free_poses'
            
        pltf_pose = userdata.rear_platform_free_poses.pop()
        # 
        sss.move_arm(pltf_pose+"_pre")
        #
        sss.move_arm(pltf_pose)
        
        
        sss.move_gripper("open")
        rospy.sleep(2)

        userdata.rear_platform_occupied_poses.append(pltf_pose)
        
        sss.move_arm(pltf_pose+"_pre")
        sss.move_arm("platform_intermediate")

        return 'succeeded'
    

  
class move_arm_out_of_view(smach.State):

    def __init__(self, do_blocking = True):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.do_blocking = do_blocking

    def execute(self, userdata):   
        sss.move_arm("zeroposition")
        sss.move_arm("arm_out_of_view")
           
        return 'succeeded'        
        

    
class grasp_obj_from_pltf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_obj_on_pltf'], 
                             input_keys=['rear_platform_occupied_poses'],
                             output_keys=['rear_platform_occupied_poses'])

    def execute(self, userdata):   

        if len(userdata.rear_platform_occupied_poses) == 0:
            rospy.logerr("NO more objects on platform")
            return 'no_more_obj_on_pltf'

        pltf_obj_pose = userdata.rear_platform_occupied_poses.pop()
        
        sss.move_arm("platform_intermediate")
        # 
        sss.move_arm(pltf_obj_pose+"_pre")
        #
        sss.move_arm(pltf_obj_pose)
        
        sss.move_gripper("close")
        rospy.sleep(3)
        
        # untested
        sss.move_arm(pltf_obj_pose+"_pre")
        #
        sss.move_arm("platform_intermediate")
        sss.move_arm("zeroposition")
           
        return 'succeeded'
    
    
class place_object_in_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'no_more_cfg_poses'],
            input_keys=['obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
        
    def execute(self, userdata):
        
        if len(userdata.obj_goal_configuration_poses) == 0:
            rospy.logerr("no more configuration poses")
            return 'no_more_cfg_poses'
        
        cfg_goal_pose = userdata.obj_goal_configuration_poses.pop()
        print "goal pose taken: ",cfg_goal_pose
        print "rest poses: ", userdata.obj_goal_configuration_poses
        
        sss.move_arm(cfg_goal_pose)
        
        sss.move_gripper("open")
        rospy.sleep(2)
                
        return 'succeeded'
        
        
class pick_object(smach.State):

    def __init__(self, do_blocking = True):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_list'])
        self.do_blocking = do_blocking

    def execute(self, userdata):   
	# grasps first object in list
	# THIS SHOULD BE BROUGHT OUT AS A SEPARATE STATE
        ah = action_cmdr.pick_up(userdata.object_list[0])
        if ah.get_state() == 3: 
	        return 'succeeded'
	else:
		return 'failed'
	# TODO: add failure check
