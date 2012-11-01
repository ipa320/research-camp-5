#!/usr/bin/python
PKG = 'raw_generic_states'
import roslib
roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import math

import action_cmdr
action_cmdr.init(PKG)

class grasp_random_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_list'])
        
    def execute(self, userdata):
        action_cmdr.move_gripper("open")
        action_cmdr.move_arm("zeroposition")
        
        for object in userdata.object_list:         
                
            action_cmdr.move_arm("zeroposition")                             

            ##object.pose.pose.position.z = object.pose.pose.position.z + 0.02
            #object.pose.pose.position.x = object.pose.pose.position.x + 0.01
            #object.pose.pose.position.y = object.pose.pose.position.y - 0.005

            #object.transform.transform.translation.z = object.transform.transform.translation.z + 0.02
            #object.transform.transform.translation.x = object.transform.transform.translation.x + 0.01
            #object.transform.transform.translation.y = object.transform.transform.translation.y - 0.005

            #handle_arm = action_cmdr.move_arm([object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, "/base_link"])
            handle_arm = action_cmdr.move_arm_cartesian([object.transform.transform.translation.x, object.transform.transform.translation.y, object.transform.transform.translation.z, "/base_link"])

## component needs to specified explicitly or be configured via yaml file
## component = "arm_controller" ### direct
## component = "arm" ### via paramter arm --> namespace of FollowJointTrajectory-ActionServer
#sss.move_joint_goal(component, string, blocking = True)
#sss.move_joint_goal(component, [[],string,...], blocking = True) # trajectory[list] of joint configs[list]; joint config can be string or [double]
#sss.move_joint_goal_planned(component, string, blocking = True)
#sss.move_joint_goal_planned(component, [[],string,...], blocking = True) # trajectory[list] of joint configs[list]; joint config can be string or [double]
#
#sss.move_cartesian_planned(component,pose_stamped,blocking)
#sss.move_cartesian_planned(component,pose_stamped) # blocking = True
#sss.move_cartesian_planned(component,[[1,2,3],[4,5,6],frame_id]) # blocking = True
#sss.move_cartesian_planned(component,[[1,2,3],frame_id]) # rpy=[current], blocking = True
#sss.move_cartesian_planned(component,[[1,2,3]]) # rpy=[current],frame_id='/base_link', blocking = True
#
#
## sss.move will be replaced by the upper functions

            if handle_arm.get_state() == 3:
                action_cmdr.move_gripper("close")
                rospy.sleep(3.0)
                action_cmdr.move_arm("zeroposition")        
                return 'succeeded'    
            else:
                rospy.logerr('could not find IK for current object')

        return 'failed'
 

class place_obj_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_free_poses'], input_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'], 
								output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'])

    def execute(self, userdata):   
        
        action_cmdr.move_arm("zeroposition")
        action_cmdr.move_arm("platform_intermediate")

        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on platform")
            return 'no_more_free_poses'
            
        pltf_pose = userdata.rear_platform_free_poses.pop()
        # 
        action_cmdr.move_arm(pltf_pose+"_pre")
        #
        action_cmdr.move_arm(pltf_pose)
        
        
        action_cmdr.move_gripper("open")
        rospy.sleep(2)

        userdata.rear_platform_occupied_poses.append(pltf_pose)
        
        action_cmdr.move_arm(pltf_pose+"_pre")
        action_cmdr.move_arm("platform_intermediate")

        return 'succeeded'
    

  
class move_arm_out_of_view(smach.State):

    def __init__(self, do_blocking = True):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.do_blocking = do_blocking

    def execute(self, userdata):   
        action_cmdr.move_arm("zeroposition")
        action_cmdr.move_arm("arm_out_of_view")
           
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
        
        action_cmdr.move_arm("platform_intermediate")
        # 
        action_cmdr.move_arm(pltf_obj_pose+"_pre")
        #
        action_cmdr.move_arm(pltf_obj_pose)
        
        action_cmdr.move_gripper("close")
        rospy.sleep(3)
        
        # untested
        action_cmdr.move_arm(pltf_obj_pose+"_pre")
        #
        action_cmdr.move_arm("platform_intermediate")
        action_cmdr.move_arm("zeroposition")
           
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
        
        action_cmdr.move_arm(cfg_goal_pose)
        
        action_cmdr.move_gripper("open")
        rospy.sleep(2)
                
        return 'succeeded'
        
        
class pick_up(smach.State):

    def __init__(self, do_blocking = True):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.do_blocking = do_blocking

    def execute(self, userdata):   
        action_cmdr.pick_up()
           
        return 'succeeded'           
