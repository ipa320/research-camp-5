#!/usr/bin/python
import roslib; roslib.load_manifest('raw_fetch_and_carry')
import rospy

import smach
import smach_ros

# generic states
from generic_basic_states import *
from generic_navigation_states import *
from generic_state_machines import *

# scenario specific states
from fetch_and_carry_demo_states import *

# main
def main():
    rospy.init_node('fetch_and_carry_demo')

    SM = smach.StateMachine(outcomes=['overall_failed', 'overall_success'])
    
    # world knowledge
    SM.userdata.base_pose_list = ["S2", "S1"]
    SM.userdata.base_pose_to_approach = -1; 
    SM.userdata.object_list = [];
                                            # x, y, z, roll, pitch, yaw
    SM.userdata.rear_platform_free_poses = ['platform_centre']
    SM.userdata.rear_platform_occupied_poses = []
    
    SM.userdata.obj_goal_configuration_poses = []

    # open the container
    with SM:
        # add states to the container
        
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH', 
                         'failed':'overall_failed'})
        
        # collect objects
        smach.StateMachine.add('SELECT_POSE_TO_APPROACH', select_pose_to_approach(),
            transitions={'succeeded':'MOVE_TO_GRASP_POSE'})
        
        smach.StateMachine.add('MOVE_TO_GRASP_POSE', approach_pose(),
            transitions={'succeeded':'ADJUST_POSE', 
                        'failed':'overall_failed'})

        smach.StateMachine.add('ADJUST_POSE', sleep(),
            transitions={'succeeded':'SM_GRASP_OBJECT'})
                      
        smach.StateMachine.add('SM_GRASP_OBJECT', sm_grasp_random_object(),
            transitions={'object_grasped':'PLACE_OBJECT_ON_REAR_PLATFORM', 
                         'failed':'SM_GRASP_OBJECT'})
                                
        smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
            transitions={'succeeded':'MOVE_BACK_FIXED_DISTANCE', 
                        'no_more_free_poses':'MOVE_BACK_FIXED_DISTANCE3'})

        smach.StateMachine.add('MOVE_BACK_FIXED_DISTANCE', move_base_rel(-0.15,0),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH'})


        smach.StateMachine.add('MOVE_BACK_FIXED_DISTANCE3', move_base_rel(-0.15,0),
            transitions={'succeeded':'MOVE_TO_DESTINATION_POSE'})

        
        # place object at destination pose
        smach.StateMachine.add('MOVE_TO_DESTINATION_POSE', approach_pose("D1"),
            transitions={'succeeded':'ADJUST_POSE_WRT_PLATFORM', 
                        'failed':'overall_failed'})

        smach.StateMachine.add('ADJUST_POSE_WRT_PLATFORM', sleep(),
            transitions={'succeeded':'GET_OBJ_POSES_FOR_CONFIGURATION'})
        
        smach.StateMachine.add('GET_OBJ_POSES_FOR_CONFIGURATION', get_obj_poses_for_goal_configuration("line"),
            transitions={'succeeded':'PLACE_OBJ_IN_CONFIGURATION',
                         'configuration_poses_not_available':'overall_failed'})
         
        #place first obj in gripper
        smach.StateMachine.add('PLACE_OBJ_IN_CONFIGURATION', place_object_in_configuration(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                        'no_more_cfg_poses':'MOVE_ARM_OUT_OF_VIEW2'}) 
        
        
        #ToDo: implement state
        smach.StateMachine.add('GRASP_OBJECT_FROM_PLTF', grasp_obj_from_pltf(),
            transitions={'succeeded':'PLACE_OBJ_IN_CONFIGURATION_2',
                        'no_more_obj_on_pltf':'MOVE_ARM_OUT_OF_VIEW2'})
        
        smach.StateMachine.add('PLACE_OBJ_IN_CONFIGURATION_2', place_object_in_configuration(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                        'no_more_cfg_poses':'MOVE_ARM_OUT_OF_VIEW2'})
        
        
        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW2', move_arm_out_of_view(),
                transitions={'succeeded':'MOVE_BACK_FIXED_DISTANCE2'})

        smach.StateMachine.add('MOVE_BACK_FIXED_DISTANCE2', sleep(),
            transitions={'succeeded':'MOVE_TO_EXIT'})
        
        
        smach.StateMachine.add('MOVE_TO_EXIT', approach_pose("EXIT"),
            transitions={'succeeded':'overall_success', 
                        'failed':'overall_failed'})
              
            
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('FETCH_AND_CARRY_DEMO', SM, 'FETCH_AND_CARRY_DEMO')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
