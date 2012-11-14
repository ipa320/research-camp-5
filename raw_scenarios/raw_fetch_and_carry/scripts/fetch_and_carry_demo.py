#!/usr/bin/python
import roslib; roslib.load_manifest('raw_fetch_and_carry')
import rospy

import smach
import smach_ros

# generic states from package raw_generic_states
import generic_basic_states as gbs
import generic_state_machines as gsm

# scenario specific states from package raw_fetch_and_carry
import fetch_and_carry_demo_states as fcds

# main
def main():
    rospy.init_node('fetch_and_carry_demo')

    SM = smach.StateMachine(outcomes=['overall_failed', 'overall_success'])
    
    # world knowledge
    SM.userdata.perception_attempts = 10
    SM.userdata.base_pose_list = ["S2", "S1"]
    SM.userdata.base_pose_to_approach = -1; 
    SM.userdata.object_list = [];
    SM.userdata.target_object = 'uninitialized'
                                            # x, y, z, roll, pitch, yaw
    SM.userdata.rear_platform_free_poses = ['platform_centre']
    SM.userdata.rear_platform_occupied_poses = []
    
    SM.userdata.obj_goal_configuration_poses = []

    # open the container
    with SM:
        # add states to the container

        smach.StateMachine.add('INIT_ROBOT', gbs.init_robot(),
            transitions={'succeeded':'MOVE_TO_SOURCE',
                         'failed':'overall_failed'})

        smach.StateMachine.add('MOVE_TO_SOURCE', gsm.approach_pose('S2'),
            transitions={'succeeded':'PERCEIVE_OBJECT',
                         'failed':'overall_failed'})

        smach.StateMachine.add('PERCEIVE_OBJECT', gsm.perceive_object(),
            transitions={'found_objects':'PICK_UP_OBJECT',
                         'found_no_objects':'MOVE_TO_SOURCE',
			 'attempt_limit_exceeded':'overall_failed'})
 
        # needs to get object_pose from previous perception state - chooses first object in retrieved list
        smach.StateMachine.add('PICK_UP_OBJECT', gsm.pick_object(),
            transitions={'succeeded':'MOVE_TO_DESTINATION',
                         'failed':'PERCEIVE_OBJECT'})

        smach.StateMachine.add('MOVE_TO_DESTINATION', gsm.approach_pose('S1'),
            transitions={'succeeded':'overall_success',
                         'failed':'overall_failed'})

        '''
        # place object at destination pose
        smach.StateMachine.add('MOVE_TO_DESTINATION_POSE', gsm.approach_pose("D1"),
            transitions={'succeeded':'ADJUST_POSE_WRT_PLATFORM_AT_DESTINATION', 
                        'failed':'overall_failed'})

        smach.StateMachine.add('ADJUST_POSE_WRT_PLATFORM_AT_DESTINATION', gsm.adjust_pose_wrt_platform(),
            transitions={'succeeded':'GET_OBJ_POSES_FOR_CONFIGURATION',
                         'failed':'ADJUST_POSE_WRT_PLATFORM_AT_DESTINATION'})
        
        smach.StateMachine.add('GET_OBJ_POSES_FOR_CONFIGURATION', fcds.get_obj_poses_for_goal_configuration("line"),
            transitions={'succeeded':'PLACE_OBJ_IN_CONFIGURATION',
                         'configuration_poses_not_available':'overall_failed'})
         
        smach.StateMachine.add('PLACE_OBJ_IN_CONFIGURATION', gsm.place_object_in_configuration(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                        'no_more_cfg_poses':'MOVE_ARM_OUT_OF_VIEW2'}) 
        
        smach.StateMachine.add('GRASP_OBJECT_FROM_PLTF', gsm.grasp_obj_from_pltf(),
            transitions={'succeeded':'PLACE_OBJ_IN_CONFIGURATION_2',
                        'no_more_obj_on_pltf':'MOVE_ARM_OUT_OF_VIEW2'})
        
        smach.StateMachine.add('PLACE_OBJ_IN_CONFIGURATION_2', gsm.place_object_in_configuration(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                        'no_more_cfg_poses':'MOVE_ARM_OUT_OF_VIEW2'})
        
        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW2', gsm.move_arm_out_of_view(),
                transitions={'succeeded':'MOVE_TO_EXIT'})
        
        smach.StateMachine.add('MOVE_TO_EXIT', gsm.approach_pose("EXIT"),
            transitions={'succeeded':'overall_success', 
                        'failed':'overall_failed'})
        smach.StateMachine.add('INIT_ROBOT', gbs.init_robot(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH'})

        # collect objects
        smach.StateMachine.add('SELECT_POSE_TO_APPROACH', fcds.select_pose_to_approach(),
            transitions={'succeeded':'MOVE_TO_GRASP_POSE'})
        
        smach.StateMachine.add('MOVE_TO_GRASP_POSE', gsm.approach_pose(),
            transitions={'succeeded':'ADJUST_POSE_WRT_PLATFORM_AT_SOURCE', 
                        'failed':'overall_failed'})
        
        smach.StateMachine.add('ADJUST_POSE_WRT_PLATFORM_AT_SOURCE', gsm.adjust_pose_wrt_platform(),
            transitions={'succeeded':'SM_GRASP_OBJECT',
                         'failed':'ADJUST_POSE_WRT_PLATFORM_AT_SOURCE'})
                      
        smach.StateMachine.add('SM_GRASP_OBJECT', gsm.sm_grasp_random_object(),
            transitions={'object_grasped':'PLACE_OBJECT_ON_REAR_PLATFORM', 
                         'failed':'SM_GRASP_OBJECT'})
                                
        smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', gsm.place_obj_on_rear_platform(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH', 
                        'no_more_free_poses':'MOVE_TO_DESTINATION_POSE'})
        '''


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
