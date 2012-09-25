#!/usr/bin/python
import roslib; roslib.load_manifest('raw_fetch_and_carry')
import rospy

import smach
import smach_ros

class select_pose_to_approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['base_pose_list', 'base_pose_to_approach'],
            output_keys=['base_pose_list', 'base_pose_to_approach'])

    def execute(self, userdata):
        if(userdata.base_pose_to_approach == -1):
            userdata.base_pose_to_approach = userdata.base_pose_list[0]
        else:
            count = 0
            for item in userdata.base_pose_list:
                if userdata.base_pose_to_approach == item:
                    userdata.base_pose_to_approach = userdata.base_pose_list[((count+1) % len(userdata.base_pose_list))]
                    break;
                count = count + 1 
        
        rospy.loginfo("selected pose: %s", userdata.base_pose_to_approach)
                
        return 'succeeded'


class get_obj_poses_for_goal_configuration(smach.State):
    def __init__(self, config):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'configuration_poses_not_available'],
            input_keys=['obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
        
        self.config = config
        
    def execute(self, userdata):
        
        
        if (not rospy.has_param("/script_server/arm/" + self.config)):
            rospy.logerr("configuration <<" + self.config + ">> NOT available on parameter server")
            return 'configuration_poses_not_available'
            
        pose_names = rospy.get_param("/script_server/arm/" + self.config)

        print pose_names
        
        for pose_name in pose_names:
            print "cfg pose: ", pose_name
            userdata.obj_goal_configuration_poses.append((self.config + "/" + pose_name))
    

        userdata.obj_goal_configuration_poses.sort()
        
                
        return 'succeeded'
