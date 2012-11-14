#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import raw_srvs.srv
import std_srvs.srv
import tf 
import geometry_msgs.msg
from brics_3d_msgs.srv import GetSceneObjects

from os import getenv
robot_platform = getenv("ROBOT")
# yes, this is a hack, a better solution is known and has been discussed,
# cf. the Lua-based Behavior Engine, but is pending implementation after RC5
if robot_platform == "cob3-3":
    robot_platform = "cob"

import action_cmdr
action_cmdr.load(["generic_actions", robot_platform + "_actions"])

class perceive_object(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['found_objects', 'found_no_objects','attempt_limit_exceeded'],
            input_keys=['perception_attempts'], 
            output_keys=['object_list'])
	self.attempts = 0

    def execute(self, userdata):
	self.attempts += 1
	if self.attempts > userdata.perception_attempts:
		return 'attempt_limit_exceeded'

        rospy.loginfo("started looking for objects")
        data = action_cmdr.perceive_object()
        userdata.object_list = data
        rospy.loginfo("ended looking for objects")
	
	if len(data) > 0:
		return 'found_objects'
	else:
		return 'found_no_objects'


class detect_object(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            output_keys=['object_list'])
        
        #self.object_finder_srv = rospy.ServiceProxy('/raw_perception/object_segmentation/get_segmented_objects', raw_srvs.srv.GetObjects)
        self.scene_object_finder_srv = rospy.ServiceProxy('/raw_perception/object_segmentation/get_scene_objects', GetSceneObjects)

    def execute(self, userdata):     
        #get object pose list
        #rospy.wait_for_service('/raw_perception/object_segmentation/get_segmented_objects', 30)
	rospy.wait_for_service('/raw_perception/object_segmentation/get_scene_objects', 30)

        for i in range(10): 
            print "find object try: ", i
            #resp = self.object_finder_srv()
            resp = self.scene_object_finder_srv()
              
            #if (len(resp.objects) <= 0):
            if (len(resp.results.sceneObjects) <= 0):
                rospy.loginfo('found no objects')
                rospy.sleep(0.1);
            else:    
                rospy.loginfo('found {0} objects'.format(len(resp.results.sceneObjects)))
                break
            
        #if (len(resp.objects) <= 0):
        if (len(resp.results.sceneObjects) <= 0):
            rospy.logerr("no graspable objects found");
            userdata.object_list = []            
            return 'failed'
        
        else:
            #userdata.object_list = resp.objects
            userdata.object_list = resp.results.sceneObjects
            return 'succeeded'
