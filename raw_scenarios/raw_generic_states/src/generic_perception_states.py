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
