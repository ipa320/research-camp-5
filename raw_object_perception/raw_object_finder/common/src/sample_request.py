#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_object_finder')
import rospy


import sys

from brics_3d_msgs.srv import GetSceneObjects

def sample_query():
	rospy.wait_for_service('/raw_perception/object_segmentation/get_scene_objects')
	try:
		getSceneObject = rospy.ServiceProxy('/raw_perception/object_segmentation/get_scene_objects', GetSceneObjects)
		sceneObjects = getSceneObject()
		rospy.loginfo('found {0} objects'.format(len(sceneObjects.results.sceneObjects)))
		for object in sceneObjects.results.sceneObjects:
			print object.transform.transform.translation.x 
			print object.transform.transform.translation.y
			print object.transform.transform.translation.z
			print
 
		print sceneObjects

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
	print "Requesting scene objects."
	sample_query()
	print "Done."
	
