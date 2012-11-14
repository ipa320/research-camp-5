#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_2dnav')

import rospy
import tf


if __name__ == "__main__":
    rospy.init_node("save_base_map_pose_to_file")


    tf_listener = tf.TransformListener()

    tf_received = False

    while(not rospy.is_shutdown()):
        # get pose name from user
        pose_name = raw_input("\nPlease enter the pose name: ")

        # get transformation between map and base_link
        while(not tf_received):
            try:
                tf_listener.waitForTransform('map', '/base_link', rospy.Time.now(), rospy.Duration(1))
                (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0));
                
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
                
                pose_description = "%s: [%lf, %lf, %lf]\n" % (pose_name, trans[0], trans[1], yaw)
                print pose_description
                tf_received = True
            except Exception, e:
                rospy.sleep(1)
                tf_received = False
        tf_received = False

        #write position into a file
        pose_file = open('navigation_goals.yaml', 'a')
        pose_file.write(pose_description)	    
        pose_file.close()
    
