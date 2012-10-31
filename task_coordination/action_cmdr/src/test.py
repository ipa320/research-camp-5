#!/usr/bin/python
import roslib
roslib.load_manifest('action_cmdr')
import rospy
import action_cmdr
action_cmdr.init("action_cmdr")
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node('test') # needed to add this so I could make trajectory messages, require timestamp
    rospy.sleep(2)
    #action_cmdr.test("foo")
    action_cmdr.move_arm(target="zeroposition", blocking=True)
    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0.590
    pose.pose.position.y = -0.004
    pose.pose.position.z = 0.168
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0
    action_cmdr.move_arm(target=pose, blocking=True)
