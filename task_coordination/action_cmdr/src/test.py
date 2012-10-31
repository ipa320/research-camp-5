#!/usr/bin/python
import roslib
roslib.load_manifest('action_cmdr')
import rospy
import action_cmdr
action_cmdr.init(["youbot_actions"])


if __name__ == "__main__":
    rospy.init_node('test') # needed to add this so I could make trajectory messages, require timestamp
    rospy.sleep(2)
    action_cmdr.test("foo")
    action_cmdr.move_arm(target="zeroposition", blocking=True)
