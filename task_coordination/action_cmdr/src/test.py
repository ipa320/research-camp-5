#!/usr/bin/python
import roslib; roslib.load_manifest('action_cmdr')
import rospy 
import time

import action_cmdr
action_cmdr.init(["cob_actions"])

if __name__ == "__main__":
    rospy.init_node('test') # needed to add this so I could make trajectory messages, require timestamp
    time.sleep(1)
    action_cmdr.move("arm","home")
