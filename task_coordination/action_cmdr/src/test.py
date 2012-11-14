#!/usr/bin/python
import roslib
roslib.load_manifest('action_cmdr')
import rospy
import action_cmdr
action_cmdr.load(["generic_actions","youbot_actions"])
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node('test') # needed to add this so I could make trajectory messages, require timestamp
    rospy.sleep(2)

    action_cmdr.move_base(target="S1",blocking=True)
    
    action_cmdr.prepare_perception()
    
    det_objects = action_cmdr.execute_perception(wait_time=1,retries=10,blocking=True)
    if len(det_objects) > 0:
        print("Got: %s",det_objects);
	obj = det_objects[0]
	ret = action_cmdr.pick_up(target=obj,blocking=True)
        print("Got: %s",ret);
	action_cmdr.move_base(target="D1",blocking=True)
	action_cmdr.move_gripper(target="open",blocking=True)
    else:
	print("didn't find anything!")

    #action_cmdr.test("foo")
    
    
    #action_cmdr.move_gripper(target="cylopen", blocking=True)
    
    #action_cmdr.move_torso(target="front", blocking=True)
    
    #action_cmdr.move_tray(target="down", blocking=True)
    
    #action_cmdr.move_head(target="front", blocking=True)
    
    
    #action_cmdr.move_arm(target=["wavein","waveout","wavein"], blocking=True)
    #action_cmdr.move_arm_planned(target=["waveout","wavein","waveout"], blocking=True)
    

#### youbot
    ##pose = PoseStamped()
    ##pose.header.frame_id = "/base_link"
    ##pose.header.stamp = rospy.Time.now()
    ##pose.pose.position.x = 0.590
    ##pose.pose.position.y = -0.004
    ##pose.pose.position.z = 0.168
    ##pose.pose.orientation.x = 0
    ##pose.pose.orientation.y = 0
    ##pose.pose.orientation.z = 0
    ##pose.pose.orientation.w = 0
    
### cob
    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    #pose.pose.position.x = -0.626
    #pose.pose.position.y = -0.005
    #pose.pose.position.z = 0.963
    #pose.pose.orientation.x = 0.225
    #pose.pose.orientation.y = 0.673
    #pose.pose.orientation.z = -0.664
    #pose.pose.orientation.w = -0.236  
    pose.pose.position.x = -0.715
    pose.pose.position.y = -0.130
    pose.pose.position.z = 0.882
 

    #action_cmdr.move_arm_planned(target=pose, blocking=True)
    
    action_cmdr.pick_up(target=pose, blocking=True)

