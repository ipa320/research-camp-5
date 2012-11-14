#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <raw_base_placement/OrientToBaseAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "BaseOrientation_Test");

  actionlib::SimpleActionClient<raw_base_placement::OrientToBaseAction> ac("/raw_base_placement/adjust_to_workspace", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  raw_base_placement::OrientToBaseActionGoal goal;

  goal.goal.distance = 0.07;

  ac.sendGoal(goal.goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration());

  while(!finished_before_timeout)
  {
	  finished_before_timeout = ac.waitForResult(ros::Duration());
	  
  }
  
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}



