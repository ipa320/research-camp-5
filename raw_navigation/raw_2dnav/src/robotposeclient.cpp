#include "ros/ros.h"
#include "../srv_gen/cpp/include/raw_basic_navigation/RobotPose.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getRobotPose");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<raw_basic_navigation::RobotPose>("robot_pose");
  raw_basic_navigation::RobotPose srv;
  geometry_msgs::Pose pose;
  if (client.call(srv))
  {
    pose = srv.response.pose;
  }
  else
  {
    ROS_ERROR("Failed to call service robot_pose");
    return 1;
  }

  return 0;
}
