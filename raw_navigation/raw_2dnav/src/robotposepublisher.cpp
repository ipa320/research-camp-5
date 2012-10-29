/*
 * Robot Pose Publisher. Publishes pose of the robot at every 10Hz.
 * Precise enough for planning ..
 *
 * Author : Praveen Ramanujam (H-BRS)
 *          Ravi Kumar Venkat (H-BRS)
 */


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>


class RobotPose
{
private :

	tf::TransformListener listener;
	geometry_msgs::Pose pose;

public:
	RobotPose(){};
	geometry_msgs::Pose getRobotPose();

};

geometry_msgs::Pose RobotPose::getRobotPose()
{
	  tf::StampedTransform transform;
	  try
	  {
	     listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
	     pose.position.x = transform.getOrigin().getX();
	     pose.position.y = transform.getOrigin().getY();
	     pose.orientation.x = transform.getRotation().x();
	     pose.orientation.y = transform.getRotation().y();
	     pose.orientation.z = transform.getRotation().z();
	     pose.orientation.w = transform.getRotation().w();

      }
	  catch (tf::TransformException ex)
	  {
	        //ROS_ERROR("%s",ex.what());
		    ROS_WARN("Waiting for transform.. Nothing in Cache");
	  }
	  //tf::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	  //double pitch,roll;
	  //btMatrix3x3(q).getEulerYPR(yaw,pitch,roll);
	  return pose;

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "tf_");
	ros::NodeHandle node;
	tf::TransformListener tf(ros::Duration(10));
	RobotPose pose;
	geometry_msgs::Pose robotpose;
	ros::Rate rate(10.0);
	ros::Duration(2).sleep();
	while (ros::ok())
	{
		//Get Current Robot Pose
	     robotpose = pose.getRobotPose();
	     //robotpose.header.stamp = ros::Time::now();
	     ros::Publisher pub = node.advertise<geometry_msgs::Pose>("basic_navigation", 10);
	     pub.publish(robotpose);
	     rate.sleep();
	}

	return 0;
}
