#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>
#include <ocl/Component.hpp>

#include <string>
#include <vector>

#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <brics_actuator/typekit/Types.h>
#include <actionlib/server/simple_action_server.h>
#include <raw_arm_navigation/MoveToCartesianPoseAction.h>
#include <raw_arm_navigation/MoveToJointConfigurationAction.h>
#include <tf/transform_datatypes.h>

#include <boost/ptr_container/ptr_vector.hpp>


using namespace RTT;
using namespace std;

class ArmBridgeRosOrocos: public TaskContext
{
  public:
    ArmBridgeRosOrocos(const string& name);
    virtual ~ArmBridgeRosOrocos();

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    /**
     * @brief Callback that is executed when an action goal to perform a arm movement in joint space comes in.
     * @param joint_cfg_goal Actionlib goal that contains the joint configuration.
     */
    void armJointConfigurationGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction>::GoalHandle joint_cfg_goal);


    /**
	  * @brief Callback that is executed when an action goal to go to a Cartesian pose with the arm comes in.
	  * @param cartesian_pose_goal Actionlib goal that contains the 6DOF pose.
	  */
    void armCartesianPoseWithImpedanceCtrlGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction>::GoalHandle cartesian_pose_goal);

    void writeJointPositionsToPort(brics_actuator::JointPositions brics_joint_positions, std_msgs::Float64MultiArray& orocos_data_array, OutputPort<std_msgs::Float64MultiArray>& output_port);

  private:

    InputPort<brics_actuator::JointPositions> brics_joint_positions;
    OutputPort<std_msgs::Float64MultiArray> orocos_joint_positions;
    OutputPort<std_msgs::Float64MultiArray> orocos_homog_matrix;
    OutputPort<std_msgs::Float64MultiArray> orocos_arm_stiffness;
    OutputPort<std_msgs::Float64MultiArray> orocos_HtipCC;

    brics_actuator::JointPositions m_brics_joint_positions;
    std_msgs::Float64MultiArray m_orocos_joint_positions;
    std_msgs::Float64MultiArray m_orocos_homog_matrix;
    std_msgs::Float64MultiArray m_orocos_arm_stiffness;
    std_msgs::Float64MultiArray m_orocos_HtipCC;

	ros::NodeHandle m_nh;

	/* Action Server */
    actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction> *m_joint_config_as;
    actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction> *m_cartesian_pose_with_impedance_ctrl_as;

    const size_t m_youbot_arm_dof;
};

