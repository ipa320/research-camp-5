#include "arm_bridge_ros_orocos.hpp"

ArmBridgeRosOrocos::ArmBridgeRosOrocos(const string& name) :  TaskContext(name, PreOperational), m_youbot_arm_dof(5)
{
	m_joint_config_as = new actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction > (
      m_nh, "/arm_1/arm_controller/MoveToJointConfigurationDirect", boost::bind(&ArmBridgeRosOrocos::armJointConfigurationGoalCallback, this, _1), false);

	m_cartesian_pose_with_impedance_ctrl_as = new actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction > (
	      m_nh, "/arm_1/arm_controller/MoveToCartesianPoseDirect", boost::bind(&ArmBridgeRosOrocos::armCartesianPoseWithImpedanceCtrlGoalCallback, this, _1), false);

	m_orocos_joint_positions.data.resize(m_youbot_arm_dof, 0.0);
	m_orocos_homog_matrix.data.resize(16, 0.0);
	m_orocos_arm_stiffness.data.resize(9, 0.0);
	m_orocos_HtipCC.data.resize(16, 0.0);

	orocos_joint_positions.setDataSample(m_orocos_joint_positions);
	orocos_homog_matrix.setDataSample(m_orocos_homog_matrix);
	orocos_arm_stiffness.setDataSample(m_orocos_arm_stiffness);
	orocos_HtipCC.setDataSample(m_orocos_HtipCC);

	this->addPort("brics_joint_positions", brics_joint_positions).doc("Input of joint positions in BRICS data types");
	this->addPort("orocos_joint_positions", orocos_joint_positions).doc("Output of joint positions in Orocos data type");
	this->addPort("orocos_homog_matrix", orocos_homog_matrix).doc("Output of a Cartesian Pose as homogeneous coordinates in Orocos data type");
	this->addPort("orocos_arm_stiffness", orocos_arm_stiffness).doc("Output of a arm stiffness to set");
	this->addPort("orocos_HtipCC", orocos_HtipCC).doc("Output of HtipCC to set");
}

ArmBridgeRosOrocos::~ArmBridgeRosOrocos()
{
	delete m_cartesian_pose_with_impedance_ctrl_as;
	delete m_joint_config_as;
}

bool ArmBridgeRosOrocos::configureHook()
{
	return TaskContext::configureHook();
}

bool ArmBridgeRosOrocos::startHook()
{
	m_cartesian_pose_with_impedance_ctrl_as->start();
	m_joint_config_as->start();

	ROS_INFO("arm actions started");

	if (!brics_joint_positions.connected())
	{
		log(Error) << "BRICS joint positions not connected." << endlog();
		return false;
	}

	if (!orocos_joint_positions.connected())
	{
		log(Error) << "Orocos joint positions not connected." << endlog();
		return false;
	}

	if (!orocos_homog_matrix.connected())
	{
		log(Error) << "Orocos homog_matrix not connected." << endlog();
		return false;
	}

	if (!orocos_arm_stiffness.connected())
	{
		log(Error) << "arm stiffness port not connected." << endlog();
		return false;
	}

	return TaskContext::startHook();
}

void ArmBridgeRosOrocos::updateHook()
{
	TaskContext::updateHook();

	ros::spinOnce();

	// check if new joint position arrived at topic
	if (brics_joint_positions.read(m_brics_joint_positions) == NoData)
		return;

	ROS_INFO("received new joint configuration on topic");
	writeJointPositionsToPort(m_brics_joint_positions, m_orocos_joint_positions, orocos_joint_positions);
}

void ArmBridgeRosOrocos::stopHook()
{
	TaskContext::stopHook();
}

void ArmBridgeRosOrocos::cleanupHook()
{
	TaskContext::cleanupHook();
}

void ArmBridgeRosOrocos::writeJointPositionsToPort(brics_actuator::JointPositions brics_joint_positions, std_msgs::Float64MultiArray& orocos_data_array, OutputPort<std_msgs::Float64MultiArray>& output_port)
{
	for (size_t i = 0; i < brics_joint_positions.positions.size(); i++)
		orocos_data_array.data[i] = brics_joint_positions.positions[i].value;

	output_port.write(orocos_data_array);
}

void ArmBridgeRosOrocos::armJointConfigurationGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction>::GoalHandle joint_cfg_goal)
{
	ROS_INFO("MoveToJointConfigurationDirect action called");

	joint_cfg_goal.setAccepted();

	writeJointPositionsToPort(m_brics_joint_positions, m_orocos_joint_positions, orocos_joint_positions);


	// TDB: check if pose is reached


	joint_cfg_goal.setSucceeded();
}

void ArmBridgeRosOrocos::armCartesianPoseWithImpedanceCtrlGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction>::GoalHandle cartesian_pose_goal)
{
	btVector3 trans_vec;
	btQuaternion bt_quat;

	geometry_msgs::PoseStamped goal_pose = cartesian_pose_goal.getGoal()->goal;

	ROS_INFO("MoveToCartesianPoseDirect action called");

	std::cout << "\nx: " << goal_pose.pose.position.x << " y: " << goal_pose.pose.position.y << " z: " << goal_pose.pose.position.z;

	// create rotation matrix from quaternion
	tf::quaternionMsgToTF(goal_pose.pose.orientation, bt_quat);
	btMatrix3x3 rot_mat = btMatrix3x3(bt_quat);

	double r, p, y;
	rot_mat.getRPY(r, p, y);
	std::cout << " -- r: " << r << " p: " << p << " y: " << y << std::endl;

	m_orocos_homog_matrix.data[0] = rot_mat[0][0];
	m_orocos_homog_matrix.data[1] = rot_mat[0][1];
	m_orocos_homog_matrix.data[2] = rot_mat[0][2];
	m_orocos_homog_matrix.data[3] = goal_pose.pose.position.x;
	m_orocos_homog_matrix.data[4] = rot_mat[1][0];
	m_orocos_homog_matrix.data[5] = rot_mat[1][1];
	m_orocos_homog_matrix.data[6] = rot_mat[1][2];
	m_orocos_homog_matrix.data[7] = goal_pose.pose.position.y;
	m_orocos_homog_matrix.data[8] = rot_mat[2][0];
	m_orocos_homog_matrix.data[9] = rot_mat[2][1];
	m_orocos_homog_matrix.data[10] = rot_mat[2][2];
	m_orocos_homog_matrix.data[11] = goal_pose.pose.position.z;
	m_orocos_homog_matrix.data[12] = 0.0;
	m_orocos_homog_matrix.data[13] = 0.0;
	m_orocos_homog_matrix.data[14] = 0.0;
	m_orocos_homog_matrix.data[15] = 1.0;

	m_orocos_arm_stiffness.data[0] = 10;
	m_orocos_arm_stiffness.data[1] = 10;
	m_orocos_arm_stiffness.data[2] = 10;
	m_orocos_arm_stiffness.data[3] = 10;
	m_orocos_arm_stiffness.data[4] = 10;
	m_orocos_arm_stiffness.data[5] = 10;
	m_orocos_arm_stiffness.data[6] = 0;
	m_orocos_arm_stiffness.data[7] = 0;
	m_orocos_arm_stiffness.data[8] = 0;

	/* identity */
	m_orocos_arm_stiffness.data[0] = m_orocos_arm_stiffness.data[5] = m_orocos_arm_stiffness.data[10] = m_orocos_arm_stiffness.data[15] = 1.0;

	std::cout << "write homog matrix to output port" << std::endl;

	cartesian_pose_goal.setAccepted();

	orocos_HtipCC.write(m_orocos_HtipCC);
	orocos_arm_stiffness.write(m_orocos_arm_stiffness);
	orocos_homog_matrix.write(m_orocos_homog_matrix);

	// TDB: check if pose is reached


	cartesian_pose_goal.setSucceeded();

}


ORO_CREATE_COMPONENT( ArmBridgeRosOrocos )
