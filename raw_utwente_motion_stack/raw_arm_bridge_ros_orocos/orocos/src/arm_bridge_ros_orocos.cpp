#include "arm_bridge_ros_orocos.hpp"


ArmBridgeRosOrocos::ArmBridgeRosOrocos(const string& name) :  TaskContext(name, PreOperational), m_youbot_arm_dof(5)
{
	for(size_t i=0; i < m_youbot_arm_dof; ++i)
		m_trajectory_controller.push_back(new JointTrajectoryController);

	m_trajectory_as_srv = new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction > (
	  m_nh, "/arm_1/arm_controller/joint_trajectory_action",
	  boost::bind(&ArmBridgeRosOrocos::armJointTrajectoryGoalCallback, this, _1),
	  boost::bind(&ArmBridgeRosOrocos::armJointTrajectoryCancelCallback, this, _1), false);

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

	// Ports
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
	delete m_trajectory_as_srv;
}

bool ArmBridgeRosOrocos::configureHook()
{
	//Operations
	if(this->getPeer("executive"))
	{
		std::cout << "get operation jointspaceControl" << std::endl;
		m_joint_space_ctrl_op = this->getPeer("executive")->getOperation("jointspaceControl");

		std::cout << "get operation useArmOnly" << std::endl;
		m_use_arm_only_op = this->getPeer("executive")->getOperation("useArmOnly");

		std::cout << "get operation cartesianControl" << std::endl;
		m_cartesian_ctrl_op = this->getPeer("executive")->getOperation("cartesianControl");

		std::cout << "get operation gravityCompensation" << std::endl;
		m_gravity_compensation_ctrl_op = this->getPeer("executive")->getOperation("gravityCompensation");

		std::cout << "get operation execute" << std::endl;
		m_execute_op = this->getPeer("executive")->getOperation("execute");
	}

	return TaskContext::configureHook();
}

bool ArmBridgeRosOrocos::startHook()
{
	m_cartesian_pose_with_impedance_ctrl_as->start();
	m_joint_config_as->start();
	m_trajectory_as_srv->start();

	m_arm_has_active_joint_trajectory_goal = false;

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

	// check if trajectory controller is finished
	bool areTrajectoryControllersDone = true;

	for (size_t i = 0; i < m_youbot_arm_dof; ++i)
	{
		if (m_trajectory_controller[i].isTrajectoryControllerActive())
		{
			areTrajectoryControllersDone = false;
			break;
		}
	}

	if (areTrajectoryControllersDone && m_arm_has_active_joint_trajectory_goal)
	{
		m_arm_has_active_joint_trajectory_goal = false;
		control_msgs::FollowJointTrajectoryResult trajectoryResult;
		trajectoryResult.error_code = trajectoryResult.SUCCESSFUL;
		m_arm_active_joint_trajectory_goal.setSucceeded(trajectoryResult, "trajectory successful");
	}


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

void ArmBridgeRosOrocos::armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbot_arm_goal)
{
	trajectory_msgs::JointTrajectory trajectory = youbot_arm_goal.getGoal()->trajectory;

	// validate that the correct number of joints is provided in the goal
	if (trajectory.joint_names.size() != m_youbot_arm_dof)
	{
		log(Error) << "Trajectory is malformed! Goal has " << trajectory.joint_names.size() << " joint names, but only " << m_youbot_arm_dof << " joints are supported" << endlog();
		youbot_arm_goal.setRejected();
		return;
	}

	std::vector<JointTrajectory> jointTrajectories(m_youbot_arm_dof);

	// convert from the ROS trajectory representation to the controller's representation
	std::vector<std::vector< double > > positions(m_youbot_arm_dof);
	std::vector<std::vector< double > > velocities(m_youbot_arm_dof);
	std::vector<std::vector< double > > accelerations(m_youbot_arm_dof);
	TrajectorySegment segment;
	for (size_t i = 0; i < trajectory.points.size(); i++)
	{
		trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
		// validate the trajectory point
		if ((point.positions.size() != m_youbot_arm_dof
						|| point.velocities.size() != m_youbot_arm_dof
						|| point.accelerations.size() != m_youbot_arm_dof))
		{
			log(Error) << "A trajectory point is malformed! " << m_youbot_arm_dof << " positions, velocities and accelerations must be provided" << endlog();
			youbot_arm_goal.setRejected();
			return;
		}

		for (size_t j = 0; j < m_youbot_arm_dof; j++)
		{
			segment.positions = point.positions[j];
			segment.velocities = point.velocities[j];
			segment.accelerations = point.accelerations[j];
			segment.time_from_start = boost::posix_time::microsec(point.time_from_start.toNSec()/1000);
			jointTrajectories[j].segments.push_back(segment);
		}
	}

	for (size_t j = 0; j < m_youbot_arm_dof; j++)
	{
        jointTrajectories[j].start_time = boost::posix_time::microsec_clock::local_time(); //TODO is this correct to set the trajectory start time to now
	}


	// cancel the old goal
	/*
	if (m_arm_has_active_joint_trajectory_goal)
	{
		m_arm_active_joint_trajectory_goal.setCanceled();
		m_arm_has_active_joint_trajectory_goal = false;
		for (int i = 0; i < m_youbot_arm_dof; ++i)
		{
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).cancelTrajectory();
		}
	}
	 */

	// replace the old goal with the new one
	youbot_arm_goal.setAccepted();
	m_arm_active_joint_trajectory_goal = youbot_arm_goal;
	m_arm_has_active_joint_trajectory_goal = true;


	// send the trajectory to the controller
	for (size_t i = 0; i < m_youbot_arm_dof; ++i)
	{
		try
		{
			// youBot joints start with 1 not with 0 -> i + 1
			m_trajectory_controller[i].setTrajectory(jointTrajectories[i]);
			log(Info) << "set trajectories " << i << endlog();
		} catch (std::exception& e)
		{
			std::string errorMessage = e.what();
			log(Warning) << "Cannot set trajectory for joint " << i + 1 << "\n " << errorMessage.c_str() << endlog();
		}
	}
	log(Info) << "set all trajectories" << endlog();
}

void ArmBridgeRosOrocos::armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbot_arm_goal)
{
	// stop the controller
	for (size_t i = 0; i < m_youbot_arm_dof; ++i)
	{
		try
		{
			m_trajectory_controller[i].cancelCurrentTrajectory();
		} catch (std::exception& e)
		{
			std::string errorMessage = e.what();
			log(Warning) << "Cannot stop joint " << i + 1 << "\n" << errorMessage.c_str();
		}
	}

	if (m_arm_active_joint_trajectory_goal == youbot_arm_goal)
	{
		// Marks the current goal as canceled.
		youbot_arm_goal.setCanceled();
		m_arm_has_active_joint_trajectory_goal = false;
	}
}

void ArmBridgeRosOrocos::writeJointPositionsToPort(brics_actuator::JointPositions brics_joint_positions, std_msgs::Float64MultiArray& orocos_data_array, OutputPort<std_msgs::Float64MultiArray>& output_port)
{
	for (size_t i = 0; i < brics_joint_positions.positions.size(); i++)
	{	
		std::cout << brics_joint_positions.positions[i].value << " ";
		orocos_data_array.data[i] = brics_joint_positions.positions[i].value;
	}

	output_port.write(orocos_data_array);
}

void ArmBridgeRosOrocos::armJointConfigurationGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction>::GoalHandle joint_cfg_goal)
{
	ROS_INFO("MoveToJointConfigurationDirect action called");

	joint_cfg_goal.setAccepted();

	if(!m_joint_space_ctrl_op.ready() || !m_use_arm_only_op.ready() || !m_execute_op.ready())
	{
		std::cout << "operation(s) not available" << std::endl;
		joint_cfg_goal.setAborted();
		return;
	}


	std::cout << std::endl << std::endl;
	writeJointPositionsToPort(joint_cfg_goal.getGoal()->goal, m_orocos_joint_positions, orocos_joint_positions);
	std::cout << std::endl << std::endl;

	m_joint_space_ctrl_op();
	m_use_arm_only_op();
	m_execute_op();

	
	// TDB: check if pose is reached


	joint_cfg_goal.setSucceeded();
}

void ArmBridgeRosOrocos::armCartesianPoseWithImpedanceCtrlGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction>::GoalHandle cartesian_pose_goal)
{
	btVector3 trans_vec;
	btQuaternion bt_quat;

	ROS_INFO("MoveToCartesianPoseDirect action called");

	geometry_msgs::PoseStamped goal_pose = cartesian_pose_goal.getGoal()->goal;

	cartesian_pose_goal.setAccepted();


	if(!m_gravity_compensation_ctrl_op.ready() || !m_execute_op.ready())
	{
		std::cout << "operation(s) not available" << std::endl;
		cartesian_pose_goal.setAborted();
		return;
	}


	std::cout << "\nx: " << goal_pose.pose.position.x << " y: " << goal_pose.pose.position.y << " z: " << goal_pose.pose.position.z << std::endl;

	double qw, qx, qy, qz;
	qx = goal_pose.pose.orientation.x;
	qy = goal_pose.pose.orientation.y;
	qz = goal_pose.pose.orientation.z;
	qw = goal_pose.pose.orientation.w;

	m_orocos_homog_matrix.data[0] = pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2);
	m_orocos_homog_matrix.data[1] = 2*qx*qy - 2*qz*qw;
	m_orocos_homog_matrix.data[2] = 2*qx*qz + 2*qy*qw;
	m_orocos_homog_matrix.data[3] = goal_pose.pose.position.x;
	m_orocos_homog_matrix.data[4] = 2*qx*qy + 2*qz*qw;
	m_orocos_homog_matrix.data[5] = pow(qw,2) - pow(qx,2) + pow(qy,2) - pow(qz,2);
	m_orocos_homog_matrix.data[6] = 2*qy*qz - 2*qx*qw;
	m_orocos_homog_matrix.data[7] = goal_pose.pose.position.y;
	m_orocos_homog_matrix.data[8] = 2*qx*qz - 2*qy*qw;
	m_orocos_homog_matrix.data[9] = 2*qy*qz + 2*qx*qw;
	m_orocos_homog_matrix.data[10] = pow(qw,2) - pow(qx,2) - pow(qy,2) + pow(qz,2);
	m_orocos_homog_matrix.data[11] = goal_pose.pose.position.z;
	m_orocos_homog_matrix.data[12] = 0.0;
	m_orocos_homog_matrix.data[13] = 0.0;
	m_orocos_homog_matrix.data[14] = 0.0;
	m_orocos_homog_matrix.data[15] = 1.0;

	m_orocos_arm_stiffness.data[0] = 200;
	m_orocos_arm_stiffness.data[1] = 200;
	m_orocos_arm_stiffness.data[2] = 200;
	m_orocos_arm_stiffness.data[3] = 10;
	m_orocos_arm_stiffness.data[4] = 10;
	m_orocos_arm_stiffness.data[5] = 10;
	m_orocos_arm_stiffness.data[6] = 0;
	m_orocos_arm_stiffness.data[7] = 0;
	m_orocos_arm_stiffness.data[8] = 0;


	// identity
	//m_orocos_HtipCC.data[0] = m_orocos_arm_stiffness.data[5] = m_orocos_arm_stiffness.data[10] = m_orocos_arm_stiffness.data[15] = 1.0;

	std::cout << "write homog matrix to output port" << std::endl;

	//orocos_HtipCC.write(m_orocos_HtipCC);
	orocos_arm_stiffness.write(m_orocos_arm_stiffness);
	orocos_homog_matrix.write(m_orocos_homog_matrix);


	m_cartesian_ctrl_op();
	m_execute_op();



	// TDB: check if pose is reached


	cartesian_pose_goal.setSucceeded();

}


ORO_CREATE_COMPONENT( ArmBridgeRosOrocos )
