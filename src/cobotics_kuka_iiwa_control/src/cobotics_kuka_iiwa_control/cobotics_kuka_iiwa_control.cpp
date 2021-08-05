#include "cobotics_kuka_iiwa_control/cobotics_kuka_iiwa_control.h"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <mutex>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <moveit_msgs/GetPositionIK.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

/***********************************************************************************************************
 * Most of the code was taken from the MoveIt Tutorial at
 * https://ros-planning.github.io/moveit_tutorials/doc/motion_planning_pipeline/motion_planning_pipeline_tutorial.html
 ***********************************************************************************************************/

std::string initPlanningGroup(ros::NodeHandle &nh)
{
	std::string planningGroup;
	nh.param("planning_group", planningGroup, std::string("move_group"));

	return planningGroup;
}

CoboticsKukaIIWAControl::CoboticsKukaIIWAControl()
    : _nh(ros::NodeHandle()),
      _planningGroup(initPlanningGroup(this->_nh)),
      _moveGroup(moveit::planning_interface::MoveGroupInterface(this->_planningGroup))
{
	{
		// Run a spinner to get joint data
		ros::AsyncSpinner spinner(1);
		spinner.start();

		// Raw pointers are frequently used to refer to the planning group for improved performance.
		this->_jointModelGroup = this->_moveGroup.getCurrentState()->getJointModelGroup(this->_planningGroup);

		this->_moveGroup.allowReplanning(true);
	}

	std::string moveTopic;
	this->_nh.param("move_topic", moveTopic, std::string("move"));
	this->_moveSub = this->_nh.subscribe(moveTopic, 10, &CoboticsKukaIIWAControl::MoveSubCallback, this);

	this->_ikRequest = this->_nh.serviceClient<moveit_msgs::GetPositionIK>("/iiwa/compute_ik");
	this->_jointStatesPub = this->_nh.advertise<trajectory_msgs::JointTrajectory>("/iiwa/move_arm_controller/command", 10);
}

moveit::planning_interface::MoveGroupInterface::Plan CoboticsKukaIIWAControl::PlanPose(const geometry_msgs::PoseStamped &target,
                                                                                       double tolerancePose, double toleranceAngle)
{
//	this->_moveGroup.clearPathConstraints();
//	moveit_msgs::Constraints poseGoal = kinematic_constraints::constructGoalConstraints(this->_moveGroup.getEndEffectorLink(), target, tolerancePose, toleranceAngle);
//	this->_moveGroup.setPathConstraints(poseGoal);

	this->_moveGroup.setGoalPositionTolerance(tolerancePose);
	this->_moveGroup.setGoalOrientationTolerance(toleranceAngle);

	this->_moveGroup.setPoseTarget(target);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	const auto res = this->_moveGroup.plan(plan);
	if(res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_STREAM("Failed to plan path. Returned error code " << res);
	}

	return plan;
}

void CoboticsKukaIIWAControl::PlanExecutePose(const geometry_msgs::PoseStamped &target,
                                              double tolerancePose, double toleranceAngle)
{
	this->_moveGroup.setGoalPositionTolerance(tolerancePose);
	this->_moveGroup.setGoalOrientationTolerance(toleranceAngle);

	this->_moveGroup.setPoseTarget(target);

	{
		std::lock_guard<std::mutex> lock(this->_lockMove);
		const auto res = this->_moveGroup.move();

		if(res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_ERROR_STREAM("Failed to plan and execute path. Returned error code " << res);
		}
	}
}

void CoboticsKukaIIWAControl::PlanExecutePoseAsync(const geometry_msgs::PoseStamped &target, double tolerancePose, double toleranceAngle)
{
	this->_moveGroup.setGoalPositionTolerance(tolerancePose);
	this->_moveGroup.setGoalOrientationTolerance(toleranceAngle);

	this->_moveGroup.setPoseTarget(target);

	{
		std::lock_guard<std::mutex> lock(this->_lockMove);
		const auto res = this->_moveGroup.asyncMove();

		if(res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_ERROR_STREAM("Failed to start async path execution. Returned error code " << res);
		}
	}
}

std::vector<CoboticsKukaIIWAControl::LinkTarget> CoboticsKukaIIWAControl::CalcJointPoses(const geometry_msgs::PoseStamped &target)
{
	this->_moveGroup.setPoseTarget(target);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	const auto res = this->_moveGroup.plan(plan);
	if(res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_STREAM("Failed plan path for joint targets. Returned error code " << res);
	}

	std::vector<double> jVal;
	this->_moveGroup.getJointValueTarget(jVal);

	auto curState = this->_moveGroup.getCurrentState();
	curState->setFromIK(this->_jointModelGroup, target.pose);

	std::vector<double> rVal;
	for(const auto &jModel : this->_jointModelGroup->getJointModels())
	{
		rVal.push_back(*(curState->getJointPositions(jModel)));
	}

	std::vector<LinkTarget> ret;
	ret.reserve(plan.trajectory_.joint_trajectory.joint_names.size());
	for(size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); ++i)
	{
		ret.push_back(LinkTarget({plan.trajectory_.joint_trajectory.joint_names[i],
		                          plan.trajectory_.joint_trajectory.points.back().positions.at(i)}));

		ROS_ERROR_STREAM(ret.back().Name << " " << ret.back().Target << "\t" << jVal.at(i) << "\t" << rVal.at(i));
	}

	return ret;
}

void CoboticsKukaIIWAControl::Execute(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
	std::lock_guard<std::mutex> lock(this->_lockMove);
	const auto res = this->_moveGroup.execute(plan);
	if(res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_STREAM("Failed to plan and execute path. Returned error code " << res);
	}
}

void CoboticsKukaIIWAControl::Run()
{
	ros::AsyncSpinner spinner(1);

	ros::Rate rate(60);
	while(ros::ok())
	{
		if(this->_newMoveRequest)
		{
			spinner.start();

			this->PlanExecutePose(this->_moveRequestPose, TolerancePose, ToleranceAngle);

			this->_newMoveRequest = false;

			spinner.stop();
		}

		ros::spinOnce();
		rate.sleep();
	}
}

void CoboticsKukaIIWAControl::RunCalcJointPoses()
{
	ros::AsyncSpinner spinner(1);

	ros::Rate rate(60);
	while(ros::ok())
	{
		if(this->_newMoveRequest)
		{
			spinner.start();

			this->CalcJointPoses(this->_moveRequestPose);

			this->_newMoveRequest = false;

			spinner.stop();
		}

		ros::spinOnce();
		rate.sleep();
	}
}

void CoboticsKukaIIWAControl::Shutdown()
{
}

void CoboticsKukaIIWAControl::FollowObject(const std::string &objectFrame, const tf2::Transform &offset)
{
	ros::AsyncSpinner spinner(1);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped objTf;
	geometry_msgs::PoseStamped objPose;

	// Wait for TFs
	tfBuffer.lookupTransform("map", objectFrame, ros::Time(0), ros::Duration(1));

	spinner.start();

	ros::Rate rate(60);
	while(ros::ok())
	{
		const auto newObjTfMsg = tfBuffer.lookupTransform("map", objectFrame, ros::Time(0));
		tf2::Transform newObjTf(tf2::Quaternion(newObjTfMsg.transform.rotation.w,
		                                        newObjTfMsg.transform.rotation.x,
		                                        newObjTfMsg.transform.rotation.y,
		                                        newObjTfMsg.transform.rotation.z),
		                        tf2::Vector3(newObjTfMsg.transform.translation.x,
		                                     newObjTfMsg.transform.translation.y,
		                                     newObjTfMsg.transform.translation.z));

		if(newObjTf.getOrigin().distance2(tf2::Vector3(objTf.transform.translation.x,
		                                               objTf.transform.translation.y,
		                                               objTf.transform.translation.z)) > 0.01*0.01)
		{
			ROS_ERROR_STREAM("MOVING ARM");

			objTf = newObjTfMsg;

			const tf2::Quaternion objQuat(objTf.transform.rotation.w, objTf.transform.rotation.x,
			                              objTf.transform.rotation.y, objTf.transform.rotation.z);

			tf2::Quaternion targetRot = offset.getRotation() * objQuat;
			targetRot.setRPY(0, -1.57, 0);

			objPose.header = objTf.header;
			objPose.pose.position.x = offset.getOrigin().x() + objTf.transform.translation.x;
			objPose.pose.position.y = offset.getOrigin().y() + objTf.transform.translation.y;
			objPose.pose.position.z = offset.getOrigin().z() + objTf.transform.translation.z;
			objPose.pose.orientation.w = targetRot.w();
			objPose.pose.orientation.x = targetRot.x();
			objPose.pose.orientation.y = targetRot.y();
			objPose.pose.orientation.z = targetRot.z();

//			objPose.pose.position.x = 0;
//			objPose.pose.position.y = 0;
//			objPose.pose.position.z = 1.5;
//			objPose.pose.orientation.w = 1;
//			objPose.pose.orientation.x = 0;
//			objPose.pose.orientation.y = 0;
//			objPose.pose.orientation.z = 0;

			this->_moveGroup.setGoalOrientationTolerance(2.0*M_PI);

			this->CalcJointPoses(objPose);
			this->PlanExecutePoseAsync(objPose, TolerancePose, ToleranceAngle);

			//this->_newMoveRequest = false;
		}

		rate.sleep();
	}
}

void CoboticsKukaIIWAControl::FollowObjectUpdate(const std::string &objectFrame, const tf2::Transform &offset)
{
	ros::AsyncSpinner spinner(1);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped objTf;
	geometry_msgs::PoseStamped objPose;
	tf2::Quaternion objQuat, targetRos;

	moveit::planning_interface::MoveGroupInterface::Plan plan;

	ros::Rate rate(60);
	while(ros::ok())
	{
		if(this->_newMoveRequest)
		{
			spinner.start();

			objTf = tfBuffer.lookupTransform(objectFrame, "map",
			                                 ros::Time(0));

			objQuat = tf2::Quaternion(objTf.transform.rotation.w, objTf.transform.rotation.x,
			                              objTf.transform.rotation.y, objTf.transform.rotation.z);

			const tf2::Quaternion targetRot = offset.getRotation() * objQuat;

			objPose.header = objTf.header;
			objPose.pose.position.x = offset.getOrigin().x() + objTf.transform.translation.x;
			objPose.pose.position.y = offset.getOrigin().y() + objTf.transform.translation.y;
			objPose.pose.position.z = offset.getOrigin().z() + objTf.transform.translation.z;
			objPose.pose.orientation.w = targetRot.w();
			objPose.pose.orientation.x = targetRot.x();
			objPose.pose.orientation.y = targetRot.y();
			objPose.pose.orientation.z = targetRot.z();

			this->_moveGroup.plan(plan);

			this->_newMoveRequest = false;

			spinner.stop();
		}

		ros::spinOnce();
		rate.sleep();
	}
}

const std::array<std::string, 7> validJoints = {
    "iiwa_joint_1",
    "iiwa_joint_2",
    "iiwa_joint_3",
    "iiwa_joint_4",
    "iiwa_joint_5",
    "iiwa_joint_6",
    "iiwa_joint_7"
};

void CoboticsKukaIIWAControl::IKFollowObject(geometry_msgs::PoseStamped &target)
{
	ros::AsyncSpinner spinner(1);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped objTf;
	geometry_msgs::PoseStamped objPose;
	tf2::Quaternion objQuat, targetRos;

//	while(debugWait)
//	{}

	spinner.start();

	ros::Rate rate(200);
	while(ros::ok())
	{
		target.header.stamp = ros::Time(0); //::now();
		objPose = tfBuffer.transform(target, this->_jointModelGroup->getSolverInstance()->getBaseFrame(), ros::Duration(1));

		// TODO: Fake Pose inputs. REMOVE
		{
			objPose.pose.position.x = 0.1;

			tf2::Quaternion quat(0., -M_PI/2.0, 0.);
			objPose.pose.orientation.w = quat.w();
			objPose.pose.orientation.x = quat.x();
			objPose.pose.orientation.y = quat.y();
			objPose.pose.orientation.z = quat.z();
		}

		// Copy current robot state
		moveit::core::RobotState robState = *this->_moveGroup.getCurrentState();

		// Compute inverse kinematics for requested pose
		moveit_msgs::GetPositionIK ik;
		ik.request.ik_request.group_name = this->_planningGroup;
		const auto res = robState.setFromIK(this->_jointModelGroup, objPose.pose, 0.05);
//		ROS_ERROR_STREAM("IK RESULT: " << res);
//		ROS_ERROR_STREAM("POSE: " << objPose);
//		ROS_ERROR_STREAM("TARGET: " << target);
//		ROS_ERROR_STREAM("TIME: " << ros::Time::now());

		// If solution found, move to joint positions
		if(res)
		{
			sensor_msgs::JointState jState;
			moveit::core::robotStateToJointStateMsg(robState, jState);

			// Create trajectory message with the new pose as only traj. point
			trajectory_msgs::JointTrajectory jTrack;
			jTrack.header = jState.header;
			jTrack.points.push_back(trajectory_msgs::JointTrajectoryPoint());
			jTrack.points[0].time_from_start = ros::Duration(0.01);

			jTrack.header.frame_id = "";

			const auto &jGroup = robState.getJointModelGroup(this->_planningGroup);
			const auto vCount = jGroup->getVariableCount();
			jTrack.joint_names.resize(vCount);
			jTrack.points[0].effort.resize(vCount, 0);
			jTrack.points[0].positions.resize(vCount, 0);
			jTrack.points[0].velocities.resize(vCount, 0);
			jTrack.points[0].accelerations.resize(vCount, 0);

			// Set joint names and target positions
			for(size_t i=0; i<vCount; ++i)
			{
				jTrack.joint_names[i] = robState.getVariableNames()[jGroup->getVariableIndexList()[i]];
				jTrack.points[0].positions[i] = robState.getVariablePositions()[jGroup->getVariableIndexList()[i]];
			}

			this->_jointStatesPub.publish(jTrack);
		}

		rate.sleep();
	}
}

void CoboticsKukaIIWAControl::MoveSubCallback(const geometry_msgs::PoseStamped &pose)
{
	this->_newMoveRequest = true;
	this->_moveRequestPose = pose;
}

bool CoboticsKukaIIWAControl::followTrajectory()
{
	ros::Rate rate(60);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan = this->_plan;

	const auto startTime = ros::Time::now();

	std::size_t pIndex = 0;
	while(ros::ok())
	{
		const auto dt = ros::Time::now() - startTime;

		if(!plan.trajectory_.joint_trajectory.points.empty())
		{
			auto pIt = plan.trajectory_.joint_trajectory.points.end();
			for(auto pointIt = plan.trajectory_.joint_trajectory.points.begin() + pIndex;
			    pointIt != plan.trajectory_.joint_trajectory.points.end();
			    ++pointIt)
			{
				if(dt >= pointIt->time_from_start)
					pIt = pointIt;
				else
					break;
			}

			if(pIt != plan.trajectory_.joint_trajectory.points.end())
			{

			}
		}
	}
}


//void CoboticsKukaIIWAControl::Init()
//{
//	this->_nh = ros::NodeHandle();

//	std::string planningGroup, robotDescription;
//	this->_nh.param("planning_group", planningGroup, std::string("move_group"));
//	this->_nh.param("robot_description", robotDescription, std::string("robot_description"));

//	if(!this->_nh.getParam("end_effector_link", this->_endEffectorLink))
//		ROS_FATAL_STREAM_NAMED(this->_nh.getNamespace(), "Could not find end effector_link param");

//	robot_model_loader::RobotModelLoader robot_model_loader(robotDescription);
//	this->_robotModel = robot_model_loader.getModel();
//	/* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
//	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(this->_robotModel));
//	this->_jointModelGroup = robot_state->getJointModelGroup(planningGroup);

//	// Using the :moveit_core:`RobotModel`, we can construct a :planning_scene:`PlanningScene`
//	// that maintains the state of the world (including the robot).
//	this->_planningScene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(this->_robotModel));

//	// Configure a valid robot state
//	this->_planningScene->getCurrentStateNonConst().setToDefaultValues(this->_jointModelGroup, "ready");

//	// We will now construct a loader to load a planner, by name.
//	// Note that we are using the ROS pluginlib library here.
//	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

//	// We will get the name of planning plugin we want to load
//	// from the ROS parameter server, and then load the planner
//	// making sure to catch all exceptions.
//	std::string plannerPluginName;
//	if(!this->_nh.getParam("planning_plugin", plannerPluginName))
//		ROS_FATAL_STREAM("Could not find planning_plugin name param");
//	try
//	{
//		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
//		                                "moveit_core", "planning_interface::PlannerManager"));
//	}
//	catch (pluginlib::PluginlibException& ex)
//	{
//		ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
//	}
//	try
//	{
//		this->_plannerInstance.reset(planner_plugin_loader->createUnmanagedInstance(plannerPluginName));
//		if (!this->_plannerInstance->initialize(this->_robotModel, this->_nh.getNamespace()))
//			ROS_FATAL_STREAM("Could not initialize planner instance");
//		ROS_INFO_STREAM("Using planning interface '" << this->_plannerInstance->getDescription() << "'");
//	}
//	catch (pluginlib::PluginlibException& ex)
//	{
//		const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
//		std::stringstream ss;
//		for (const auto& cls : classes)
//			ss << cls << " ";
//		ROS_ERROR_STREAM("Exception while loading planner '" << plannerPluginName << "': " << ex.what() << std::endl
//		                 << "Available plugins: " << ss.str());
//	}
//}

//moveit_msgs::MotionPlanResponse CoboticsKukaIIWAControl::PlanPose(const geometry_msgs::PoseStamped &target,
//                                                                  const std::vector<double> &tolerancePose, const std::vector<double> &toleranceAngle)
//{
//	moveit_msgs::Constraints poseGoal = kinematic_constraints::constructGoalConstraints(this->_endEffectorLink, target, tolerancePose, toleranceAngle);

//	// We now construct a planning context that encapsulate the scene,
//	// the request and the response. We call the planner using this
//	// planning context
//	planning_interface::MotionPlanResponse res;
//	planning_interface::MotionPlanRequest req;
//	req.group_name = this->_planningScene->getName();
//	req.goal_constraints.push_back(poseGoal);

//	planning_interface::PlanningContextPtr context = this->_plannerInstance->getPlanningContext(this->_planningScene, req, res.error_code_);
//	context->solve(res);
//	if (res.error_code_.val != res.error_code_.SUCCESS)
//	{
//		ROS_ERROR("Could not compute plan successfully");
//		throw std::runtime_error("Could not compute plan successfully");
//	}

//	moveit_msgs::MotionPlanResponse mp;
//	res.getMessage(mp);

//	return mp;
//}
