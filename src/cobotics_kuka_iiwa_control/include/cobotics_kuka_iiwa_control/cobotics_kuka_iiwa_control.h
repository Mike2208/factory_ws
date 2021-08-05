#ifndef COBOTICS_KUKA_IIWA_CONTROL_H
#define COBOTICS_KUKA_IIWA_CONTROL_H

#include <ros/ros.h>

#include <future>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/utils.h>

class CoboticsKukaIIWAControl
{
	public:
		struct LinkTarget
		{
			std::string Name;
			double Target;
		};

		static constexpr double TolerancePose = 0.01;
		static constexpr double ToleranceAngle = 0.01;

		CoboticsKukaIIWAControl();

		moveit::planning_interface::MoveGroupInterface::Plan PlanPose(const geometry_msgs::PoseStamped &target,
		                                                              double tolerancePose = 0.01,
		                                                              double toleranceAngle = 0.01);

		void PlanExecutePose(const geometry_msgs::PoseStamped &target,
		                     double tolerancePose = 0.01,
		                     double toleranceAngle = 0.01);

		void PlanExecutePoseAsync(const geometry_msgs::PoseStamped &target,
		                          double tolerancePose = 0.01,
		                          double toleranceAngle = 0.01);

		std::vector<LinkTarget> CalcJointPoses(const geometry_msgs::PoseStamped &target);

		void Execute(const moveit::planning_interface::MoveGroupInterface::Plan &plan);

		void Run();
		void RunCalcJointPoses();
		void Shutdown();

		void FollowObject(const std::string &objectFrame, const tf2::Transform &offset);

		void FollowObjectUpdate(const std::string &objectFrame, const tf2::Transform &offset);

		//void IKFollowObject(const std::string &objectFrame, const tf2::Transform &offset);
		void IKFollowObject(geometry_msgs::PoseStamped &target);

	private:
		ros::NodeHandle _nh;

		std::string _planningGroup;

		moveit::planning_interface::MoveGroupInterface _moveGroup;
		moveit::planning_interface::PlanningSceneInterface _planningScene;

		ros::Subscriber _moveSub;

		const moveit::core::JointModelGroup *_jointModelGroup = nullptr;

		volatile bool _newMoveRequest = false;
		geometry_msgs::PoseStamped _moveRequestPose;

		ros::ServiceClient _ikRequest;
		ros::Publisher _jointStatesPub;

		/*!
		 * \brief Make sure only one move operation is requested
		 */
		std::mutex _lockMove;
		void MoveSubCallback(const geometry_msgs::PoseStamped &pose);

		std::future<bool> _followTrajectory;

		volatile bool _planUpdated = false;
		moveit::planning_interface::MoveGroupInterface::Plan _plan;
		bool followTrajectory();
};


//#include <moveit/moveit_cpp/moveit_cpp.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/plan_execution/plan_execution.h>
//#include <moveit_ros_planning/PlanExecutionDynamicReconfigureConfig.h>

//class CoboticsKukaIIWAControl
//{
//	public:
//		void InitRos(char **argv, int argc, const std::string &name);
//		void Init();

//		moveit_msgs::MotionPlanResponse PlanPose(const geometry_msgs::PoseStamped &target,
//		              const std::vector<double> &tolerancePose, const std::vector<double> &toleranceAngle);

//	private:
//		ros::NodeHandle _nh;

//		std::string _endEffectorLink;

//		/*!
//		 * \brief Robot Model
//		 */
//		moveit::core::RobotModelPtr _robotModel;

//		moveit::core::RobotStatePtr _robotState;

//		const moveit::core::JointModelGroup *_jointModelGroup;

//		/*!
//		 * \brief Planning Scene for robot
//		 */
//		planning_scene::PlanningScenePtr _planningScene;

//		/*!
//		 * \brief Planner
//		 */
//		planning_interface::PlannerManagerPtr _plannerInstance;
//};

#endif // COBOTICS_KUKA_IIWA_CONTROL_H
