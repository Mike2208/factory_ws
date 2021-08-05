#ifndef IIWA_ROBOT_HW_SIM_H
#define IIWA_ROBOT_HW_SIM_H

#include <gazebo/gazebo.hh>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <gazebo_ros_control/default_robot_hw_sim.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>


class IIWARobotHWSim
        : public gazebo_ros_control::RobotHWSim
{
		// Methods used to control a joint.
		enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

		struct ExtraJointData
		{
			double Pos;
			double Vel;
			double Eff;

			double PosCmd;
			double PosLowLim;
			double PosUppLim;
			double EffLim;

			double EStopTarget = 0;

			ControlMethod CtrlMeth = ControlMethod::EFFORT;
			int Type = urdf::Joint::PRISMATIC;

			control_toolbox::Pid EffPID;
			gazebo::physics::JointPtr GzJoint;

			ExtraJointData() = default;
		};

	public:
		virtual ~IIWARobotHWSim() = default;

		bool initSim(
		    const std::string& robot_namespace,
		    ros::NodeHandle model_nh,
		    gazebo::physics::ModelPtr parent_model,
		    const urdf::Model *const urdf_model,
		    std::vector<transmission_interface::TransmissionInfo> transmissions) override;

		virtual void readSim(ros::Time time, ros::Duration period) override;
		virtual void writeSim(ros::Time time, ros::Duration period) override;
		virtual void eStopActive(const bool active) override;

//		void read(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;
//		void write(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

	private:
		bool _eStopActive = false;
		std::array<double, 7> _eStopPosTargets = {0,0,0,0,0,0,0};

//		std::array<urdf::Vector3, 6> _linkToJoint;
//		std::array<urdf::Vector3, 6> _jointToLink;

		std::array<control_toolbox::Pid, 7> _jPosPID;

		using pos_vector_t = ignition::math::Vector3<double>;
//		std::array<pos_vector_t, 6> _gzLinkToJoint;
//		std::array<pos_vector_t, 6> _gzJointToLink;

		hardware_interface::JointStateInterface    _jsInterface;
		hardware_interface::PositionJointInterface _pjInterface;
		hardware_interface::EffortJointInterface   _ejInterface;

		joint_limits_interface::PositionJointSaturationInterface _pjSatInterface;
		joint_limits_interface::PositionJointSoftLimitsInterface _pjLimInterface;
		joint_limits_interface::EffortJointSaturationInterface _ejSatInterface;
		joint_limits_interface::EffortJointSoftLimitsInterface _ejLimInterface;

		std::array<double, 7> _jPos;
		std::array<double, 7> _jVel;
		std::array<double, 7> _jEff;

		std::array<double, 7> _jPosCmd;
		std::array<double, 7> _jPosLowLim;
		std::array<double, 7> _jPosUppLim;
		std::array<double, 7> _jEffLim;

		std::string _baseLinkName;
		std::string _physicsType;
		gazebo::physics::ModelPtr _model;
		gazebo::physics::WorldPtr _world;
		std::array<gazebo::physics::LinkPtr, 8>  _gzLinks;
		std::array<gazebo::physics::JointPtr, 7> _gzJoints;

		std::list<ExtraJointData> _extraJointData;

		bool registerJointInterfaces(const std::string& robot_namespace,
		                             ros::NodeHandle model_nh,
		                             gazebo::physics::ModelPtr parent_model,
		                             const urdf::Model *const urdf_model,
		                             const transmission_interface::TransmissionInfo &transmission,
		                             const std::string &joint_name,
		                             std::size_t joint_index);

		bool registerExtraJointInterfaces(const std::string& robot_namespace,
		                                  ros::NodeHandle model_nh,
		                                  gazebo::physics::ModelPtr parent_model,
		                                  const urdf::Model *const urdf_model,
		                                  const transmission_interface::TransmissionInfo &transmission,
		                                  const std::string &joint_name,
		                                  ExtraJointData *pJointData);

		// Register the limits of the joint specified by joint_name and joint_handle. The limits are
		// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
		// Return the joint's type, lower position limit, upper position limit, and effort limit.
		void registerJointLimits(const std::string& joint_name,
		                         const hardware_interface::JointHandle& joint_handle,
		                         const ControlMethod ctrl_method,
		                         const ros::NodeHandle& joint_limit_nh,
		                         const urdf::Model *const urdf_model, int *const joint_type,
		                         double *const lower_limit, double *const upper_limit,
		                         double *const effort_limit);

		void writeExtraJoints(ros::Time time, ros::Duration period);
};

#endif // IIWA_ROBOT_HW_SIM_H
