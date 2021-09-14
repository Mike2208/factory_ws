#ifndef TELEPORT_GZ_PLUGIN_H
#define TELEPORT_GZ_PLUGIN_H

#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Pose.h>

class TeleportGzPlugin
        : public gazebo::ModelPlugin
{
		static constexpr std::string_view ROS_CMD_TOPIC = "teleport";
		static constexpr std::string_view SDF_PLATFORM_JOINT_PROP = "base_lift";

	public:
		~TeleportGzPlugin() override = default;

		void Init() override;
		void Reset() override;

		void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

	private:
		gazebo::physics::ModelPtr _model;
		gazebo::physics::JointPtr _liftJoint;

		double _baseOffset = 0.;

		ros::NodeHandlePtr _rn;
		ros::CallbackQueue _rCb;
		ros::Subscriber _commands;

		double _curLiftElevation = 0.;
		ignition::math::Pose3d _curPose;
		void OnRosCommand(const geometry_msgs::Pose &pose);

		gazebo::event::ConnectionPtr _onWorldBegin;
		void OnWorldBegin();

		gazebo::event::ConnectionPtr _onUpdateBegin;
		void OnUpdateBegin();
};

#endif // TELEPORT_GZ_PLUGIN_H
