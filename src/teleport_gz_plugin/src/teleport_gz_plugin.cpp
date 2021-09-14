#include "teleport_gz_plugin/teleport_gz_plugin.h"

#include <gazebo/physics/physics.hh>

GZ_REGISTER_MODEL_PLUGIN(TeleportGzPlugin);

void TeleportGzPlugin::Init()
{}

void TeleportGzPlugin::Reset()
{}

void TeleportGzPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
	this->_model = model;

	if(const auto sdfElement = sdf->GetElementImpl(SDF_PLATFORM_JOINT_PROP.data()))
	{
		const std::string liftJointName = sdfElement->GetValue()->GetAsString();
		this->_liftJoint = this->_model->GetJoint(liftJointName);
	}

	this->_rn = ros::NodeHandlePtr(new ros::NodeHandle());
	this->_rn->setCallbackQueue(&this->_rCb);
	this->_commands = this->_rn->subscribe(ROS_CMD_TOPIC.data(), 1, &TeleportGzPlugin::OnRosCommand, this);

	this->_onWorldBegin  = gazebo::event::Events::ConnectWorldCreated(std::bind(&TeleportGzPlugin::OnWorldBegin, this));
	this->_onUpdateBegin = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&TeleportGzPlugin::OnUpdateBegin, this));
}

void TeleportGzPlugin::OnRosCommand(const geometry_msgs::Pose &pose)
{
	auto newPose = this->_model->WorldPose();

	// Set Translation
	newPose.SetX(pose.position.x);
	newPose.SetY(pose.position.y);

	// Update z translation if above ground
	if(pose.position.z >= 0.)
	{
		if(this->_liftJoint)
			this->_curLiftElevation = pose.position.z;
		else
			newPose.SetZ(pose.position.z + this->_baseOffset);
	}

	// Set rotation, only rotate around z-Axis
	const auto eulerRot = ignition::math::Quaternion(pose.orientation.w, pose.orientation.x,
	                                                 pose.orientation.y, pose.orientation.z).Euler();
	newPose.Rot().Euler(0, 0, eulerRot.Z());

	// Update world pose (Done later in OnUpdateBegin)
	this->_curPose = newPose;
	//this->_model->SetWorldPose(newPose);
}

void TeleportGzPlugin::OnWorldBegin()
{
	// At world load, save model z-Pose
	this->_baseOffset = this->_model->WorldPose().Z();

	// Save current pose and lift elevation
	this->_curPose = this->_model->WorldPose();
	if(this->_liftJoint)
		this->_curLiftElevation = this->_liftJoint->Position(0);
}

void TeleportGzPlugin::OnUpdateBegin()
{
	// Check for new commands
	this->_rCb.callOne(ros::WallDuration(0.001));

	// Set model position
	this->_model->SetWorldPose(this->_curPose);
	if(this->_liftJoint)
		this->_liftJoint->SetPosition(0, this->_curLiftElevation);
}
