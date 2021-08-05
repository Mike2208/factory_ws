#include "ros_robot_pose_publisher/ros_robot_pose_publisher.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>

void ROSRobotPosePublisher::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
	this->_model = model;

	{
		const auto robotNsPtr = sdf->GetElementImpl("ROSFrameID");
		if(robotNsPtr != nullptr)
		{
			this->_rosFrameID = robotNsPtr->GetValue()->GetAsString();
			//if(!this->_rosNs.empty())
			//	this->_rosNs = this->_rosNs;
		}
		else
		{
			this->_rosFrameID = this->_model->GetLink()->GetName();
		}
	}

	{
		const auto frameIdPtr = sdf->GetElementImpl("ROSWorldFrameID");
		if(frameIdPtr != nullptr)
			this->_rosWorldFrameID = frameIdPtr->GetValue()->GetAsString();
	}

//	{
//		const auto gzBaseLinkPtr = sdf->GetElement("GzBaseLink");
//		if(gzBaseLinkPtr != nullptr)
//		{
//			const auto linkName = gzBaseLinkPtr->GetValue()->GetAsString();
//			this->_baseLink = this->_model->GetLink(linkName);
//		}
//		else
//			this->_baseLink = this->_model->GetLink();
//	}

	this->_onWorldEndCon = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&ROSRobotPosePublisher::OnWorldEnd, this));
}

void ROSRobotPosePublisher::Reset()
{
	this->_prevPubTime = ros::Time(0);
}

void ROSRobotPosePublisher::OnWorldEnd()
{
	geometry_msgs::TransformStamped tfStamped;
	tfStamped.header.stamp = ros::Time::now();

	// Publish TF if ROS time has updates
	if(tfStamped.header.stamp > this->_prevPubTime + this->_pubRate.expectedCycleTime())
	{
		this->_prevPubTime = tfStamped.header.stamp;

		tfStamped.header.frame_id = this->_rosWorldFrameID;
		tfStamped.child_frame_id  = this->_rosFrameID;

		tfStamped.transform.translation.x = this->_model->WorldPose().Pos().X();
		tfStamped.transform.translation.y = this->_model->WorldPose().Pos().Y();
		tfStamped.transform.translation.z = this->_model->WorldPose().Pos().Z();

		tfStamped.transform.rotation.w    = this->_model->WorldPose().Rot().W();
		tfStamped.transform.rotation.x    = this->_model->WorldPose().Rot().X();
		tfStamped.transform.rotation.y    = this->_model->WorldPose().Rot().Y();
		tfStamped.transform.rotation.z    = this->_model->WorldPose().Rot().Z();

		this->_rtfBroadcaster.sendTransform(tfStamped);
	}
}

GZ_REGISTER_MODEL_PLUGIN(ROSRobotPosePublisher);
