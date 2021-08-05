#include "cobotics_kuka_iiwa_control/cobotics_kuka_iiwa_control.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cobotics_kuka_iiwa_control");

	CoboticsKukaIIWAControl ctrl;

//	ctrl.FollowObject("target::base_link", tf2::Transform());
//	auto offset = tf2::Transform::getIdentity();
//	offset.setRotation(tf2::Quaternion(tf2::Vector3(0.,1.,0.), 1.57));

//	ctrl.IKFollowObject("skeleton_model::hand_palm_target", tf2::Transform::getIdentity());
	geometry_msgs::PoseStamped pose;
	{
		pose.header.frame_id = "skeleton_model::hand_palm_target";
		pose.header.stamp = ros::Time::now();

		pose.pose.position.x = 0.15;
		pose.pose.position.y = 0.0;
		pose.pose.position.z = 0.1;

		tf2::Quaternion quat(tf2::Vector3(1,0,0), -M_PI/2);
		pose.pose.orientation.w = quat.w();
		pose.pose.orientation.x = quat.x();
		pose.pose.orientation.y = quat.y();
		pose.pose.orientation.z = quat.z();
	}

	ctrl.IKFollowObject(pose);


	ctrl.Shutdown();

	return 0;
}
