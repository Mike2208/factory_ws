#ifndef ROS_ROBOT_POSE_PUBLISHER_H
#define ROS_ROBOT_POSE_PUBLISHER_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/*!
 * \brief Gazebo Model Plugin to publish object pose as ROS TF
 */
class ROSRobotPosePublisher
        : public gazebo::ModelPlugin
{
	public:
		virtual ~ROSRobotPosePublisher() = default;

		void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

//		void Init() override;
		void Reset() override;

	private:
		tf2_ros::TransformBroadcaster _rtfBroadcaster;

		tf2_ros::Buffer _rtfBuffer;
		tf2_ros::TransformListener _rtfListener = tf2_ros::TransformListener(this->_rtfBuffer);

		gazebo::event::ConnectionPtr _onWorldEndCon;
		gazebo::physics::ModelPtr _model;
		//gazebo::physics::LinkPtr _baseLink;

		/*!
		 * \brief ID of published frame
		 */
		std::string _rosFrameID = "";

		/*!
		 * \brief Frame ID of Gazebo world origin.
		 * Can be different from ROS origin
		 */
		std::string _rosWorldFrameID = "map";

		/*!
		 * \brief TF publish rate
		 */
		ros::Rate _pubRate = ros::Rate(100);

		ros::Time _prevPubTime = ros::Time(0);

		void OnWorldEnd();
};

#endif // ROS_ROBOT_STATE_PUBLISHER_H
