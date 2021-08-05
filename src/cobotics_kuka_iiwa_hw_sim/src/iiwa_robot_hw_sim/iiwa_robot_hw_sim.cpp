#include "iiwa_robot_hw_sim/iiwa_robot_hw_sim.h"

#include <angles/angles.h>
#include <assert.h>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <pluginlib/class_list_macros.h>


constexpr inline double clamp(const double val, const double min_val, const double max_val)
{
	return std::min(std::max(val, min_val), max_val);
}

bool IIWARobotHWSim::initSim(const std::string &robot_namespace,
                             ros::NodeHandle model_nh,
                             gazebo::physics::ModelPtr parent_model,
                             const urdf::Model *const urdf_model,
                             std::vector<transmission_interface::TransmissionInfo> transmissions)
{
	std::string gzRootLink = "iiwa_link_0";
	std::string rosRootLink = "iiwa::iiwa_link_0";
	{
		XmlRpc::XmlRpcValue params;
		model_nh.getParam("iiwa_ctrl", params);

		if(params.hasMember("gazebo_root_link"))
			gzRootLink = params["gazebo_root_link"].operator std::__cxx11::basic_string<char> &();
		if(params.hasMember("ros_root_link"))
			rosRootLink = params["ros_root_link"].operator std::__cxx11::basic_string<char> &();
	}

	this->_model = parent_model;
	this->_world = parent_model->GetWorld();
	this->_physicsType = this->_world->Physics()->GetType();

	std::array<std::string, 7> jointNames;

	// Iterate over URDF, extract information
	{
		auto curLink = urdf_model->getLink(rosRootLink);
		for(uint linkNum = 0; linkNum < 8; ++linkNum)
		{
			if(curLink == nullptr)
			{
				ROS_ERROR("Unable to find child link %u in iiwa chain for robot \"%s\". Started at link \"%s\"",
				          linkNum, robot_namespace.c_str(), rosRootLink.c_str());
				throw std::runtime_error("Failed to get all child links for iiwa");
			}

			// Extract iiwa chain from urdf
			if(linkNum > 0)
			{
//				this->_jointToLink[linkNum] = curLink->parent_joint->parent_to_joint_origin_transform.position;

//				// Invert, to get proper vector direction (joint --> link)
//				this->_jointToLink[linkNum].x = -this->_jointToLink[linkNum].x;
//				this->_jointToLink[linkNum].y = -this->_jointToLink[linkNum].y;
//				this->_jointToLink[linkNum].z = -this->_jointToLink[linkNum].z;
			}

			if(linkNum < 7)
			{
				if(curLink->child_joints.size() != 1)
				{
					ROS_ERROR("Unexpected number of child joints for link \"%s\" in iiwa chain. Expected 1, got %zu",
					          curLink->name.c_str(), curLink->child_joints.size());

					throw std::runtime_error("Unexpected number of child joints for link");
				}

//				this->_linkToJoint[linkNum] = curLink->child_joints[0]->parent_to_joint_origin_transform.position;

				auto transIt = transmissions.begin();
				for(; transIt != transmissions.end(); ++transIt)
				{
					if(transIt->joints_.at(0).name_ == curLink->child_joints[0]->name)
						break;
				}

				if(transIt == transmissions.end())
				{
					ROS_ERROR("Unable to find transmission for joint \"%s\"", curLink->child_joints[0]->name.c_str());

					throw std::runtime_error("Unable to find transmission for joint");
				}

				this->registerJointInterfaces(robot_namespace, model_nh, parent_model, urdf_model,
				                              *transIt, curLink->child_joints[0]->name, linkNum);

				transmissions.erase(transIt);
			}

			// Next joint
			if(linkNum < 7)
			{
				curLink = curLink->child_links.at(0);
			}
		}

		// Register any additional interfaces (all of the transmissions handled by our hw_sim have been removed from `transmissions`)
		for(auto transIt = transmissions.begin(); transIt != transmissions.end(); ++transIt)
		{
			this->_extraJointData.push_back(ExtraJointData());

			// TODO: Add check that this transmission only has one joint
			this->registerExtraJointInterfaces(robot_namespace, model_nh, parent_model, urdf_model,
			                                   *transIt, transIt->joints_.at(0).name_, &(this->_extraJointData.back()));

			auto gzJoint = this->_model->GetJoint(transIt->joints_.at(0).name_);
			if(gzJoint == nullptr)
			{
				ROS_ERROR("Unable to find gazebo model joint \"%s\"", transIt->joints_.at(0).name_.c_str());
				throw std::runtime_error("Unable to find gazebo model joint");
			}

			this->_extraJointData.back().GzJoint = gzJoint;
		}
	}

	{
		auto curLink = this->_model->GetLink(gzRootLink);
		for(uint linkNum = 0; linkNum < 8; ++linkNum)
		{
			if(curLink == nullptr)
			{
				ROS_ERROR("Unable to find gazebo model child link %u in iiwa chain for robot \"%s\". Started at link \"%s\"",
				          linkNum, parent_model->GetName().c_str(), gzRootLink.c_str());
				throw std::runtime_error("Failed to get all gazebo model child links for iiwa");
			}

			// Store joint
			this->_gzLinks[linkNum] = curLink;

			if(linkNum > 0)
			{
//				this->_gzJointToLink[linkNum] = curLink->InitialRelativePose().Pos() - curLink->GetParentJoints().at(0)->InitialAnchorPose().Pos();
			}

			if(linkNum < 7)
			{
				if(const auto numChildJoints = curLink->GetChildJoints().size(); numChildJoints != 1)
				{
					ROS_ERROR("Unexpected number of gazebo model child joints for link \"%s\" in iiwa chain. Expected 1, got %zu",
					          curLink->GetName().c_str(), numChildJoints);

					throw std::runtime_error("Unexpected number of child joints for link");
				}

				this->_gzJoints[linkNum] = curLink->GetChildJoints().at(0);

//				this->_gzLinkToJoint[linkNum] = this->_gzJoints[linkNum]->InitialAnchorPose().Pos() - curLink->InitialRelativePose().Pos();

				this->_jPos[linkNum] = this->_gzJoints[linkNum]->Position(0);
				this->_jVel[linkNum] = this->_gzJoints[linkNum]->GetVelocity(0);
				this->_jEff[linkNum] = this->_gzJoints[linkNum]->GetForce(0);
			}

			// Next joint
			if(linkNum < 7)
			{
				curLink = curLink->GetChildJointsLinks().at(0);
			}
		}
	}

	registerInterface(&this->_jsInterface);
	registerInterface(&this->_pjInterface);
	registerInterface(&this->_ejInterface);

	return true;
}

void IIWARobotHWSim::readSim(ros::Time time, ros::Duration period)
{
	for(unsigned int j=0; j < 7; j++)
	{
		double position = this->_gzJoints[j]->Position(0);
		this->_jPos[j] += angles::shortest_angular_distance(this->_jPos[j], position);

		this->_jVel[j] = this->_gzJoints[j]->GetVelocity(0);
		this->_jEff[j] = this->_gzJoints[j]->GetForce((unsigned int)(0));
	}

	for(auto jointIt = this->_extraJointData.begin(); jointIt != this->_extraJointData.end(); ++jointIt)
	{
		double position = jointIt->GzJoint->Position(0);
		if (jointIt->Type == urdf::Joint::PRISMATIC)
		{
			jointIt->Pos = position;
		}
		else
		{
			jointIt->Pos += angles::shortest_angular_distance(jointIt->Pos,
			                                                  position);
		}
		jointIt->Vel = jointIt->GzJoint->GetVelocity(0);
		jointIt->Eff = jointIt->GzJoint->GetForce((unsigned int)(0));
	}
}


void IIWARobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
	std::array<double, 7> jointTargets;

	if(this->_eStopActive)
	{
		jointTargets = this->_eStopPosTargets;
	}
	else
	{
		for(uint i=0; i<7; ++i)
		{
			double error;
			angles::shortest_angular_distance_with_limits(this->_jPos[i],
			                                              this->_jPosCmd[i],
			                                              this->_jPosLowLim[i],
			                                              this->_jPosUppLim[i],
			                                              error);

			const double target = this->_jPosPID[i].computeCommand(error, period);
			jointTargets[i] = target;

		}
	}

	this->_pjSatInterface.enforceLimits(period);
	this->_pjLimInterface.enforceLimits(period);

	std::array<pos_vector_t, 8> lLinAcc;
	std::array<pos_vector_t, 8> lAngAcc;

	std::array<pos_vector_t, 7> forces;
	std::array<pos_vector_t, 7> torques;

	// Base link doesn't move
	lLinAcc[0].Set(0,0,0);
	lAngAcc[0].Set(0,0,0);

	// Convert to global coordinates
	std::array<pos_vector_t, 7> jAccTarget;
	for(uint i=0; i<7; ++i)
	{
		//jAccTarget[i] = this->_gzJoints[i]->AxisFrame(0)*(this->_gzJoints[i]->LocalAxis(0)*jointTargets[i]);

		const auto axisFrame = this->_gzJoints[i]->AxisFrame(0);
		const auto localAxis = this->_gzJoints[i]->LocalAxis(0);
		jAccTarget[i] = this->_gzJoints[i]->AxisFrame(0)*(this->_gzJoints[i]->LocalAxis(0)*jointTargets[i]);
	}

	// Compute rotational link acceleration targets
	for(uint i=1; i<8; ++i)
	{
		lAngAcc[i] = lAngAcc[i-1] + jAccTarget[i-1];
	}

	// Compute linear link acceleration targets
	for(uint i=1; i<8; ++i)
	{
		const auto &jParent = this->_gzJoints[i-1];

		const auto &lParent = this->_gzLinks[i-1];
		const auto &lLink = this->_gzLinks[i];

		const auto parentJointVec = jParent->WorldPose().Pos() - lParent->WorldPose().Pos();
		const auto jointLinkVec   = lLink->WorldPose().Pos()  - jParent->WorldPose().Pos();

		const auto lParentAngVel = lParent->WorldAngularVel();
		const auto lLinkAngVel = lLink->WorldAngularVel();

		lLinAcc[i] = lLinAcc[i-1]
		        + lParent->WorldAngularVel().Cross(lParent->WorldAngularVel().Cross(parentJointVec))
		        + lLink->WorldAngularVel().Cross(  lLink->WorldAngularVel().Cross(  jointLinkVec  ))
		        + lAngAcc[i-1].Cross(parentJointVec)
		        + lAngAcc[i  ].Cross(jointLinkVec  );
	}


	// Compute forces on end of chain first
	{
		const auto &jParent = this->_gzJoints[6];
		const auto &lLink = this->_gzLinks[7];
		const auto jParentLinkVec = lLink->WorldPose().Pos() - jParent->WorldPose().Pos();

		const auto mass = this->_gzLinks[6]->GetInertial()->Mass();
		const auto moi = this->_gzLinks[6]->GetInertial()->MOI();

		forces[6] = lLink->GetInertial()->Mass() * (lLinAcc[7] - this->_world->Gravity());
		torques[6] = lLink->GetInertial()->MOI().operator*(lAngAcc[7])
		        + jParentLinkVec.Cross(forces[6]);
	}

	// Compute forces for rest of chain
	for(int i=5; i>=0; --i)
	{
		const auto &jParent = this->_gzJoints[i];
		const auto &jChild = this->_gzJoints[i+1];

		const auto &lLink = this->_gzLinks[i+1];

		const auto jParentLinkVec = lLink->WorldPose().Pos()  - jParent->WorldPose().Pos();
		const auto linkjChildVec  = jChild->WorldPose().Pos() - lLink->WorldPose().Pos();

		const auto mass = this->_gzLinks[6]->GetInertial()->Mass();
		const auto moi = this->_gzLinks[6]->GetInertial()->MOI();

		forces[i]  = lLink->GetInertial()->Mass() * (lLinAcc[i+1] - this->_world->Gravity())
		        + forces[i+1];

		torques[i] = lLink->GetInertial()->MOI()  * lAngAcc[i+1]
		        + torques[i+1]
		        + jParentLinkVec.Cross(forces[i])
		        + linkjChildVec.Cross(forces[i+1]);
	}

	// Compute Joint Torques and apply
	std::array<double, 7> jointTorques;
	for(uint i=0; i<7; i++)
	{
//		jointTorques[i] = clamp(this->_gzJoints[i]->AxisFrame(0).RotateVectorReverse(torques[i]).Dot(this->_gzJoints[i]->LocalAxis(0)),
//		                        -this->_jEffLim[i], this->_jEffLim[i]);
		jointTorques[i] = this->_gzJoints[i]->AxisFrame(0).RotateVectorReverse(torques[i]).Dot(this->_gzJoints[i]->LocalAxis(0));

		// Apply torques
		this->_gzJoints[i]->SetForce(0, jointTorques[i]);
	}

	this->writeExtraJoints(time, period);
}

void IIWARobotHWSim::eStopActive(const bool active)
{
	this->_eStopActive = active;
	for(uint i=0; i<6; ++i)
	{
		this->_eStopPosTargets[i] = this->_gzJoints[i]->Position(0);
	}

	for(auto &joint : this->_extraJointData)
	{
		switch (joint.CtrlMeth)
		{
			case POSITION:
			case POSITION_PID:
				joint.EStopTarget = joint.Pos;

			case VELOCITY:
			case VELOCITY_PID:
				joint.EStopTarget = joint.Vel;

			case EFFORT:
				joint.EStopTarget = joint.Eff;
		}
	}
}

bool IIWARobotHWSim::registerJointInterfaces(const std::string &robot_namespace, ros::NodeHandle model_nh,
                                             gazebo::physics::ModelPtr parent_model, const urdf::Model *const urdf_model,
                                             const transmission_interface::TransmissionInfo &transmission,
                                             const std::string &joint_name, std::size_t joint_index)
{
	// getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
	// parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
	const ros::NodeHandle joint_limit_nh(model_nh);

	// Check that this transmission has one joint
	if(transmission.joints_.size() == 0)
	{
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmission.name_ << " has no associated joints.");
		return true;
	}
	else if(transmission.joints_.size() > 1)
	{
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmission.name_
		    << " has more than one joint. Currently the default robot hardware simulation "
		    << " interface only supports one.");
		return true;
	}

	std::vector<std::string> joint_interfaces = transmission.joints_[0].hardware_interfaces_;
	if (joint_interfaces.empty() &&
	    !(transmission.actuators_.empty()) &&
	    !(transmission.actuators_[0].hardware_interfaces_.empty()))
	{
		// TODO: Deprecate HW interface specification in actuators in ROS J
		joint_interfaces = transmission.actuators_[0].hardware_interfaces_;
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
		    transmission.name_ << " should be nested inside the <joint> element, not <actuator>. " <<
		    "The transmission will be properly loaded, but please update " <<
		    "your robot model to remain compatible with future versions of the plugin.");
	}
	if (joint_interfaces.empty())
	{
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmission.joints_[0].name_ <<
		    " of transmission " << transmission.name_ << " does not specify any hardware interface. " <<
		    "Not adding it to the robot hardware simulation.");
		return true;
	}
	else if (joint_interfaces.size() > 1)
	{
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmission.joints_[0].name_ <<
		    " of transmission " << transmission.name_ << " specifies multiple hardware interfaces. " <<
		    "Currently the default robot hardware simulation interface only supports one. Using the first entry");
		//continue;
	}

	// Add data from transmission
	//joint_names_[j] = transmissions[j].joints_[0].name_;
	this->_jPos[joint_index] = 1.0;
	this->_jVel[joint_index] = 0.0;
	this->_jEff[joint_index] = 0.0;;  // N/m for continuous joints
	//joint_effort_command_[j] = 0.0;
	this->_jPosCmd[joint_index] = 0.0;
	//joint_velocity_command_[j] = 0.0;

	const std::string& hardware_interface = joint_interfaces.front();

	// Debug
	ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_name
	    << "' of type '" << hardware_interface << "'");

	// Create joint state interface for all joints
	this->_jsInterface.registerHandle(hardware_interface::JointStateHandle(
	    joint_name, &this->_jPos[joint_index], &this->_jVel[joint_index], &this->_jEff[joint_index]));

	// Decide what kind of command interface this actuator/joint has
	hardware_interface::JointHandle joint_handle;
	if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")
	{
		// Create position joint interface
		//joint_control_methods_[j] = POSITION;
		joint_handle = hardware_interface::JointHandle(this->_jsInterface.getHandle(joint_name),
		                                               &this->_jPosCmd[joint_index]);
		this->_pjInterface.registerHandle(joint_handle);
	}
	else
	{
		ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
		<< hardware_interface << "' while loading interfaces for " << joint_name );
		return false;
	}

	if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface")
	{
		ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface <<
		                "' within the <hardwareInterface> tag in joint '" << joint_name << "'.");
	}

	// get physics engine type
//	gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
//	physics_type_ = physics->GetType();
//	if (physics_type_.empty())
//	{
//		ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "No physics type found.");
//	}

	registerJointLimits(joint_name, joint_handle, POSITION,
	                    joint_limit_nh, urdf_model,
	                    nullptr,
	                    &this->_jPosLowLim[joint_index], &this->_jPosUppLim[joint_index],
	                    &this->_jEffLim[joint_index]);

	// Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
	// joint->SetParam("vel") to control the joint.
	const ros::NodeHandle nh(robot_namespace + "/gazebo_ros_control/pid_gains/" +
	                         joint_name);
	this->_jPosPID[joint_index].init(nh);

	return true;
}

bool IIWARobotHWSim::registerExtraJointInterfaces(const std::string &robot_namespace,
                                                  ros::NodeHandle model_nh,
                                                  gazebo::physics::ModelPtr parent_model,
                                                  const urdf::Model *const urdf_model,
                                                  const transmission_interface::TransmissionInfo &transmission,
                                                  const std::string &joint_name,
                                                  IIWARobotHWSim::ExtraJointData *pJointData)
{
	// getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
	// parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
	const ros::NodeHandle joint_limit_nh(model_nh);

	// Check that this transmission has one joint
	if(transmission.joints_.size() == 0)
	{
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmission.name_ << " has no associated joints.");
		return true;
	}
	else if(transmission.joints_.size() > 1)
	{
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmission.name_
		    << " has more than one joint. Currently the default robot hardware simulation "
		    << " interface only supports one.");
		return true;
	}

	std::vector<std::string> joint_interfaces = transmission.joints_[0].hardware_interfaces_;
	if (joint_interfaces.empty() &&
	    !(transmission.actuators_.empty()) &&
	    !(transmission.actuators_[0].hardware_interfaces_.empty()))
	{
		// TODO: Deprecate HW interface specification in actuators in ROS J
		joint_interfaces = transmission.actuators_[0].hardware_interfaces_;
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
		    transmission.name_ << " should be nested inside the <joint> element, not <actuator>. " <<
		    "The transmission will be properly loaded, but please update " <<
		    "your robot model to remain compatible with future versions of the plugin.");
	}
	if (joint_interfaces.empty())
	{
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmission.joints_[0].name_ <<
		    " of transmission " << transmission.name_ << " does not specify any hardware interface. " <<
		    "Not adding it to the robot hardware simulation.");
		return true;
	}
	else if (joint_interfaces.size() > 1)
	{
		ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmission.joints_[0].name_ <<
		    " of transmission " << transmission.name_ << " specifies multiple hardware interfaces. " <<
		    "Currently the default robot hardware simulation interface only supports one. Using the first entry");
		//continue;
	}

	// Add data from transmission
	//joint_names_[j] = transmissions[j].joints_[0].name_;
	pJointData->Pos = 1.0;
	pJointData->Vel = 0.0;
	pJointData->Eff = 0.0;;  // N/m for continuous joints
	//joint_effort_command_[j] = 0.0;
	pJointData->PosCmd = 0.0;
	//joint_velocity_command_[j] = 0.0;

	const std::string& hardware_interface = joint_interfaces.front();

	// Debug
	ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_name
	    << "' of type '" << hardware_interface << "'");

	// Create joint state interface for all joints
	this->_jsInterface.registerHandle(hardware_interface::JointStateHandle(
	    joint_name, &pJointData->Pos, &pJointData->Vel, &pJointData->Eff));

	// Decide what kind of command interface this actuator/joint has
	// TODO: Add other controller methods from DefaultRobotHWSim back
	hardware_interface::JointHandle joint_handle;
	if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
	{
		// Create effort joint interface
		pJointData->CtrlMeth = EFFORT;
		joint_handle = hardware_interface::JointHandle(this->_jsInterface.getHandle(joint_name),
		                                               &pJointData->PosCmd);
		this->_ejInterface.registerHandle(joint_handle);
	}
	else
	{
		ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
		<< hardware_interface << "' while loading interfaces for " << joint_name );
		return false;
	}

	if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface")
	{
		ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface <<
		                "' within the <hardwareInterface> tag in joint '" << joint_name << "'.");
	}

	registerJointLimits(joint_name, joint_handle, pJointData->CtrlMeth,
	                    joint_limit_nh, urdf_model,
	                    &pJointData->Type,
	                    &pJointData->PosLowLim, &pJointData->PosUppLim,
	                    &pJointData->EffLim);

	if (pJointData->CtrlMeth != EFFORT)
	{
		// Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
		// joint->SetParam("vel") to control the joint.
		const ros::NodeHandle nh(robot_namespace + "/gazebo_ros_control/pid_gains/" +
		                         joint_name);
		if(pJointData->EffPID.init(nh))
		{
			switch (pJointData->CtrlMeth)
			{
				case POSITION:
					pJointData->CtrlMeth = POSITION_PID;
					break;
				case VELOCITY:
					pJointData->CtrlMeth = VELOCITY_PID;
					break;
				default:
					break;
			}
		}
		else
		{
			// joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
			// going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
			// going to be called.
			pJointData->GzJoint->SetParam("fmax", 0, pJointData->EffLim);
		}
	}

	return true;
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void IIWARobotHWSim::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle,
                         const ControlMethod ctrl_method,
                         const ros::NodeHandle& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         int *const joint_type,
                         double *const lower_limit, double *const upper_limit,
                         double *const effort_limit)
{
	*lower_limit = -std::numeric_limits<double>::max();
	*upper_limit = std::numeric_limits<double>::max();
	*effort_limit = std::numeric_limits<double>::max();

	joint_limits_interface::JointLimits limits;
	bool has_limits = false;
	joint_limits_interface::SoftJointLimits soft_limits;
	bool has_soft_limits = false;

	if (urdf_model != NULL)
	{
		const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
		if (urdf_joint != NULL)
		{
			if(joint_type != nullptr)
			{
				*joint_type = urdf_joint->type;
			}
			else
			{
				// Make sure that all IIWA joints are the correct type
				assert(urdf_joint->type == urdf::Joint::REVOLUTE);
			}

			// Get limits from the URDF file.
			if (joint_limits_interface::getJointLimits(urdf_joint, limits))
				has_limits = true;
			if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
				has_soft_limits = true;
		}
	}

	// Get limits from the parameter server.
	if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
		has_limits = true;

	if (!has_limits)
		return;

	if (limits.has_position_limits)
	{
		*lower_limit = limits.min_position;
		*upper_limit = limits.max_position;
	}
	if (limits.has_effort_limits)
		*effort_limit = limits.max_effort;

	if (has_soft_limits)
	{
		switch (ctrl_method)
		{
			case EFFORT:
			{
				const joint_limits_interface::EffortJointSoftLimitsHandle
				        limits_handle(joint_handle, limits, soft_limits);
				this->_ejLimInterface.registerHandle(limits_handle);
			}
			break;

			case POSITION:
			{
				const joint_limits_interface::PositionJointSoftLimitsHandle
				limits_handle(joint_handle, limits, soft_limits);
				this->_pjLimInterface.registerHandle(limits_handle);
				break;
			}

			default:
				ROS_FATAL_STREAM_NAMED("default_robot_hw_sim", "Unsupported control interface specified for '" << joint_name << "'" );
				throw std::runtime_error("Unsupported control interface type");
				break;
		}
	}
	else
	{
		switch (ctrl_method)
		{
			case EFFORT:
			{
				const joint_limits_interface::EffortJointSaturationHandle
				        sat_handle(joint_handle, limits);
				this->_ejSatInterface.registerHandle(sat_handle);
				break;
			}
			case POSITION:
			{
				const joint_limits_interface::PositionJointSaturationHandle
				sat_handle(joint_handle, limits);
				this->_pjSatInterface.registerHandle(sat_handle);
				break;
			}
			default:
				ROS_FATAL_STREAM_NAMED("default_robot_hw_sim", "Unsupported control interface specified for '" << joint_name << "'" );
				throw std::runtime_error("Unsupported control interface type");
				break;
		}
	}
}

void IIWARobotHWSim::writeExtraJoints(ros::Time time, ros::Duration period)
{
	if(this->_eStopActive)
	{
		for(auto &joint : this->_extraJointData)
		{
			joint.PosCmd = joint.EStopTarget;
		}
	}

	this->_ejSatInterface.enforceLimits(period);
	this->_ejLimInterface.enforceLimits(period);

	for(auto jointIt = this->_extraJointData.begin(); jointIt != this->_extraJointData.end(); ++jointIt)
	{
		switch (jointIt->CtrlMeth)
		{
			case EFFORT:
			{
				const double effort = this->_eStopActive ? 0 : jointIt->PosCmd;
				jointIt->GzJoint->SetForce(0, effort);
				break;
			}

			case POSITION:
				jointIt->GzJoint->SetPosition(0, jointIt->PosCmd, true);
				break;

			case POSITION_PID:
			{
				double error;
				switch (jointIt->Type)
				{
					case urdf::Joint::REVOLUTE:
						angles::shortest_angular_distance_with_limits(jointIt->Pos,
						                                              jointIt->PosCmd,
						                                              jointIt->PosLowLim,
						                                              jointIt->PosUppLim,
						                                              error);
						break;

					case urdf::Joint::CONTINUOUS:
						error = angles::shortest_angular_distance(jointIt->Pos,
						                                          jointIt->PosCmd);
						break;

					default:
						error = jointIt->PosCmd - jointIt->Pos;
				}

				const double effort_limit = jointIt->EffLim;
				const double effort = clamp(jointIt->EffPID.computeCommand(error, period),
				                            -effort_limit, effort_limit);
				jointIt->GzJoint->SetForce(0, effort);
				break;
			}

			case VELOCITY:
			{
				if (this->_physicsType.compare("dart") == 0)
				{
					jointIt->GzJoint->SetVelocity(0, this->_eStopActive ? 0 : jointIt->PosCmd);
				}
				else
				{
					jointIt->GzJoint->SetParam("vel", 0, this->_eStopActive ? 0 : jointIt->PosCmd);
				}
				break;
			}

			case VELOCITY_PID:
			{
				double error;
				if (this->_eStopActive)
					error = -jointIt->Vel;
				else
					error = jointIt->PosCmd - jointIt->Vel;
				const double effort_limit = jointIt->EffLim;
				const double effort = clamp(jointIt->EffPID.computeCommand(error, period),
				                            -effort_limit, effort_limit);
				jointIt->GzJoint->SetForce(0, effort);
				break;
			}
		}
	}
}

PLUGINLIB_EXPORT_CLASS(IIWARobotHWSim, gazebo_ros_control::RobotHWSim);
