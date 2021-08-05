#include "iiwa_robot_hw_sim/iiwa_robot_hw_sim_plugin.h"
#include "iiwa_robot_hw_sim/iiwa_robot_hw_sim.h"

#include <class_loader/class_loader.hpp>

void IIWARobotHWSimPlugin::Load(int, char **)
{
	gzerr << "Loading IIWA Plugin";
	ROS_ERROR("Loading IIWA Plugin");
}

GZ_REGISTER_SYSTEM_PLUGIN(IIWARobotHWSimPlugin);
