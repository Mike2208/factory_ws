#ifndef IIWA_ROBOT_HW_SIM_PLUGIN_H
#define IIWA_ROBOT_HW_SIM_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>

class IIWARobotHWSimPlugin
        : public gazebo::SystemPlugin
{
	public:
		virtual void Load(int _argc = 0, char **_argv = nullptr) override;
//		virtual void Init() override;
//		virtual void Reset() override;
};

#endif // IIWA_ROBOT_HW_SIM_PLUGIN_H
