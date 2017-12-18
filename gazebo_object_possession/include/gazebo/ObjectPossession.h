#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <cstdlib>
#include <iostream>
#include <functional>
#include <stdio.h>

namespace supplementary {
	class SystemConfig;
}

namespace gazebo {
class ObjectPossession: public WorldPlugin {
public:
	ObjectPossession();
	void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo & /*_info*/);
private:

	// Pointer to the model
	physics::WorldPtr model;

	// Pointer to the update event connection
	event::ConnectionPtr updateConnection;

	ros::Subscriber doorCmdSub;
	ros::AsyncSpinner* spinner;

	supplementary::SystemConfig* sc;
};
}
