#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "hinged_door_controller/DoorCmd.h"

#include <cstdlib>
#include <iostream>
#include <functional>
#include <stdio.h>

namespace gazebo {
class HingedDoorController: public ModelPlugin {
public:
	HingedDoorController();
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo & /*_info*/);
	void handleDoorCmd (hinged_door_controller::DoorCmdPtr cmd);
private:

	// Pointer to the model
	physics::ModelPtr model;

	// Pointer to the update event connection
	event::ConnectionPtr updateConnection;

	ros::Subscriber doorCmdSub;
	ros::AsyncSpinner* spinner;
};
}
