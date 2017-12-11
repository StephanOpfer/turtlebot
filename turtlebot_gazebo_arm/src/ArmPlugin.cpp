#include <gazebo/common/Plugin.hh>

#include "turtlebot_gazebo_arm/ArmPlugin.h"

#include <string>
#include <vector>
#include <cstdlib>
#include <numeric>
#include <ros/ros.h>

namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ArmPlugin)
////////////////////////////////////////////////////////////////////////////////
// Constructor
ArmPlugin::ArmPlugin() {
	this->spinner = nullptr;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ArmPlugin::~ArmPlugin() {
	ROS_DEBUG_STREAM_NAMED("arm", "Unloaded");
}

void ArmPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	// Make sure the ROS node for Gazebo has already been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM(
				"A ROS node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
	}

	this->model = _parent;
	this->world = this->model->GetWorld();

	ros::NodeHandle n;
	this->doorCmdSub = n.subscribe("/ArmCmd", 10, &ArmPlugin::onGrabDropObjectCmd, (ArmPlugin *) this);
	this->spinner = new ros::AsyncSpinner(4);
	this->spinner->start();

	ROS_INFO("ArmPlugin PlugIn loaded!");
}

void ArmPlugin::OnUpdate() {

}

void ArmPlugin::onGrabDropObjectCmd(turtlebot_gazebo_arm::GrabDropObjectPtr msg) {
	auto object = this->world->GetModel(msg->objectName);
}

}
