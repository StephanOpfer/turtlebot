#pragma once

#include <string>

#include "turtlebot_gazebo_arm/GrabDropObject.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <chrono>
#include <ros/ros.h>
#include <map>

namespace gazebo
{
	class ArmPlugin : public ModelPlugin
	{
	public:
		/**
		 * Constructor
		 */
		ArmPlugin();

		/**
		 * Destructor.
		 */
		virtual ~ArmPlugin();

		/**
		 * Load the sensor plugin.
		 * @param _sensor Pointer to the sensor that loaded this plugin.
		 * @param _sdf SDF element that describes the plugin.
		 */

		virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	private:
		/**
		 * Callback that receives the contact sensor's update signal.
		 */
		void OnUpdate(const common::UpdateInfo& info);

		void onGrabDropObjectCmd(turtlebot_gazebo_arm::GrabDropObjectPtr msg);

		ros::Subscriber doorCmdSub;
		ros::AsyncSpinner* spinner;

		// Pointer to the model
		physics::ModelPtr model;
		physics::ModelPtr transportedModel;
		physics::WorldPtr world;

		event::ConnectionPtr updateConnection;
		double armRange = 1.0;
	};
}
