#pragma once

#include <string>

#include <ttb_msgs/GrabDropObject.h>
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

		void onGrabDropObjectCmd(ttb_msgs::GrabDropObjectPtr msg);

		ros::Subscriber armCmdSub;
		ros::AsyncSpinner* spinner;

		// Pointer to the model
		physics::ModelPtr model;
		physics::ModelPtr transportedModel;
		physics::ModelPtr previousTransportedModel;
		physics::WorldPtr world;

		event::ConnectionPtr updateConnection;
		double armRange = 3.0;
	};
}
