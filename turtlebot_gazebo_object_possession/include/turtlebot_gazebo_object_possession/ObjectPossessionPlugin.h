#pragma once

#include <string>

#include <ttb_msgs/GrabDropObject.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <chrono>
#include <ros/ros.h>
#include <map>
#include <mutex>

namespace gazebo
{
	class ObjectPossessionPlugin : public WorldPlugin
	{
	public:
		/**
		 * Constructor
		 */
		ObjectPossessionPlugin();

		/**
		 * Destructor.
		 */
		virtual ~ObjectPossessionPlugin();

		/**
		 * Load the sensor plugin.
		 * @param _sensor Pointer to the sensor that loaded this plugin.
		 * @param _sdf SDF element that describes the plugin.
		 */

		virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

	private:
		/**
		 * Callback that receives the contact sensor's update signal.
		 */
		void OnUpdate(const common::UpdateInfo& info);

		void onGrabDropObjectCmd(ttb_msgs::GrabDropObjectPtr msg);

		ros::Subscriber armCmdSub;
		ros::Publisher pub;
		ros::AsyncSpinner* spinner;
		ros::NodeHandle n;

		// Pointer to the model
		physics::WorldPtr world;

		event::ConnectionPtr updateConnection;
		std::mutex publisherMutex;
		std::map<std::string, std::string> objectPossession;
	};
}
