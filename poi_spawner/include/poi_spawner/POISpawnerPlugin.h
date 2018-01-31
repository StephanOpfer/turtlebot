#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
	class POISpawnerPlugin : public WorldPlugin
	{
	public:
		/**
		 * Constructor
		 */
		POISpawnerPlugin();

		/**
		 * Destructor.
		 */
		virtual ~POISpawnerPlugin();

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

		// Pointer to the model
		physics::WorldPtr world;
		event::ConnectionPtr updateConnection;
	};
}
