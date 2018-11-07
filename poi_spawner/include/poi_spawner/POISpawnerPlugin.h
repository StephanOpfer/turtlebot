#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <map>
#include <ros/ros.h>
#include <string>

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
    void spawnPOI(std::string name, double x, double y);

private:
    /**
     * Callback that receives the contact sensor's update signal.
     */
    void OnModelUpdate(const common::UpdateInfo& info);

    // Pointer to the model
    physics::WorldPtr world;
    transport::PublisherPtr factoryPub;
    transport::NodePtr node;
    rendering::ScenePtr scene;
    std::map<std::string, bool> poiTextMap;
    std::string poi_model_xml;
    bool spawned;
    event::ConnectionPtr updateConnection;
};
} // namespace gazebo
