#include "poi_spawner/POISpawnerPlugin.h"

#include <SystemConfig.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/rendering/rendering.hh>
#include <ros/package.h>

#include <memory>
#include <vector>

namespace gazebo
{

POISpawnerPlugin::POISpawnerPlugin()
{
    std::string path = ros::package::getPath("turtlebot_bringup");
    std::ifstream in(supplementary::FileSystem::combinePaths(path, "/models/poi/poi.sdf"));
    this->poi_model_xml = std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    spawned = false;
}

POISpawnerPlugin::~POISpawnerPlugin() {}

void POISpawnerPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    this->world = _parent;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&POISpawnerPlugin::OnModelUpdate, this, _1));

    // Create a new transport node
    node = boost::make_shared<transport::Node>();

    // Initialize the node with the world name
    node->Init(this->world->Name());

    // Create a publisher on the ~/factory topic
    this->factoryPub = node->Advertise<msgs::Factory>("~/factory");

    std::cout << "Info: POISpawnerPlugin PlugIn loaded!" << std::endl;
}

void POISpawnerPlugin::spawnPOI(std::string name, double x, double y)
{
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(this->poi_model_xml);
    // Demonstrate using a custom model name.
    sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    std::string modelName = "poi_" + name;
    model->GetAttribute("name")->SetFromString(modelName);
    model->GetElement("pose")->Set(std::to_string(x) + " " + std::to_string(y) + " 0.02 0 0 0");
    world->InsertModelSDF(sphereSDF);
    this->poiTextMap.emplace(modelName, false);
}

void POISpawnerPlugin::OnModelUpdate(const common::UpdateInfo& info)
{
    if (!spawned) {
        // Read rooms with its connected areas and pois from config
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        auto roomNames = (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", NULL);
        for (auto& roomName : (*roomNames)) {
            try {
                auto poisIDStrings = (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", roomName.c_str(), "POIs", NULL);
                for (auto poiIDString : (*poisIDStrings)) {
                    double x = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs", poiIDString.c_str(), "X", NULL);
                    double y = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs", poiIDString.c_str(), "Y", NULL);
                    this->spawnPOI(poiIDString, x, y);
                }
            } catch (std::exception& e) {
                continue;
            }
        }
        spawned = true;
    }
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(POISpawnerPlugin)
} // namespace gazebo
