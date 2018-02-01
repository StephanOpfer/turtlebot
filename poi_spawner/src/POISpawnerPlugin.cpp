#include "poi_spawner/POISpawnerPlugin.h"

#include <SystemConfig.h>

#include <gazebo/physics/World.hh>
#include <gazebo_msgs/SpawnModel.h>
#include <ros/master.h>
#include <ros/package.h>

#include <memory>
#include <vector>

namespace gazebo
{

POISpawnerPlugin::POISpawnerPlugin()
{
    string path = ros::package::getPath("turtlebot_bringup");
    cout << "POISpawnerPlugin: Path to turtlebot_bringup " << path << endl;
    ifstream in(supplementary::FileSystem::combinePaths(path, "/models/poi/poi.sdf"));
    this->poi_model_xml = string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    spawned = false;
}

POISpawnerPlugin::~POISpawnerPlugin()
{
}

void POISpawnerPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    robotSpawnServiceClient = this->n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    this->world = _parent;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&POISpawnerPlugin::OnUpdate, this, _1));
}

void POISpawnerPlugin::spawnPOI(string name, double x, double y)
{
    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(this->world->GetName());

    // Create a publisher on the ~/factory topic
    transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

    // Create the message
    msgs::Factory msg;

    // Model file to load
    msg.set_sdf(this->poi_model_xml);

    // Pose to initialize the model to
    msgs::Set(msg.mutable_pose(),
              ignition::math::Pose3d(ignition::math::Vector3d(x, y, 0), ignition::math::Quaterniond(0, 0, 0)));

    // Send the message
    factoryPub->Publish(msg);
}

void POISpawnerPlugin::OnUpdate(const common::UpdateInfo &info)
{
    if (spawned)
        return;

    // Read rooms with its connected areas and pois from config
    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    auto roomNames = (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", NULL);
    for (auto &roomName : (*roomNames))
    {
        try
        {
            auto poisIDStrings =
                (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", roomName.c_str(), "POIs", NULL);
            for (auto poiIDString : (*poisIDStrings))
            {
                double x = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs",
                                                                  poiIDString.c_str(), "X", NULL);
                double y = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs",
                                                                  poiIDString.c_str(), "Y", NULL);
                this->spawnPOI(poiIDString, x, y);
            }
        }
        catch (exception &e)
        {
            continue;
        }
    }
    spawned = true;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(POISpawnerPlugin)
}
